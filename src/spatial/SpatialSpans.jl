"""Shared marker-to-marker span coordinates for spatial elements."""
module SpatialSpans

using LinearAlgebra
using ..AutomaticAnalysis
using ..SpatialComponentAssembly
using ..SpatialModeling: spatial_marker_position, spatial_marker_velocity,
    spatial_marker_acceleration
using ..SpatialDirectedDistances: marker_point_kinematics

import ..SpatialComponentAssembly: component_registration,
    executable_blocks, equation_contributions

export SpatialSpan, SpatialSpanMeasure, spatial_span_measure_registration,
       allocated_spatial_span_measure, initialize_spatial_span_measure!,
       spatial_span_variable_declarations,
       spatial_span_equation_blocks, allocated_spatial_span,
       initialize_spatial_span!, spatial_span_values,
       spatial_span_executable_blocks

"""
Explicit separation, direction, distance, velocity, and acceleration shared by
elements acting along the line between two markers.
"""
struct SpatialSpan{M1,M2}
    name::Symbol
    marker_1::M1
    marker_2::M2
    separation_variables::UnitRange{Int}
    distance_variable::Int
    direction_variables::UnitRange{Int}
    velocity_variable::Int
    acceleration_variable::Int
    separation_equations::UnitRange{Int}
    distance_equation::Int
    direction_equations::UnitRange{Int}
    velocity_equation::Int
    acceleration_equation::Int
end

"""Reaction-free marker-to-marker span measurement."""
struct SpatialSpanMeasure{S}
    name::Symbol
    span::S
end

function spatial_span_measure_registration(name::Symbol)
    ComponentRegistration(name, spatial_span_variable_declarations(),
        spatial_span_equation_blocks())
end

component_registration(measure::SpatialSpanMeasure) =
    spatial_span_measure_registration(measure.name)

allocated_spatial_span_measure(layout, name, marker_1, marker_2) =
    SpatialSpanMeasure(name,
        allocated_spatial_span(layout, name, marker_1, marker_2))

initialize_spatial_span_measure!(initial, measure::SpatialSpanMeasure) =
    initialize_spatial_span!(initial, measure.span)

function spatial_span_variable_declarations(; distance_name = :distance,
        velocity_name = :velocity, acceleration_name = :acceleration)
    VariableDeclaration[
        [VariableDeclaration(Symbol(:s_, axis), :applied_geometry, 0)
            for axis in (:x, :y, :z)];
        VariableDeclaration(distance_name, :relative_position, 0);
        [VariableDeclaration(Symbol(:u_, axis), :applied_geometry, 0)
            for axis in (:x, :y, :z)];
        VariableDeclaration(velocity_name, :relative_velocity, 1);
        VariableDeclaration(acceleration_name, :relative_acceleration, 2);
    ]
end

function spatial_span_equation_blocks()
    EquationBlockDeclaration[
        EquationBlockDeclaration(:span_geometry, EquationDeclaration[
            [EquationDeclaration(Symbol(:span_, axis), :applied_definition,
                0, :span) for axis in (:x, :y, :z)];
            EquationDeclaration(:distance, :coordinate_relation, 0,
                :spanning_distance);
            [EquationDeclaration(Symbol(:direction_, axis),
                :applied_definition, 0, :unit)
                for axis in (:x, :y, :z)];
        ]),
        EquationBlockDeclaration(:span_velocity, EquationDeclaration[
            EquationDeclaration(:velocity, :coordinate_relation, 1,
                :spanning_distance),
        ]),
        EquationBlockDeclaration(:span_acceleration, EquationDeclaration[
            EquationDeclaration(:acceleration, :coordinate_relation, 2,
                :spanning_distance),
        ]),
    ]
end

function allocated_spatial_span(layout, name, marker_1, marker_2)
    variables = component_variable_indices(layout, name)
    geometry = component_equation_indices(layout, name, :span_geometry)
    SpatialSpan(name, marker_1, marker_2,
        variables[1:3], variables[4], variables[5:7], variables[8],
        variables[9], geometry[1:3], geometry[4], geometry[5:7],
        only(component_equation_indices(layout, name, :span_velocity)),
        only(component_equation_indices(layout, name, :span_acceleration)))
end

function spatial_span_values(span::SpatialSpan, z)
    marker_1 = marker_point_kinematics(span.marker_1, z)
    marker_2 = marker_point_kinematics(span.marker_2, z)
    distance = z[span.distance_variable]
    distance > zero(distance) || throw(DomainError(distance,
        "span '$(span.name)' distance must be positive"))
    relative_velocity = marker_2.velocity - marker_1.velocity
    relative_acceleration = marker_2.acceleration - marker_1.acceleration
    velocity = z[span.velocity_variable]
    transverse_speed_squared =
        dot(relative_velocity, relative_velocity) - velocity^2
    (; marker_1, marker_2,
       separation = @view(z[span.separation_variables]), distance,
       direction = @view(z[span.direction_variables]),
       relative_velocity, relative_acceleration, velocity,
       acceleration = z[span.acceleration_variable],
       transverse_speed_squared)
end

function initialize_spatial_span!(initial, span::SpatialSpan)
    point_1 = spatial_marker_position(span.marker_1, initial)
    point_2 = spatial_marker_position(span.marker_2, initial)
    separation = point_2 - point_1
    distance = norm(separation)
    distance > 0 || throw(ArgumentError(
        "span '$(span.name)' has zero initial distance"))
    direction = separation ./ distance
    relative_velocity = spatial_marker_velocity(span.marker_2, initial) -
        spatial_marker_velocity(span.marker_1, initial)
    velocity = dot(direction, relative_velocity)
    relative_acceleration =
        spatial_marker_acceleration(span.marker_2, initial) -
        spatial_marker_acceleration(span.marker_1, initial)
    transverse_speed_squared =
        dot(relative_velocity, relative_velocity) - velocity^2
    acceleration = dot(direction, relative_acceleration) +
        transverse_speed_squared / distance
    initial[span.separation_variables] .= separation
    initial[span.distance_variable] = distance
    initial[span.direction_variables] .= direction
    initial[span.velocity_variable] = velocity
    initial[span.acceleration_variable] = acceleration
    initial
end

function add_separation_marker_jacobian!(jacobian, rows, values, sign)
    isnothing(values.body) && return nothing
    body = values.body
    jacobian[rows, body.position_variables] .+=
        sign .* Matrix{eltype(jacobian)}(I, 3, 3)
    jacobian[rows, body.euler_parameter_variables] .+=
        sign .* values.position_parameters
    nothing
end

function add_velocity_marker_jacobian!(jacobian, row, values, direction,
        sign)
    isnothing(values.body) && return nothing
    body = values.body
    jacobian[row, body.velocity_variables] .+= sign .* direction
    jacobian[row, body.euler_parameter_variables] .+=
        sign .* vec(transpose(direction) * values.velocity_parameters)
    jacobian[row, body.angular_velocity_variables] .+=
        sign .* vec(transpose(direction) * values.velocity_omega)
    nothing
end

function add_acceleration_marker_jacobian!(jacobian, row, values,
        velocity_gradient, acceleration_gradient)
    isnothing(values.body) && return nothing
    body = values.body
    jacobian[row, body.velocity_variables] .+= velocity_gradient
    jacobian[row, body.acceleration_variables] .+= acceleration_gradient
    jacobian[row, body.euler_parameter_variables] .+= vec(
        transpose(velocity_gradient) * values.velocity_parameters +
        transpose(acceleration_gradient) * values.acceleration_parameters)
    jacobian[row, body.angular_velocity_variables] .+= vec(
        transpose(velocity_gradient) * values.velocity_omega +
        transpose(acceleration_gradient) * values.acceleration_omega)
    jacobian[row, body.angular_acceleration_variables] .+= vec(
        transpose(acceleration_gradient) * values.acceleration_alpha)
    nothing
end

function spatial_span_executable_blocks(span::SpatialSpan)
    geometry_rows = [collect(span.separation_equations);
        span.distance_equation; collect(span.direction_equations)]
    geometry! = function (equations, t, z, zdot)
        values = spatial_span_values(span, z)
        equations[span.separation_equations] .= values.separation .-
            (values.marker_2.position - values.marker_1.position)
        equations[span.distance_equation] =
            values.distance - norm(values.separation)
        equations[span.direction_equations] .= values.direction .-
            values.separation ./ values.distance
    end
    geometry_jacobian! = function (jacobian, t, z, zdot, coefficient)
        values = spatial_span_values(span, z)
        jacobian[span.separation_equations,
            span.separation_variables] .+= I(3)
        add_separation_marker_jacobian!(jacobian,
            span.separation_equations, values.marker_1, 1)
        add_separation_marker_jacobian!(jacobian,
            span.separation_equations, values.marker_2, -1)
        jacobian[span.distance_equation, span.distance_variable] += 1
        jacobian[span.distance_equation, span.separation_variables] .-=
            values.separation ./ norm(values.separation)
        for component in 1:3
            row = span.direction_equations[component]
            jacobian[row, span.direction_variables[component]] += 1
            jacobian[row, span.separation_variables[component]] -=
                1 / values.distance
            jacobian[row, span.distance_variable] +=
                values.separation[component] / values.distance^2
        end
    end

    velocity! = function (equations, t, z, zdot)
        values = spatial_span_values(span, z)
        equations[span.velocity_equation] = values.velocity -
            dot(values.direction, values.relative_velocity)
    end
    velocity_jacobian! = function (jacobian, t, z, zdot, coefficient)
        values = spatial_span_values(span, z)
        row = span.velocity_equation
        jacobian[row, span.velocity_variable] += 1
        jacobian[row, span.direction_variables] .-=
            values.relative_velocity
        add_velocity_marker_jacobian!(jacobian, row, values.marker_1,
            values.direction, 1)
        add_velocity_marker_jacobian!(jacobian, row, values.marker_2,
            values.direction, -1)
    end

    acceleration! = function (equations, t, z, zdot)
        values = spatial_span_values(span, z)
        equations[span.acceleration_equation] = values.acceleration -
            dot(values.direction, values.relative_acceleration) -
            values.transverse_speed_squared / values.distance
    end
    acceleration_jacobian! = function (jacobian, t, z, zdot, coefficient)
        values = spatial_span_values(span, z)
        row = span.acceleration_equation
        jacobian[row, span.acceleration_variable] += 1
        jacobian[row, span.direction_variables] .-=
            values.relative_acceleration
        jacobian[row, span.velocity_variable] +=
            2values.velocity / values.distance
        jacobian[row, span.distance_variable] +=
            values.transverse_speed_squared / values.distance^2
        relative_velocity_gradient =
            -2 .* values.relative_velocity ./ values.distance
        relative_acceleration_gradient = -values.direction
        add_acceleration_marker_jacobian!(jacobian, row, values.marker_1,
            -relative_velocity_gradient, -relative_acceleration_gradient)
        add_acceleration_marker_jacobian!(jacobian, row, values.marker_2,
            relative_velocity_gradient, relative_acceleration_gradient)
    end

    ExecutableEquationBlock[
        ExecutableEquationBlock(span.name, :span_geometry, geometry_rows,
            geometry!, geometry_jacobian!),
        ExecutableEquationBlock(span.name, :span_velocity,
            [span.velocity_equation], velocity!, velocity_jacobian!),
        ExecutableEquationBlock(span.name, :span_acceleration,
            [span.acceleration_equation], acceleration!,
            acceleration_jacobian!),
    ]
end

executable_blocks(measure::SpatialSpanMeasure) =
    spatial_span_executable_blocks(measure.span)

equation_contributions(::SpatialSpanMeasure) = EquationContribution[]

end
