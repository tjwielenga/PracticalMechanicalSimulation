"""Rolling-tire contact and force components for spatial models."""
module SpatialTires

using ForwardDiff
using LinearAlgebra
using ..AutomaticAnalysis
using ..ScalarExpressions: ScalarLaw
using ..SpatialComponentAssembly
using ..SpatialModeling
using ..SpatialDirectedDistances: marker_axis_kinematics,
    marker_point_kinematics
using ..SpatialConstraints: marker_angular_kinematics

import ..SpatialComponentAssembly: component_registration,
    executable_blocks, equation_contributions

export SpatialTireComponent, spatial_tire_registration,
       allocated_spatial_tire, initialize_spatial_tire!,
       spatial_tire_kinematics, spatial_tire_values,
       tire_deformation_rates,
       tire_contact_surface, tire_normal_force_surface

"""
A tire between a wheel-center marker and a planar-road marker.

The wheel marker's local z-axis is the axle. The road marker's local z-axis is
the outward road normal. Forces are applied at the projection of the wheel
center onto the road plane. Tangential expressions define trial forces; an
optional friction ellipse limits their combined magnitude.

When positive longitudinal and lateral relaxation lengths are supplied, two
first-order internal deformation states provide transient tangential forces
and retain static tangential deformation at zero transport speed.
"""
struct SpatialTireComponent{W,R,N,X,Y,T}
    name::Symbol
    wheel_marker::W
    road_marker::R
    radius::T
    regularization_speed::T
    normal_law::N
    longitudinal_law::X
    lateral_law::Y
    normal_stiffness::T
    normal_damping::T
    normal_expression::Bool
    friction_limit::Symbol
    mu_longitudinal::T
    mu_lateral::T
    transient::Bool
    longitudinal_relaxation_length::T
    lateral_relaxation_length::T
    deflection_variable::Int
    deflection_rate_variable::Int
    forward_velocity_variable::Int
    lateral_velocity_variable::Int
    longitudinal_slip_velocity_variable::Int
    lateral_slip_velocity_variable::Int
    slip_ratio_variable::Int
    slip_angle_variable::Int
    camber_angle_variable::Int
    normal_force_variable::Int
    longitudinal_trial_force_variable::Int
    lateral_trial_force_variable::Int
    longitudinal_force_variable::Int
    lateral_force_variable::Int
    global_force_variables::UnitRange{Int}
    longitudinal_deformation_variable::Int
    lateral_deformation_variable::Int
    kinematic_equations::UnitRange{Int}
    load_equations::UnitRange{Int}
    deformation_equations::UnitRange{Int}
end

function spatial_tire_registration(name::Symbol; transient = false)
    variables = VariableDeclaration[
        VariableDeclaration(:deflection, :applied_geometry, 0),
        VariableDeclaration(:deflection_rate, :applied_rate, 1),
        VariableDeclaration(:forward_velocity, :applied_rate, 1),
        VariableDeclaration(:lateral_velocity, :applied_rate, 1),
        VariableDeclaration(:longitudinal_slip_velocity, :applied_rate, 1),
        VariableDeclaration(:lateral_slip_velocity, :applied_rate, 1),
        VariableDeclaration(:slip_ratio, :applied_rate, 1),
        VariableDeclaration(:slip_angle, :applied_rate, 1),
        VariableDeclaration(:camber_angle, :applied_geometry, 0),
        VariableDeclaration(:normal_force, :applied_load, 2),
        VariableDeclaration(:longitudinal_trial_force, :applied_load, 2),
        VariableDeclaration(:lateral_trial_force, :applied_load, 2),
        VariableDeclaration(:longitudinal_force, :applied_load, 2),
        VariableDeclaration(:lateral_force, :applied_load, 2),
    ]
    append!(variables, [VariableDeclaration(Symbol(:F_, axis),
        :applied_load, 2) for axis in (:x, :y, :z)])
    if transient
        push!(variables,
            VariableDeclaration(:longitudinal_deformation, :internal_state, 0),
            VariableDeclaration(:lateral_deformation, :internal_state, 0))
    end
    kinematics = EquationDeclaration[
        EquationDeclaration(:deflection, :applied_definition, 0,
            :tire_contact),
        EquationDeclaration(:deflection_rate, :applied_definition, 1,
            :tire_contact),
        EquationDeclaration(:forward_velocity, :applied_definition, 1,
            :tire_slip),
        EquationDeclaration(:lateral_velocity, :applied_definition, 1,
            :tire_slip),
        EquationDeclaration(:longitudinal_slip_velocity,
            :applied_definition, 1, :tire_slip),
        EquationDeclaration(:lateral_slip_velocity,
            :applied_definition, 1, :tire_slip),
        EquationDeclaration(:slip_ratio, :applied_definition, 1,
            :tire_slip),
        EquationDeclaration(:slip_angle, :applied_definition, 1,
            :tire_slip),
        EquationDeclaration(:camber_angle, :applied_definition, 0,
            :tire_contact),
    ]
    loads = EquationDeclaration[
        EquationDeclaration(:normal_force, :applied_definition, 2,
            :constitutive_force),
        EquationDeclaration(:longitudinal_trial_force,
            :applied_definition, 2, :constitutive_force),
        EquationDeclaration(:lateral_trial_force,
            :applied_definition, 2, :constitutive_force),
        EquationDeclaration(:longitudinal_force, :applied_definition, 2,
            :combined_slip),
        EquationDeclaration(:lateral_force, :applied_definition, 2,
            :combined_slip),
    ]
    append!(loads, [EquationDeclaration(Symbol(:global_force_, axis),
        :applied_definition, 2, :global_force)
        for axis in (:x, :y, :z)])
    blocks = EquationBlockDeclaration[
        EquationBlockDeclaration(:kinematics, kinematics),
        EquationBlockDeclaration(:load, loads),
    ]
    if transient
        push!(blocks, EquationBlockDeclaration(:deformation,
            EquationDeclaration[
                EquationDeclaration(:longitudinal_deformation,
                    :internal_differential, 0, :tire_relaxation),
                EquationDeclaration(:lateral_deformation,
                    :internal_differential, 0, :tire_relaxation),
            ]))
    end
    ComponentRegistration(name, variables, blocks)
end

component_registration(tire::SpatialTireComponent) =
    spatial_tire_registration(tire.name; transient = tire.transient)

function allocated_spatial_tire(layout, name, wheel_marker, road_marker,
        radius, regularization_speed, normal_law::ScalarLaw,
        longitudinal_law::ScalarLaw, lateral_law::ScalarLaw;
        normal_stiffness = 0.0, normal_damping = 0.0,
        normal_expression = false, friction_limit = :none,
        mu_longitudinal = Inf, mu_lateral = Inf,
        longitudinal_relaxation_length = 0.0,
        lateral_relaxation_length = 0.0)
    variables = component_variable_indices(layout, name)
    kinematics = component_equation_indices(layout, name, :kinematics)
    loads = component_equation_indices(layout, name, :load)
    transient = longitudinal_relaxation_length > 0
    deformation = transient ?
        component_equation_indices(layout, name, :deformation) : (1:0)
    SpatialTireComponent(name, wheel_marker, road_marker, Float64(radius),
        Float64(regularization_speed), normal_law, longitudinal_law,
        lateral_law, Float64(normal_stiffness), Float64(normal_damping),
        Bool(normal_expression), Symbol(friction_limit),
        Float64(mu_longitudinal), Float64(mu_lateral),
        transient, Float64(longitudinal_relaxation_length),
        Float64(lateral_relaxation_length),
        variables[1], variables[2], variables[3], variables[4], variables[5],
        variables[6], variables[7], variables[8], variables[9], variables[10],
        variables[11], variables[12], variables[13], variables[14],
        variables[15:17], transient ? variables[18] : 0,
        transient ? variables[19] : 0, kinematics, loads, deformation)
end

function spatial_tire_kinematics(tire::SpatialTireComponent, z)
    wheel = marker_point_kinematics(tire.wheel_marker, z)
    road = marker_point_kinematics(tire.road_marker, z)
    axle = marker_axis_kinematics(tire.wheel_marker, z, 3).direction
    normal = marker_axis_kinematics(tire.road_marker, z, 3).direction
    tangent = cross(axle, normal)
    tangent_norm = norm(tangent)
    tangent_norm > sqrt(eps(Float64)) || throw(DomainError(tangent_norm,
        "tire axle must not be parallel to the road normal"))
    forward = tangent ./ tangent_norm
    lateral = cross(normal, forward)

    separation = wheel.position - road.position
    height = dot(separation, normal)
    normal_velocity = dot(wheel.velocity - road.velocity, normal) +
        dot(separation,
            marker_axis_kinematics(tire.road_marker, z, 3).velocity)
    deflection = tire.radius - height
    deflection_rate = -normal_velocity
    contact_point = wheel.position - height .* normal

    wheel_angular = marker_angular_kinematics(tire.wheel_marker, z).omega
    road_angular = marker_angular_kinematics(tire.road_marker, z).omega
    road_contact_velocity = road.velocity +
        cross(road_angular, contact_point - road.position)
    transport_velocity = wheel.velocity - road_contact_velocity
    wheel_contact_velocity = wheel.velocity +
        cross(wheel_angular, contact_point - wheel.position)
    slip_velocity = wheel_contact_velocity - road_contact_velocity

    forward_velocity = dot(transport_velocity, forward)
    lateral_velocity = dot(transport_velocity, lateral)
    longitudinal_slip_velocity = dot(slip_velocity, forward)
    lateral_slip_velocity = dot(slip_velocity, lateral)
    speed_scale = hypot(forward_velocity, tire.regularization_speed)
    slip_ratio = -longitudinal_slip_velocity / speed_scale
    slip_angle = atan(lateral_velocity, speed_scale)
    camber_angle = atan(dot(axle, normal), tangent_norm)
    (; deflection, deflection_rate, forward_velocity, lateral_velocity,
       longitudinal_slip_velocity, lateral_slip_velocity, slip_ratio,
       slip_angle, camber_angle, contact_point, forward, lateral, normal,
       height, wheel, road, transport_velocity, slip_velocity)
end

function spatial_tire_values(tire::SpatialTireComponent, z)
    values = spatial_tire_kinematics(tire, z)
    (; values...,
       normal_force = z[tire.normal_force_variable],
       longitudinal_trial_force = z[tire.longitudinal_trial_force_variable],
       lateral_trial_force = z[tire.lateral_trial_force_variable],
       longitudinal_force = z[tire.longitudinal_force_variable],
       lateral_force = z[tire.lateral_force_variable],
       global_force = @view(z[tire.global_force_variables]),
       longitudinal_deformation = tire.transient ?
           z[tire.longitudinal_deformation_variable] : zero(eltype(z)),
       lateral_deformation = tire.transient ?
           z[tire.lateral_deformation_variable] : zero(eltype(z)))
end

"""
Return the two first-order tread-deformation rates.

The deformation is convected out of a loaded contact patch according to its
relaxation length. At zero forward transport speed it is retained, allowing
the tire to carry a static tangential load. When normal force reaches zero,
an unloaded tire releases the stored deformation on the regularization-speed
time scale. Normal force, rather than geometric deflection, defines unloading
because rebound damping can remove the contact force shortly before geometric
separation.
"""
function tire_deformation_rates(tire::SpatialTireComponent, z)
    tire.transient || return (zero(eltype(z)), zero(eltype(z)))
    longitudinal = z[tire.longitudinal_deformation_variable]
    lateral = z[tire.lateral_deformation_variable]
    if z[tire.normal_force_variable] <= 0
        return (-tire.regularization_speed * longitudinal /
                    tire.longitudinal_relaxation_length,
                -tire.regularization_speed * lateral /
                    tire.lateral_relaxation_length)
    end
    forward_velocity = z[tire.forward_velocity_variable]
    transport_speed = forward_velocity^2 /
        hypot(forward_velocity, tire.regularization_speed)
    (-z[tire.longitudinal_slip_velocity_variable] -
         transport_speed * longitudinal /
             tire.longitudinal_relaxation_length,
     z[tire.lateral_slip_velocity_variable] -
         transport_speed * lateral / tire.lateral_relaxation_length)
end

function limited_tire_forces(tire, normal_force, longitudinal, lateral)
    normal_force > zero(normal_force) ||
        return (zero(longitudinal), zero(lateral))
    tire.friction_limit == :none && return (longitudinal, lateral)
    # Compare an equivalent force demand directly with normal force. This is
    # algebraically identical to the usual utilization calculation but avoids
    # dividing by a vanishing normal load during tire lift-off.
    equivalent_demand = hypot(longitudinal / tire.mu_longitudinal,
        lateral / tire.mu_lateral)
    equivalent_demand <= normal_force && return (longitudinal, lateral)
    scale = normal_force / equivalent_demand
    (scale * longitudinal, scale * lateral)
end

function initialize_spatial_tire!(initial, tire, time)
    values = spatial_tire_kinematics(tire, initial)
    kinematic_values = [values.deflection, values.deflection_rate,
        values.forward_velocity, values.lateral_velocity,
        values.longitudinal_slip_velocity, values.lateral_slip_velocity,
        values.slip_ratio, values.slip_angle, values.camber_angle]
    initial[tire.deflection_variable:tire.camber_angle_variable] .=
        kinematic_values
    raw_normal = tire.normal_law(time, initial)
    normal_force = values.deflection > 0 ? max(raw_normal, 0.0) : 0.0
    initial[tire.normal_force_variable] = normal_force
    # Trial forces describe the tire's tangential demand and remain defined
    # through lift-off. The friction limiter, not the constitutive law, makes
    # the transmitted tangential forces zero when normal load is zero.
    longitudinal_trial = tire.longitudinal_law(time, initial)
    lateral_trial = tire.lateral_law(time, initial)
    all(isfinite, (normal_force, longitudinal_trial, lateral_trial)) ||
        throw(ArgumentError("tire '$(tire.name)' force is not finite initially"))
    initial[tire.longitudinal_trial_force_variable] = longitudinal_trial
    initial[tire.lateral_trial_force_variable] = lateral_trial
    longitudinal, lateral = limited_tire_forces(tire, normal_force,
        longitudinal_trial, lateral_trial)
    initial[tire.longitudinal_force_variable] = longitudinal
    initial[tire.lateral_force_variable] = lateral
    initial[tire.global_force_variables] .= longitudinal .* values.forward .+
        lateral .* values.lateral .+ normal_force .* values.normal
    initial
end

tire_contact_surface(tire::SpatialTireComponent, z) =
    z[tire.deflection_variable]
tire_normal_force_surface(tire::SpatialTireComponent, time, z) =
    tire.normal_law(time, z)

function body_kinematic_dependencies(marker)
    marker isa SpatialGroundMarker && return Int[]
    body = marker.body
    [collect(body.position_variables);
     collect(body.velocity_variables);
     collect(body.angular_velocity_variables);
     collect(body.euler_parameter_variables)]
end

function body_point_dependencies(marker)
    marker isa SpatialGroundMarker && return Int[]
    body = marker.body
    [collect(body.position_variables);
     collect(body.euler_parameter_variables)]
end

function local_state_jacobian(function_value, z, columns)
    isempty(columns) && return zeros(eltype(z), length(function_value(z)), 0)
    inputs = collect(z[columns])
    ForwardDiff.jacobian(inputs) do local_values
        state = Vector{eltype(local_values)}(undef, length(z))
        state .= z
        state[columns] .= local_values
        function_value(state)
    end
end

function tire_kinematic_vector(tire, z)
    values = spatial_tire_kinematics(tire, z)
    [values.deflection, values.deflection_rate, values.forward_velocity,
     values.lateral_velocity, values.longitudinal_slip_velocity,
     values.lateral_slip_velocity, values.slip_ratio, values.slip_angle,
     values.camber_angle]
end

function tire_direction_force(tire, z)
    values = spatial_tire_kinematics(tire, z)
    z[tire.longitudinal_force_variable] .* values.forward .+
        z[tire.lateral_force_variable] .* values.lateral .+
        z[tire.normal_force_variable] .* values.normal
end

function executable_blocks(tire::SpatialTireComponent)
    kinematic_variables = tire.deflection_variable:tire.camber_angle_variable
    kinematic_columns = sort!(unique!([
        body_kinematic_dependencies(tire.wheel_marker);
        body_kinematic_dependencies(tire.road_marker)]))
    orientation_columns = sort!(unique!([
        tire.wheel_marker.body.euler_parameter_variables;
        tire.road_marker isa SpatialGroundMarker ? Int[] :
            collect(tire.road_marker.body.euler_parameter_variables)]))

    kinematics! = function (equations, t, z, zdot)
        equations[tire.kinematic_equations] .=
            z[kinematic_variables] .- tire_kinematic_vector(tire, z)
    end
    kinematics_jacobian! = function (jacobian, t, z, zdot, coefficient)
        for (row, variable) in zip(tire.kinematic_equations,
                kinematic_variables)
            jacobian[row, variable] += 1
        end
        partials = local_state_jacobian(
            state -> tire_kinematic_vector(tire, state), z,
            kinematic_columns)
        jacobian[tire.kinematic_equations, kinematic_columns] .-= partials
    end

    load! = function (equations, t, z, zdot)
        deflection = z[tire.deflection_variable]
        raw_normal = tire.normal_law(t, z)
        target_normal = deflection > 0 ? max(raw_normal, zero(raw_normal)) :
            zero(raw_normal)
        normal_force = z[tire.normal_force_variable]
        target_longitudinal = tire.longitudinal_law(t, z)
        target_lateral = tire.lateral_law(t, z)
        equations[tire.load_equations[1]] = normal_force - target_normal
        equations[tire.load_equations[2]] =
            z[tire.longitudinal_trial_force_variable] - target_longitudinal
        equations[tire.load_equations[3]] =
            z[tire.lateral_trial_force_variable] - target_lateral
        limited = limited_tire_forces(tire, normal_force,
            z[tire.longitudinal_trial_force_variable],
            z[tire.lateral_trial_force_variable])
        equations[tire.load_equations[4]] =
            z[tire.longitudinal_force_variable] - limited[1]
        equations[tire.load_equations[5]] =
            z[tire.lateral_force_variable] - limited[2]
        equations[tire.load_equations[6:8]] .=
            z[tire.global_force_variables] .- tire_direction_force(tire, z)
    end
    load_jacobian! = function (jacobian, t, z, zdot, coefficient)
        for (row, variable) in zip(tire.load_equations,
                [tire.normal_force_variable,
                 tire.longitudinal_trial_force_variable,
                 tire.lateral_trial_force_variable,
                 tire.longitudinal_force_variable,
                 tire.lateral_force_variable,
                 collect(tire.global_force_variables)...])
            jacobian[row, variable] += 1
        end
        raw_normal = tire.normal_law(t, z)
        normal_active = z[tire.deflection_variable] > 0 && raw_normal > 0
        normal_factor = normal_active ? 1 : 0
        for (column, partial) in zip(tire.normal_law.dependencies,
                tire.normal_law.gradient(t, z))
            # Write the inactive zeros as well so the contact transition does
            # not change the established sparse pattern.
            jacobian[tire.load_equations[1], column] -=
                normal_factor * partial
        end
        for (column, partial) in zip(tire.longitudinal_law.dependencies,
                tire.longitudinal_law.gradient(t, z))
            jacobian[tire.load_equations[2], column] -=
                partial
        end
        for (column, partial) in zip(tire.lateral_law.dependencies,
                tire.lateral_law.gradient(t, z))
            jacobian[tire.load_equations[3], column] -=
                partial
        end
        limiter_inputs = [z[tire.normal_force_variable],
            z[tire.longitudinal_trial_force_variable],
            z[tire.lateral_trial_force_variable]]
        limiter_partial = ForwardDiff.jacobian(limiter_inputs) do inputs
            collect(limited_tire_forces(tire, inputs...))
        end
        limiter_columns = [tire.normal_force_variable,
            tire.longitudinal_trial_force_variable,
            tire.lateral_trial_force_variable]
        jacobian[tire.load_equations[4:5], limiter_columns] .-=
            limiter_partial

        values = spatial_tire_kinematics(tire, z)
        jacobian[tire.load_equations[6:8], tire.normal_force_variable] .-=
            values.normal
        jacobian[tire.load_equations[6:8],
            tire.longitudinal_force_variable] .-= values.forward
        jacobian[tire.load_equations[6:8],
            tire.lateral_force_variable] .-= values.lateral
        direction_partial = local_state_jacobian(
            state -> tire_direction_force(tire, state), z,
            orientation_columns)
        jacobian[tire.load_equations[6:8], orientation_columns] .-=
            direction_partial
    end

    blocks = ExecutableEquationBlock[
        ExecutableEquationBlock(tire.name, :kinematics,
            collect(tire.kinematic_equations), kinematics!,
            kinematics_jacobian!),
        ExecutableEquationBlock(tire.name, :load,
            collect(tire.load_equations), load!, load_jacobian!),
    ]
    if tire.transient
        deformation_variables = [tire.longitudinal_deformation_variable,
            tire.lateral_deformation_variable]
        deformation_columns = [tire.deflection_variable,
            tire.normal_force_variable,
            tire.forward_velocity_variable,
            tire.longitudinal_slip_velocity_variable,
            tire.lateral_slip_velocity_variable,
            deformation_variables...]
        deformation! = function (equations, t, z, zdot)
            equations[tire.deformation_equations] .=
                zdot[deformation_variables] .-
                collect(tire_deformation_rates(tire, z))
        end
        deformation_jacobian! = function (jacobian, t, z, zdot, coefficient)
            for (row, variable) in zip(tire.deformation_equations,
                    deformation_variables)
                jacobian[row, variable] += coefficient
            end
            partials = local_state_jacobian(
                state -> collect(tire_deformation_rates(tire, state)), z,
                deformation_columns)
            jacobian[tire.deformation_equations, deformation_columns] .-=
                partials
        end
        push!(blocks, ExecutableEquationBlock(tire.name, :deformation,
            collect(tire.deformation_equations), deformation!,
            deformation_jacobian!))
    end
    blocks
end

function body_force_contribution(body, z, contact_point, global_force, sign)
    parameters = @view z[body.euler_parameter_variables]
    orientation = rotation_matrix(parameters)
    signed_force = sign .* global_force
    lever_body = transpose(orientation) *
        (contact_point - z[body.position_variables])
    force_body = transpose(orientation) * signed_force
    [-signed_force; -cross(lever_body, force_body)]
end

function tire_body_contribution(tire, z)
    values = spatial_tire_kinematics(tire, z)
    global_force = @view z[tire.global_force_variables]
    wheel = body_force_contribution(tire.wheel_marker.body, z,
        values.contact_point, global_force, 1)
    tire.road_marker isa SpatialGroundMarker && return wheel
    [wheel; body_force_contribution(tire.road_marker.body, z,
        values.contact_point, global_force, -1)]
end

function equation_contributions(tire::SpatialTireComponent)
    rows = collect(tire.wheel_marker.body.balance_equations)
    if !(tire.road_marker isa SpatialGroundMarker)
        append!(rows, collect(tire.road_marker.body.balance_equations))
    end
    columns = sort!(unique!([
        body_point_dependencies(tire.wheel_marker);
        body_point_dependencies(tire.road_marker);
        collect(tire.global_force_variables)]))
    residual! = function (equations, t, z, zdot)
        equations[rows] .+= tire_body_contribution(tire, z)
    end
    jacobian! = function (jacobian, t, z, zdot, coefficient)
        partials = local_state_jacobian(
            state -> tire_body_contribution(tire, state), z, columns)
        jacobian[rows, columns] .+= partials
    end
    EquationContribution[EquationContribution(tire.name, :load_to_bodies,
        rows, residual!, jacobian!)]
end

end
