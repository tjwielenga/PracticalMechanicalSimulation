"""Six-component compliant bushings for spatial models."""
module SpatialBushings

using LinearAlgebra
using ..AutomaticAnalysis
using ..SpatialComponentAssembly
using ..SpatialModeling
using ..SpatialDirectedDistances: marker_axis_kinematics,
    marker_point_kinematics
using ..SpatialConstraints: body_force_rows, add_reaction_to_body!,
    add_reaction_jacobian!, marker_angular_kinematics
using ..SpatialAppliedForces: add_floating_force!,
    add_floating_force_jacobian!, add_applied_torque_to_body!,
    add_applied_torque_jacobian!

import ..SpatialComponentAssembly: component_registration,
    executable_blocks, equation_contributions

export SpatialBushingComponent, spatial_bushing_registration,
       allocated_spatial_bushing, initialize_spatial_bushing!,
       spatial_bushing_values, set_spatial_bushing_stage!

"""
A six-component spring-damper between two oriented markers.

Translation, translation rate, Bryant rotation angles, relative angular
velocity, and the constitutive loads are expressed in the second marker's
frame. The optional reaction marker is owned by the second marker's body and
follows the first marker point, so the opposite wrench is applied at the same
global point as the first wrench.
"""
struct SpatialBushingComponent{I,J,R,T,S}
    name::Symbol
    marker_i::I
    marker_j::J
    reaction_marker::R
    translational_stiffness::Vector{T}
    translational_damping::Vector{T}
    rotational_stiffness::Vector{T}
    rotational_damping::Vector{T}
    damping_time_scale::T
    active_during::S
    active::Base.RefValue{Bool}
    translation_variables::UnitRange{Int}
    translation_rate_variables::UnitRange{Int}
    angle_variables::UnitRange{Int}
    angular_velocity_variables::UnitRange{Int}
    local_force_variables::UnitRange{Int}
    local_torque_variables::UnitRange{Int}
    global_force_variables::UnitRange{Int}
    global_torque_variables::UnitRange{Int}
    kinematic_equations::UnitRange{Int}
    load_equations::UnitRange{Int}
end

function spatial_bushing_registration(name::Symbol)
    variables = VariableDeclaration[
        [VariableDeclaration(Symbol(:r_, axis), :applied_geometry, 0)
            for axis in (:x, :y, :z)];
        [VariableDeclaration(Symbol(:v_, axis), :applied_rate, 1)
            for axis in (:x, :y, :z)];
        [VariableDeclaration(Symbol(:angle_, axis), :applied_geometry, 0)
            for axis in (:x, :y, :z)];
        [VariableDeclaration(Symbol(:omega_, axis), :applied_rate, 1)
            for axis in (:x, :y, :z)];
        [VariableDeclaration(Symbol(:f_, axis), :applied_load, 2)
            for axis in (:x, :y, :z)];
        [VariableDeclaration(Symbol(:tau_, axis), :applied_load, 2)
            for axis in (:x, :y, :z)];
        [VariableDeclaration(Symbol(:F_, axis), :applied_load, 2)
            for axis in (:x, :y, :z)];
        [VariableDeclaration(Symbol(:T_, axis), :applied_load, 2)
            for axis in (:x, :y, :z)]
    ]
    kinematics = EquationDeclaration[
        [EquationDeclaration(Symbol(:translation_, axis),
            :applied_definition, 0, :bushing_translation)
            for axis in (:x, :y, :z)];
        [EquationDeclaration(Symbol(:translation_rate_, axis),
            :applied_definition, 1, :bushing_translation_rate)
            for axis in (:x, :y, :z)];
        [EquationDeclaration(Symbol(:angle_, axis),
            :applied_definition, 0, :bushing_angle)
            for axis in (:x, :y, :z)];
        [EquationDeclaration(Symbol(:angular_velocity_, axis),
            :applied_definition, 1, :bushing_angular_velocity)
            for axis in (:x, :y, :z)]
    ]
    loads = EquationDeclaration[
        [EquationDeclaration(Symbol(:local_force_, axis),
            :applied_definition, 2, :constitutive_force)
            for axis in (:x, :y, :z)];
        [EquationDeclaration(Symbol(:local_torque_, axis),
            :applied_definition, 2, :constitutive_torque)
            for axis in (:x, :y, :z)];
        [EquationDeclaration(Symbol(:global_force_, axis),
            :applied_definition, 2, :global_force)
            for axis in (:x, :y, :z)];
        [EquationDeclaration(Symbol(:global_torque_, axis),
            :applied_definition, 2, :global_torque)
            for axis in (:x, :y, :z)]
    ]
    ComponentRegistration(name, variables, EquationBlockDeclaration[
        EquationBlockDeclaration(:kinematics, kinematics),
        EquationBlockDeclaration(:load, loads),
    ])
end

component_registration(bushing::SpatialBushingComponent) =
    spatial_bushing_registration(bushing.name)

function allocated_spatial_bushing(layout, name, marker_i, marker_j,
        reaction_marker, translational_stiffness, translational_damping,
        rotational_stiffness, rotational_damping, damping_time_scale,
        active_during = (:static, :dynamic, :modal))
    variables = component_variable_indices(layout, name)
    kinematics = component_equation_indices(layout, name, :kinematics)
    loads = component_equation_indices(layout, name, :load)
    SpatialBushingComponent(name, marker_i, marker_j, reaction_marker,
        Float64.(translational_stiffness),
        Float64.(translational_damping), Float64.(rotational_stiffness),
        Float64.(rotational_damping), Float64(damping_time_scale),
        active_during, Ref(:dynamic in active_during),
        variables[1:3], variables[4:6], variables[7:9], variables[10:12],
        variables[13:15], variables[16:18], variables[19:21],
        variables[22:24], kinematics, loads)
end

"""Select whether this bushing is active in the current analysis stage."""
function set_spatial_bushing_stage!(bushing::SpatialBushingComponent, stage)
    bushing.active[] = stage in bushing.active_during
    bushing
end

function bushing_bryant_angles(orientation_i, orientation_j)
    relative = transpose(orientation_j) * orientation_i
    [atan(relative[3, 2], relative[3, 3]),
     atan(-relative[3, 1], hypot(relative[3, 2], relative[3, 3])),
     atan(relative[2, 1], relative[1, 1])]
end

function spatial_bushing_kinematics(bushing::SpatialBushingComponent, z)
    first = marker_point_kinematics(bushing.marker_i, z)
    second = marker_point_kinematics(bushing.marker_j, z)
    angular_i = marker_angular_kinematics(bushing.marker_i, z)
    angular_j = marker_angular_kinematics(bushing.marker_j, z)
    orientation_i = spatial_marker_orientation(bushing.marker_i, z)
    orientation_j = spatial_marker_orientation(bushing.marker_j, z)
    separation = first.position - second.position
    relative_velocity = first.velocity - second.velocity
    translation = transpose(orientation_j) * separation
    frame_omega = transpose(orientation_j) * angular_j.omega
    translation_rate = transpose(orientation_j) * relative_velocity -
        cross(frame_omega, translation)
    angles = bushing_bryant_angles(orientation_i, orientation_j)
    angular_velocity = transpose(orientation_j) *
        (angular_i.omega - angular_j.omega)
    (; translation, translation_rate, angles, angular_velocity,
       first, second, angular_i, angular_j, orientation_i, orientation_j,
       separation, relative_velocity, frame_omega)
end

function spatial_bushing_values(bushing::SpatialBushingComponent, z)
    kinematics = spatial_bushing_kinematics(bushing, z)
    (; translation = kinematics.translation,
       translation_rate = kinematics.translation_rate,
       angles = kinematics.angles,
       angular_velocity = kinematics.angular_velocity,
       local_force = @view(z[bushing.local_force_variables]),
       local_torque = @view(z[bushing.local_torque_variables]),
       global_force = @view(z[bushing.global_force_variables]),
       global_torque = @view(z[bushing.global_torque_variables]))
end

function initialize_spatial_bushing!(initial, bushing)
    values = spatial_bushing_kinematics(bushing, initial)
    initial[bushing.translation_variables] .= values.translation
    initial[bushing.translation_rate_variables] .= values.translation_rate
    initial[bushing.angle_variables] .= values.angles
    initial[bushing.angular_velocity_variables] .= values.angular_velocity
    local_force = if bushing.active[]
        -bushing.translational_stiffness .* values.translation .-
            bushing.translational_damping .* values.translation_rate
    else
        zeros(eltype(initial), 3)
    end
    local_torque = if bushing.active[]
        -bushing.rotational_stiffness .* values.angles .-
            bushing.rotational_damping .* values.angular_velocity
    else
        zeros(eltype(initial), 3)
    end
    orientation_j = spatial_marker_orientation(bushing.marker_j, initial)
    initial[bushing.local_force_variables] .= local_force
    initial[bushing.local_torque_variables] .= local_torque
    initial[bushing.global_force_variables] .= orientation_j * local_force
    initial[bushing.global_torque_variables] .= orientation_j * local_torque
    initial
end

function add_bushing_partials!(jacobian, rows, columns, partials)
    isempty(columns) || (jacobian[rows, columns] .-= partials)
    nothing
end

function axis_dot_partials(axis_j, axis_i)
    value = dot(axis_j.direction, axis_i.direction)
    first = isnothing(axis_i.body) ? nothing :
        vec(transpose(axis_i.direction_parameters) * axis_j.direction)
    second = isnothing(axis_j.body) ? nothing :
        vec(transpose(axis_j.direction_parameters) * axis_i.direction)
    value, first, second
end

function combine_gradient(first, second, first_factor, second_factor)
    isnothing(first) && return nothing
    first_factor .* first .+ second_factor .* second
end

function atan_gradient(sine, cosine, sine_gradient, cosine_gradient)
    isnothing(sine_gradient) && return nothing
    scale = sine^2 + cosine^2
    scale > eps(Float64) || throw(DomainError(scale,
        "a spatial bushing Bryant angle is undefined"))
    (cosine .* sine_gradient .- sine .* cosine_gradient) ./ scale
end

function bushing_bryant_angle_partials(bushing, z)
    x_i = marker_axis_kinematics(bushing.marker_i, z, 1)
    y_i = marker_axis_kinematics(bushing.marker_i, z, 2)
    z_i = marker_axis_kinematics(bushing.marker_i, z, 3)
    x_j = marker_axis_kinematics(bushing.marker_j, z, 1)
    y_j = marker_axis_kinematics(bushing.marker_j, z, 2)
    z_j = marker_axis_kinematics(bushing.marker_j, z, 3)

    r32, r32_i, r32_j = axis_dot_partials(z_j, y_i)
    r33, r33_i, r33_j = axis_dot_partials(z_j, z_i)
    r31, r31_i, r31_j = axis_dot_partials(z_j, x_i)
    r21, r21_i, r21_j = axis_dot_partials(y_j, x_i)
    r11, r11_i, r11_j = axis_dot_partials(x_j, x_i)

    transverse = hypot(r32, r33)
    transverse > eps(Float64) || throw(DomainError(transverse,
        "spatial bushing Bryant angles are singular when abs(alpha_y) " *
        "is 90 degrees"))
    transverse_i = combine_gradient(r32_i, r33_i,
        r32 / transverse, r33 / transverse)
    transverse_j = combine_gradient(r32_j, r33_j,
        r32 / transverse, r33 / transverse)

    first_parameters = isnothing(r32_i) ? nothing : zeros(eltype(z), 3, 4)
    second_parameters = isnothing(r32_j) ? nothing : zeros(eltype(z), 3, 4)
    if !isnothing(first_parameters)
        first_parameters[1, :] .= atan_gradient(
            r32, r33, r32_i, r33_i)
        first_parameters[2, :] .= atan_gradient(
            -r31, transverse, -r31_i, transverse_i)
        first_parameters[3, :] .= atan_gradient(
            r21, r11, r21_i, r11_i)
    end
    if !isnothing(second_parameters)
        second_parameters[1, :] .= atan_gradient(
            r32, r33, r32_j, r33_j)
        second_parameters[2, :] .= atan_gradient(
            -r31, transverse, -r31_j, transverse_j)
        second_parameters[3, :] .= atan_gradient(
            r21, r11, r21_j, r11_j)
    end
    first_parameters, second_parameters
end

function periodic_angle_difference(angle, measured)
    sine_angle, cosine_angle = sincos(angle)
    sine_measured, cosine_measured = sincos(measured)
    atan(sine_angle * cosine_measured - cosine_angle * sine_measured,
        cosine_angle * cosine_measured + sine_angle * sine_measured)
end

function executable_blocks(bushing::SpatialBushingComponent)
    kinematics! = function (equations, t, z, zdot)
        values = spatial_bushing_kinematics(bushing, z)
        angle_residuals = [periodic_angle_difference(
            z[bushing.angle_variables[component]], values.angles[component])
            for component in 1:3]
        equations[bushing.kinematic_equations] .=
            [z[bushing.translation_variables] - values.translation;
             z[bushing.translation_rate_variables] - values.translation_rate;
             angle_residuals;
             z[bushing.angular_velocity_variables] - values.angular_velocity]
    end
    kinematics_jacobian! = function (jacobian, t, z, zdot, coefficient)
        variables = [collect(bushing.translation_variables);
                     collect(bushing.translation_rate_variables);
                     collect(bushing.angle_variables);
                     collect(bushing.angular_velocity_variables)]
        for (row, variable) in zip(bushing.kinematic_equations, variables)
            jacobian[row, variable] += 1
        end
        values = spatial_bushing_kinematics(bushing, z)
        transpose_j = transpose(values.orientation_j)
        translation_rows = bushing.kinematic_equations[1:3]
        rate_rows = bushing.kinematic_equations[4:6]
        angle_rows = bushing.kinematic_equations[7:9]
        omega_rows = bushing.kinematic_equations[10:12]
        translation_skew = skew(values.translation)
        frame_omega_skew = skew(values.frame_omega)

        if !isnothing(values.first.body)
            body = values.first.body
            translation_position = transpose_j
            translation_parameters = transpose_j *
                values.first.position_parameters
            rate_parameters = transpose_j *
                values.first.velocity_parameters -
                frame_omega_skew * translation_parameters
            add_bushing_partials!(jacobian, translation_rows,
                body.position_variables, translation_position)
            add_bushing_partials!(jacobian, translation_rows,
                body.euler_parameter_variables, translation_parameters)
            add_bushing_partials!(jacobian, rate_rows,
                body.position_variables,
                -frame_omega_skew * translation_position)
            add_bushing_partials!(jacobian, rate_rows,
                body.euler_parameter_variables, rate_parameters)
            add_bushing_partials!(jacobian, rate_rows,
                body.velocity_variables, transpose_j)
            add_bushing_partials!(jacobian, rate_rows,
                body.angular_velocity_variables,
                transpose_j * values.first.velocity_omega)
            add_bushing_partials!(jacobian, omega_rows,
                body.euler_parameter_variables,
                transpose_j * values.angular_i.omega_parameters)
            add_bushing_partials!(jacobian, omega_rows,
                body.angular_velocity_variables,
                transpose_j * values.angular_i.orientation)
        end

        if !isnothing(values.second.body)
            body = values.second.body
            parameters = @view z[body.euler_parameter_variables]
            marker_orientation = bushing.marker_j.orientation_body
            frame_translation_parameters = transpose(marker_orientation) *
                rotation_transpose_vector_jacobian(
                    parameters, values.separation)
            translation_position = -transpose_j
            translation_parameters = frame_translation_parameters -
                transpose_j * values.second.position_parameters
            frame_velocity_parameters = transpose(marker_orientation) *
                rotation_transpose_vector_jacobian(
                    parameters, values.relative_velocity)
            frame_omega_parameters = transpose(marker_orientation) *
                rotation_transpose_vector_jacobian(
                    parameters, values.angular_j.omega) +
                transpose_j * values.angular_j.omega_parameters
            rate_parameters = frame_velocity_parameters -
                transpose_j * values.second.velocity_parameters -
                frame_omega_skew * translation_parameters +
                translation_skew * frame_omega_parameters
            add_bushing_partials!(jacobian, translation_rows,
                body.position_variables, translation_position)
            add_bushing_partials!(jacobian, translation_rows,
                body.euler_parameter_variables, translation_parameters)
            add_bushing_partials!(jacobian, rate_rows,
                body.position_variables,
                -frame_omega_skew * translation_position)
            add_bushing_partials!(jacobian, rate_rows,
                body.euler_parameter_variables, rate_parameters)
            add_bushing_partials!(jacobian, rate_rows,
                body.velocity_variables, -transpose_j)
            add_bushing_partials!(jacobian, rate_rows,
                body.angular_velocity_variables,
                -transpose_j * values.second.velocity_omega +
                translation_skew * transpose_j *
                    values.angular_j.orientation)
            relative_omega = values.angular_i.omega - values.angular_j.omega
            omega_parameters = transpose(marker_orientation) *
                rotation_transpose_vector_jacobian(
                    parameters, relative_omega) -
                transpose_j * values.angular_j.omega_parameters
            add_bushing_partials!(jacobian, omega_rows,
                body.euler_parameter_variables, omega_parameters)
            add_bushing_partials!(jacobian, omega_rows,
                body.angular_velocity_variables,
                -transpose_j * values.angular_j.orientation)
        end

        first_parameters, second_parameters =
            bushing_bryant_angle_partials(bushing, z)
        if !isnothing(first_parameters)
            add_bushing_partials!(jacobian, angle_rows,
                bushing.marker_i.body.euler_parameter_variables,
                first_parameters)
        end
        if !isnothing(second_parameters)
            add_bushing_partials!(jacobian, angle_rows,
                bushing.marker_j.body.euler_parameter_variables,
                second_parameters)
        end
        nothing
    end
    load! = function (equations, t, z, zdot)
        local_force = @view z[bushing.local_force_variables]
        local_torque = @view z[bushing.local_torque_variables]
        orientation_j = spatial_marker_orientation(bushing.marker_j, z)
        equations[bushing.load_equations[1:3]] .= local_force
        equations[bushing.load_equations[4:6]] .= local_torque
        if bushing.active[]
            equations[bushing.load_equations[1:3]] .+=
                bushing.translational_stiffness .*
                    z[bushing.translation_variables] .+
                bushing.translational_damping .*
                    z[bushing.translation_rate_variables]
            equations[bushing.load_equations[4:6]] .+=
                bushing.rotational_stiffness .* z[bushing.angle_variables] .+
                bushing.rotational_damping .*
                    z[bushing.angular_velocity_variables]
        end
        equations[bushing.load_equations[7:9]] .=
            z[bushing.global_force_variables] - orientation_j * local_force
        equations[bushing.load_equations[10:12]] .=
            z[bushing.global_torque_variables] - orientation_j * local_torque
    end
    load_jacobian! = function (jacobian, t, z, zdot, coefficient)
        for component in 1:3
            force_row = bushing.load_equations[component]
            torque_row = bushing.load_equations[component + 3]
            jacobian[force_row, bushing.local_force_variables[component]] += 1
            if bushing.active[]
                jacobian[force_row,
                    bushing.translation_variables[component]] +=
                    bushing.translational_stiffness[component]
                jacobian[force_row,
                    bushing.translation_rate_variables[component]] +=
                    bushing.translational_damping[component]
            end
            jacobian[torque_row,
                bushing.local_torque_variables[component]] += 1
            if bushing.active[]
                jacobian[torque_row, bushing.angle_variables[component]] +=
                    bushing.rotational_stiffness[component]
                jacobian[torque_row,
                    bushing.angular_velocity_variables[component]] +=
                    bushing.rotational_damping[component]
            end
        end
        orientation_j = spatial_marker_orientation(bushing.marker_j, z)
        jacobian[bushing.load_equations[7:9],
            bushing.global_force_variables] .+= Matrix{eltype(z)}(I, 3, 3)
        jacobian[bushing.load_equations[7:9],
            bushing.local_force_variables] .-= orientation_j
        jacobian[bushing.load_equations[10:12],
            bushing.global_torque_variables] .+= Matrix{eltype(z)}(I, 3, 3)
        jacobian[bushing.load_equations[10:12],
            bushing.local_torque_variables] .-= orientation_j
        if bushing.marker_j isa SpatialBodyMarker
            body = bushing.marker_j.body
            parameters = @view z[body.euler_parameter_variables]
            marker_orientation = bushing.marker_j.orientation_body
            local_force = marker_orientation *
                z[bushing.local_force_variables]
            local_torque = marker_orientation *
                z[bushing.local_torque_variables]
            jacobian[bushing.load_equations[7:9],
                body.euler_parameter_variables] .-=
                rotation_vector_jacobian(parameters, local_force)
            jacobian[bushing.load_equations[10:12],
                body.euler_parameter_variables] .-=
                rotation_vector_jacobian(parameters, local_torque)
        end
        nothing
    end
    ExecutableEquationBlock[
        ExecutableEquationBlock(bushing.name, :kinematics,
            collect(bushing.kinematic_equations), kinematics!,
            kinematics_jacobian!),
        ExecutableEquationBlock(bushing.name, :load,
            collect(bushing.load_equations), load!, load_jacobian!),
    ]
end

function equation_contributions(bushing::SpatialBushingComponent)
    rows = body_force_rows(bushing.marker_i)
    if !isnothing(bushing.reaction_marker)
        append!(rows, collect(bushing.reaction_marker.body.balance_equations))
    end
    unique!(rows)
    residual! = function (equations, t, z, zdot)
        global_force = @view z[bushing.global_force_variables]
        global_torque = @view z[bushing.global_torque_variables]
        add_reaction_to_body!(equations, z, bushing.marker_i,
            global_force, 1)
        add_applied_torque_to_body!(equations, z, bushing.marker_i,
            global_torque, 1)
        if !isnothing(bushing.reaction_marker)
            add_floating_force!(equations, z, bushing.reaction_marker,
                -global_force)
            add_applied_torque_to_body!(equations, z,
                bushing.reaction_marker, global_torque, -1)
        end
    end
    jacobian! = function (jacobian, t, z, zdot, coefficient)
        global_force = @view z[bushing.global_force_variables]
        global_torque = @view z[bushing.global_torque_variables]
        add_reaction_jacobian!(jacobian, z, bushing.marker_i,
            global_force, 1, bushing.global_force_variables)
        add_applied_torque_jacobian!(jacobian, z, bushing.marker_i,
            global_torque, 1, bushing.global_torque_variables)
        if !isnothing(bushing.reaction_marker)
            add_floating_force_jacobian!(jacobian, z,
                bushing.reaction_marker, global_force,
                bushing.global_force_variables, -1)
            add_applied_torque_jacobian!(jacobian, z,
                bushing.reaction_marker, global_torque, -1,
                bushing.global_torque_variables)
        end
    end
    EquationContribution[EquationContribution(bushing.name, :wrench_to_bodies,
        rows, residual!, jacobian!)]
end

end
