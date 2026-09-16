"""
    PlanarAppliedForces

Low-level planar marker kinematics and applied-force equations. These routines
know how a point or orientation marker maps canonical body variables into
global positions, velocities, forces, and torques. Higher-level component
assembly wraps them with names, allocation, and equation ownership.

Force routines add their action to the first marker and their equal-and-
opposite reaction to the second marker when that marker belongs to a body.
Ground markers intentionally have no balance equations.
"""
module PlanarAppliedForces

using LinearAlgebra

export PlanarBodyOrientationMarker, PlanarGroundOrientationMarker,
       PlanarTorsionalSpringDamper, relative_rotation,
       add_torsional_spring_damper!, add_torsional_spring_damper_jacobian!,
       PlanarBodyPointMarker, PlanarGroundPointMarker,
       PlanarFloatingPointMarker,
       PlanarSpanningSpringDamper, spanning_force_kinematics,
       add_spanning_spring_damper!, add_spanning_spring_damper_jacobian!

abstract type AbstractPlanarOrientationMarker end

"""Orientation marker fixed to a planar body."""
struct PlanarBodyOrientationMarker{T} <: AbstractPlanarOrientationMarker
    theta_variable::Int
    omega_variable::Int
    torque_equation::Int
    angle_offset::T
end

abstract type AbstractPlanarPointMarker end

"""Point marker fixed to a planar body."""
struct PlanarBodyPointMarker{T} <: AbstractPlanarPointMarker
    position_variables::UnitRange{Int}
    theta_variable::Int
    velocity_variables::UnitRange{Int}
    omega_variable::Int
    force_equations::UnitRange{Int}
    torque_equation::Int
    r_body::Vector{T}
end


"""Point marker fixed to ground."""
struct PlanarGroundPointMarker{T} <: AbstractPlanarPointMarker
    position::Vector{T}
end

"""
Point marker whose translation follows another marker while force ownership
remains with a planar body.
"""
struct PlanarFloatingPointMarker{M} <: AbstractPlanarPointMarker
    owner_position_variables::UnitRange{Int}
    force_equations::UnitRange{Int}
    torque_equation::Int
    follower::M
end

"""
Axial spring-damper spanning two planar point markers.

The nine local scalar variables are the global spanning vector, its length,
unit direction, length rate, signed axial force, and global force vector.
"""
struct PlanarSpanningSpringDamper{M1,M2,T}
    marker_1::M1
    marker_2::M2
    stiffness::T
    damping::T
    free_length::T
    spanning_variables::UnitRange{Int}
    length_variable::Int
    unit_variables::UnitRange{Int}
    length_rate_variable::Int
    force_variable::Int
    global_force_variables::UnitRange{Int}
    spanning_equations::UnitRange{Int}
    length_equation::Int
    unit_equations::UnitRange{Int}
    length_rate_equation::Int
    force_equation::Int
    global_force_equations::UnitRange{Int}
end

function point_marker_kinematics(marker::PlanarBodyPointMarker, z)
    theta = z[marker.theta_variable]
    omega = z[marker.omega_variable]
    cosine = cos(theta)
    sine = sin(theta)
    rotation = [cosine -sine; sine cosine]
    skew = [zero(theta) -one(theta); one(theta) zero(theta)]
    r_global = rotation * marker.r_body
    d_global = rotation * skew * marker.r_body
    return (
        position = z[marker.position_variables] + r_global,
        velocity = z[marker.velocity_variables] + d_global .* omega,
        r = r_global,
        d = d_global,
    )
end

function point_marker_kinematics(marker::PlanarGroundPointMarker, z)
    T = eltype(z)
    return (
        position = T.(marker.position),
        velocity = zeros(T, 2),
        r = zeros(T, 2),
        d = zeros(T, 2),
    )
end

function point_marker_kinematics(marker::PlanarFloatingPointMarker, z)
    follower = point_marker_kinematics(marker.follower, z)
    r = follower.position - z[marker.owner_position_variables]
    return (
        position = follower.position,
        velocity = follower.velocity,
        r,
        d = [-r[2], r[1]],
    )
end

"""Return the explicit local variables and marker kinematics."""
function spanning_force_kinematics(element::PlanarSpanningSpringDamper, z)
    marker_1 = point_marker_kinematics(element.marker_1, z)
    marker_2 = point_marker_kinematics(element.marker_2, z)
    spanning = z[element.spanning_variables]
    length = z[element.length_variable]
    length > zero(length) ||
        throw(DomainError(length, "spanning-force length must be positive"))
    unit = z[element.unit_variables]
    relative_velocity = marker_2.velocity - marker_1.velocity
    length_rate = z[element.length_rate_variable]
    scalar_force = z[element.force_variable]
    global_force = z[element.global_force_variables]
    return (;
        marker_1,
        marker_2,
        spanning,
        length,
        unit,
        relative_velocity,
        length_rate,
        scalar_force,
        global_force,
    )
end

function add_body_point_force!(equations, marker::PlanarBodyPointMarker,
                               kinematics, global_force)
    equations[marker.force_equations] .-= global_force
    equations[marker.torque_equation] -= dot(kinematics.d, global_force)
    return nothing
end


add_body_point_force!(equations, marker::PlanarGroundPointMarker,
                      kinematics, global_force) = nothing

function add_body_point_force!(equations, marker::PlanarFloatingPointMarker,
                               kinematics, global_force)
    equations[marker.force_equations] .-= global_force
    equations[marker.torque_equation] -= dot(kinematics.d, global_force)
    return nothing
end

"""Add the six equation blocks, comprising nine scalar equations."""
function add_spanning_spring_damper!(equations, z,
                                     element::PlanarSpanningSpringDamper)
    local_values = spanning_force_kinematics(element, z)
    equations[element.spanning_equations] .= local_values.spanning .-
        (local_values.marker_2.position - local_values.marker_1.position)
    equations[element.length_equation] = local_values.length -
        sqrt(dot(local_values.spanning, local_values.spanning))
    equations[element.unit_equations] .= local_values.unit .-
        local_values.spanning ./ local_values.length
    equations[element.length_rate_equation] = local_values.length_rate -
        dot(local_values.unit, local_values.relative_velocity)
    equations[element.force_equation] = local_values.scalar_force +
        element.stiffness * (local_values.length - element.free_length) +
        element.damping * local_values.length_rate
    equations[element.global_force_equations] .= local_values.global_force .+
        local_values.unit .* local_values.scalar_force
    add_body_point_force!(equations, element.marker_1,
        local_values.marker_1, local_values.global_force)
    add_body_point_force!(equations, element.marker_2,
        local_values.marker_2, -local_values.global_force)
    return nothing
end

function add_spanning_marker_partials!(jacobian, rows,
        marker::PlanarBodyPointMarker, sign, kinematics)
    for component in 1:2
        jacobian[rows[component], marker.position_variables[component]] += sign
        jacobian[rows[component], marker.theta_variable] +=
            sign * kinematics.d[component]
    end
    return nothing
end


add_spanning_marker_partials!(jacobian, rows,
    marker::PlanarGroundPointMarker, sign, kinematics) = nothing

function add_length_rate_marker_partials!(jacobian, row,
        marker::PlanarBodyPointMarker, relative_sign, kinematics,
        unit, omega)
    jacobian[row, marker.velocity_variables] .+=
        -relative_sign .* unit
    jacobian[row, marker.omega_variable] +=
        -relative_sign * dot(unit, kinematics.d)
    jacobian[row, marker.theta_variable] +=
        relative_sign * dot(unit, kinematics.r) * omega
    return nothing
end


add_length_rate_marker_partials!(jacobian, row,
    marker::PlanarGroundPointMarker, relative_sign, kinematics,
    unit, omega) = nothing

function add_point_force_partials!(jacobian,
        marker::PlanarBodyPointMarker, force_sign, kinematics,
        force_variables, global_force)
    for component in 1:2
        jacobian[marker.force_equations[component],
            force_variables[component]] += -force_sign
        jacobian[marker.torque_equation, force_variables[component]] +=
            -force_sign * kinematics.d[component]
    end
    jacobian[marker.torque_equation, marker.theta_variable] +=
        force_sign * dot(kinematics.r, global_force)
    return nothing
end


add_point_force_partials!(jacobian, marker::PlanarGroundPointMarker,
    force_sign, kinematics, force_variables, global_force) = nothing

"""Add analytical partials for the nine-equation spanning element."""
function add_spanning_spring_damper_jacobian!(jacobian, z,
        element::PlanarSpanningSpringDamper)
    local_values = spanning_force_kinematics(element, z)
    s = element.spanning_variables
    ell = element.length_variable
    unit_variables = element.unit_variables
    ell_dot = element.length_rate_variable
    scalar_force = element.force_variable
    global_force_variables = element.global_force_variables

    for component in 1:2
        jacobian[element.spanning_equations[component], s[component]] += 1
    end
    add_spanning_marker_partials!(jacobian, element.spanning_equations,
        element.marker_1, 1, local_values.marker_1)
    add_spanning_marker_partials!(jacobian, element.spanning_equations,
        element.marker_2, -1, local_values.marker_2)

    jacobian[element.length_equation, ell] += 1
    span_norm = sqrt(dot(local_values.spanning, local_values.spanning))
    jacobian[element.length_equation, s] .-=
        local_values.spanning ./ span_norm

    for component in 1:2
        row = element.unit_equations[component]
        jacobian[row, unit_variables[component]] += 1
        jacobian[row, s[component]] -= 1 / local_values.length
        jacobian[row, ell] +=
            local_values.spanning[component] / local_values.length^2
    end

    rate_row = element.length_rate_equation
    jacobian[rate_row, ell_dot] += 1
    jacobian[rate_row, unit_variables] .-=
        local_values.relative_velocity
    omega_1 = element.marker_1 isa PlanarBodyPointMarker ?
        z[element.marker_1.omega_variable] : zero(eltype(z))
    omega_2 = element.marker_2 isa PlanarBodyPointMarker ?
        z[element.marker_2.omega_variable] : zero(eltype(z))
    add_length_rate_marker_partials!(jacobian, rate_row,
        element.marker_1, -1, local_values.marker_1,
        local_values.unit, omega_1)
    add_length_rate_marker_partials!(jacobian, rate_row,
        element.marker_2, 1, local_values.marker_2,
        local_values.unit, omega_2)

    force_row = element.force_equation
    jacobian[force_row, scalar_force] += 1
    jacobian[force_row, ell] += element.stiffness
    jacobian[force_row, ell_dot] += element.damping

    for component in 1:2
        row = element.global_force_equations[component]
        jacobian[row, global_force_variables[component]] += 1
        jacobian[row, unit_variables[component]] += local_values.scalar_force
        jacobian[row, scalar_force] += local_values.unit[component]
    end

    add_point_force_partials!(jacobian, element.marker_1, 1,
        local_values.marker_1, global_force_variables,
        local_values.global_force)
    add_point_force_partials!(jacobian, element.marker_2, -1,
        local_values.marker_2, global_force_variables,
        local_values.global_force)
    return nothing
end

"""Orientation marker fixed to ground."""
struct PlanarGroundOrientationMarker{T} <: AbstractPlanarOrientationMarker
    angle::T
end

"""
Marker-to-marker linear torsional spring and damper.

Positive torque is defined by
    torque = -stiffness*(relative_angle - free_angle) -
              damping*relative_angular_velocity.
The torque is the load on marker_1; marker_2 receives its opposite.
"""
struct PlanarTorsionalSpringDamper{M1,M2,T}
    marker_1::M1
    marker_2::M2
    stiffness::T
    damping::T
    free_angle::T
    torque_variable::Int
    constitutive_equation::Int
end

marker_angle(marker::PlanarBodyOrientationMarker, z) =
    z[marker.theta_variable] + marker.angle_offset
marker_angle(marker::PlanarGroundOrientationMarker, z) = marker.angle

marker_angular_velocity(marker::PlanarBodyOrientationMarker, z) =
    z[marker.omega_variable]
marker_angular_velocity(marker::PlanarGroundOrientationMarker, z) =
    zero(eltype(z))

"""Relative angle from marker 2 to marker 1 and its angular velocity."""
function relative_rotation(element::PlanarTorsionalSpringDamper, z)
    angle_1 = marker_angle(element.marker_1, z)
    angle_2 = marker_angle(element.marker_2, z)
    axis_1 = (cos(angle_1), sin(angle_1))
    axis_2 = (cos(angle_2), sin(angle_2))
    x_relative = axis_2[1] * axis_1[1] + axis_2[2] * axis_1[2]
    y_relative = axis_2[1] * axis_1[2] - axis_2[2] * axis_1[1]
    relative_angle = atan(y_relative, x_relative)
    relative_angular_velocity =
        marker_angular_velocity(element.marker_1, z) -
        marker_angular_velocity(element.marker_2, z)
    return (
        angle = relative_angle,
        angular_velocity = relative_angular_velocity,
        x = x_relative,
        y = y_relative,
    )
end

function add_marker_torque!(equations, marker::PlanarBodyOrientationMarker,
                            torque)
    # Balance equations have the form inertia*alpha - applied_torque = 0.
    equations[marker.torque_equation] -= torque
    return nothing
end

add_marker_torque!(equations, marker::PlanarGroundOrientationMarker,
                   torque) = nothing

"""Add the constitutive equation and equal-and-opposite body torques."""
function add_torsional_spring_damper!(equations, z,
                                      element::PlanarTorsionalSpringDamper)
    relative = relative_rotation(element, z)
    torque = z[element.torque_variable]
    equations[element.constitutive_equation] += torque +
        element.stiffness * (relative.angle - element.free_angle) +
        element.damping * relative.angular_velocity
    add_marker_torque!(equations, element.marker_1, torque)
    add_marker_torque!(equations, element.marker_2, -torque)
    return nothing
end

function add_marker_torque_jacobian!(jacobian,
        marker::PlanarBodyOrientationMarker, torque_variable, sign)
    # Applied marker torque is sign*torque and is subtracted in the balance.
    jacobian[marker.torque_equation, torque_variable] -= sign
    return nothing
end

add_marker_torque_jacobian!(jacobian,
    marker::PlanarGroundOrientationMarker, torque_variable, sign) = nothing

function add_constitutive_partials!(jacobian,
        marker::PlanarBodyOrientationMarker, row, angle_coefficient,
        omega_coefficient)
    jacobian[row, marker.theta_variable] += angle_coefficient
    jacobian[row, marker.omega_variable] += omega_coefficient
    return nothing
end

add_constitutive_partials!(jacobian,
    marker::PlanarGroundOrientationMarker, row, angle_coefficient,
    omega_coefficient) = nothing

"""Add analytical partials of the constitutive and torque contributions."""
function add_torsional_spring_damper_jacobian!(jacobian, z,
        element::PlanarTorsionalSpringDamper)
    row = element.constitutive_equation
    jacobian[row, element.torque_variable] += one(eltype(jacobian))
    add_constitutive_partials!(jacobian, element.marker_1, row,
        element.stiffness, element.damping)
    add_constitutive_partials!(jacobian, element.marker_2, row,
        -element.stiffness, -element.damping)
    add_marker_torque_jacobian!(jacobian, element.marker_1,
        element.torque_variable, 1)
    add_marker_torque_jacobian!(jacobian, element.marker_2,
        element.torque_variable, -1)
    return nothing
end

end
