using LinearAlgebra

if !isdefined(@__MODULE__, :select_velocity_columns)
    include(joinpath(@__DIR__, "pendulum_state_selection.jl"))
end

const TWO_BODY_VELOCITY_NAMES =
    [:V_1x, :V_1y, :omega_1, :V_2x, :V_2y, :omega_2]

"""Infinitesimal rotational direction of a body-fixed marker vector."""
function planar_marker_direction(theta, r_body)
    S = [0.0 -1.0; 1.0 0.0]
    return rotation(theta) * S * r_body
end

"""
Velocity-constraint partial for two planar bodies connected in a chain.

Body 1 is pinned to ground at its upper marker. Its lower marker is connected
by a revolute joint to the upper marker of body 2. The velocity ordering is
[V_1x, V_1y, omega_1, V_2x, V_2y, omega_2].
"""
function two_body_constraint_partial(theta_1, theta_2;
                                     upper_1 = [0.0, 0.5],
                                     lower_1 = [0.0, -0.5],
                                     upper_2 = [0.0, 0.5])
    d_upper_1 = planar_marker_direction(theta_1, upper_1)
    d_lower_1 = planar_marker_direction(theta_1, lower_1)
    d_upper_2 = planar_marker_direction(theta_2, upper_2)

    D = zeros(4, 6)

    # Ground pin: V_1 + d_upper_1*omega_1 = 0.
    D[1:2, 1:2] .= I(2)
    D[1:2, 3] .= d_upper_1

    # Interbody pin: V_1 + d_lower_1*omega_1
    #                  - V_2 - d_upper_2*omega_2 = 0.
    D[3:4, 1:2] .= I(2)
    D[3:4, 3] .= d_lower_1
    D[3:4, 4:5] .= -I(2)
    D[3:4, 6] .= -d_upper_2

    return D
end

"""Run the unscaled QR state selection for the two-body pendulum."""
function two_body_state_selection(theta_1, theta_2;
                                  row_scale = ones(4),
                                  column_scale = ones(6))
    D = two_body_constraint_partial(theta_1, theta_2)
    selection = select_velocity_columns(D; row_scale, column_scale)
    P = coordinate_tangent_mapping(
        D, selection.dependent, selection.independent
    )
    return merge(selection, (D = D, P = P))
end

"""Velocity partial after prescribing the base angular velocity."""
function driven_base_constraint_partial(theta_1, theta_2; kwargs...)
    D = two_body_constraint_partial(theta_1, theta_2; kwargs...)
    motion_row = zeros(eltype(D), 1, size(D, 2))
    motion_row[1, 3] = 1
    return [D; motion_row]
end

function driven_base_state_selection(theta_1, theta_2;
        preferred_independent = [6])
    D = driven_base_constraint_partial(theta_1, theta_2)
    qr_selection = select_velocity_columns(D)
    preferred = preferred_velocity_selection(D, preferred_independent)
    return (; D, qr_selection, preferred)
end

if abspath(PROGRAM_FILE) == @__FILE__
    theta_1 = deg2rad(35.0)
    theta_2 = deg2rad(-20.0)
    result = two_body_state_selection(theta_1, theta_2)
    preferred = preferred_velocity_selection(result.D, [3, 6])

    println("Two-body planar-pendulum state selection")
    println("  theta_1: 35 degrees")
    println("  theta_2: -20 degrees")
    println("  constraint matrix size: ", size(result.D))
    println("  numerical rank: ", result.rank)
    println("  unrestricted QR")
    println("    column pivots: ", TWO_BODY_VELOCITY_NAMES[result.pivots])
    println("    dependent: ", TWO_BODY_VELOCITY_NAMES[result.dependent])
    println("    independent: ", TWO_BODY_VELOCITY_NAMES[result.independent])
    println("    condition(D_d): ", result.dependent_condition)
    println("    ||D P||_inf: ", norm(result.D * result.P, Inf))
    println("  preferred angular velocities")
    println("    dependent: ", TWO_BODY_VELOCITY_NAMES[preferred.dependent])
    println("    independent: ", TWO_BODY_VELOCITY_NAMES[preferred.independent])
    println("    condition(D_d): ", preferred.dependent_condition)
    println("    ||D P||_inf: ", norm(preferred.D * preferred.P, Inf))
end
