using LinearAlgebra

if !isdefined(@__MODULE__, :select_velocity_columns)
    include(joinpath(@__DIR__, "pendulum_state_selection.jl"))
end

const INPLANE_VELOCITY_NAMES = [:V_x, :V_y, :omega]

inplane_marker_direction(theta, r_body) =
    rotation(theta) * [0.0 -1.0; 1.0 0.0] * r_body

"""
Velocity partial for a planar body guided by two parallel inplane constraints.

Both constraint directions are the global x direction. The body marker
locations are separated along the body y axis.
"""
function parallel_inplane_constraint_partial(theta;
        marker_1 = [0.0, 0.5], marker_2 = [0.0, -0.5],
        unit = [1.0, 0.0])
    direction_1 = inplane_marker_direction(theta, marker_1)
    direction_2 = inplane_marker_direction(theta, marker_2)
    normalized_unit = unit / norm(unit)
    return [normalized_unit' dot(direction_1, normalized_unit);
            normalized_unit' dot(direction_2, normalized_unit)]
end

function parallel_inplane_state_selection(theta = 0.0)
    D = parallel_inplane_constraint_partial(theta)
    selection = select_velocity_columns(D)
    mapping = coordinate_tangent_mapping(
        D, selection.dependent, selection.independent)
    return merge(selection, (D = D, P = mapping))
end

if abspath(PROGRAM_FILE) == @__FILE__
    result = parallel_inplane_state_selection()
    println("Parallel inplane-constraint state selection")
    println("  constraint matrix:")
    display(result.D)
    println("  rank: ", result.rank)
    println("  dependent: ", INPLANE_VELOCITY_NAMES[result.dependent])
    println("  independent: ", INPLANE_VELOCITY_NAMES[result.independent])
    println("  ||D P||_inf: ", norm(result.D * result.P, Inf))
end
