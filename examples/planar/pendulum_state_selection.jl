using LinearAlgebra

if !isdefined(@__MODULE__, :PendulumParameters)
    include(joinpath(@__DIR__, "minimal_cartesian_pendulum.jl"))
end

const PLANAR_VELOCITY_NAMES = [:V_x, :V_y, :omega]

"""Select dependent and independent columns of a full-row-rank matrix."""
function select_velocity_columns(D; row_scale = ones(size(D, 1)),
                                    column_scale = ones(size(D, 2)),
                                    rank_tolerance = nothing)
    D_scaled = Diagonal(row_scale) * D * Diagonal(column_scale)
    factorization = qr(D_scaled, ColumnNorm())
    pivots = collect(factorization.p)
    diagonal = abs.(diag(factorization.R))

    tolerance = if isnothing(rank_tolerance)
        isempty(diagonal) ? zero(eltype(D_scaled)) :
            max(size(D_scaled)...) * eps(real(float(one(eltype(D_scaled))))) *
            maximum(diagonal)
    else
        rank_tolerance
    end
    rank = count(value -> value > tolerance, diagonal)

    dependent = pivots[1:rank]
    independent = pivots[(rank + 1):end]
    return (
        D_scaled = D_scaled,
        pivots = pivots,
        rank = rank,
        dependent = dependent,
        independent = independent,
        dependent_condition = cond(D_scaled[:, dependent]),
    )
end

"""
Construct the null-space mapping without forming a null-space basis that
mixes physical coordinates. The independent entries retain unit coefficients.
"""
function coordinate_tangent_mapping(D, dependent, independent)
    D_d = D[:, dependent]
    D_i = D[:, independent]
    H = zeros(eltype(D), size(D, 2), length(independent))
    H[dependent, :] .= -(D_d \ D_i)
    H[independent, :] .= I(length(independent))
    return H
end

"""Run state selection for one planar-pendulum configuration."""
function pendulum_state_selection(theta;
                                  p = PendulumParameters(),
                                  row_scale = ones(2),
                                  column_scale = ones(3))
    q = [p.pin; theta]
    D = constraint_partial(q, p)
    selection = select_velocity_columns(D; row_scale, column_scale)
    P = coordinate_tangent_mapping(
        D, selection.dependent, selection.independent
    )
    return merge(selection, (D = D, P = P))
end

"""Check a preferred independent-column set and construct its mapping."""
function preferred_velocity_selection(D, independent)
    dependent = setdiff(collect(axes(D, 2)), independent)
    D_d = D[:, dependent]
    size(D_d, 1) == size(D_d, 2) ||
        error("The preferred dependent block must be square")
    rank(D_d) == size(D_d, 1) ||
        error("The preferred dependent block is singular")
    return (
        D = D,
        dependent = dependent,
        independent = independent,
        dependent_condition = cond(D_d),
        P = coordinate_tangent_mapping(D, dependent, independent),
    )
end

"""Check a user-preferred independent set for the one-body pendulum."""
function preferred_pendulum_selection(theta;
                                      p = PendulumParameters(),
                                      independent = [3])
    q = [p.pin; theta]
    D = constraint_partial(q, p)
    return preferred_velocity_selection(D, independent)
end

if abspath(PROGRAM_FILE) == @__FILE__
    theta = deg2rad(35.0)
    p = PendulumParameters()
    unscaled = pendulum_state_selection(theta; p)

    # Scale angular velocity by the pendulum's characteristic length. This
    # gives all columns compatible velocity units. The preferred-coordinate
    # check below makes the intended physical choice explicit.
    length_scale = norm(p.r_body)
    scaled = pendulum_state_selection(
        theta; p, column_scale = [length_scale, length_scale, 1.0]
    )
    preferred = preferred_pendulum_selection(theta; p)

    println("Planar-pendulum state selection at theta = 35 degrees")
    for (label, result) in (("unscaled QR", unscaled),
                            ("dimensionally scaled QR", scaled),
                            ("preferred omega", preferred))
        println("  ", label)
        println("    dependent: ", PLANAR_VELOCITY_NAMES[result.dependent])
        println("    independent: ", PLANAR_VELOCITY_NAMES[result.independent])
        println("    condition(D_d): ", result.dependent_condition)
        println("    ||D P||_inf: ", norm(result.D * result.P, Inf))
    end
end
