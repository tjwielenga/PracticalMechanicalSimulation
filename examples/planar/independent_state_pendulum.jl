using LinearAlgebra
using OrdinaryDiffEq

if !isdefined(@__MODULE__, :HistoricalDDASSL)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "HistoricalDDASSL.jl"))
end
if !isdefined(@__MODULE__, :reconstruct_cartesian)
    include(joinpath(@__DIR__, "reduced_coordinate_pendulum.jl"))
end

const INDEPENDENT_STATE_VARIABLE_LEVELS =
    [2, 2, 2, 1, 1, 1, 0, 0, 0, 2, 2]
const INDEPENDENT_STATE_EQUATION_LEVELS =
    [2, 2, 2, 2, 2, 1, 1, 0, 0, 2, 1]
const INDEPENDENT_STATE_DIFFERENTIAL_VARS =
    Bool[false, false, false, false, false, true,
         false, false, true, false, false]

"""
Eleven implicit equations with only omega and theta as differential states.

z = [a_x, a_y, alpha, V_x, V_y, omega,
     R_x, R_y, theta, lambda_x, lambda_y]
"""
function independent_state_pendulum!(equations, t, z, zdot,
                                     p::PendulumParameters)
    acceleration = view(z, 1:2)
    alpha = z[3]
    velocity = view(z, 4:5)
    omega = z[6]
    position = view(z, 7:8)
    theta = z[9]
    reaction = view(z, 10:11)
    r_global, d_global = marker_vectors(theta, p)

    equations[1:2] .= p.mass .* acceleration .- reaction .-
                      p.mass .* p.gravity
    equations[3] = p.inertia * alpha - dot(d_global, reaction)
    equations[4:5] .= acceleration .+ d_global .* alpha .-
                       r_global .* omega^2
    equations[6:7] .= velocity .+ d_global .* omega
    equations[8:9] .= position .+ r_global .- p.pin
    equations[10] = alpha - zdot[6]
    equations[11] = omega - zdot[9]
    return nothing
end

"""Analytical matrix F_z + coefficient*F_zdot."""
function independent_state_pendulum_jacobian!(jacobian, t, z, zdot,
                                               coefficient,
                                               p::PendulumParameters)
    fill!(jacobian, zero(eltype(jacobian)))
    alpha = z[3]
    omega = z[6]
    theta = z[9]
    reaction = view(z, 10:11)
    r_global, d_global = marker_vectors(theta, p)

    jacobian[1, 1] = p.mass
    jacobian[2, 2] = p.mass
    jacobian[1, 10] = -1
    jacobian[2, 11] = -1

    jacobian[3, 3] = p.inertia
    jacobian[3, 9] = dot(r_global, reaction)
    jacobian[3, 10:11] .= -d_global

    jacobian[4, 1] = 1
    jacobian[5, 2] = 1
    jacobian[4:5, 3] .= d_global
    jacobian[4:5, 6] .= -2 .* r_global .* omega
    jacobian[4:5, 9] .= -r_global .* alpha .-
                          d_global .* omega^2

    jacobian[6, 4] = 1
    jacobian[7, 5] = 1
    jacobian[6:7, 6] .= d_global
    jacobian[6:7, 9] .= -r_global .* omega

    jacobian[8, 7] = 1
    jacobian[9, 8] = 1
    jacobian[8:9, 9] .= d_global

    jacobian[10, 3] = 1
    jacobian[10, 6] = -coefficient
    jacobian[11, 6] = 1
    jacobian[11, 9] = -coefficient
    return nothing
end

function independent_state_initial_conditions(theta, omega,
                                              p::PendulumParameters)
    reduced_state = [theta, omega]
    reconstructed = reconstruct_cartesian(reduced_state, p)
    reaction = reduced_pin_reaction(reduced_state, p)
    z0 = [
        reconstructed.a_global
        reconstructed.alpha_p
        reconstructed.v[1:2]
        omega
        reconstructed.q[1:2]
        theta
        reaction
    ]
    zdot0 = zeros(eltype(z0), length(z0))
    zdot0[6] = reconstructed.alpha_p
    zdot0[9] = omega
    return z0, zdot0
end

function run_independent_state_pendulum(;
        theta0 = deg2rad(45.0), omega0 = 0.0,
        tspan = (0.0, 5.0), atol = 1.0e-7,
        rtol = 1.0e-5, dt = 1.0e-5, dtmax = 0.05)
    p = PendulumParameters()
    z0, zdot0 = independent_state_initial_conditions(theta0, omega0, p)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol,
        rtol = rtol,
        initial_step = dt,
        maximum_step = dtmax,
    )
    return HistoricalDDASSL.dassl(
        independent_state_pendulum!, z0, zdot0, tspan;
        parameter = p,
        jacobian! = independent_state_pendulum_jacobian!,
        options,
        variable_levels = INDEPENDENT_STATE_VARIABLE_LEVELS,
        equation_levels = INDEPENDENT_STATE_EQUATION_LEVELS,
        differential_vars = INDEPENDENT_STATE_DIFFERENTIAL_VARS,
        deficit = 0,
    )
end

function independent_state_diagnostics(solution, p::PendulumParameters)
    reduced_problem = ODEProblem(
        reduced_pendulum_rhs!, [solution.u[1][9], solution.u[1][6]],
        (first(solution.t), last(solution.t)), p,
    )
    reduced_solution = solve(
        reduced_problem, Tsit5(); abstol = 1.0e-11, reltol = 1.0e-11
    )

    equation_errors = Float64[]
    state_errors = Float64[]
    position_errors = Float64[]
    velocity_errors = Float64[]
    acceleration_errors = Float64[]
    reaction_errors = Float64[]
    energy_errors = Float64[]
    initial_energy = nothing

    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(eltype(z), length(z))
        independent_state_pendulum!(equations, t, z, zdot, p)
        push!(equation_errors, norm(equations, Inf))

        reduced_state = reduced_solution(t)
        reconstructed = reconstruct_cartesian(reduced_state, p)
        reference_reaction = reduced_pin_reaction(reduced_state, p)
        push!(state_errors,
            norm([z[9], z[6]] - reduced_state, Inf))
        push!(position_errors,
            norm([z[7], z[8], z[9]] - reconstructed.q, Inf))
        push!(velocity_errors,
            norm([z[4], z[5], z[6]] - reconstructed.v, Inf))
        push!(acceleration_errors,
            norm(z[1:3] -
                 [reconstructed.a_global; reconstructed.alpha_p], Inf))
        push!(reaction_errors, norm(z[10:11] - reference_reaction, Inf))

        energy = total_energy([z[7:9]; z[4:6]], p)
        isnothing(initial_energy) && (initial_energy = energy)
        push!(energy_errors, abs(energy - initial_energy))
    end

    return (
        return_code = solution.retcode,
        final_time = last(solution.t),
        accepted_time_points = length(solution.t),
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_integrated_state_difference = maximum(state_errors),
        maximum_position_difference = maximum(position_errors),
        maximum_velocity_difference = maximum(velocity_errors),
        maximum_acceleration_difference = maximum(acceleration_errors),
        maximum_reaction_difference = maximum(reaction_errors),
        maximum_energy_error = maximum(energy_errors),
    )
end

function independent_state_solver_diagnostics(solution)
    nonzero_steps = abs.(solution.steps[2:end])
    return (
        accepted_steps = solution.stats.accepted_steps,
        rejected_steps = solution.stats.rejected_steps,
        residual_evaluations = solution.stats.residual_evaluations,
        jacobian_evaluations = solution.stats.jacobian_evaluations,
        factorizations = solution.stats.factorizations,
        newton_iterations = solution.stats.newton_iterations,
        maximum_order = maximum(solution.orders),
        minimum_step = minimum(nonzero_steps),
        maximum_step = maximum(nonzero_steps),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution = run_independent_state_pendulum()
    parameters = PendulumParameters()
    println("Independent-state implicit pendulum")
    println("  simultaneous variables: 11")
    println("  differential states: 2")
    for (key, value) in pairs(
            independent_state_diagnostics(solution, parameters))
        println("  ", key, ": ", value)
    end
    for (key, value) in pairs(independent_state_solver_diagnostics(solution))
        println("  ", key, ": ", value)
    end
end
