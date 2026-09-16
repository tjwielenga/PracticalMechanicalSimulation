using LinearAlgebra
using OrdinaryDiffEq

# Reuse the physical parameters, rotation convention, and Cartesian model.
if !isdefined(@__MODULE__, :PendulumParameters)
    include(joinpath(@__DIR__, "minimal_cartesian_pendulum.jl"))
end

"""Reduced angular acceleration for the state [s, omega_p]."""
function reduced_acceleration(u, p::PendulumParameters)
    s = u[1]
    _, d_global = marker_vectors(s, p)
    inertia_about_pin = p.inertia + p.mass * dot(d_global, d_global)
    return -p.mass * dot(d_global, p.gravity) / inertia_about_pin
end

"""Two-state first-order right-hand side for SciML."""
function reduced_pendulum_rhs!(du, u, p::PendulumParameters, t)
    du[1] = u[2]
    du[2] = reduced_acceleration(u, p)
    return nothing
end

"""
Reconstruct Cartesian position, velocity, and acceleration from [s, omega_p].

Returns a named tuple containing q = [R_x, R_y, theta],
v = [V_x, V_y, omega], a_global, and alpha_p.
"""
function reconstruct_cartesian(u, p::PendulumParameters)
    s, omega_p = u
    r_global, d_global = marker_vectors(s, p)
    alpha_p = reduced_acceleration(u, p)

    position_global = p.pin - r_global
    velocity_global = -d_global * omega_p
    acceleration_global = -d_global * alpha_p + r_global * omega_p^2

    return (
        q = [position_global; s],
        v = [velocity_global; omega_p],
        a_global = acceleration_global,
        alpha_p = alpha_p,
    )
end

"""Recover the ideal-pin reaction from the reconstructed force equation."""
function reduced_pin_reaction(u, p::PendulumParameters)
    reconstructed = reconstruct_cartesian(u, p)
    return p.mass .* (reconstructed.a_global .- p.gravity)
end

"""Mechanical energy evaluated from the reduced state."""
function reduced_total_energy(u, p::PendulumParameters)
    reconstructed = reconstruct_cartesian(u, p)
    cartesian_state = [reconstructed.q; reconstructed.v]
    return total_energy(cartesian_state, p)
end

"""Run the two-state reduced-coordinate pendulum simulation."""
function run_reduced_example(; theta0 = deg2rad(45.0),
                               omega0 = 0.0,
                               tspan = (0.0, 5.0),
                               abstol = 1.0e-10,
                               reltol = 1.0e-10,
                               saveat = nothing)
    p = PendulumParameters()
    u0 = [theta0, omega0]
    problem = ODEProblem(reduced_pendulum_rhs!, u0, tspan, p)

    solution = if isnothing(saveat)
        solve(problem, Tsit5(); abstol, reltol)
    else
        solve(problem, Tsit5(); abstol, reltol, saveat)
    end

    energies = [reduced_total_energy(u, p) for u in solution.u]
    energy_errors = energies .- first(energies)
    diagnostics = (
        maximum_energy_error = maximum(abs, energy_errors),
    )

    return solution, diagnostics
end

"""
Integrate the reduced and minimal Cartesian formulations from the same state.

The returned diagnostics compare reconstructed Cartesian states, accelerations,
pin reactions, and energy at common output times.
"""
function compare_formulations(; theta0 = deg2rad(45.0),
                                omega0 = 0.0,
                                tspan = (0.0, 5.0),
                                samples = 201,
                                abstol = 1.0e-10,
                                reltol = 1.0e-10)
    p = PendulumParameters()
    times = range(tspan[1], tspan[2]; length = samples)

    reduced_initial = [theta0, omega0]
    initial_cartesian = reconstruct_cartesian(reduced_initial, p)
    cartesian_initial = [initial_cartesian.q; initial_cartesian.v]

    reduced_problem = ODEProblem(
        reduced_pendulum_rhs!, reduced_initial, tspan, p
    )
    cartesian_problem = ODEProblem(
        pendulum_rhs!, cartesian_initial, tspan, p
    )

    reduced_solution = solve(
        reduced_problem, Tsit5(); abstol, reltol, saveat = times
    )
    cartesian_solution = solve(
        cartesian_problem, Tsit5(); abstol, reltol, saveat = times
    )

    state_errors = Float64[]
    acceleration_errors = Float64[]
    reaction_errors = Float64[]
    reduced_energies = Float64[]
    cartesian_energies = Float64[]

    for (u_reduced, u_cartesian) in zip(
        reduced_solution.u, cartesian_solution.u
    )
        reconstructed = reconstruct_cartesian(u_reduced, p)
        reconstructed_state = [reconstructed.q; reconstructed.v]
        cartesian_acceleration, cartesian_alpha, cartesian_reaction =
            acceleration_solution(u_cartesian, p)
        reduced_reaction = reduced_pin_reaction(u_reduced, p)

        push!(state_errors, norm(reconstructed_state - u_cartesian, Inf))
        push!(
            acceleration_errors,
            norm(
                [reconstructed.a_global; reconstructed.alpha_p] -
                [cartesian_acceleration; cartesian_alpha],
                Inf,
            ),
        )
        push!(reaction_errors, norm(reduced_reaction - cartesian_reaction, Inf))
        push!(reduced_energies, reduced_total_energy(u_reduced, p))
        push!(cartesian_energies, total_energy(u_cartesian, p))
    end

    diagnostics = (
        maximum_state_difference = maximum(state_errors),
        maximum_acceleration_difference = maximum(acceleration_errors),
        maximum_reaction_difference = maximum(reaction_errors),
        maximum_reduced_energy_error =
            maximum(abs, reduced_energies .- first(reduced_energies)),
        maximum_cartesian_energy_error =
            maximum(abs, cartesian_energies .- first(cartesian_energies)),
        maximum_energy_difference =
            maximum(abs, reduced_energies .- cartesian_energies),
    )

    return reduced_solution, cartesian_solution, diagnostics
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution, diagnostics = run_reduced_example()
    println("Reduced-coordinate pendulum")
    println("  integrated states: 2")
    println("  accepted time points: ", length(solution.t))
    for (name, value) in pairs(diagnostics)
        println("  ", name, ": ", value)
    end

    _, _, comparison = compare_formulations()
    println("Comparison with minimal Cartesian formulation")
    for (name, value) in pairs(comparison)
        println("  ", name, ": ", value)
    end
end
