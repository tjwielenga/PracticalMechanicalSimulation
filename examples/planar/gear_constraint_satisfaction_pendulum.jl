using LinearAlgebra
using OrdinaryDiffEq

if !isdefined(@__MODULE__, :HistoricalDDASSL)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "HistoricalDDASSL.jl"))
end
if !isdefined(@__MODULE__, :reconstruct_cartesian)
    include(joinpath(@__DIR__, "reduced_coordinate_pendulum.jl"))
end

const GEAR_VARIABLE_LEVELS =
    [2, 2, 2, 1, 1, 1, 0, 0, 0, 2, 2, 1, 1]
const GEAR_EQUATION_LEVELS =
    [2, 2, 2, 1, 1, 0, 0, 2, 2, 2, 1, 1, 1]
const GEAR_DIFFERENTIAL_VARS =
    BitVector([i in (4, 5, 6, 7, 8, 9) for i in 1:13])

const COMPLETE_GEAR_VARIABLE_LEVELS =
    [2, 2, 2, 1, 1, 1, 0, 0, 0, 2, 2, 1, 1, 2, 2]
const COMPLETE_GEAR_EQUATION_LEVELS =
    [2, 2, 2, 2, 2, 1, 1, 0, 0, 2, 2, 2, 1, 1, 1]
const COMPLETE_GEAR_DIFFERENTIAL_VARS =
    BitVector([i in (4, 5, 6, 7, 8, 9) for i in 1:15])

"""
GearStableV position-and-velocity constraint-satisfaction equations.

z = [a_x, a_y, alpha, V_x, V_y, omega,
     R_x, R_y, theta, lambda_x, lambda_y, mu_x, mu_y]
"""
function gear_constraint_satisfaction_pendulum!(equations, t, z, zdot,
                                                p::PendulumParameters)
    acceleration = view(z, 1:2)
    alpha = z[3]
    velocity = view(z, 4:5)
    omega = z[6]
    position = view(z, 7:8)
    theta = z[9]
    reaction = view(z, 10:11)
    satisfaction_multiplier = view(z, 12:13)
    r_global, d_global = marker_vectors(theta, p)

    # Force and torque equations.
    equations[1:2] .= p.mass .* acceleration .- reaction .-
        p.mass .* p.gravity
    equations[3] = p.inertia * alpha - dot(d_global, reaction)

    # Velocity and position constraints.
    equations[4:5] .= velocity .+ d_global .* omega
    equations[6:7] .= position .+ r_global .- p.pin

    # Explicit acceleration definitions.
    equations[8:9] .= acceleration .- zdot[4:5]
    equations[10] = alpha - zdot[6]

    # Modified kinematic differential equations:
    # qdot - nu + D' * mu = 0, with D = [I d].
    equations[11:12] .= zdot[7:8] .- velocity .+
        satisfaction_multiplier
    equations[13] = zdot[9] - omega +
        dot(d_global, satisfaction_multiplier)
    return nothing
end

"""Analytical matrix F_z + coefficient*F_zdot."""
function gear_constraint_satisfaction_jacobian!(jacobian, t, z, zdot,
                                                coefficient,
                                                p::PendulumParameters)
    fill!(jacobian, zero(eltype(jacobian)))
    omega = z[6]
    theta = z[9]
    reaction = view(z, 10:11)
    satisfaction_multiplier = view(z, 12:13)
    r_global, d_global = marker_vectors(theta, p)

    jacobian[1, 1] = p.mass
    jacobian[2, 2] = p.mass
    jacobian[1, 10] = -1
    jacobian[2, 11] = -1
    jacobian[3, 3] = p.inertia
    jacobian[3, 9] = dot(r_global, reaction)
    jacobian[3, 10:11] .= -d_global

    jacobian[4, 4] = 1
    jacobian[5, 5] = 1
    jacobian[4:5, 6] .= d_global
    jacobian[4:5, 9] .= -r_global .* omega

    jacobian[6, 7] = 1
    jacobian[7, 8] = 1
    jacobian[6:7, 9] .= d_global

    jacobian[8, 1] = 1
    jacobian[9, 2] = 1
    jacobian[8, 4] = -coefficient
    jacobian[9, 5] = -coefficient
    jacobian[10, 3] = 1
    jacobian[10, 6] = -coefficient

    jacobian[11, 4] = -1
    jacobian[12, 5] = -1
    jacobian[11, 7] = coefficient
    jacobian[12, 8] = coefficient
    jacobian[11, 12] = 1
    jacobian[12, 13] = 1
    jacobian[13, 6] = -1
    jacobian[13, 9] = coefficient -
        dot(r_global, satisfaction_multiplier)
    jacobian[13, 12:13] .= d_global
    return nothing
end

"""
GearStableA constraint satisfaction at position, velocity, and acceleration levels.

z = [a_x, a_y, alpha, V_x, V_y, omega,
     R_x, R_y, theta, lambda_x, lambda_y,
     mu_x, mu_y, eta_x, eta_y]
"""
function complete_gear_constraint_satisfaction_pendulum!(equations, t, z,
                                                         zdot,
                                                         p::PendulumParameters)
    acceleration = view(z, 1:2)
    alpha = z[3]
    velocity = view(z, 4:5)
    omega = z[6]
    position = view(z, 7:8)
    theta = z[9]
    reaction = view(z, 10:11)
    velocity_multiplier = view(z, 12:13)
    acceleration_multiplier = view(z, 14:15)
    r_global, d_global = marker_vectors(theta, p)

    # Force and torque equations.
    equations[1:2] .= p.mass .* acceleration .- reaction .-
        p.mass .* p.gravity
    equations[3] = p.inertia * alpha - dot(d_global, reaction)

    # Acceleration, velocity, and position constraints.
    equations[4:5] .= acceleration .+ d_global .* alpha .-
        r_global .* omega^2
    equations[6:7] .= velocity .+ d_global .* omega
    equations[8:9] .= position .+ r_global .- p.pin

    # Minimum discrepancy between explicit acceleration and velocity derivative.
    equations[10:11] .= acceleration .- zdot[4:5] .+
        acceleration_multiplier
    equations[12] = alpha - zdot[6] +
        dot(d_global, acceleration_multiplier)

    # Minimum discrepancy between velocity and position derivative.
    equations[13:14] .= zdot[7:8] .- velocity .+ velocity_multiplier
    equations[15] = zdot[9] - omega +
        dot(d_global, velocity_multiplier)
    return nothing
end

"""Analytical matrix F_z + coefficient*F_zdot for the complete Gear case."""
function complete_gear_constraint_satisfaction_jacobian!(jacobian, t, z,
                                                          zdot, coefficient,
                                                          p::PendulumParameters)
    fill!(jacobian, zero(eltype(jacobian)))
    alpha = z[3]
    omega = z[6]
    theta = z[9]
    reaction = view(z, 10:11)
    velocity_multiplier = view(z, 12:13)
    acceleration_multiplier = view(z, 14:15)
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

    jacobian[10, 1] = 1
    jacobian[11, 2] = 1
    jacobian[10, 4] = -coefficient
    jacobian[11, 5] = -coefficient
    jacobian[10, 14] = 1
    jacobian[11, 15] = 1
    jacobian[12, 3] = 1
    jacobian[12, 6] = -coefficient
    jacobian[12, 9] = -dot(r_global, acceleration_multiplier)
    jacobian[12, 14:15] .= d_global

    jacobian[13, 4] = -1
    jacobian[14, 5] = -1
    jacobian[13, 7] = coefficient
    jacobian[14, 8] = coefficient
    jacobian[13, 12] = 1
    jacobian[14, 13] = 1
    jacobian[15, 6] = -1
    jacobian[15, 9] = coefficient -
        dot(r_global, velocity_multiplier)
    jacobian[15, 12:13] .= d_global
    return nothing
end

function gear_constraint_satisfaction_initial_conditions(theta, omega,
                                                         p)
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
        0.0
        0.0
    ]
    zdot0 = zeros(eltype(z0), length(z0))
    zdot0[4:5] .= reconstructed.a_global
    zdot0[6] = reconstructed.alpha_p
    zdot0[7:8] .= reconstructed.v[1:2]
    zdot0[9] = omega
    return z0, zdot0
end

function run_gear_constraint_satisfaction_pendulum(;
        theta0 = deg2rad(45.0), omega0 = 0.0,
        tspan = (0.0, 5.0), atol = 1.0e-7,
        rtol = 1.0e-5, dt = 1.0e-5, dtmax = 0.05)
    p = PendulumParameters()
    z0, zdot0 = gear_constraint_satisfaction_initial_conditions(
        theta0, omega0, p)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol, rtol = rtol, initial_step = dt, maximum_step = dtmax)
    return HistoricalDDASSL.dassl(
        gear_constraint_satisfaction_pendulum!, z0, zdot0, tspan;
        parameter = p,
        jacobian! = gear_constraint_satisfaction_jacobian!,
        options,
        variable_levels = GEAR_VARIABLE_LEVELS,
        equation_levels = GEAR_EQUATION_LEVELS,
        differential_vars = GEAR_DIFFERENTIAL_VARS,
        error_control = GEAR_DIFFERENTIAL_VARS,
        deficit = 1,
    )
end

function complete_gear_constraint_satisfaction_initial_conditions(theta,
                                                                  omega, p)
    z0, zdot0 = gear_constraint_satisfaction_initial_conditions(
        theta, omega, p)
    return [z0; 0.0; 0.0], [zdot0; 0.0; 0.0]
end

function run_complete_gear_constraint_satisfaction_pendulum(;
        theta0 = deg2rad(45.0), omega0 = 0.0,
        tspan = (0.0, 5.0), atol = 1.0e-7,
        rtol = 1.0e-5, dt = 1.0e-5, dtmax = 0.05)
    p = PendulumParameters()
    z0, zdot0 = complete_gear_constraint_satisfaction_initial_conditions(
        theta0, omega0, p)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol, rtol = rtol, initial_step = dt, maximum_step = dtmax)
    return HistoricalDDASSL.dassl(
        complete_gear_constraint_satisfaction_pendulum!, z0, zdot0, tspan;
        parameter = p,
        jacobian! = complete_gear_constraint_satisfaction_jacobian!,
        options,
        variable_levels = COMPLETE_GEAR_VARIABLE_LEVELS,
        equation_levels = COMPLETE_GEAR_EQUATION_LEVELS,
        differential_vars = COMPLETE_GEAR_DIFFERENTIAL_VARS,
        error_control = COMPLETE_GEAR_DIFFERENTIAL_VARS,
        deficit = 0,
    )
end

function gear_constraint_satisfaction_diagnostics(solution, p)
    reduced_problem = ODEProblem(reduced_pendulum_rhs!,
        [solution.u[1][9], solution.u[1][6]],
        (first(solution.t), last(solution.t)), p)
    reduced_solution = solve(reduced_problem, Tsit5();
        abstol = 1.0e-11, reltol = 1.0e-11)

    equation_errors = Float64[]
    position_errors = Float64[]
    velocity_errors = Float64[]
    acceleration_constraint_errors = Float64[]
    kinematic_discrepancies = Float64[]
    multiplier_magnitudes = Float64[]
    state_errors = Float64[]
    reaction_errors = Float64[]
    energy_errors = Float64[]
    initial_energy = total_energy(
        [solution.u[1][7:9]; solution.u[1][4:6]], p)

    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(eltype(z), 13)
        gear_constraint_satisfaction_pendulum!(
            equations, t, z, zdot, p)
        push!(equation_errors, norm(equations, Inf))
        push!(position_errors, norm(equations[6:7], Inf))
        push!(velocity_errors, norm(equations[4:5], Inf))

        r_global, d_global = marker_vectors(z[9], p)
        acceleration_constraint = z[1:2] + d_global .* z[3] -
            r_global .* z[6]^2
        push!(acceleration_constraint_errors,
            norm(acceleration_constraint, Inf))
        discrepancy = zdot[7:9] - z[4:6]
        push!(kinematic_discrepancies, norm(discrepancy, Inf))
        push!(multiplier_magnitudes, norm(z[12:13], Inf))

        reduced_state = reduced_solution(t)
        reconstructed = reconstruct_cartesian(reduced_state, p)
        reference_reaction = reduced_pin_reaction(reduced_state, p)
        push!(state_errors, norm(
            [z[7:9]; z[4:6]] - [reconstructed.q; reconstructed.v], Inf))
        push!(reaction_errors, norm(z[10:11] - reference_reaction, Inf))
        energy = total_energy([z[7:9]; z[4:6]], p)
        push!(energy_errors, abs(energy - initial_energy))
    end

    return (
        return_code = solution.retcode,
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_position_constraint_error = maximum(position_errors),
        maximum_velocity_constraint_error = maximum(velocity_errors),
        maximum_acceleration_constraint_error =
            maximum(acceleration_constraint_errors),
        maximum_kinematic_discrepancy = maximum(kinematic_discrepancies),
        maximum_satisfaction_multiplier = maximum(multiplier_magnitudes),
        maximum_state_difference = maximum(state_errors),
        maximum_reaction_difference = maximum(reaction_errors),
        maximum_energy_error = maximum(energy_errors),
    )
end

function complete_gear_constraint_satisfaction_diagnostics(solution, p)
    reduced_problem = ODEProblem(reduced_pendulum_rhs!,
        [solution.u[1][9], solution.u[1][6]],
        (first(solution.t), last(solution.t)), p)
    reduced_solution = solve(reduced_problem, Tsit5();
        abstol = 1.0e-11, reltol = 1.0e-11)

    equation_errors = Float64[]
    position_errors = Float64[]
    velocity_errors = Float64[]
    acceleration_errors = Float64[]
    velocity_discrepancies = Float64[]
    acceleration_discrepancies = Float64[]
    velocity_multiplier_magnitudes = Float64[]
    acceleration_multiplier_magnitudes = Float64[]
    state_errors = Float64[]
    acceleration_state_errors = Float64[]
    reaction_errors = Float64[]
    energy_errors = Float64[]
    initial_energy = total_energy(
        [solution.u[1][7:9]; solution.u[1][4:6]], p)

    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(eltype(z), 15)
        complete_gear_constraint_satisfaction_pendulum!(
            equations, t, z, zdot, p)
        push!(equation_errors, norm(equations, Inf))
        push!(acceleration_errors, norm(equations[4:5], Inf))
        push!(velocity_errors, norm(equations[6:7], Inf))
        push!(position_errors, norm(equations[8:9], Inf))
        push!(acceleration_discrepancies,
            norm(z[1:3] - zdot[4:6], Inf))
        push!(velocity_discrepancies,
            norm(zdot[7:9] - z[4:6], Inf))
        push!(velocity_multiplier_magnitudes, norm(z[12:13], Inf))
        push!(acceleration_multiplier_magnitudes, norm(z[14:15], Inf))

        reduced_state = reduced_solution(t)
        reconstructed = reconstruct_cartesian(reduced_state, p)
        reference_reaction = reduced_pin_reaction(reduced_state, p)
        push!(state_errors, norm(
            [z[7:9]; z[4:6]] - [reconstructed.q; reconstructed.v], Inf))
        push!(acceleration_state_errors,
            norm(z[1:3] -
                [reconstructed.a_global; reconstructed.alpha_p], Inf))
        push!(reaction_errors, norm(z[10:11] - reference_reaction, Inf))
        energy = total_energy([z[7:9]; z[4:6]], p)
        push!(energy_errors, abs(energy - initial_energy))
    end

    return (
        return_code = solution.retcode,
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_position_constraint_error = maximum(position_errors),
        maximum_velocity_constraint_error = maximum(velocity_errors),
        maximum_acceleration_constraint_error = maximum(acceleration_errors),
        maximum_velocity_discrepancy = maximum(velocity_discrepancies),
        maximum_acceleration_discrepancy =
            maximum(acceleration_discrepancies),
        maximum_velocity_multiplier =
            maximum(velocity_multiplier_magnitudes),
        maximum_acceleration_multiplier =
            maximum(acceleration_multiplier_magnitudes),
        maximum_state_difference = maximum(state_errors),
        maximum_acceleration_difference = maximum(acceleration_state_errors),
        maximum_reaction_difference = maximum(reaction_errors),
        maximum_energy_error = maximum(energy_errors),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution = run_gear_constraint_satisfaction_pendulum()
    diagnostics = gear_constraint_satisfaction_diagnostics(
        solution, PendulumParameters())
    println("GearStableV constraint-satisfaction pendulum")
    println("  simultaneous variables: 13")
    println("  differential and controlled variables: 6")
    for (name, value) in pairs(diagnostics)
        println("  ", name, ": ", value)
    end
    println("  accepted steps: ", solution.stats.accepted_steps)
    println("  rejected steps: ", solution.stats.rejected_steps)
    println("  Newton iterations: ", solution.stats.newton_iterations)
    println("  maximum BDF order: ", maximum(solution.orders))

    complete_solution =
        run_complete_gear_constraint_satisfaction_pendulum()
    complete_diagnostics =
        complete_gear_constraint_satisfaction_diagnostics(
            complete_solution, PendulumParameters())
    println("GearStableA constraint-satisfaction pendulum")
    println("  simultaneous variables: 15")
    println("  differential and controlled variables: 6")
    for (name, value) in pairs(complete_diagnostics)
        println("  ", name, ": ", value)
    end
    println("  accepted steps: ", complete_solution.stats.accepted_steps)
    println("  rejected steps: ", complete_solution.stats.rejected_steps)
    println("  Newton iterations: ",
        complete_solution.stats.newton_iterations)
    println("  maximum BDF order: ", maximum(complete_solution.orders))
end
