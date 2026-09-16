using LinearAlgebra
using OrdinaryDiffEq

if !isdefined(@__MODULE__, :implicit_initial_conditions)
    include(joinpath(@__DIR__, "implicit_displacement_pendulum.jl"))
end

const BDF1_ACCELERATION = 1:3
const BDF1_REACTION = 4:5
const BDF1_VELOCITY = 6:8
const BDF1_POSITION = 9:11

"""Integrated state [Vx, Vy, omega, Rx, Ry, theta] contained in z."""
bdf1_state(z) = z[6:11]

"""State derivative [ax, ay, alpha, Vx, Vy, omega] contained in z."""
bdf1_state_derivative(z) = [z[1:3]; z[6:8]]

"""Eleven equations for the eleven current-step unknowns."""
function generalized_bdf1_equations!(equations, z, previous_state, step,
                                      p::PendulumParameters)
    acceleration = z[1:2]
    alpha = z[3]
    reaction = z[4:5]
    velocity = z[6:7]
    omega = z[8]
    position = z[9:10]
    theta = z[11]
    r_global, d_global = marker_vectors(theta, p)

    equations[1:2] .= p.mass .* acceleration .- reaction .-
                      p.mass .* p.gravity
    equations[3] = p.inertia * alpha - dot(d_global, reaction)
    equations[4:5] .= position .+ r_global .- p.pin

    equations[6:8] .= z[1:3] .- (z[6:8] .- previous_state[1:3]) ./ step
    equations[9:11] .= z[6:8] .- (z[9:11] .- previous_state[4:6]) ./ step
    return nothing
end

function generalized_bdf1_jacobian(z, step, p::PendulumParameters)
    r_global, d_global = marker_vectors(z[11], p)
    jacobian = zeros(eltype(z), 11, 11)
    jacobian[1, 1] = p.mass
    jacobian[2, 2] = p.mass
    jacobian[1, 4] = -1
    jacobian[2, 5] = -1
    jacobian[3, 3] = p.inertia
    jacobian[3, 4:5] .= -d_global
    jacobian[3, 11] = dot(r_global, z[4:5])
    jacobian[4, 9] = 1
    jacobian[5, 10] = 1
    jacobian[4:5, 11] .= d_global
    jacobian[6:8, 1:3] .= I(3)
    jacobian[6:8, 6:8] .= -I(3) ./ step
    jacobian[9:11, 6:8] .= I(3)
    jacobian[9:11, 9:11] .= -I(3) ./ step
    return jacobian
end

function bdf1_initial_unknown(theta, omega, p::PendulumParameters)
    y, dy = implicit_initial_conditions(theta, omega, p)
    return [dy[4:6]; y[7:8]; y[4:6]; y[1:3]]
end

function scaled_rms(values, scales)
    return sqrt(sum(abs2, values ./ scales) / length(values))
end

"""Solve one generalized BDF1 step with independent Newton controls."""
function generalized_bdf1_step(previous_z, step, p::PendulumParameters;
        newton_rtol = 1.0e-10, newton_atol = 1.0e-12,
        equation_tolerance = 1.0e-9, maximum_iterations = 10)
    previous_state = bdf1_state(previous_z)
    predicted_state = previous_state .+
        step .* bdf1_state_derivative(previous_z)
    z = copy(previous_z)
    z[6:11] .= predicted_state
    equations = zeros(eltype(z), 11)
    force_scale = max(p.mass * norm(p.gravity), one(eltype(z)))
    length_scale = max(norm(p.r_body), one(eltype(z)))
    acceleration_scale = max(norm(p.gravity), one(eltype(z)))
    velocity_scale = max(sqrt(acceleration_scale * length_scale), one(eltype(z)))
    equation_scales = [fill(force_scale, 2); force_scale * length_scale;
        fill(length_scale, 2); fill(acceleration_scale, 3);
        fill(velocity_scale, 3)]

    for iteration in 1:maximum_iterations
        generalized_bdf1_equations!(equations, z, previous_state, step, p)
        equation_norm = scaled_rms(equations,
            equation_tolerance .* equation_scales)
        equation_norm <= 1 && return z, iteration, equation_norm
        correction = -(generalized_bdf1_jacobian(z, step, p) \ equations)
        z .+= correction
        correction_scales = newton_atol .+ newton_rtol .* max.(abs.(z), 1.0)
        correction_norm = scaled_rms(correction, correction_scales)
        if correction_norm <= 1
            generalized_bdf1_equations!(equations, z, previous_state, step, p)
            equation_norm = scaled_rms(equations,
                equation_tolerance .* equation_scales)
            equation_norm <= 1 && return z, iteration, equation_norm
        end
    end
    error("Generalized BDF1 Newton iteration did not converge")
end

"""Adaptive BDF1 using step doubling for state-only integration error."""
function run_generalized_bdf1(; theta0 = deg2rad(45.0), omega0 = 0.0,
        tspan = (0.0, 1.0), initial_step = 1.0e-3,
        integration_rtol = 1.0e-3, integration_atol = 1.0e-5,
        minimum_step = 1.0e-8, maximum_step = 0.05)
    p = PendulumParameters()
    z = bdf1_initial_unknown(theta0, omega0, p)
    times = [first(tspan)]
    unknowns = [copy(z)]
    step = initial_step
    rejected_steps = 0
    newton_iterations = 0

    while last(times) < last(tspan)
        step = min(step, last(tspan) - last(times), maximum_step)
        full, iterations_full, _ = generalized_bdf1_step(z, step, p)
        half1, iterations_half1, _ = generalized_bdf1_step(z, step / 2, p)
        half2, iterations_half2, _ = generalized_bdf1_step(half1, step / 2, p)
        newton_iterations += iterations_full + iterations_half1 + iterations_half2
        full_state = bdf1_state(full)
        half_state = bdf1_state(half2)
        scales = integration_atol .+
            integration_rtol .* max.(abs.(full_state), abs.(half_state))
        integration_error = scaled_rms(half_state - full_state, scales)

        if integration_error <= 1
            z = half2
            push!(times, last(times) + step)
            push!(unknowns, copy(z))
            factor = integration_error == 0 ? 2.0 :
                clamp(0.9 * integration_error^(-0.5), 0.5, 2.0)
            step = min(maximum_step, step * factor)
        else
            rejected_steps += 1
            step *= max(0.2, 0.9 * integration_error^(-0.5))
            step >= minimum_step || error("Generalized BDF1 step became too small")
        end
    end
    return (t = times, z = unknowns, rejected_steps,
        newton_iterations, parameters = p)
end

function generalized_bdf1_diagnostics(solution)
    p = solution.parameters
    reduced_problem = ODEProblem(reduced_pendulum_rhs!,
        [solution.z[1][11], solution.z[1][8]],
        (first(solution.t), last(solution.t)), p)
    reference = solve(reduced_problem, Tsit5(); abstol = 1.0e-11,
        reltol = 1.0e-11)
    state_errors = Float64[]
    constraint_errors = Float64[]
    for (time, z) in zip(solution.t, solution.z)
        reduced = reconstruct_cartesian(reference(time), p)
        cartesian = [z[9:11]; z[6:8]]
        push!(state_errors, norm(cartesian - [reduced.q; reduced.v], Inf))
        push!(constraint_errors, norm(position_constraint(z[9:11], p), Inf))
    end
    return (accepted_steps = length(solution.t) - 1,
        rejected_steps = solution.rejected_steps,
        newton_iterations = solution.newton_iterations,
        maximum_state_difference = maximum(state_errors),
        maximum_position_constraint_error = maximum(constraint_errors))
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution = run_generalized_bdf1()
    println("Generalized BDF1 pendulum")
    for (key, value) in pairs(generalized_bdf1_diagnostics(solution))
        println("  ", key, ": ", value)
    end
end
