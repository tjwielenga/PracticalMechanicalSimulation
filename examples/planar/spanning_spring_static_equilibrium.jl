using LinearAlgebra

if !isdefined(@__MODULE__, :spanning_spring_damper_pendulum!)
    include(joinpath(@__DIR__, "spanning_spring_damper_pendulum.jl"))
end
if !isdefined(@__MODULE__, :spanning_pendulum_executable_model)
    include(joinpath(@__DIR__, "spanning_pendulum_executable_assembly.jl"))
end

const SPANNING_ANALYSIS_CATALOG = spanning_pendulum_analysis_catalog()

spanning_static_selection() =
    select_analysis(SPANNING_ANALYSIS_CATALOG, StaticEQ())

"""
Static unknowns:
y = [R_x, R_y, theta, lambda_x, lambda_y,
     s_x, s_y, ell, u_x, u_y, f, F_x, F_y]
"""
function expand_static_spanning_state(y)
    z = zeros(eltype(y), 20)
    expand_analysis_values!(z, y, spanning_static_selection())
    # Acceleration, velocity, angular velocity, and length rate remain zero.
    return z
end

function spanning_spring_static_equations!(equations, y, system)
    z = expand_static_spanning_state(y)
    model = spanning_pendulum_executable_model(system)
    evaluate_analysis_equations!(equations, model,
        spanning_static_selection(), 0.0, z, zeros(eltype(y), 20))
    return nothing
end

function spanning_spring_static_jacobian(y, system)
    z = expand_static_spanning_state(y)
    model = spanning_pendulum_executable_model(system)
    return evaluate_analysis_jacobian(model, spanning_static_selection(),
        0.0, z, zeros(eltype(y), 20), 0.0)
end

function spanning_spring_static_initial_guess(theta, system)
    p = system.mechanical
    reconstructed = reconstruct_cartesian([theta, zero(theta)], p)
    z = zeros(typeof(theta), 20)
    z[7:9] .= reconstructed.q
    set_spanning_spring_variables!(z, system)
    global_force = z[19:20]
    reaction = -p.mass .* p.gravity .- global_force
    return [z[7:9]; reaction; z[12:16]; z[18:20]]
end

function solve_spanning_spring_static_equilibrium(;
        theta_guess = deg2rad(35.0), stiffness = 20.0,
        damping = 0.5, free_length = 0.5,
        ground_anchor = [0.8, -0.2], tolerance = 1.0e-11,
        maximum_iterations = 30)
    system = SpanningSpringPendulumSystem(;
        stiffness, damping, free_length, ground_anchor)
    y = spanning_spring_static_initial_guess(theta_guess, system)
    equations = zeros(eltype(y), 13)
    for iteration in 1:maximum_iterations
        spanning_spring_static_equations!(equations, y, system)
        norm(equations, Inf) <= tolerance &&
            return y, system, iteration - 1
        jacobian = spanning_spring_static_jacobian(y, system)
        correction = -(jacobian \ equations)
        initial_norm = norm(equations, Inf)
        factor = one(eltype(y))
        accepted = false
        while factor >= 1 / 1024
            trial = y + factor .* correction
            trial_equations = similar(equations)
            try
                spanning_spring_static_equations!(
                    trial_equations, trial, system)
                if norm(trial_equations, Inf) < initial_norm
                    y = trial
                    accepted = true
                    break
                end
            catch error
                error isa DomainError || rethrow()
            end
            factor /= 2
        end
        accepted || error("Static-equilibrium Newton line search failed")
    end
    error("Static-equilibrium Newton iteration did not converge")
end

function spanning_spring_static_diagnostics(y, system, iterations)
    equations = zeros(eltype(y), 13)
    spanning_spring_static_equations!(equations, y, system)
    z = expand_static_spanning_state(y)
    p = system.mechanical
    _, d_pin = marker_vectors(z[9], p)
    force_marker = PlanarAppliedForces.point_marker_kinematics(
        system.spring_damper.marker_1, z)
    reduced_moment = p.mass * dot(d_pin, p.gravity) +
        dot(d_pin - force_marker.d, z[19:20])
    return (
        iterations,
        maximum_equation_error = norm(equations, Inf),
        equilibrium_angle = z[9],
        equilibrium_angle_degrees = rad2deg(z[9]),
        pin_reaction = copy(z[10:11]),
        spring_length = z[14],
        scalar_spring_force = z[18],
        global_spring_force = copy(z[19:20]),
        reduced_moment_error = abs(reduced_moment),
        jacobian_condition = cond(spanning_spring_static_jacobian(y, system)),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    y, system, iterations = solve_spanning_spring_static_equilibrium()
    println("Spanning spring-damper static equilibrium")
    println("  simultaneous variables and equations: 13")
    for (name, value) in pairs(
            spanning_spring_static_diagnostics(y, system, iterations))
        println("  ", name, ": ", value)
    end
end
