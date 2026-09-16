using LinearAlgebra
using OrdinaryDiffEq
import DASSL
import Sundials

if !isdefined(@__MODULE__, :HistoricalDDASSL)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "HistoricalDDASSL.jl"))
end

if !isdefined(@__MODULE__, :PendulumParameters)
    include(joinpath(@__DIR__, "minimal_cartesian_pendulum.jl"))
end
if !isdefined(@__MODULE__, :reconstruct_cartesian)
    include(joinpath(@__DIR__, "reduced_coordinate_pendulum.jl"))
end

const POSITION_INDICES = 1:3
const VELOCITY_INDICES = 4:6
const REACTION_INDICES = 7:8

"""
Eight implicit equations for the displacement-constraint pendulum.

y  = [R_x, R_y, theta, V_x, V_y, omega, lambda_x, lambda_y]
dy = [Rdot_x, Rdot_y, thetadot, a_x, a_y, alpha,
      lambdadot_x, lambdadot_y]
"""
function implicit_pendulum!(equations, dy, y, p::PendulumParameters, t)
    theta = y[3]
    r_global, d_global = marker_vectors(theta, p)

    # Sum of forces and sum of torques.
    equations[1:2] .= p.mass .* dy[4:5] .- y[7:8] .-
                      p.mass .* p.gravity
    equations[3] = p.inertia * dy[6] - dot(d_global, y[7:8])

    # Kinematic differential equations.
    equations[4:5] .= dy[1:2] .- y[4:5]
    equations[6] = dy[3] - y[6]

    # Displacement-level pin constraint.
    equations[7:8] .= y[1:2] .+ r_global .- p.pin
    return nothing
end

"""Analytical matrix F_y + coefficient*F_dy for the implicit equations."""
function implicit_pendulum_jacobian(t, y, dy, coefficient,
                                    p::PendulumParameters)
    r_global, d_global = marker_vectors(y[3], p)
    jacobian = zeros(eltype(y), 8, 8)

    jacobian[1, 4] = coefficient * p.mass
    jacobian[2, 5] = coefficient * p.mass
    jacobian[1, 7] = -1
    jacobian[2, 8] = -1

    jacobian[3, 3] = dot(r_global, y[7:8])
    jacobian[3, 6] = coefficient * p.inertia
    jacobian[3, 7:8] .= -d_global

    jacobian[4, 1] = coefficient
    jacobian[5, 2] = coefficient
    jacobian[4, 4] = -1
    jacobian[5, 5] = -1
    jacobian[6, 3] = coefficient
    jacobian[6, 6] = -1

    jacobian[7, 1] = 1
    jacobian[8, 2] = 1
    jacobian[7:8, 3] .= d_global
    return jacobian
end

function implicit_pendulum_jacobian!(jacobian, dy, y,
                                     p::PendulumParameters, coefficient, t)
    jacobian .= implicit_pendulum_jacobian(t, y, dy, coefficient, p)
    return nothing
end

function historical_displacement_residual!(equations, t, y, dy,
                                            p::PendulumParameters)
    implicit_pendulum!(equations, dy, y, p, t)
    return nothing
end

function historical_displacement_jacobian!(jacobian, t, y, dy, coefficient,
                                            p::PendulumParameters)
    jacobian .= implicit_pendulum_jacobian(t, y, dy, coefficient, p)
    return nothing
end

"""Construct exactly consistent y0 and dy0 from theta0 and omega0."""
function implicit_initial_conditions(theta0, omega0, p::PendulumParameters)
    reduced_state = [theta0, omega0]
    reconstructed = reconstruct_cartesian(reduced_state, p)
    lambda_global = reduced_pin_reaction(reduced_state, p)

    y0 = [reconstructed.q; reconstructed.v; lambda_global]
    dy0 = [
        reconstructed.v
        reconstructed.a_global
        reconstructed.alpha_p
        0.0
        0.0
    ]
    return y0, dy0
end

function implicit_problem(; theta0 = deg2rad(45.0),
                            omega0 = 0.0,
                            tspan = (0.0, 5.0),
                            parameters = PendulumParameters())
    y0, dy0 = implicit_initial_conditions(theta0, omega0, parameters)
    differential_variables = [trues(6); falses(2)]
    implicit_function = DAEFunction(
        implicit_pendulum!; jac = implicit_pendulum_jacobian!
    )
    return DAEProblem(
        implicit_function, dy0, y0, tspan, parameters;
        differential_vars = differential_variables,
    )
end

"""
Componentwise absolute tolerances for the intended DASSL error test.

Position and orientation retain the requested controlled tolerance. Velocity
and reaction components receive a deliberately large tolerance so they make a
negligible contribution to the weighted local-error norm.
"""
function displacement_control_tolerances(;
        controlled = 1.0e-9, suppressed = 1.0e6)
    return [fill(controlled, 3); fill(suppressed, 5)]
end

function run_implicit_dassl(; theta0 = deg2rad(45.0),
                              omega0 = 0.0,
                              tspan = (0.0, 5.0),
                              controlled_abstol = 1.0e-9,
                              suppressed_abstol = 1.0e6,
                              reltol = 1.0e-8,
                              dt = 1.0e-5,
                              dtmax = 0.05)
    p = PendulumParameters()
    y0, dy0 = implicit_initial_conditions(theta0, omega0, p)
    abstol = displacement_control_tolerances(
        controlled = controlled_abstol,
        suppressed = suppressed_abstol,
    )
    reltols = fill(reltol, 8)

    equations = zeros(8)
    residual = (t, y, dy) -> begin
        implicit_pendulum!(equations, dy, y, p, t)
        return copy(equations)
    end
    jacobian = (t, y, dy, coefficient) ->
        implicit_pendulum_jacobian(t, y, dy, coefficient, p)

    # The direct DASSL API is used because it accepts the analytical Jacobian
    # and returns the derivative history. The high-level DAEProblem path in
    # DASSL v3.0.1 currently discards that history.
    times, values, derivatives = DASSL.dasslSolve(
        residual, y0, collect(tspan);
        abstol,
        reltol = reltols,
        initstep = dt,
        maxstep = dtmax,
        dy0,
        jacobian,
    )
    return (
        t = times,
        u = values,
        du = derivatives,
        retcode = ReturnCode.Success,
    )
end

function run_implicit_ida(; theta0 = deg2rad(45.0),
                            omega0 = 0.0,
                            tspan = (0.0, 5.0),
                            controlled_abstol = 1.0e-9,
                            suppressed_abstol = nothing,
                            reltol = 1.0e-8)
    problem = implicit_problem(; theta0, omega0, tspan)
    abstol = if isnothing(suppressed_abstol)
        controlled_abstol
    else
        displacement_control_tolerances(
            controlled = controlled_abstol,
            suppressed = suppressed_abstol,
        )
    end
    return solve(
        problem, Sundials.IDA();
        abstol,
        reltol,
        initializealg = CheckInit(),
    )
end

"""Run the deficit-two pendulum with the Julia DDASSL reconstruction."""
function run_displacement_historical_ddassl(;
        theta0 = deg2rad(45.0),
        omega0 = 0.0,
        tspan = (0.0, 5.0),
        atol = 1.0e-7,
        rtol = 1.0e-5,
        dt = 1.0e-5,
        dtmax = 0.05)
    p = PendulumParameters()
    y0, dy0 = implicit_initial_conditions(theta0, omega0, p)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol,
        rtol = rtol,
        initial_step = dt,
        maximum_step = dtmax,
    )

    variable_levels = [0, 0, 0, 1, 1, 1, 2, 2]
    equation_levels = [2, 2, 2, 1, 1, 1, 0, 0]
    differential_vars = [trues(6); falses(2)]

    return HistoricalDDASSL.dassl(
        historical_displacement_residual!, y0, dy0, tspan;
        parameter = p,
        jacobian! = historical_displacement_jacobian!,
        options,
        variable_levels,
        equation_levels,
        differential_vars,
        deficit = 2,
    )
end

function implicit_solution_diagnostics(solution, p::PendulumParameters)
    position_errors = Float64[]
    velocity_errors = Float64[]
    acceleration_errors = Float64[]
    force_errors = Float64[]
    torque_errors = Float64[]
    energies = Float64[]

    for (y, dy, t) in zip(solution.u, solution.du, solution.t)
        equations = zeros(8)
        implicit_pendulum!(equations, dy, y, p, t)
        push!(position_errors, norm(equations[7:8], Inf))

        q = y[1:3]
        cartesian_state = y[1:6]
        push!(velocity_errors, norm(velocity_constraint(cartesian_state, p), Inf))
        r_global, d_global = marker_vectors(y[3], p)
        push!(
            acceleration_errors,
            norm(dy[4:5] .+ d_global .* dy[6] .-
                 r_global .* y[6]^2, Inf),
        )
        push!(force_errors, norm(equations[1:2], Inf))
        push!(torque_errors, abs(equations[3]))
        push!(energies, total_energy(y[1:6], p))
    end

    return (
        return_code = solution.retcode,
        final_time = last(solution.t),
        accepted_time_points = length(solution.t),
        maximum_position_constraint_error = maximum(position_errors),
        maximum_velocity_constraint_error = maximum(velocity_errors),
        maximum_acceleration_constraint_error = maximum(acceleration_errors),
        maximum_force_equation_error = maximum(force_errors),
        maximum_torque_equation_error = maximum(torque_errors),
        maximum_energy_error = maximum(abs, energies .- first(energies)),
    )
end

function historical_displacement_solver_diagnostics(solution)
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

function compare_implicit_with_reduced(solution, p::PendulumParameters)
    reduced_problem = ODEProblem(
        reduced_pendulum_rhs!, [solution.u[1][3], solution.u[1][6]],
        (first(solution.t), last(solution.t)), p,
    )
    reduced_solution = solve(
        reduced_problem, Tsit5(); abstol = 1.0e-11, reltol = 1.0e-11
    )

    state_errors = Float64[]
    reaction_errors = Float64[]
    for (t, y) in zip(solution.t, solution.u)
        reduced_state = reduced_solution(t)
        reconstructed = reconstruct_cartesian(reduced_state, p)
        reference_state = [reconstructed.q; reconstructed.v]
        reference_reaction = reduced_pin_reaction(reduced_state, p)
        push!(state_errors, norm(y[1:6] - reference_state, Inf))
        push!(reaction_errors, norm(y[7:8] - reference_reaction, Inf))
    end

    return (
        maximum_state_difference = maximum(state_errors),
        maximum_reaction_difference = maximum(reaction_errors),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    p = PendulumParameters()
    for (name, runner) in (
        ("Julia DDASSL reconstruction", run_displacement_historical_ddassl),
        ("Native SciML DASSL", run_implicit_dassl),
        ("Sundials IDA", run_implicit_ida),
    )
        println(name)
        try
            solution = runner()
            diagnostics = implicit_solution_diagnostics(solution, p)
            comparison = compare_implicit_with_reduced(solution, p)
            for (key, value) in pairs(diagnostics)
                println("  ", key, ": ", value)
            end
            for (key, value) in pairs(comparison)
                println("  ", key, ": ", value)
            end
            if solution isa HistoricalDDASSL.DASSLResult
                for (key, value) in pairs(
                        historical_displacement_solver_diagnostics(solution))
                    println("  ", key, ": ", value)
                end
            end
        catch error
            println("  failed: ", sprint(showerror, error))
        end
    end
end
