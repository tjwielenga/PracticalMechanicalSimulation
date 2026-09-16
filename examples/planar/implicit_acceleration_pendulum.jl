using LinearAlgebra
using OrdinaryDiffEq
import DASSL
import Sundials

if !isdefined(@__MODULE__, :HistoricalDDASSL)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "HistoricalDDASSL.jl"))
end

if !isdefined(@__MODULE__, :implicit_initial_conditions)
    include(joinpath(@__DIR__, "implicit_displacement_pendulum.jl"))
end

"""Eight implicit equations for the deficit-zero acceleration-constraint model."""
function implicit_acceleration_pendulum!(equations, dy, y,
                                         p::PendulumParameters, t)
    theta = y[3]
    r_global, d_global = marker_vectors(theta, p)

    equations[1:2] .= p.mass .* dy[4:5] .- y[7:8] .-
                      p.mass .* p.gravity
    equations[3] = p.inertia * dy[6] - dot(d_global, y[7:8])

    equations[4:5] .= dy[1:2] .- y[4:5]
    equations[6] = dy[3] - y[6]

    equations[7:8] .= dy[4:5] .+ d_global .* dy[6] .-
                       r_global .* y[6]^2
    return nothing
end

"""Analytical matrix F_y + coefficient*F_dy for the deficit-zero model."""
function implicit_acceleration_jacobian(t, y, dy, coefficient,
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

    jacobian[7:8, 3] .= -r_global .* dy[6] .-
                         d_global .* y[6]^2
    jacobian[7, 4] = coefficient
    jacobian[8, 5] = coefficient
    jacobian[7:8, 6] .= coefficient .* d_global .-
                         2 .* r_global .* y[6]
    return jacobian
end

function implicit_acceleration_jacobian!(jacobian, dy, y,
                                         p::PendulumParameters, coefficient, t)
    jacobian .= implicit_acceleration_jacobian(t, y, dy, coefficient, p)
    return nothing
end

function historical_acceleration_residual!(equations, t, y, dy,
                                            p::PendulumParameters)
    implicit_acceleration_pendulum!(equations, dy, y, p, t)
    return nothing
end

function historical_acceleration_jacobian!(jacobian, t, y, dy, coefficient,
                                            p::PendulumParameters)
    jacobian .= implicit_acceleration_jacobian(t, y, dy, coefficient, p)
    return nothing
end

function implicit_acceleration_problem(; theta0 = deg2rad(45.0),
                                         omega0 = 0.0,
                                         tspan = (0.0, 5.0),
                                         parameters = PendulumParameters())
    y0, dy0 = implicit_initial_conditions(theta0, omega0, parameters)
    differential_variables = [trues(6); falses(2)]
    implicit_function = DAEFunction(
        implicit_acceleration_pendulum!;
        jac = implicit_acceleration_jacobian!,
    )
    return DAEProblem(
        implicit_function, dy0, y0, tspan, parameters;
        differential_vars = differential_variables,
    )
end

function acceleration_control_tolerances(;
        controlled = 1.0e-5, suppressed = 1.0e-3)
    return [fill(controlled, 6); fill(suppressed, 2)]
end

function run_acceleration_dassl(; theta0 = deg2rad(45.0),
                                  omega0 = 0.0,
                                  tspan = (0.0, 5.0),
                                  controlled_abstol = 1.0e-5,
                                  suppressed_abstol = 1.0e-3,
                                  reltol = 1.0e-5,
                                  dt = 1.0e-5,
                                  dtmax = 0.05)
    p = PendulumParameters()
    y0, dy0 = implicit_initial_conditions(theta0, omega0, p)
    abstol = acceleration_control_tolerances(
        controlled = controlled_abstol,
        suppressed = suppressed_abstol,
    )
    reltols = fill(reltol, 8)
    equations = zeros(8)
    residual = (t, y, dy) -> begin
        implicit_acceleration_pendulum!(equations, dy, y, p, t)
        return copy(equations)
    end
    jacobian = (t, y, dy, coefficient) ->
        implicit_acceleration_jacobian(t, y, dy, coefficient, p)

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

function run_acceleration_ida(; theta0 = deg2rad(45.0),
                                omega0 = 0.0,
                                tspan = (0.0, 5.0),
                                controlled_abstol = 1.0e-5,
                                suppressed_abstol = 1.0e-3,
                                reltol = 1.0e-5)
    problem = implicit_acceleration_problem(; theta0, omega0, tspan)
    abstol = acceleration_control_tolerances(
        controlled = controlled_abstol,
        suppressed = suppressed_abstol,
    )
    return solve(
        problem, Sundials.IDA();
        abstol,
        reltol,
        initializealg = CheckInit(),
    )
end

"""Run the deficit-zero pendulum with the Julia DDASSL reconstruction."""
function run_acceleration_historical_ddassl(;
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

    # Positions have level zero, velocities level one, and pin reactions
    # level two. For a deficit-zero system, levels zero and one participate
    # in integration-error control; reactions are still stored and predicted.
    variable_levels = [0, 0, 0, 1, 1, 1, 2, 2]
    equation_levels = [2, 2, 2, 1, 1, 1, 2, 2]
    differential_vars = [trues(6); falses(2)]

    return HistoricalDDASSL.dassl(
        historical_acceleration_residual!, y0, dy0, tspan;
        parameter = p,
        jacobian! = historical_acceleration_jacobian!,
        options,
        variable_levels,
        equation_levels,
        differential_vars,
        deficit = 0,
    )
end

function acceleration_solution_diagnostics(solution, p::PendulumParameters)
    position_errors = Float64[]
    velocity_errors = Float64[]
    acceleration_errors = Float64[]
    force_errors = Float64[]
    torque_errors = Float64[]
    energies = Float64[]

    for (y, dy, t) in zip(solution.u, solution.du, solution.t)
        equations = zeros(8)
        implicit_acceleration_pendulum!(equations, dy, y, p, t)
        push!(position_errors, norm(position_constraint(y[1:3], p), Inf))
        push!(velocity_errors, norm(velocity_constraint(y[1:6], p), Inf))
        push!(acceleration_errors, norm(equations[7:8], Inf))
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

function historical_acceleration_solver_diagnostics(solution)
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
    p = PendulumParameters()
    for (name, runner) in (
        ("Julia DDASSL reconstruction", run_acceleration_historical_ddassl),
        ("Native SciML DASSL", run_acceleration_dassl),
        ("Sundials IDA", run_acceleration_ida),
    )
        println(name)
        try
            solution = runner()
            diagnostics = acceleration_solution_diagnostics(solution, p)
            comparison = compare_implicit_with_reduced(solution, p)
            for (key, value) in pairs(diagnostics)
                println("  ", key, ": ", value)
            end
            for (key, value) in pairs(comparison)
                println("  ", key, ": ", value)
            end
            if solution isa HistoricalDDASSL.DASSLResult
                for (key, value) in pairs(
                        historical_acceleration_solver_diagnostics(solution))
                    println("  ", key, ": ", value)
                end
            end
        catch error
            println("  failed: ", sprint(showerror, error))
        end
    end
end
