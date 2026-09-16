using LinearAlgebra
using OrdinaryDiffEq

if !isdefined(@__MODULE__, :HistoricalDDASSL)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "HistoricalDDASSL.jl"))
end
if !isdefined(@__MODULE__, :PlanarAppliedForces)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarAppliedForces.jl"))
end
using .PlanarAppliedForces
if !isdefined(@__MODULE__, :independent_state_pendulum!)
    include(joinpath(@__DIR__, "independent_state_pendulum.jl"))
end

struct SpanningSpringPendulumSystem{P,S}
    mechanical::P
    spring_damper::S
end

function SpanningSpringPendulumSystem(;
        stiffness = 20.0, damping = 0.5, free_length = 0.5,
        ground_anchor = [0.8, -0.2])
    mechanical = PendulumParameters()
    body_marker = PlanarBodyPointMarker(
        7:8, 9, 4:5, 6, 1:2, 3, [0.0, -0.5])
    ground_marker = PlanarGroundPointMarker(Float64.(ground_anchor))
    spring_damper = PlanarSpanningSpringDamper(
        body_marker, ground_marker,
        stiffness, damping, free_length,
        12:13, 14, 15:16, 17, 18, 19:20,
        12:13, 14, 15:16, 17, 18, 19:20)
    return SpanningSpringPendulumSystem(mechanical, spring_damper)
end

const SPANNING_SPRING_VARIABLE_LEVELS =
    [INDEPENDENT_STATE_VARIABLE_LEVELS;
     [0, 0, 0, 0, 0, 1, 2, 2, 2]]
const SPANNING_SPRING_EQUATION_LEVELS =
    [INDEPENDENT_STATE_EQUATION_LEVELS;
     [0, 0, 0, 0, 0, 1, 2, 2, 2]]
const SPANNING_SPRING_DIFFERENTIAL_VARS =
    BitVector([INDEPENDENT_STATE_DIFFERENTIAL_VARS; falses(9)])

"""
Full-equation pendulum with a nine-scalar-equation spanning spring-damper.

z = [a_x, a_y, alpha, V_x, V_y, omega,
     R_x, R_y, theta, lambda_x, lambda_y,
     s_x, s_y, ell, u_x, u_y, ell_dot, f, F_x, F_y]
"""
function spanning_spring_damper_pendulum!(equations, t, z, zdot, system)
    independent_state_pendulum!(
        equations, t, z, zdot, system.mechanical)
    equations[12:20] .= zero(eltype(equations))
    add_spanning_spring_damper!(equations, z, system.spring_damper)
    return nothing
end

function spanning_spring_damper_pendulum_jacobian!(jacobian, t, z, zdot,
                                                   coefficient, system)
    independent_state_pendulum_jacobian!(
        jacobian, t, z, zdot, coefficient, system.mechanical)
    add_spanning_spring_damper_jacobian!(
        jacobian, z, system.spring_damper)
    return nothing
end

function set_spanning_spring_variables!(z, system)
    element = system.spring_damper
    marker_1 = PlanarAppliedForces.point_marker_kinematics(
        element.marker_1, z)
    marker_2 = PlanarAppliedForces.point_marker_kinematics(
        element.marker_2, z)
    spanning = marker_2.position - marker_1.position
    length = norm(spanning)
    length > 0 || throw(DomainError(length,
        "initial spanning-force length must be positive"))
    unit = spanning ./ length
    length_rate = dot(unit, marker_2.velocity - marker_1.velocity)
    scalar_force = -element.stiffness * (length - element.free_length) -
        element.damping * length_rate
    z[element.spanning_variables] .= spanning
    z[element.length_variable] = length
    z[element.unit_variables] .= unit
    z[element.length_rate_variable] = length_rate
    z[element.force_variable] = scalar_force
    z[element.global_force_variables] .= -unit .* scalar_force
    return z
end

function spanning_spring_initial_conditions(theta, omega, system)
    p = system.mechanical
    reconstructed = reconstruct_cartesian([theta, omega], p)
    z0 = zeros(promote_type(typeof(theta), typeof(omega)), 20)
    z0[4:5] .= reconstructed.v[1:2]
    z0[6] = omega
    z0[7:9] .= reconstructed.q
    set_spanning_spring_variables!(z0, system)

    r_pin, d_pin = marker_vectors(theta, p)
    force_marker = PlanarAppliedForces.point_marker_kinematics(
        system.spring_damper.marker_1, z0)
    global_force = z0[19:20]
    effective_inertia = p.inertia + p.mass * dot(d_pin, d_pin)
    alpha = -(p.mass * dot(d_pin, p.gravity) +
        dot(d_pin - force_marker.d, global_force)) / effective_inertia
    acceleration = -d_pin .* alpha .+ r_pin .* omega^2
    reaction = p.mass .* (acceleration .- p.gravity) .- global_force
    z0[1:2] .= acceleration
    z0[3] = alpha
    z0[10:11] .= reaction

    zdot0 = zeros(eltype(z0), length(z0))
    zdot0[6] = alpha
    zdot0[9] = omega
    return z0, zdot0
end

function run_spanning_spring_damper_pendulum(;
        theta0 = deg2rad(45.0), omega0 = 0.0,
        stiffness = 20.0, damping = 0.5, free_length = 0.5,
        ground_anchor = [0.8, -0.2],
        tspan = (0.0, 5.0), atol = 1.0e-7, rtol = 1.0e-5,
        dt = 1.0e-5, dtmax = 0.05)
    system = SpanningSpringPendulumSystem(;
        stiffness, damping, free_length, ground_anchor)
    z0, zdot0 = spanning_spring_initial_conditions(theta0, omega0, system)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol, rtol = rtol, initial_step = dt, maximum_step = dtmax)
    solution = HistoricalDDASSL.dassl(
        spanning_spring_damper_pendulum!, z0, zdot0, tspan;
        parameter = system,
        jacobian! = spanning_spring_damper_pendulum_jacobian!,
        options,
        variable_levels = SPANNING_SPRING_VARIABLE_LEVELS,
        equation_levels = SPANNING_SPRING_EQUATION_LEVELS,
        differential_vars = SPANNING_SPRING_DIFFERENTIAL_VARS,
        error_control = SPANNING_SPRING_DIFFERENTIAL_VARS,
        deficit = 0)
    return solution, system
end

function reduced_spanning_spring_rhs!(du, u, system, t)
    theta, omega = u
    p = system.mechanical
    reconstructed = reconstruct_cartesian(u, p)
    z = zeros(eltype(u), 20)
    z[4:5] .= reconstructed.v[1:2]
    z[6] = omega
    z[7:9] .= reconstructed.q
    set_spanning_spring_variables!(z, system)
    _, d_pin = marker_vectors(theta, p)
    force_marker = PlanarAppliedForces.point_marker_kinematics(
        system.spring_damper.marker_1, z)
    global_force = z[19:20]
    effective_inertia = p.inertia + p.mass * dot(d_pin, d_pin)
    du[1] = omega
    du[2] = -(p.mass * dot(d_pin, p.gravity) +
        dot(d_pin - force_marker.d, global_force)) / effective_inertia
    return nothing
end

function spanning_spring_energy(z, system)
    mechanical_energy = total_energy([z[7:9]; z[4:6]], system.mechanical)
    extension = z[14] - system.spring_damper.free_length
    return mechanical_energy + 0.5 * system.spring_damper.stiffness * extension^2
end

function spanning_spring_diagnostics(solution, system)
    initial = solution.u[1]
    reference_problem = ODEProblem(reduced_spanning_spring_rhs!,
        [initial[9], initial[6]],
        (first(solution.t), last(solution.t)), system)
    reference = solve(reference_problem, Tsit5();
        abstol = 1.0e-11, reltol = 1.0e-11)
    equation_errors = Float64[]
    state_errors = Float64[]
    energies = Float64[]
    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(eltype(z), 20)
        spanning_spring_damper_pendulum!(equations, t, z, zdot, system)
        push!(equation_errors, norm(equations, Inf))
        push!(state_errors, norm([z[9], z[6]] - reference(t), Inf))
        push!(energies, spanning_spring_energy(z, system))
    end
    return (
        return_code = solution.retcode,
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_selected_state_difference = maximum(state_errors),
        minimum_length = minimum(z[14] for z in solution.u),
        maximum_length = maximum(z[14] for z in solution.u),
        minimum_scalar_force = minimum(z[18] for z in solution.u),
        maximum_scalar_force = maximum(z[18] for z in solution.u),
        total_energy_change = last(energies) - first(energies),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution, system = run_spanning_spring_damper_pendulum()
    println("Spanning spring-damper pendulum")
    println("  simultaneous variables: 20")
    println("  applied-force variables and equations: 9")
    for (name, value) in pairs(spanning_spring_diagnostics(solution, system))
        println("  ", name, ": ", value)
    end
    println("  accepted steps: ", solution.stats.accepted_steps)
    println("  rejected steps: ", solution.stats.rejected_steps)
    println("  maximum BDF order: ", maximum(solution.orders))
end
