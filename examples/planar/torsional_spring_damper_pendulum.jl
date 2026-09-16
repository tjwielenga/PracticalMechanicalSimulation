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

struct TorsionalSpringPendulumSystem{P,S}
    mechanical::P
    spring_damper::S
end

function TorsionalSpringPendulumSystem(;
        stiffness = 2.0, damping = 0.1, free_angle = 0.0)
    mechanical = PendulumParameters()
    body_marker = PlanarBodyOrientationMarker(9, 6, 3, 0.0)
    ground_marker = PlanarGroundOrientationMarker(0.0)
    spring_damper = PlanarTorsionalSpringDamper(
        body_marker, ground_marker, stiffness, damping, free_angle, 12, 12)
    return TorsionalSpringPendulumSystem(mechanical, spring_damper)
end

const TORSIONAL_SPRING_VARIABLE_LEVELS =
    [INDEPENDENT_STATE_VARIABLE_LEVELS; 2]
const TORSIONAL_SPRING_EQUATION_LEVELS =
    [INDEPENDENT_STATE_EQUATION_LEVELS; 2]
const TORSIONAL_SPRING_DIFFERENTIAL_VARS =
    BitVector([INDEPENDENT_STATE_DIFFERENTIAL_VARS; false])

"""
Full-equation pendulum with an explicit torsional spring-damper torque.

z = [a_x, a_y, alpha, V_x, V_y, omega,
     R_x, R_y, theta, lambda_x, lambda_y, T]
"""
function torsional_spring_damper_pendulum!(equations, t, z, zdot,
                                           system)
    independent_state_pendulum!(
        equations, t, z, zdot, system.mechanical)
    equations[12] = zero(eltype(equations))
    add_torsional_spring_damper!(
        equations, z, system.spring_damper)
    return nothing
end

function torsional_spring_damper_pendulum_jacobian!(jacobian, t, z, zdot,
                                                    coefficient, system)
    independent_state_pendulum_jacobian!(
        jacobian, t, z, zdot, coefficient, system.mechanical)
    add_torsional_spring_damper_jacobian!(
        jacobian, z, system.spring_damper)
    return nothing
end

function spring_torque(z, system)
    relative = relative_rotation(system.spring_damper, z)
    element = system.spring_damper
    return -element.stiffness * (relative.angle - element.free_angle) -
        element.damping * relative.angular_velocity
end

function torsional_spring_initial_conditions(theta, omega, system)
    p = system.mechanical
    reconstructed = reconstruct_cartesian([theta, omega], p)
    z0 = zeros(promote_type(typeof(theta), typeof(omega)), 12)
    z0[4:5] .= reconstructed.v[1:2]
    z0[6] = omega
    z0[7:9] .= reconstructed.q
    z0[12] = spring_torque(z0, system)

    r_global, d_global = marker_vectors(theta, p)
    effective_inertia = p.inertia + p.mass * dot(d_global, d_global)
    alpha = -(p.mass * dot(d_global, p.gravity) - z0[12]) /
        effective_inertia
    acceleration = -d_global .* alpha .+ r_global .* omega^2
    reaction = p.mass .* (acceleration .- p.gravity)
    z0[1:2] .= acceleration
    z0[3] = alpha
    z0[10:11] .= reaction

    zdot0 = zeros(eltype(z0), length(z0))
    zdot0[6] = alpha
    zdot0[9] = omega
    return z0, zdot0
end

function run_torsional_spring_damper_pendulum(;
        theta0 = deg2rad(45.0), omega0 = 0.0,
        stiffness = 2.0, damping = 0.1, free_angle = 0.0,
        tspan = (0.0, 5.0), atol = 1.0e-7, rtol = 1.0e-5,
        dt = 1.0e-5, dtmax = 0.05)
    system = TorsionalSpringPendulumSystem(;
        stiffness, damping, free_angle)
    z0, zdot0 = torsional_spring_initial_conditions(
        theta0, omega0, system)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol, rtol = rtol, initial_step = dt, maximum_step = dtmax)
    solution = HistoricalDDASSL.dassl(
        torsional_spring_damper_pendulum!, z0, zdot0, tspan;
        parameter = system,
        jacobian! = torsional_spring_damper_pendulum_jacobian!,
        options,
        variable_levels = TORSIONAL_SPRING_VARIABLE_LEVELS,
        equation_levels = TORSIONAL_SPRING_EQUATION_LEVELS,
        differential_vars = TORSIONAL_SPRING_DIFFERENTIAL_VARS,
        error_control = TORSIONAL_SPRING_DIFFERENTIAL_VARS,
        deficit = 0,
    )
    return solution, system
end

function reduced_spring_pendulum_rhs!(du, u, system, t)
    theta, omega = u
    p = system.mechanical
    z = zeros(eltype(u), 12)
    z[6] = omega
    z[9] = theta
    torque = spring_torque(z, system)
    _, d_global = marker_vectors(theta, p)
    effective_inertia = p.inertia + p.mass * dot(d_global, d_global)
    du[1] = omega
    du[2] = -(p.mass * dot(d_global, p.gravity) - torque) /
        effective_inertia
    return nothing
end

function torsional_spring_damper_energy(z, system)
    mechanical_energy = total_energy([z[7:9]; z[4:6]], system.mechanical)
    relative = relative_rotation(system.spring_damper, z)
    spring_deflection = relative.angle - system.spring_damper.free_angle
    return mechanical_energy +
        0.5 * system.spring_damper.stiffness * spring_deflection^2
end

function torsional_spring_damper_diagnostics(solution, system)
    initial = solution.u[1]
    reference_problem = ODEProblem(reduced_spring_pendulum_rhs!,
        [initial[9], initial[6]],
        (first(solution.t), last(solution.t)), system)
    reference = solve(reference_problem, Tsit5();
        abstol = 1.0e-11, reltol = 1.0e-11)

    equation_errors = Float64[]
    state_errors = Float64[]
    torque_errors = Float64[]
    energies = Float64[]
    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(eltype(z), 12)
        torsional_spring_damper_pendulum!(
            equations, t, z, zdot, system)
        push!(equation_errors, norm(equations, Inf))
        push!(state_errors, norm([z[9], z[6]] - reference(t), Inf))
        push!(torque_errors, abs(z[12] - spring_torque(z, system)))
        push!(energies, torsional_spring_damper_energy(z, system))
    end
    return (
        return_code = solution.retcode,
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_selected_state_difference = maximum(state_errors),
        maximum_constitutive_torque_error = maximum(torque_errors),
        minimum_torque = minimum(z[12] for z in solution.u),
        maximum_torque = maximum(z[12] for z in solution.u),
        initial_total_energy = first(energies),
        final_total_energy = last(energies),
        total_energy_change = last(energies) - first(energies),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution, system = run_torsional_spring_damper_pendulum()
    println("Torsional spring-damper pendulum")
    println("  simultaneous variables: 12")
    println("  differential and controlled variables: 2")
    for (name, value) in pairs(
            torsional_spring_damper_diagnostics(solution, system))
        println("  ", name, ": ", value)
    end
    println("  accepted steps: ", solution.stats.accepted_steps)
    println("  rejected steps: ", solution.stats.rejected_steps)
    println("  maximum BDF order: ", maximum(solution.orders))
end
