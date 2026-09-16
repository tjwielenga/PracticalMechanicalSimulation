using LinearAlgebra
using OrdinaryDiffEq

if !isdefined(@__MODULE__, :run_independent_state_pendulum)
    include(joinpath(@__DIR__, "independent_state_pendulum.jl"))
end

struct EulerParameterPendulumParameters{P}
    mechanical::P
    use_euler_parameters_for_orientation::Bool
end

const EULER_PARAMETER_VARIABLE_LEVELS =
    [INDEPENDENT_STATE_VARIABLE_LEVELS; 0; 0]
const EULER_PARAMETER_EQUATION_LEVELS =
    [INDEPENDENT_STATE_EQUATION_LEVELS; 1; 0]
const EULER_PARAMETER_DIFFERENTIAL_VARS =
    Bool[INDEPENDENT_STATE_DIFFERENTIAL_VARS; true; true]
const EULER_PARAMETER_STATE_ERROR_CONTROL =
    Bool[INDEPENDENT_STATE_DIFFERENTIAL_VARS; false; false]

function planar_euler_parameters(theta)
    return [cos(theta / 2), sin(theta / 2)]
end

function planar_euler_parameter_rates(parameters, omega)
    p0, p3 = parameters
    return 0.5 .* [-p3, p0] .* omega
end

function rotation_from_euler_parameters(parameters)
    p0, p3 = parameters
    return [p0^2 - p3^2 -2p0 * p3;
            2p0 * p3 p0^2 - p3^2]
end

function marker_vectors_from_euler_parameters(parameters,
        mechanical::PendulumParameters)
    A = rotation_from_euler_parameters(parameters)
    T = eltype(parameters)
    S = T[0 -1; 1 0]
    return A * mechanical.r_body, A * S * mechanical.r_body
end

"""
Thirteen-equation extension of the independent-state pendulum.

z = [a_x, a_y, alpha, V_x, V_y, omega,
     R_x, R_y, theta, lambda_x, lambda_y, p_0, p_3]
"""
function euler_parameter_independent_state_pendulum!(equations, t, z, zdot,
        parameters::EulerParameterPendulumParameters)
    mechanical = parameters.mechanical
    acceleration = view(z, 1:2)
    alpha = z[3]
    velocity = view(z, 4:5)
    omega = z[6]
    position = view(z, 7:8)
    theta = z[9]
    reaction = view(z, 10:11)
    euler_parameters = view(z, 12:13)

    r_global, d_global = if parameters.use_euler_parameters_for_orientation
        marker_vectors_from_euler_parameters(euler_parameters, mechanical)
    else
        marker_vectors(theta, mechanical)
    end

    equations[1:2] .= mechanical.mass .* acceleration .- reaction .-
                      mechanical.mass .* mechanical.gravity
    equations[3] = mechanical.inertia * alpha - dot(d_global, reaction)
    equations[4:5] .= acceleration .+ d_global .* alpha .-
                       r_global .* omega^2
    equations[6:7] .= velocity .+ d_global .* omega
    equations[8:9] .= position .+ r_global .- mechanical.pin
    equations[10] = alpha - zdot[6]
    equations[11] = omega - zdot[9]

    p0, p3 = euler_parameters
    pdot0, pdot3 = zdot[12], zdot[13]
    equations[12] = omega - 2(-p3 * pdot0 + p0 * pdot3)
    equations[13] = p0^2 + p3^2 - 1
    return nothing
end

function euler_parameter_initial_conditions(theta, omega,
        mechanical::PendulumParameters)
    base_z, base_zdot = independent_state_initial_conditions(
        theta, omega, mechanical)
    parameters = planar_euler_parameters(theta)
    parameter_rates = planar_euler_parameter_rates(parameters, omega)
    return [base_z; parameters], [base_zdot; parameter_rates]
end

function run_euler_parameter_independent_state_pendulum(;
        use_euler_parameters_for_orientation = false,
        control_euler_parameters = !use_euler_parameters_for_orientation,
        control_theta = true,
        theta0 = deg2rad(45.0), omega0 = 0.0,
        tspan = (0.0, 5.0), atol = 1.0e-7,
        rtol = 1.0e-5, dt = 1.0e-5, dtmax = 0.05)
    mechanical = PendulumParameters()
    parameters = EulerParameterPendulumParameters(
        mechanical, use_euler_parameters_for_orientation)
    z0, zdot0 = euler_parameter_initial_conditions(theta0, omega0, mechanical)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol,
        rtol = rtol,
        initial_step = dt,
        maximum_step = dtmax,
    )
    error_control = falses(length(z0))
    error_control[6] = true
    error_control[9] = control_theta
    error_control[12:13] .= control_euler_parameters

    return HistoricalDDASSL.dassl(
        euler_parameter_independent_state_pendulum!, z0, zdot0, tspan;
        parameter = parameters,
        options,
        variable_levels = EULER_PARAMETER_VARIABLE_LEVELS,
        equation_levels = EULER_PARAMETER_EQUATION_LEVELS,
        differential_vars = EULER_PARAMETER_DIFFERENTIAL_VARS,
        error_control,
        deficit = 0,
    )
end

function euler_parameter_pendulum_diagnostics(solution,
        parameters::EulerParameterPendulumParameters)
    mechanical = parameters.mechanical
    reduced_problem = ODEProblem(
        reduced_pendulum_rhs!, [solution.u[1][9], solution.u[1][6]],
        (first(solution.t), last(solution.t)), mechanical,
    )
    reduced_solution = solve(
        reduced_problem, Tsit5(); abstol = 1.0e-11, reltol = 1.0e-11
    )

    equation_errors = Float64[]
    normalization_errors = Float64[]
    orientation_differences = Float64[]
    mechanical_orientation_errors = Float64[]
    integrated_state_errors = Float64[]
    position_errors = Float64[]
    velocity_errors = Float64[]
    acceleration_errors = Float64[]
    reaction_errors = Float64[]

    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(eltype(z), length(z))
        euler_parameter_independent_state_pendulum!(
            equations, t, z, zdot, parameters)
        push!(equation_errors, norm(equations, Inf))
        push!(normalization_errors, abs(dot(z[12:13], z[12:13]) - 1))
        push!(orientation_differences,
            norm(rotation(z[9]) -
                 rotation_from_euler_parameters(z[12:13]), Inf))

        reduced_state = reduced_solution(t)
        reconstructed = reconstruct_cartesian(reduced_state, mechanical)
        reference_reaction = reduced_pin_reaction(reduced_state, mechanical)
        mechanical_rotation =
            parameters.use_euler_parameters_for_orientation ?
            rotation_from_euler_parameters(z[12:13]) : rotation(z[9])
        push!(mechanical_orientation_errors,
            norm(mechanical_rotation - rotation(reduced_state[1]), Inf))
        push!(integrated_state_errors,
            norm([z[9], z[6]] - reduced_state, Inf))
        push!(position_errors,
            norm([z[7], z[8], z[9]] - reconstructed.q, Inf))
        push!(velocity_errors,
            norm([z[4], z[5], z[6]] - reconstructed.v, Inf))
        push!(acceleration_errors,
            norm(z[1:3] -
                 [reconstructed.a_global; reconstructed.alpha_p], Inf))
        push!(reaction_errors, norm(z[10:11] - reference_reaction, Inf))
    end

    return (
        return_code = solution.retcode,
        final_time = last(solution.t),
        accepted_time_points = length(solution.t),
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_euler_normalization_error = maximum(normalization_errors),
        maximum_orientation_matrix_difference =
            maximum(orientation_differences),
        maximum_mechanical_orientation_difference =
            maximum(mechanical_orientation_errors),
        maximum_integrated_state_difference = maximum(integrated_state_errors),
        maximum_position_difference = maximum(position_errors),
        maximum_velocity_difference = maximum(velocity_errors),
        maximum_acceleration_difference = maximum(acceleration_errors),
        maximum_reaction_difference = maximum(reaction_errors),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    mechanical = PendulumParameters()
    for (name, uses_parameters, controls_parameters, controls_theta) in (
            ("A(theta), theta and Euler parameters controlled",
             false, true, true),
            ("A(p), theta controlled", true, false, true),
            ("A(p), Euler parameters controlled", true, true, false))
        solution = run_euler_parameter_independent_state_pendulum(;
            use_euler_parameters_for_orientation = uses_parameters,
            control_euler_parameters = controls_parameters,
            control_theta = controls_theta)
        parameters = EulerParameterPendulumParameters(
            mechanical, uses_parameters)
        println(name)
        for (key, value) in pairs(
                euler_parameter_pendulum_diagnostics(solution, parameters))
            println("  ", key, ": ", value)
        end
        for (key, value) in pairs(
                independent_state_solver_diagnostics(solution))
            println("  ", key, ": ", value)
        end
    end
end
