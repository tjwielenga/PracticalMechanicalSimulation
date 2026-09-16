using Test
using LinearAlgebra

include(joinpath(@__DIR__, "..", "..", "examples", "planar", "minimal_cartesian_pendulum.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "reduced_coordinate_pendulum.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "pendulum_state_selection.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "two_body_pendulum_state_selection.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "inplane_constraint_state_selection.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "parallel_inplane_constraint_components.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "driven_planar_four_bar.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "torque_driven_planar_four_bar.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "constant_speed_slider_crank.jl"))
include(joinpath(@__DIR__, "..", "..", "src", "common", "ResultIO.jl"))
include(joinpath(@__DIR__, "..", "..", "src", "common", "ModalAnalysis.jl"))
include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarModelIO.jl"))
include(joinpath(@__DIR__, "..", "..", "src", "planar", "SimulationRunner.jl"))
include(joinpath(@__DIR__, "..", "..", "src", "common", "CSVExport.jl"))
include(joinpath(@__DIR__, "..", "..", "src", "common", "ModelExtraction.jl"))
include(joinpath(@__DIR__, "..", "..", "src", "planar", "CommandLine.jl"))

using .PlanarModelIO
using .SimulationRunner
using .ResultIO
using .CSVExport
using .ModelExtraction
using .CommandLine

@testset "Parallel inplane constraints" begin
    selection = parallel_inplane_state_selection()
    @test size(selection.D) == (2, 3)
    @test selection.rank == 2
    @test selection.dependent == [1, 3]
    @test selection.independent == [2]
    @test INPLANE_VELOCITY_NAMES[selection.independent] == [:V_y]
    @test norm(selection.D * selection.P, Inf) < 1.0e-12

    assembly = parallel_inplane_component_assembly()
    z = zeros(11)
    z[assembly.body.position_variables] .= [0.0, 1.2]
    z[assembly.body.velocity_variables] .= [0.0, 2.0]
    z[assembly.guide_1.reaction_variable] = 1.3
    z[assembly.guide_2.reaction_variable] = -0.8
    zdot = zeros(11)
    position = select_analysis(assembly.layout.catalog, PositionIC())
    velocity = select_analysis(assembly.layout.catalog, VelocityIC())
    acceleration = select_analysis(assembly.layout.catalog, AccelerationIC())
    @test length(position.variable_indices) == 3
    @test length(position.equation_indices) == 2
    @test length(velocity.variable_indices) == 3
    @test length(velocity.equation_indices) == 2
    @test length(acceleration.variable_indices) == 5
    @test length(acceleration.equation_indices) == 5
    for analysis in (position, velocity)
        equations = zeros(length(analysis.equation_indices))
        evaluate_analysis_equations!(equations, assembly.model, analysis,
            0.0, z, zdot)
        @test norm(equations, Inf) < 1.0e-12
    end

    dynamics = select_analysis(assembly.layout.catalog, Dynamics())
    equations = zeros(length(dynamics.equation_indices))
    evaluate_analysis_equations!(equations, assembly.model, dynamics,
        0.0, z, zdot)
    jacobian = evaluate_analysis_jacobian(assembly.model, dynamics,
        0.0, z, zdot, 0.0)
    direction = collect(range(-0.3, 0.4; length = 11))
    epsilon = 1.0e-6
    perturbed = similar(equations)
    evaluate_analysis_equations!(perturbed, assembly.model, dynamics,
        0.0, z + epsilon * direction, zdot)
    finite_difference = (perturbed - equations) / epsilon
    @test norm(jacobian * direction - finite_difference, Inf) < 2.0e-5
end

@testset "Driven planar four-bar" begin
    parameters = FourBarParameters()
    assembly = driven_four_bar_assembly(parameters)
    expected_sizes = (
        KinematicPosition() => 10,
        KinematicVelocity() => 10,
        KinematicAcceleration() => 10,
        KinematicForces() => 9,
    )
    for (policy, expected) in expected_sizes
        selection = select_analysis(assembly.layout.catalog, policy)
        @test length(selection.variable_indices) == expected
        @test length(selection.equation_indices) == expected
    end

    for time in (0.0, 0.7, 1.4)
        diagnostics = driven_four_bar_diagnostics(time; parameters)
        @test maximum(values(diagnostics.errors)) < 1.0e-10
        @test all(isfinite, values(diagnostics.conditions))
        @test isfinite(diagnostics.driving_torque)
    end

    history = driven_four_bar_history(range(0.0, 1.4; length = 15);
        parameters)
    @test length(history.states) == 15
    @test maximum(c.position for c in history.corrections) <= 4
    @test all(state[history.system.assembly.coupler.position_variables][2] > 0
        for state in history.states)

    revolution = driven_four_bar_history(range(0.0, 5.0; length = 101);
        parameters)
    revolution_assembly = revolution.system.assembly
    crank_angle = revolution_assembly.crank.orientation_variable
    @test revolution.states[end][crank_angle] -
        revolution.states[1][crank_angle] ≈ 2pi
    for body in revolution_assembly.bodies
        @test revolution.states[end][body.position_variables] ≈
            revolution.states[1][body.position_variables] atol = 1.0e-10
    end
    @test maximum(c.position for c in revolution.corrections) <= 3

    state, system, _ = driven_four_bar_state(0.7; parameters)
    for policy in (KinematicPosition(), KinematicVelocity(),
                   KinematicAcceleration(), KinematicForces())
        selection = select_analysis(system.assembly.layout.catalog, policy)
        derivative = zeros(length(state))
        equations = zeros(length(selection.equation_indices))
        evaluate_analysis_equations!(equations, system.assembly.model,
            selection, 0.7, state, derivative)
        jacobian = evaluate_analysis_jacobian(system.assembly.model,
            selection, 0.7, state, derivative, 0.0)
        sparse_jacobian = evaluate_analysis_sparse_jacobian(
            system.assembly.model, selection, 0.7, state, derivative, 0.0)
        @test Matrix(sparse_jacobian) ≈ jacobian
        direction = collect(range(-0.2, 0.3;
            length = length(selection.variable_indices)))
        perturbed_state = copy(state)
        epsilon = 1.0e-7
        perturbed_state[selection.variable_indices] .+= epsilon .* direction
        perturbed = similar(equations)
        evaluate_analysis_equations!(perturbed, system.assembly.model,
            selection, 0.7, perturbed_state, derivative)
        finite_difference = (perturbed - equations) / epsilon
        @test norm(jacobian * direction - finite_difference, Inf) < 2.0e-6
    end
end

@testset "Torque-driven planar four-bar" begin
    parameters = FourBarParameters()
    assembly = torque_driven_four_bar_assembly(parameters; applied_torque = 10.0)
    zero_torque = torque_driven_four_bar_assembly(parameters; applied_torque = 0.0)
    @test length(assembly.layout.catalog.variables) == 35
    @test length(assembly.layout.catalog.equations) == 35

    z0, zdot0 = torque_four_bar_initial_conditions(assembly, parameters)
    equations = zeros(35)
    torque_driven_four_bar_equations!(equations, 0.0, z0, zdot0, assembly)
    @test norm(equations, Inf) < 1.0e-10

    equations_without_torque = zeros(35)
    torque_driven_four_bar_equations!(equations_without_torque,
        0.0, z0, zdot0, zero_torque)
    torque_difference = equations - equations_without_torque
    @test torque_difference[assembly.crank.balance_equations[3]] ≈ -10.0
    @test count(!iszero, torque_difference) == 1

    coefficient = 2.3
    jacobian = torque_driven_four_bar_jacobian(
        0.0, z0, zdot0, coefficient, assembly)
    direction = collect(range(-0.2, 0.3; length = 35))
    epsilon = 1.0e-7
    perturbed = zeros(35)
    torque_driven_four_bar_equations!(perturbed, 0.0,
        z0 + epsilon .* direction,
        zdot0 + coefficient * epsilon .* direction, assembly)
    finite_difference = (perturbed - equations) / epsilon
    @test norm(jacobian * direction - finite_difference, Inf) < 2.0e-6

    solution, dynamic_assembly, dynamic_parameters =
        run_torque_driven_four_bar(tspan = (0.0, 0.5))
    diagnostics = torque_driven_four_bar_diagnostics(
        solution, dynamic_assembly, dynamic_parameters)
    @test diagnostics.maximum_implicit_equation_error < 1.0e-9
    @test diagnostics.maximum_position_constraint_error < 1.0e-12
    @test diagnostics.maximum_energy_balance_error < 3.0e-4
    @test count(solution.differential_vars) == 2
    @test findall(solution.differential_vars) == [
        dynamic_assembly.crank.angular_velocity_variable,
        dynamic_assembly.crank.orientation_variable]

    toml_path = joinpath(@__DIR__, "..", "..", "models", "planar", "torque-driven-four-bar.toml")
    toml_history = run_planar_model(toml_path; duration = 0.5, samples = 21)
    @test toml_history.loaded.analysis.mode == :automatic
    @test toml_history.loaded.analysis.degrees_of_freedom == 1
    @test toml_history.loaded.state_selection.method == :automatic
    @test length(toml_history.loaded.state_selection.preferred_velocities) == 1
    @test toml_history.loaded.state_selection.qr_rank == 8
    @test length(toml_history.loaded.state_selection.qr_pivots) == 9
    @test count(toml_history.solution.differential_vars) == 2
    inconsistent_source = replace(read(toml_path, String),
        "position = [1.134713287347731, 0.634147265352224]" =>
        "position = [1.16, 0.61]")
    corrected = load_planar_model(IOBuffer(inconsistent_source))
    @test corrected.state_selection.position_corrections > 0
    position_selection = select_analysis(
        corrected.layout.catalog, KinematicPosition())
    position_equations = zeros(length(position_selection.equation_indices))
    evaluate_analysis_equations!(position_equations, corrected.model,
        position_selection, corrected.simulation.start_time,
        corrected.initial_values, zeros(length(corrected.initial_values)))
    @test norm(position_equations, Inf) < 1.0e-11
    @test corrected.state_selection.preferred_velocities ==
        toml_history.loaded.state_selection.preferred_velocities
    mktempdir() do directory
        path = joinpath(directory, "dynamic-four-bar.simp")
        write_result(path, toml_history)
        stored = read_result(path)
        @test stored.analysis_mode == :dynamic
        @test stored.degrees_of_freedom == 1
        @test stored.solver_statistics.accepted_steps ==
            toml_history.solution.stats.accepted_steps
    end
    for (time, state) in zip(toml_history.times, toml_history.states)
        reference = solution(time)
        for name in (:crank, :coupler, :rocker)
            loaded_body = toml_history.loaded.bodies[name]
            reference_body = getproperty(dynamic_assembly, name)
            @test state[loaded_body.position_variables] ≈
                reference[reference_body.position_variables] atol = 2.0e-9
            @test state[loaded_body.orientation_variable] ≈
                reference[reference_body.orientation_variable] atol = 2.0e-9
            @test state[loaded_body.velocity_variables] ≈
                reference[reference_body.velocity_variables] atol = 2.0e-8
        end
    end
end

@testset "Constant-speed slider-crank" begin
    parameters = SliderCrankParameters()
    @test parameters.rod_length == 2parameters.crank_length
    assembly = constant_speed_slider_crank_assembly(parameters)
    expected_sizes = (
        KinematicPosition() => 7,
        KinematicVelocity() => 7,
        KinematicAcceleration() => 7,
        KinematicForces() => 6,
    )
    for (policy, expected) in expected_sizes
        selection = select_analysis(assembly.layout.catalog, policy)
        @test length(selection.variable_indices) == expected
        @test length(selection.equation_indices) == expected
    end

    history = constant_speed_slider_crank_history(
        range(0.0, 5.0; length = 81); parameters)
    diagnostics = slider_crank_diagnostics(history)
    @test maximum(values(diagnostics.maximum_errors)) < 2.0e-10
    @test maximum(values(diagnostics.maximum_conditions)) < 10
    @test diagnostics.maximum_position_corrections <= 3
    crank_angle = assembly.crank.orientation_variable
    @test history.states[end][crank_angle] -
        history.states[1][crank_angle] ≈ 2pi

    for (time, state) in zip(history.times, history.states)
        crank_joint = PlanarAppliedForces.point_marker_kinematics(
            assembly.crank_pin.marker_a, state).position
        slider = PlanarAppliedForces.point_marker_kinematics(
            assembly.slider.marker_i, state).position
        @test abs(slider[2]) < 2.0e-10
        @test norm(slider - crank_joint) ≈ parameters.rod_length atol = 2.0e-10
        expected_x = parameters.crank_length * cos(state[crank_angle]) +
            sqrt(parameters.rod_length^2 -
                 parameters.crank_length^2 * sin(state[crank_angle])^2)
        @test slider[1] ≈ expected_x atol = 2.0e-10
    end

    state = history.states[23]
    time = history.times[23]
    derivative = zeros(length(state))
    for policy in (KinematicPosition(), KinematicVelocity(),
                   KinematicAcceleration(), KinematicForces())
        selection = select_analysis(assembly.layout.catalog, policy)
        dense = evaluate_analysis_jacobian(assembly.model, selection,
            time, state, derivative, 0.0)
        sparse = evaluate_analysis_sparse_jacobian(assembly.model, selection,
            time, state, derivative, 0.0)
        @test Matrix(sparse) ≈ dense
    end
end

include(joinpath(@__DIR__, "..", "..", "examples", "planar", "implicit_displacement_pendulum.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "implicit_velocity_pendulum.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "implicit_acceleration_pendulum.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "implicit_baumgarte_pendulum.jl"))
include(joinpath(@__DIR__, "..", "..", "examples", "planar", "generalized_bdf1_pendulum.jl"))

@testset "Minimal Cartesian pendulum" begin
    p = PendulumParameters()

    q_user = [0.03, -0.02, deg2rad(35.0)]
    q = consistent_position(q_user, p)
    @test norm(position_constraint(q, p), Inf) < 1.0e-11

    v_user = [0.1, -0.2, 0.7]
    v = consistent_velocity(v_user, q, p)
    u = vcat(q, v)
    @test norm(velocity_constraint(u, p), Inf) < 1.0e-12

    a_global, alpha, lambda_global = acceleration_solution(u, p)
    c_global, d_global = marker_vectors(q[3], p)

    @test norm(p.mass .* a_global .- lambda_global .-
               p.mass .* p.gravity, Inf) < 1.0e-12
    @test abs(p.inertia * alpha - dot(d_global, lambda_global)) < 1.0e-12
    @test norm(a_global .+ d_global .* alpha .-
               c_global .* u[6]^2, Inf) < 1.0e-12

    _, diagnostics = run_example(tspan = (0.0, 2.0))
    @test diagnostics.initial_position_error < 1.0e-12
    @test diagnostics.initial_velocity_error < 1.0e-12
    @test diagnostics.maximum_position_error < 1.0e-8
    @test diagnostics.maximum_velocity_error < 1.0e-8
    @test diagnostics.maximum_energy_error < 1.0e-8
end

@testset "Pendulum state selection" begin
    p = PendulumParameters()
    theta = deg2rad(35.0)

    unscaled = pendulum_state_selection(theta; p)
    @test unscaled.rank == 2
    @test unscaled.dependent == [1, 2]
    @test unscaled.independent == [3]
    @test PLANAR_VELOCITY_NAMES[unscaled.independent] == [:omega]
    @test norm(unscaled.D * unscaled.P, Inf) < 1.0e-14

    _, d_global = marker_vectors(theta, p)
    @test unscaled.P[:, 1] ≈ [-d_global; 1.0]

    preferred = preferred_pendulum_selection(theta; p)
    @test preferred.dependent == [1, 2]
    @test preferred.independent == [3]
    @test preferred.dependent_condition == 1.0
    @test preferred.P ≈ unscaled.P
end

@testset "Two-body pendulum state selection" begin
    theta_1 = deg2rad(35.0)
    theta_2 = deg2rad(-20.0)
    result = two_body_state_selection(theta_1, theta_2)

    @test size(result.D) == (4, 6)
    @test result.rank == 4
    @test result.dependent == [1, 2, 3, 5]
    @test result.independent == [4, 6]
    @test TWO_BODY_VELOCITY_NAMES[result.independent] == [:V_2x, :omega_2]
    @test norm(result.D * result.P, Inf) < 1.0e-14

    preferred = preferred_velocity_selection(result.D, [3, 6])
    @test preferred.dependent == [1, 2, 4, 5]
    @test preferred.independent == [3, 6]
    @test preferred.dependent_condition < result.dependent_condition
    @test norm(preferred.D * preferred.P, Inf) < 1.0e-14

    d_upper_1 = planar_marker_direction(theta_1, [0.0, 0.5])
    d_lower_1 = planar_marker_direction(theta_1, [0.0, -0.5])
    d_upper_2 = planar_marker_direction(theta_2, [0.0, 0.5])
    expected = zeros(6, 2)
    expected[1:2, 1] .= -d_upper_1
    expected[3, 1] = 1.0
    expected[4:5, 1] .= d_lower_1 - d_upper_1
    expected[4:5, 2] .= -d_upper_2
    expected[6, 2] = 1.0
    @test preferred.P ≈ expected
end

@testset "Implicit acceleration-constraint pendulum" begin
    p = PendulumParameters()
    y0, dy0 = implicit_initial_conditions(deg2rad(35.0), 0.7, p)
    equations = zeros(8)
    implicit_acceleration_pendulum!(equations, dy0, y0, p, 0.0)
    @test norm(equations, Inf) < 1.0e-12

    tolerances = acceleration_control_tolerances()
    @test all(tolerances[1:6] .== 1.0e-5)
    @test all(tolerances[REACTION_INDICES] .== 1.0e-3)

    coefficient = 2.3
    direction = collect(range(-0.2, 0.5; length = 8))
    jacobian = implicit_acceleration_jacobian(0.0, y0, dy0, coefficient, p)
    epsilon = 1.0e-7
    perturbed_equations = zeros(8)
    implicit_acceleration_pendulum!(
        perturbed_equations,
        dy0 + coefficient * epsilon * direction,
        y0 + epsilon * direction,
        p,
        0.0,
    )
    finite_difference = (perturbed_equations - equations) / epsilon
    @test norm(jacobian * direction - finite_difference, Inf) < 1.0e-6
end

@testset "Baumgarte-stabilized pendulum" begin
    p = BaumgarteParameters(PendulumParameters(), 0.1)
    position_error = [1.0e-3, -5.0e-4]
    velocity_error = [2.0e-4, -1.0e-4]
    y0, dy0 = baumgarte_initial_conditions(deg2rad(35.0), 0.7, p;
        position_error, velocity_error)
    equations = zeros(8)
    implicit_baumgarte_pendulum!(equations, dy0, y0, p, 0.0)
    @test norm(equations, Inf) < 1.0e-12
    @test position_constraint(y0[1:3], p.mechanical) ≈ position_error
    @test velocity_constraint(y0[1:6], p.mechanical) ≈ velocity_error

    coefficient = 2.1
    direction = collect(range(-0.3, 0.4; length = 8))
    jacobian = implicit_baumgarte_jacobian(0.0, y0, dy0,
        coefficient, p)
    epsilon = 1.0e-7
    perturbed = zeros(8)
    implicit_baumgarte_pendulum!(perturbed,
        dy0 + coefficient * epsilon * direction,
        y0 + epsilon * direction, p, 0.0)
    finite_difference = (perturbed - equations) / epsilon
    @test norm(jacobian * direction - finite_difference, Inf) < 1.0e-5
end

@testset "Generalized BDF1 pendulum" begin
    p = PendulumParameters()
    z0 = bdf1_initial_unknown(deg2rad(35.0), 0.7, p)
    previous_state = bdf1_state(z0)
    step = 0.002
    z1, _, equation_norm = generalized_bdf1_step(z0, step, p)
    equations = zeros(11)
    generalized_bdf1_equations!(equations, z1, previous_state, step, p)
    @test equation_norm <= 1
    @test norm(equations, Inf) < 1.0e-8

    direction = collect(range(-0.2, 0.3; length = 11))
    jacobian = generalized_bdf1_jacobian(z1, step, p)
    epsilon = 1.0e-7
    perturbed = zeros(11)
    generalized_bdf1_equations!(perturbed, z1 + epsilon * direction,
        previous_state, step, p)
    finite_difference = (perturbed - equations) / epsilon
    @test norm(jacobian * direction - finite_difference, Inf) < 1.0e-6
    @test norm(position_constraint(z1[9:11], p), Inf) < 1.0e-8
end

@testset "Implicit velocity-constraint pendulum" begin
    p = PendulumParameters()
    y0, dy0 = implicit_initial_conditions(deg2rad(35.0), 0.7, p)
    equations = zeros(8)
    implicit_velocity_pendulum!(equations, dy0, y0, p, 0.0)
    @test norm(equations, Inf) < 1.0e-12

    tolerances = velocity_control_tolerances()
    @test all(tolerances[1:6] .== 1.0e-9)
    @test all(tolerances[REACTION_INDICES] .== 1.0e6)

    coefficient = 2.9
    direction = collect(range(-0.3, 0.4; length = 8))
    jacobian = implicit_velocity_jacobian(0.0, y0, dy0, coefficient, p)
    epsilon = 1.0e-7
    perturbed_equations = zeros(8)
    implicit_velocity_pendulum!(
        perturbed_equations,
        dy0 + coefficient * epsilon * direction,
        y0 + epsilon * direction,
        p,
        0.0,
    )
    finite_difference = (perturbed_equations - equations) / epsilon
    @test norm(jacobian * direction - finite_difference, Inf) < 1.0e-6

    dassl_solution = run_velocity_dassl(tspan = (0.0, 0.2))
    dassl_diagnostics = velocity_solution_diagnostics(dassl_solution, p)
    dassl_comparison = compare_implicit_with_reduced(dassl_solution, p)
    @test dassl_diagnostics.maximum_velocity_constraint_error < 1.0e-6
    @test dassl_comparison.maximum_state_difference < 1.0e-5

    ida_solution = run_velocity_ida(tspan = (0.0, 0.2))
    ida_diagnostics = velocity_solution_diagnostics(ida_solution, p)
    ida_comparison = compare_implicit_with_reduced(ida_solution, p)
    @test ida_diagnostics.maximum_velocity_constraint_error < 1.0e-6
    @test ida_comparison.maximum_state_difference < 1.0e-5
end

@testset "Implicit displacement-constraint pendulum" begin
    p = PendulumParameters()
    y0, dy0 = implicit_initial_conditions(deg2rad(35.0), 0.7, p)
    equations = zeros(8)
    implicit_pendulum!(equations, dy0, y0, p, 0.0)
    @test norm(equations, Inf) < 1.0e-12

    tolerances = displacement_control_tolerances()
    @test all(tolerances[POSITION_INDICES] .== 1.0e-9)
    @test all(tolerances[VELOCITY_INDICES] .== 1.0e6)
    @test all(tolerances[REACTION_INDICES] .== 1.0e6)

    coefficient = 3.7
    direction = collect(range(-0.4, 0.3; length = 8))
    jacobian = implicit_pendulum_jacobian(0.0, y0, dy0, coefficient, p)
    epsilon = 1.0e-7
    perturbed_equations = zeros(8)
    implicit_pendulum!(
        perturbed_equations,
        dy0 + coefficient * epsilon * direction,
        y0 + epsilon * direction,
        p,
        0.0,
    )
    finite_difference = (perturbed_equations - equations) / epsilon
    @test norm(jacobian * direction - finite_difference, Inf) < 1.0e-6
end

@testset "Reduced-coordinate pendulum" begin
    p = PendulumParameters()
    u_reduced = [deg2rad(35.0), 0.7]
    reconstructed = reconstruct_cartesian(u_reduced, p)
    u_cartesian = [reconstructed.q; reconstructed.v]

    @test norm(position_constraint(reconstructed.q, p), Inf) < 1.0e-12
    @test norm(velocity_constraint(u_cartesian, p), Inf) < 1.0e-12

    lambda_global = reduced_pin_reaction(u_reduced, p)
    @test norm(
        p.mass .* reconstructed.a_global .- lambda_global .-
        p.mass .* p.gravity,
        Inf,
    ) < 1.0e-12
    @test abs(
        p.inertia * reconstructed.alpha_p -
        dot(marker_vectors(u_reduced[1], p)[2], lambda_global),
    ) < 1.0e-12

    _, diagnostics = run_reduced_example(tspan = (0.0, 2.0))
    @test diagnostics.maximum_energy_error < 1.0e-8

    _, _, comparison = compare_formulations(tspan = (0.0, 2.0))
    @test comparison.maximum_state_difference < 1.0e-8
    @test comparison.maximum_acceleration_difference < 1.0e-8
    @test comparison.maximum_reaction_difference < 1.0e-8
    @test comparison.maximum_reduced_energy_error < 1.0e-8
    @test comparison.maximum_cartesian_energy_error < 1.0e-8
    @test comparison.maximum_energy_difference < 1.0e-8
end
