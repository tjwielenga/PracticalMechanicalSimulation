using LinearAlgebra
using SparseArrays

if !isdefined(@__MODULE__, :automatic_two_body_executable_model)
    include(joinpath(@__DIR__, "automatic_two_body_component_assembly.jl"))
end
if !isdefined(@__MODULE__, :driven_base_state_selection)
    include(joinpath(@__DIR__, "two_body_pendulum_state_selection.jl"))
end

function driven_base_double_pendulum_assembly(system::ModularTwoBodySystem;
        initial_angle = deg2rad(20.0), amplitude = deg2rad(25.0),
        frequency = 2.0)
    registrations = (
        body_1 = planar_body_registration(:body_1),
        body_2 = planar_body_registration(:body_2),
        pin_1 = revolute_joint_registration(:pin_1),
        pin_2 = revolute_joint_registration(:pin_2),
        motion = rotational_motion_registration(:motion),
    )
    builder = ModelLayoutBuilder()
    for registration in registrations
        allocate_component_variables!(builder, registration)
    end
    allocate_component_equation_block!(builder, registrations.body_1, :balance)
    allocate_component_equation_block!(builder, registrations.body_2, :balance)
    for pin in (registrations.pin_1, registrations.pin_2)
        for block in (:acceleration, :velocity, :position)
            allocate_component_equation_block!(builder, pin, block)
        end
    end
    for block in (:acceleration, :velocity, :position)
        allocate_component_equation_block!(builder, registrations.motion, block)
    end
    allocate_component_equation_block!(builder,
        registrations.body_2, :selected_state)
    layout = finish_layout(builder)

    physical_1, physical_2 = system.bodies
    variables_1 = component_variable_indices(layout, :body_1)
    variables_2 = component_variable_indices(layout, :body_2)
    body_1 = PlanarRigidBodyComponent(:body_1, physical_1.mass,
        physical_1.inertia, variables_1[1:2], variables_1[3],
        variables_1[4:5], variables_1[6], variables_1[7:8], variables_1[9],
        component_equation_indices(layout, :body_1, :balance), 1:0)
    body_2 = PlanarRigidBodyComponent(:body_2, physical_2.mass,
        physical_2.inertia, variables_2[1:2], variables_2[3],
        variables_2[4:5], variables_2[6], variables_2[7:8], variables_2[9],
        component_equation_indices(layout, :body_2, :balance),
        component_equation_indices(layout, :body_2, :selected_state))
    gravity_1 = PlanarGravityComponent(:gravity_1, body_1, collect(system.gravity))
    gravity_2 = PlanarGravityComponent(:gravity_2, body_2, collect(system.gravity))

    old_pin_1, old_pin_2 = system.joints
    upper_1 = PlanarBodyPointMarker(body_1.position_variables,
        body_1.orientation_variable, body_1.velocity_variables,
        body_1.angular_velocity_variable, body_1.balance_equations[1:2],
        body_1.balance_equations[3], collect(old_pin_1.marker_a.r_body))
    ground = PlanarGroundPointMarker(collect(old_pin_1.marker_b.position))
    pin_1 = PlanarRevoluteJointComponent(:pin_1, body_1, upper_1,
        nothing, ground, component_variable_indices(layout, :pin_1),
        component_equation_indices(layout, :pin_1, :acceleration),
        component_equation_indices(layout, :pin_1, :velocity),
        component_equation_indices(layout, :pin_1, :position))

    lower_1 = PlanarBodyPointMarker(body_1.position_variables,
        body_1.orientation_variable, body_1.velocity_variables,
        body_1.angular_velocity_variable, body_1.balance_equations[1:2],
        body_1.balance_equations[3], collect(old_pin_2.marker_a.r_body))
    upper_2 = PlanarBodyPointMarker(body_2.position_variables,
        body_2.orientation_variable, body_2.velocity_variables,
        body_2.angular_velocity_variable, body_2.balance_equations[1:2],
        body_2.balance_equations[3], collect(old_pin_2.marker_b.r_body))
    pin_2 = PlanarRevoluteJointComponent(:pin_2, body_1, lower_1,
        body_2, upper_2, component_variable_indices(layout, :pin_2),
        component_equation_indices(layout, :pin_2, :acceleration),
        component_equation_indices(layout, :pin_2, :velocity),
        component_equation_indices(layout, :pin_2, :position))

    body_orientation = PlanarBodyOrientationMarker(
        body_1.orientation_variable, body_1.angular_velocity_variable,
        body_1.balance_equations[3], 0.0)
    ground_orientation = PlanarGroundOrientationMarker(0.0)
    motion_variables = component_variable_indices(layout, :motion)
    prescribed_angle(t) = initial_angle + amplitude * sin(frequency * t)
    prescribed_omega(t) = amplitude * frequency * cos(frequency * t)
    prescribed_alpha(t) = -amplitude * frequency^2 * sin(frequency * t)
    motion = PlanarRotationalMotionGenerator(:motion,
        body_1, body_orientation, nothing, ground_orientation,
        motion_variables[1], motion_variables[2], motion_variables[3],
        motion_variables[4],
        component_equation_indices(layout, :motion, :position),
        component_equation_indices(layout, :motion, :velocity),
        component_equation_indices(layout, :motion, :acceleration),
        prescribed_angle, prescribed_omega, prescribed_alpha)

    blocks = ExecutableEquationBlock[]
    append!(blocks, executable_blocks(body_1)[1:1])
    append!(blocks, executable_blocks(body_2))
    append!(blocks, executable_blocks(pin_1))
    append!(blocks, executable_blocks(pin_2))
    append!(blocks, executable_blocks(motion))
    contributions = EquationContribution[]
    for component in (gravity_1, gravity_2, pin_1, pin_2, motion)
        append!(contributions, equation_contributions(component))
    end
    model = ExecutableAnalysisModel(layout.catalog, blocks, contributions)
    return (; body_1, body_2, pin_1, pin_2, motion, layout, model)
end

function driven_base_double_pendulum_equations!(equations, t, z, zdot,
                                                 assembly)
    selection = select_analysis(assembly.layout.catalog, Dynamics())
    evaluate_analysis_equations!(equations, assembly.model, selection,
        t, z, zdot)
    return nothing
end

function driven_base_double_pendulum_jacobian(t, z, zdot, coefficient,
                                               assembly)
    selection = select_analysis(assembly.layout.catalog, Dynamics())
    return evaluate_analysis_sparse_jacobian(assembly.model, selection,
        t, z, zdot, coefficient)
end

function driven_base_double_pendulum_jacobian!(matrix::SparseMatrixCSC,
        t, z, zdot, coefficient, assembly)
    selection = select_analysis(assembly.layout.catalog, Dynamics())
    evaluate_analysis_sparse_jacobian!(matrix, assembly.model, selection,
        t, z, zdot, coefficient)
    return nothing
end

function driven_base_double_pendulum_initial_conditions(theta_2, omega_2,
        system, assembly, t0)
    theta_1 = assembly.motion.motion(t0)
    omega_1 = assembly.motion.motion_derivative(t0)
    reconstructed = reconstruct_two_body_state(
        theta_1, omega_1, theta_2, omega_2, system)
    z = zeros(eltype(reconstructed), 26)
    z[1:18] .= reconstructed[1:18]
    z[assembly.motion.angle_variable] = theta_1
    z[assembly.motion.angular_velocity_variable] = omega_1
    z[assembly.motion.angular_acceleration_variable] =
        assembly.motion.motion_second_derivative(t0)

    unknowns = [collect(assembly.body_1.acceleration_variables);
        assembly.body_1.angular_acceleration_variable;
        collect(assembly.body_2.acceleration_variables);
        assembly.body_2.angular_acceleration_variable;
        collect(assembly.pin_1.reaction_variables);
        collect(assembly.pin_2.reaction_variables);
        assembly.motion.angular_acceleration_variable;
        assembly.motion.torque_variable]
    equations_used = [collect(assembly.body_1.balance_equations);
        collect(assembly.body_2.balance_equations);
        collect(assembly.pin_1.acceleration_equations);
        collect(assembly.pin_2.acceleration_equations);
        collect(assembly.motion.acceleration_equations)]
    selection = select_analysis(assembly.layout.catalog, Dynamics())
    zdot = zeros(eltype(z), length(z))
    base = zeros(eltype(z), length(z))
    evaluate_analysis_equations!(base, assembly.model, selection, t0, z, zdot)
    matrix = zeros(eltype(z), length(equations_used), length(unknowns))
    trial = similar(base)
    for (column, variable) in enumerate(unknowns)
        z[variable] += one(eltype(z))
        evaluate_analysis_equations!(trial, assembly.model, selection,
            t0, z, zdot)
        matrix[:, column] .= trial[equations_used] - base[equations_used]
        z[variable] -= one(eltype(z))
    end
    z[unknowns] .+= matrix \ (-base[equations_used])
    zdot[assembly.body_2.angular_velocity_variable] =
        z[assembly.body_2.angular_acceleration_variable]
    zdot[assembly.body_2.orientation_variable] =
        z[assembly.body_2.angular_velocity_variable]
    return z, zdot
end

function run_driven_base_double_pendulum(;
        theta_2 = deg2rad(-20.0), omega_2 = 0.0,
        tspan = (0.0, 2.0), atol = 1.0e-7, rtol = 1.0e-5,
        dt = 1.0e-6, dtmax = 0.02)
    system = ModularTwoBodySystem()
    assembly = driven_base_double_pendulum_assembly(system)
    z0, zdot0 = driven_base_double_pendulum_initial_conditions(
        theta_2, omega_2, system, assembly, first(tspan))
    prototype = driven_base_double_pendulum_jacobian(
        first(tspan), z0, zdot0, 1.0, assembly)
    variable_levels = getfield.(assembly.layout.catalog.variables, :level)
    equation_levels = getfield.(assembly.layout.catalog.equations, :level)
    differential = BitVector([index in
        (assembly.body_2.angular_velocity_variable,
         assembly.body_2.orientation_variable)
        for index in eachindex(z0)])
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol, rtol = rtol, initial_step = dt, maximum_step = dtmax)
    solution = HistoricalDDASSL.dassl(
        driven_base_double_pendulum_equations!, z0, zdot0, tspan;
        parameter = assembly,
        jacobian! = driven_base_double_pendulum_jacobian!,
        jacobian_prototype = prototype,
        options, variable_levels, equation_levels,
        differential_vars = differential, error_control = differential,
        deficit = 0)
    return solution, system, assembly
end

function driven_base_double_pendulum_diagnostics(solution, assembly)
    equation_errors = Float64[]
    position_errors = Float64[]
    motion_errors = Float64[]
    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(length(z))
        driven_base_double_pendulum_equations!(
            equations, t, z, zdot, assembly)
        push!(equation_errors, norm(equations, Inf))
        push!(position_errors, norm(equations[[
            assembly.pin_1.position_equations...,
            assembly.pin_2.position_equations...]], Inf))
        push!(motion_errors, abs(
            z[assembly.body_1.orientation_variable] -
            assembly.motion.motion(t)))
    end
    return (
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_position_constraint_error = maximum(position_errors),
        maximum_prescribed_angle_error = maximum(motion_errors),
        accepted_steps = solution.stats.accepted_steps,
        rejected_steps = solution.stats.rejected_steps,
        maximum_order = maximum(solution.orders),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    selection = driven_base_state_selection(deg2rad(20.0), deg2rad(-20.0))
    println("Driven-base state selection")
    println("  rank: ", selection.qr_selection.rank)
    println("  preferred independent: ",
        TWO_BODY_VELOCITY_NAMES[selection.preferred.independent])
    solution, _, assembly = run_driven_base_double_pendulum()
    println("Driven-base double pendulum")
    for (name, value) in pairs(
            driven_base_double_pendulum_diagnostics(solution, assembly))
        println("  ", name, ": ", value)
    end
end
