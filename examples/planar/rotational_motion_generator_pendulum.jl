using LinearAlgebra

if !isdefined(@__MODULE__, :SpanningSpringPendulumSystem)
    include(joinpath(@__DIR__, "spanning_spring_damper_pendulum.jl"))
end
if !isdefined(@__MODULE__, :AutomaticAnalysis)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "AutomaticAnalysis.jl"))
end
if !isdefined(@__MODULE__, :PlanarComponentAssembly)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarComponentAssembly.jl"))
end
using .AutomaticAnalysis
using .PlanarAppliedForces
using .PlanarComponentAssembly

struct DrivenPendulumSystem{P,M}
    mechanical::P
    model::M
end

function driven_pendulum_assembly(mechanical;
        initial_angle = deg2rad(20.0), amplitude = deg2rad(25.0),
        frequency = 2.0)
    body_registration = planar_body_registration(:body)
    pin_registration = revolute_joint_registration(:pin)
    motion_registration = rotational_motion_registration(:motion)
    builder = ModelLayoutBuilder()
    for registration in (body_registration, pin_registration,
                         motion_registration)
        allocate_component_variables!(builder, registration)
    end
    allocate_component_equation_block!(builder, body_registration, :balance)
    for block in (:acceleration, :velocity, :position)
        allocate_component_equation_block!(builder, pin_registration, block)
    end
    for block in (:acceleration, :velocity, :position)
        allocate_component_equation_block!(builder, motion_registration, block)
    end
    layout = finish_layout(builder)

    body_variables = component_variable_indices(layout, :body)
    body = PlanarRigidBodyComponent(:body, mechanical.mass,
        mechanical.inertia, body_variables[1:2], body_variables[3],
        body_variables[4:5], body_variables[6], body_variables[7:8],
        body_variables[9],
        component_equation_indices(layout, :body, :balance), 1:0)
    gravity = PlanarGravityComponent(:gravity, body,
        collect(mechanical.gravity))

    point_marker = PlanarBodyPointMarker(body.position_variables,
        body.orientation_variable, body.velocity_variables,
        body.angular_velocity_variable, body.balance_equations[1:2],
        body.balance_equations[3], collect(mechanical.r_body))
    ground_point = PlanarGroundPointMarker(collect(mechanical.pin))
    pin = PlanarRevoluteJointComponent(:pin, body, point_marker,
        nothing, ground_point, component_variable_indices(layout, :pin),
        component_equation_indices(layout, :pin, :acceleration),
        component_equation_indices(layout, :pin, :velocity),
        component_equation_indices(layout, :pin, :position))

    orientation_marker = PlanarBodyOrientationMarker(
        body.orientation_variable, body.angular_velocity_variable,
        body.balance_equations[3], 0.0)
    ground_orientation = PlanarGroundOrientationMarker(0.0)
    motion_variables = component_variable_indices(layout, :motion)
    prescribed_angle(t) = initial_angle + amplitude * sin(frequency * t)
    prescribed_omega(t) = amplitude * frequency * cos(frequency * t)
    prescribed_alpha(t) = -amplitude * frequency^2 * sin(frequency * t)
    motion = PlanarRotationalMotionGenerator(:motion,
        body, orientation_marker, nothing, ground_orientation,
        motion_variables[1], motion_variables[2], motion_variables[3],
        motion_variables[4],
        component_equation_indices(layout, :motion, :position),
        component_equation_indices(layout, :motion, :velocity),
        component_equation_indices(layout, :motion, :acceleration),
        prescribed_angle, prescribed_omega, prescribed_alpha)

    blocks = ExecutableEquationBlock[]
    for component in (body, pin, motion)
        component === body ? append!(blocks, executable_blocks(body)[1:1]) :
            append!(blocks, executable_blocks(component))
    end
    contributions = EquationContribution[]
    for component in (gravity, pin, motion)
        append!(contributions, equation_contributions(component))
    end
    model = ExecutableAnalysisModel(layout.catalog, blocks, contributions)
    return (; body, gravity, pin, motion, layout, model)
end

function solve_kinematic_analysis!(canonical, assembly, policy, t;
                                   tolerance = 1.0e-12,
                                   maximum_iterations = 12)
    selection = select_analysis(assembly.layout.catalog, policy)
    values = analysis_values(canonical, selection)
    derivative = zeros(eltype(canonical), length(canonical))
    corrections = 0
    for iteration in 1:maximum_iterations
        expand_analysis_values!(canonical, values, selection)
        equations = zeros(eltype(canonical), length(selection.equation_indices))
        evaluate_analysis_equations!(equations, assembly.model, selection,
            t, canonical, derivative)
        norm(equations, Inf) <= tolerance && break
        jacobian = evaluate_analysis_jacobian(assembly.model, selection,
            t, canonical, derivative, 0.0)
        values .-= jacobian \ equations
        corrections = iteration
    end
    expand_analysis_values!(canonical, values, selection)
    equations = zeros(eltype(canonical), length(selection.equation_indices))
    evaluate_analysis_equations!(equations, assembly.model, selection,
        t, canonical, derivative)
    norm(equations, Inf) <= tolerance ||
        error("Kinematic analysis failed to converge")
    return corrections
end

function driven_pendulum_state(t; mechanical = PendulumParameters(), kwargs...)
    assembly = driven_pendulum_assembly(mechanical; kwargs...)
    canonical = zeros(Float64, length(assembly.layout.catalog.variables))
    canonical[assembly.body.orientation_variable] = assembly.motion.motion(t)
    canonical[assembly.motion.angle_variable] = assembly.motion.motion(t)
    corrections = (
        position = solve_kinematic_analysis!(canonical, assembly,
            KinematicPosition(), t),
        velocity = solve_kinematic_analysis!(canonical, assembly,
            KinematicVelocity(), t),
        acceleration = solve_kinematic_analysis!(canonical, assembly,
            KinematicAcceleration(), t),
        forces = solve_kinematic_analysis!(canonical, assembly,
            KinematicForces(), t),
    )
    return canonical, DrivenPendulumSystem(mechanical, assembly), corrections
end

function driven_pendulum_diagnostics(t; kwargs...)
    state, system, corrections = driven_pendulum_state(t; kwargs...)
    assembly = system.model
    errors = Dict{Symbol,Float64}()
    for (name, policy) in ((:position, KinematicPosition()),
                           (:velocity, KinematicVelocity()),
                           (:acceleration, KinematicAcceleration()),
                           (:forces, KinematicForces()))
        selection = select_analysis(assembly.layout.catalog, policy)
        equations = zeros(length(selection.equation_indices))
        evaluate_analysis_equations!(equations, assembly.model, selection,
            t, state, zeros(length(state)))
        errors[name] = norm(equations, Inf)
    end
    return (; state, system, corrections, errors)
end

if abspath(PROGRAM_FILE) == @__FILE__
    result = driven_pendulum_diagnostics(0.7)
    println("Rotational motion-generator pendulum")
    println("  simultaneous variables: ", length(result.state))
    println("  position/velocity/acceleration/force errors: ", result.errors)
    println("  motion torque: ", result.state[result.system.model.motion.torque_variable])
end
