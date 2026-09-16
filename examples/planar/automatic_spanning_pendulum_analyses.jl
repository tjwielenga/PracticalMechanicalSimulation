using LinearAlgebra

if !isdefined(@__MODULE__, :spanning_spring_static_equations!)
    include(joinpath(@__DIR__, "spanning_spring_static_equilibrium.jl"))
end

function automatic_position_initialization(q_user, system;
                                           weights = nothing)
    selection = select_analysis(SPANNING_ANALYSIS_CATALOG, PositionIC())
    selected_names = [SPANNING_ANALYSIS_CATALOG.variables[i].name
                      for i in selection.variable_indices]
    selected_names == [:R_x, :R_y, :theta] ||
        error("Position analysis selected an unsupported variable layout")
    isnothing(weights) &&
        (weights = Diagonal([system.mechanical.mass,
                             system.mechanical.mass,
                             system.mechanical.inertia]))
    q = consistent_position(q_user, system.mechanical; weights)
    canonical = zeros(eltype(q), 20)
    expand_analysis_values!(canonical, q, selection)
    return canonical, selection
end

function automatic_velocity_initialization(v_user, canonical_position,
                                           system; weights = nothing)
    selection = select_analysis(SPANNING_ANALYSIS_CATALOG, VelocityIC())
    selected_names = [SPANNING_ANALYSIS_CATALOG.variables[i].name
                      for i in selection.variable_indices]
    selected_names == [:V_x, :V_y, :omega] ||
        error("Velocity analysis selected an unsupported variable layout")
    isnothing(weights) &&
        (weights = Diagonal([system.mechanical.mass,
                             system.mechanical.mass,
                             system.mechanical.inertia]))
    q = canonical_position[7:9]
    velocity = consistent_velocity(v_user, q, system.mechanical; weights)
    canonical = copy(canonical_position)
    expand_analysis_values!(canonical, velocity, selection)
    return canonical, selection
end

function selected_spanning_equations!(equations, analysis_values,
                                      canonical_context, selection, system)
    canonical = copy(canonical_context)
    expand_analysis_values!(canonical, analysis_values, selection)
    model = spanning_pendulum_executable_model(system)
    evaluate_analysis_equations!(equations, model, selection, 0.0,
        canonical, zeros(eltype(canonical), 20))
    return canonical
end

function selected_spanning_jacobian(analysis_values, canonical_context,
                                    selection, system)
    canonical = copy(canonical_context)
    expand_analysis_values!(canonical, analysis_values, selection)
    model = spanning_pendulum_executable_model(system)
    return evaluate_analysis_jacobian(model, selection, 0.0,
        canonical, zeros(eltype(canonical), 20), 0.0)
end

function automatic_acceleration_initialization(canonical_position_velocity,
                                               system;
                                               tolerance = 1.0e-11,
                                               maximum_iterations = 20)
    selection = select_analysis(SPANNING_ANALYSIS_CATALOG, AccelerationIC())
    canonical = copy(canonical_position_velocity)
    set_spanning_spring_variables!(canonical, system)
    analysis = analysis_values(canonical, selection)
    equations = zeros(eltype(canonical), length(selection.equation_indices))

    for iteration in 1:maximum_iterations
        canonical = selected_spanning_equations!(
            equations, analysis, canonical_position_velocity,
            selection, system)
        norm(equations, Inf) <= tolerance &&
            return canonical, selection, iteration - 1
        jacobian = selected_spanning_jacobian(
            analysis, canonical_position_velocity, selection, system)
        analysis .-= jacobian \ equations
    end
    error("Acceleration initialization did not converge")
end

function automatic_analysis_selections()
    return (
        position = select_analysis(SPANNING_ANALYSIS_CATALOG, PositionIC()),
        velocity = select_analysis(SPANNING_ANALYSIS_CATALOG, VelocityIC()),
        acceleration = select_analysis(
            SPANNING_ANALYSIS_CATALOG, AccelerationIC()),
        static = select_analysis(SPANNING_ANALYSIS_CATALOG, StaticEQ()),
        dynamics = select_analysis(SPANNING_ANALYSIS_CATALOG, Dynamics()),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    system = SpanningSpringPendulumSystem()
    q_user = [0.03, -0.02, deg2rad(35.0)]
    position_state, position_selection =
        automatic_position_initialization(q_user, system)
    velocity_state, velocity_selection =
        automatic_velocity_initialization(
            [0.1, -0.2, 0.7], position_state, system)
    acceleration_state, acceleration_selection, iterations =
        automatic_acceleration_initialization(velocity_state, system)
    static_state, _, static_iterations =
        solve_spanning_spring_static_equilibrium()

    println("Automatic spanning-pendulum analyses")
    for (name, selection) in pairs(automatic_analysis_selections())
        println("  ", name, ": ", length(selection.variable_indices),
            " variables, ", length(selection.equation_indices),
            " component equations")
    end
    println("  position constraint error: ",
        norm(position_constraint(position_state[7:9], system.mechanical), Inf))
    println("  velocity constraint error: ",
        norm(velocity_constraint(
            [velocity_state[7:9]; velocity_state[4:6]],
            system.mechanical), Inf))
    acceleration_equations = zeros(14)
    selected_spanning_equations!(acceleration_equations,
        analysis_values(acceleration_state, acceleration_selection),
        velocity_state, acceleration_selection, system)
    println("  acceleration equation error: ",
        norm(acceleration_equations, Inf))
    println("  acceleration Newton corrections: ", iterations)
    println("  static Newton corrections: ", static_iterations)
    println("  static angle: ", rad2deg(static_state[3]), " degrees")
end
