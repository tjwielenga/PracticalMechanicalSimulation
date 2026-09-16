module PendulumChainBenchmark

using LinearAlgebra
using SparseArrays
using PracticalMechanicalSimulation

const Analysis = PracticalMechanicalSimulation.AutomaticAnalysis
const Assembly = PracticalMechanicalSimulation.PlanarComponentAssembly
const Forces = PracticalMechanicalSimulation.PlanarAppliedForces
const Runner = PracticalMechanicalSimulation.SimulationRunner

export pendulum_chain_toml, run_pendulum_chain, benchmark_pendulum_chains,
       parallelogram_chain_toml, run_parallelogram_chain,
       benchmark_parallelogram_chains, bouncing_ball_bank_toml,
       run_bouncing_ball_bank, benchmark_bouncing_ball_banks,
       rotor_train_toml, run_rotor_train, benchmark_rotor_trains

"""Generate a complete planar TOML model for an open serial pendulum chain."""
function pendulum_chain_toml(link_count::Integer;
        link_length = 0.2, initial_angle = 0.35,
        end_time = 0.25, output_samples = 11)
    link_count >= 1 || throw(ArgumentError("link_count must be positive"))
    link_length > 0 || throw(ArgumentError("link_length must be positive"))
    output_samples >= 1 || throw(ArgumentError(
        "output_samples must be positive"))
    digits = max(2, ndigits(link_count))
    link_name(index) = "link" * lpad(string(index), digits, '0')
    joint_name(index) = "joint" * lpad(string(index), digits, '0')
    mass = 1.0
    inertia = mass * link_length^2 / 12
    sine, cosine = sincos(initial_angle)
    io = IOBuffer()
    println(io, "[model]")
    println(io, "name = \"pendulum_chain_$(link_count)\"")
    println(io, "title = \"$(link_count)-link planar pendulum chain\"")
    println(io, "dimension = \"planar\"\n")
    println(io, "[analysis]")
    println(io, "mode = \"dynamic\"\n")
    println(io, "[simulation]")
    println(io, "start_time = 0.0")
    println(io, "end_time = ", repr(Float64(end_time)))
    println(io, "output_samples = ", output_samples)
    println(io, "relative_tolerance = 1.0e-7")
    println(io, "absolute_tolerance = 1.0e-9")
    println(io, "initial_step = 1.0e-6")
    println(io, "maximum_step = 0.005\n")
    println(io, "[ground]")
    println(io, "type = \"ground\"\n")
    println(io, "[ground.origin]")
    println(io, "type = \"marker\"\n")
    for index in 1:link_count
        name = link_name(index)
        center_distance = (index - 0.5) * link_length
        position = [sine * center_distance, -cosine * center_distance]
        println(io, "[$name]")
        println(io, "type = \"rigid_body\"")
        println(io, "mass = ", repr(mass))
        println(io, "inertia = ", repr(inertia))
        println(io, "position = [", repr(position[1]), ", ",
            repr(position[2]), "]")
        println(io, "angle = ", repr(Float64(initial_angle)), "\n")
        println(io, "[$name.top]")
        println(io, "type = \"marker\"")
        println(io, "position = [0.0, ", repr(link_length / 2), "]\n")
        println(io, "[$name.bottom]")
        println(io, "type = \"marker\"")
        println(io, "position = [0.0, ", repr(-link_length / 2), "]\n")
    end
    for index in 1:link_count
        name = joint_name(index)
        first_marker = "$(link_name(index)).top"
        second_marker = index == 1 ? "ground.origin" :
            "$(link_name(index - 1)).bottom"
        println(io, "[$name]")
        println(io, "type = \"revolute\"")
        println(io, "markers = [\"$first_marker\", \"$second_marker\"]\n")
    end
    println(io, "[gravity]")
    println(io, "type = \"gravity\"")
    println(io, "acceleration = [0.0, -9.81]")
    String(take!(io))
end

"""Generate a connected chain of closed-loop parallelogram four-bars."""
function parallelogram_chain_toml(cell_count::Integer;
        bay_width = 0.2, rocker_length = 0.2, initial_angle = 0.7,
        joint_damping = 0.01, end_time = 0.25, output_samples = 11)
    cell_count >= 1 || throw(ArgumentError("cell_count must be positive"))
    bay_width > 0 || throw(ArgumentError("bay_width must be positive"))
    rocker_length > 0 || throw(ArgumentError("rocker_length must be positive"))
    joint_damping >= 0 || throw(ArgumentError(
        "joint_damping must be nonnegative"))
    output_samples >= 1 || throw(ArgumentError(
        "output_samples must be positive"))
    digits = max(2, ndigits(cell_count))
    rocker_name(index) = "rocker" * lpad(string(index), digits, '0')
    coupler_name(index) = "coupler" * lpad(string(index), digits, '0')
    base_name(index) = "base" * lpad(string(index), digits, '0')
    cell_name(index) = "cell" * lpad(string(index), digits, '0')
    mass = 1.0
    rocker_inertia = mass * rocker_length^2 / 12
    coupler_inertia = mass * bay_width^2 / 12
    sine, cosine = sincos(initial_angle)
    io = IOBuffer()
    println(io, "[model]")
    println(io, "name = \"parallelogram_chain_$(cell_count)\"")
    println(io, "title = \"$(cell_count)-cell closed-loop parallelogram chain\"")
    println(io, "dimension = \"planar\"\n")
    println(io, "[analysis]")
    println(io, "mode = \"dynamic\"\n")
    println(io, "[state_selection]")
    println(io, "method = \"preferred\"")
    println(io, "preferred_velocities = [\"$(rocker_name(0)).omega\"]\n")
    println(io, "[simulation]")
    println(io, "start_time = 0.0")
    println(io, "end_time = ", repr(Float64(end_time)))
    println(io, "output_samples = ", output_samples)
    println(io, "relative_tolerance = 1.0e-7")
    println(io, "absolute_tolerance = 1.0e-9")
    println(io, "initial_step = 1.0e-6")
    println(io, "maximum_step = 0.005\n")
    println(io, "[ground]")
    println(io, "type = \"ground\"\n")
    for index in 0:cell_count
        println(io, "[ground.$(base_name(index))]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(index * bay_width), ", 0.0]\n")
    end
    for index in 0:cell_count
        name = rocker_name(index)
        position = [index * bay_width + rocker_length * cosine / 2,
                    rocker_length * sine / 2]
        println(io, "[$name]")
        println(io, "type = \"rigid_body\"")
        println(io, "mass = ", repr(mass))
        println(io, "inertia = ", repr(rocker_inertia))
        println(io, "position = [", repr(position[1]), ", ",
            repr(position[2]), "]")
        println(io, "angle = ", repr(Float64(initial_angle)), "\n")
        println(io, "[$name.base]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(-rocker_length / 2), ", 0.0]\n")
        println(io, "[$name.tip]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(rocker_length / 2), ", 0.0]\n")
    end
    for index in 1:cell_count
        name = coupler_name(index)
        position = [(index - 0.5) * bay_width + rocker_length * cosine,
                    rocker_length * sine]
        println(io, "[$name]")
        println(io, "type = \"rigid_body\"")
        println(io, "mass = ", repr(mass))
        println(io, "inertia = ", repr(coupler_inertia))
        println(io, "position = [", repr(position[1]), ", ",
            repr(position[2]), "]")
        println(io, "angle = 0.0\n")
        println(io, "[$name.left]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(-bay_width / 2), ", 0.0]\n")
        println(io, "[$name.right]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(bay_width / 2), ", 0.0]\n")
    end
    for index in 0:cell_count
        joint = base_name(index) * "_joint"
        rocker = rocker_name(index)
        base = base_name(index)
        println(io, "[$joint]")
        println(io, "type = \"revolute\"")
        println(io, "markers = [\"$rocker.base\", \"ground.$base\"]\n")
    end
    for index in 1:cell_count
        cell = cell_name(index)
        coupler = coupler_name(index)
        left_rocker = rocker_name(index - 1)
        right_rocker = rocker_name(index)
        println(io, "[$(cell)_left_joint]")
        println(io, "type = \"revolute\"")
        println(io, "markers = [\"$coupler.left\", \"$left_rocker.tip\"]\n")
        println(io, "[$(cell)_right_joint]")
        println(io, "type = \"revolute\"")
        println(io, "markers = [\"$coupler.right\", \"$right_rocker.tip\"]\n")
    end
    if joint_damping > 0
        for index in 0:cell_count
            damper = base_name(index) * "_damper"
            rocker = rocker_name(index)
            base = base_name(index)
            println(io, "[$damper]")
            println(io, "type = \"torsional_spring_damper\"")
            println(io, "markers = [\"$rocker.base\", \"ground.$base\"]")
            println(io, "stiffness = 0.0")
            println(io, "damping = ", repr(Float64(joint_damping)), "\n")
        end
    end
    println(io, "[gravity]")
    println(io, "type = \"gravity\"")
    println(io, "acceleration = [0.0, -9.81]")
    String(take!(io))
end

"""Generate independent balls with staggered impacts on one horizontal plane."""
function bouncing_ball_bank_toml(ball_count::Integer;
        radius = 0.1, stiffness = 1.0e4, damping_factor = 0.15,
        first_impact_time = 0.25, last_impact_time = 0.75,
        end_time = 1.5, output_samples = 301)
    ball_count >= 1 || throw(ArgumentError("ball_count must be positive"))
    radius > 0 || throw(ArgumentError("radius must be positive"))
    stiffness > 0 || throw(ArgumentError("stiffness must be positive"))
    damping_factor >= 0 || throw(ArgumentError(
        "damping_factor must be nonnegative"))
    0 < first_impact_time <= last_impact_time || throw(ArgumentError(
        "impact times must be positive and ordered"))
    output_samples >= 1 || throw(ArgumentError(
        "output_samples must be positive"))
    digits = max(2, ndigits(ball_count))
    ball_name(index) = "ball" * lpad(string(index), digits, '0')
    contact_name(index) = "contact" * lpad(string(index), digits, '0')
    impact_time(index) = ball_count == 1 ?
        (first_impact_time + last_impact_time) / 2 :
        first_impact_time + (index - 1) *
            (last_impact_time - first_impact_time) / (ball_count - 1)
    mass = 1.0
    inertia = mass * radius^2 / 2
    gravity = 9.81
    spacing = 3radius
    io = IOBuffer()
    println(io, "[model]")
    println(io, "name = \"bouncing_ball_bank_$(ball_count)\"")
    println(io, "title = \"$(ball_count)-ball staggered contact bank\"")
    println(io, "dimension = \"planar\"\n")
    println(io, "[analysis]")
    println(io, "mode = \"dynamic\"\n")
    println(io, "[simulation]")
    println(io, "start_time = 0.0")
    println(io, "end_time = ", repr(Float64(end_time)))
    println(io, "output_samples = ", output_samples)
    println(io, "relative_tolerance = 1.0e-7")
    println(io, "absolute_tolerance = 1.0e-9")
    println(io, "initial_step = 1.0e-6")
    println(io, "maximum_step = 0.002\n")
    println(io, "[ground]")
    println(io, "type = \"ground\"\n")
    println(io, "[ground.plane]")
    println(io, "type = \"marker\"\n")
    for index in 1:ball_count
        name = ball_name(index)
        x = (index - (ball_count + 1) / 2) * spacing
        time = impact_time(index)
        y = radius + gravity * time^2 / 2
        println(io, "[$name]")
        println(io, "type = \"rigid_body\"")
        println(io, "mass = ", repr(mass))
        println(io, "inertia = ", repr(inertia))
        println(io, "position = [", repr(Float64(x)), ", ",
            repr(Float64(y)), "]\n")
        println(io, "[$name.left]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(-radius), ", 0.0]\n")
        println(io, "[$name.right]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(radius), ", 0.0]\n")
        println(io, "[$name.center]")
        println(io, "type = \"marker\"\n")
    end
    for index in 1:ball_count
        println(io, "[$(contact_name(index))]")
        println(io, "type = \"plane_contact\"")
        println(io, "markers = [\"$(ball_name(index)).center\", \"ground.plane\"]")
        println(io, "radius = ", repr(Float64(radius)))
        println(io, "stiffness = ", repr(Float64(stiffness)))
        println(io, "damping_factor = ", repr(Float64(damping_factor)), "\n")
    end
    println(io, "[gravity]")
    println(io, "type = \"gravity\"")
    println(io, "acceleration = [0.0, -", repr(gravity), "]")
    String(take!(io))
end

"""Generate a fixed-center rotor train coupled by torsional spring-dampers."""
function rotor_train_toml(rotor_count::Integer;
        rotor_radius = 0.1, rotor_mass = 1.0, stiffness = 20.0,
        damping_time_scale = 0.001, initial_angle = 0.25,
        end_time = 1.0, output_samples = 201)
    rotor_count >= 1 || throw(ArgumentError("rotor_count must be positive"))
    rotor_radius > 0 || throw(ArgumentError("rotor_radius must be positive"))
    rotor_mass > 0 || throw(ArgumentError("rotor_mass must be positive"))
    stiffness > 0 || throw(ArgumentError("stiffness must be positive"))
    damping_time_scale >= 0 || throw(ArgumentError(
        "damping_time_scale must be nonnegative"))
    output_samples >= 1 || throw(ArgumentError(
        "output_samples must be positive"))
    digits = max(2, ndigits(rotor_count))
    rotor_name(index) = "rotor" * lpad(string(index), digits, '0')
    joint_name(index) = "bearing" * lpad(string(index), digits, '0')
    ground_name(index) = "center" * lpad(string(index), digits, '0')
    coupling_name(index) = "coupling" * lpad(string(index), digits, '0')
    inertia = rotor_mass * rotor_radius^2 / 2
    spacing = 3rotor_radius
    io = IOBuffer()
    println(io, "[model]")
    println(io, "name = \"rotor_train_$(rotor_count)\"")
    println(io, "title = \"$(rotor_count)-rotor torsional vibration train\"")
    println(io, "dimension = \"planar\"\n")
    println(io, "[analysis]")
    println(io, "mode = \"dynamic\"\n")
    println(io, "[simulation]")
    println(io, "start_time = 0.0")
    println(io, "end_time = ", repr(Float64(end_time)))
    println(io, "output_samples = ", output_samples)
    println(io, "relative_tolerance = 1.0e-7")
    println(io, "absolute_tolerance = 1.0e-9")
    println(io, "initial_step = 1.0e-7")
    println(io, "maximum_step = 0.002\n")
    println(io, "[ground]")
    println(io, "type = \"ground\"\n")
    for index in 1:rotor_count
        x = (index - 1) * spacing
        println(io, "[ground.$(ground_name(index))]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(Float64(x)), ", 0.0]\n")
    end
    for index in 1:rotor_count
        name = rotor_name(index)
        x = (index - 1) * spacing
        angle = index == rotor_count ? initial_angle : 0.0
        println(io, "[$name]")
        println(io, "type = \"rigid_body\"")
        println(io, "mass = ", repr(Float64(rotor_mass)))
        println(io, "inertia = ", repr(Float64(inertia)))
        println(io, "position = [", repr(Float64(x)), ", 0.0]")
        println(io, "angle = ", repr(Float64(angle)), "\n")
        println(io, "[$name.center]")
        println(io, "type = \"marker\"\n")
        println(io, "[$name.right]")
        println(io, "type = \"marker\"")
        println(io, "position = [", repr(Float64(rotor_radius)), ", 0.0]\n")
        println(io, "[$name.top]")
        println(io, "type = \"marker\"")
        println(io, "position = [0.0, ", repr(Float64(rotor_radius)), "]\n")
        joint = joint_name(index)
        println(io, "[$joint]")
        println(io, "type = \"revolute\"")
        println(io, "markers = [\"$name.center\", \"ground.$(ground_name(index))\"]\n")
    end
    println(io, "[ground_coupling]")
    println(io, "type = \"torsional_spring_damper\"")
    println(io, "markers = [\"$(rotor_name(1)).center\", \"ground.$(ground_name(1))\"]")
    println(io, "stiffness = ", repr(Float64(stiffness)))
    println(io, "damping_time_scale = ", repr(Float64(damping_time_scale)))
    println(io, "free_angle = 0.0\n")
    for index in 2:rotor_count
        println(io, "[$(coupling_name(index - 1))]")
        println(io, "type = \"torsional_spring_damper\"")
        println(io, "markers = [\"$(rotor_name(index)).center\", \"$(rotor_name(index - 1)).center\"]")
        println(io, "stiffness = ", repr(Float64(stiffness)))
        println(io, "damping_time_scale = ", repr(Float64(damping_time_scale)))
        println(io, "free_angle = 0.0\n")
    end
    println(io, "[state_selection]")
    println(io, "method = \"preferred\"")
    preferred = ["\"$(rotor_name(index)).omega\""
        for index in 1:rotor_count]
    println(io, "preferred_velocities = [", join(preferred, ", "), "]")
    println(io, "allow_fallback = false")
    String(take!(io))
end

"""Run one generated chain through the supported TOML modeling interface."""
function run_pendulum_chain(link_count::Integer;
        duration = 0.25, samples = 11)
    source = pendulum_chain_toml(link_count;
        end_time = duration, output_samples = samples)
    run_planar_model(IOBuffer(source); duration, samples)
end

"""Run one generated closed-loop parallelogram chain."""
function run_parallelogram_chain(cell_count::Integer;
        duration = 0.25, samples = 11)
    source = parallelogram_chain_toml(cell_count;
        end_time = duration, output_samples = samples)
    run_planar_model(IOBuffer(source); duration, samples)
end

"""Run one generated bank of staggered bouncing balls."""
function run_bouncing_ball_bank(ball_count::Integer;
        duration = 1.5, samples = 301)
    source = bouncing_ball_bank_toml(ball_count;
        end_time = duration, output_samples = samples)
    run_planar_model(IOBuffer(source); duration, samples)
end

"""Run one generated torsional rotor train."""
function run_rotor_train(rotor_count::Integer;
        duration = 1.0, samples = 201)
    source = rotor_train_toml(rotor_count;
        end_time = duration, output_samples = samples)
    run_planar_model(IOBuffer(source); duration, samples)
end

function maximum_joint_position_error(result)
    maximum((norm(
        Forces.point_marker_kinematics(joint.marker_a, state).position -
        Forces.point_marker_kinematics(joint.marker_b, state).position)
        for state in result.states
        for joint in values(result.loaded.connections)
        if joint isa Assembly.PlanarRevoluteJointComponent); init = 0.0)
end

function jacobian_nonzeros(result)
    loaded = result.loaded
    selection = Analysis.AnalysisSelection(Analysis.Dynamics(),
        loaded.active_variable_indices, loaded.active_equation_indices)
    state = first(result.states)
    derivative = Runner.initial_derivative(state, loaded)
    matrix = Analysis.evaluate_analysis_sparse_jacobian(
        loaded.model, selection, first(result.times), state, derivative, 1.0)
    nnz(matrix)
end

function benchmark_source(source)
    GC.gc()
    load_measurement = @timed load_planar_model(IOBuffer(source))
    loaded = load_measurement.value
    GC.gc()
    run_measurement = @timed run_planar_model(IOBuffer(source))
    result = run_measurement.value
    statistics = result.solution.stats
    (; variables = length(loaded.layout.catalog.variables),
       equations = length(loaded.layout.catalog.equations),
       states = loaded.analysis.degrees_of_freedom,
       jacobian_nonzeros = jacobian_nonzeros(result),
       load_seconds = load_measurement.time,
       load_mebibytes = load_measurement.bytes / 2.0^20,
       run_seconds = run_measurement.time,
       run_mebibytes = run_measurement.bytes / 2.0^20,
       variables_per_wall_second =
           length(loaded.layout.catalog.variables) / run_measurement.time,
       run_gc_seconds = run_measurement.gctime,
       run_gc_percent = 100run_measurement.gctime / run_measurement.time,
       accepted_steps = statistics.accepted_steps,
       rejected_steps = statistics.rejected_steps,
       residual_evaluations = statistics.residual_evaluations,
       jacobian_evaluations = statistics.jacobian_evaluations,
       factorizations = statistics.factorizations,
       symbolic_factorizations = statistics.symbolic_factorizations,
       newton_iterations = statistics.newton_iterations,
       corrector_failures = statistics.corrector_failures,
       maximum_order = maximum(result.solution.orders),
       maximum_joint_error = maximum_joint_position_error(result),
       result)
end

function benchmark_case(link_count; duration, samples)
    source = pendulum_chain_toml(link_count;
        end_time = duration, output_samples = samples)
    merge((; links = link_count), benchmark_source(source))
end

function parallelogram_benchmark_case(cell_count; duration, samples)
    source = parallelogram_chain_toml(cell_count;
        end_time = duration, output_samples = samples)
    merge((; cells = cell_count), benchmark_source(source))
end

function maximum_sampled_penetration(result)
    contacts = [component for components in values(result.loaded.forces)
        for component in components
        if component isa Assembly.PlanarPlaneContactComponent]
    maximum((max(-state[contact.gap_variable], 0.0)
        for state in result.states for contact in contacts); init = 0.0)
end

function contact_benchmark_case(ball_count; duration, samples)
    source = bouncing_ball_bank_toml(ball_count;
        end_time = duration, output_samples = samples)
    common = benchmark_source(source)
    statistics = common.result.solution.stats
    events = statistics.events_found
    merge((; balls = ball_count), common,
        (; root_evaluations = statistics.root_evaluations,
           events_found = events,
           history_restarts = statistics.history_restarts,
           milliseconds_per_event = events == 0 ? Inf :
               1.0e3 * common.run_seconds / events,
           maximum_sampled_penetration =
               maximum_sampled_penetration(common.result)))
end

function rotor_train_matrices(rotor_count;
        inertia = 0.005, stiffness = 20.0,
        damping_time_scale = 0.001)
    K = zeros(Float64, rotor_count, rotor_count)
    K[1, 1] += stiffness
    for index in 2:rotor_count
        K[index - 1, index - 1] += stiffness
        K[index, index] += stiffness
        K[index - 1, index] -= stiffness
        K[index, index - 1] -= stiffness
    end
    M = Diagonal(fill(Float64(inertia), rotor_count))
    C = damping_time_scale .* K
    (; M, C, K)
end

function rotor_train_reference(rotor_count, times;
        inertia = 0.005, stiffness = 20.0,
        damping_time_scale = 0.001, initial_angle = 0.25)
    matrices = rotor_train_matrices(rotor_count;
        inertia, stiffness, damping_time_scale)
    decomposition = eigen(Symmetric(matrices.K ./ inertia))
    squared_frequencies = decomposition.values
    modes = decomposition.vectors
    initial = zeros(Float64, rotor_count)
    initial[end] = initial_angle
    modal_initial = transpose(modes) * initial
    angles = zeros(Float64, length(times), rotor_count)
    angular_velocities = similar(angles)
    for (row, time) in enumerate(times)
        modal_angles = similar(modal_initial)
        modal_velocities = similar(modal_initial)
        for index in eachindex(squared_frequencies)
            lambda = squared_frequencies[index]
            decay = damping_time_scale * lambda / 2
            damped_frequency = sqrt(max(lambda - decay^2, 0.0))
            amplitude = modal_initial[index]
            exponential = exp(-decay * time)
            cosine = cos(damped_frequency * time)
            sine = sin(damped_frequency * time)
            ratio = decay / damped_frequency
            modal_angles[index] = amplitude * exponential *
                (cosine + ratio * sine)
            modal_velocities[index] = -amplitude * exponential *
                (damped_frequency + decay^2 / damped_frequency) * sine
        end
        angles[row, :] .= modes * modal_angles
        angular_velocities[row, :] .= modes * modal_velocities
    end
    frequencies = sqrt.(squared_frequencies)
    (; angles, angular_velocities, frequencies, matrices)
end

function rotor_train_diagnostics(result)
    count = length(result.loaded.bodies)
    reference = rotor_train_reference(count, result.times)
    body_names = sort!(collect(keys(result.loaded.bodies)); by=String)
    angles = zeros(Float64, length(result.times), count)
    angular_velocities = similar(angles)
    for (column, name) in enumerate(body_names)
        body = result.loaded.bodies[name]
        for row in eachindex(result.states)
            angles[row, column] =
                result.states[row][body.orientation_variable]
            angular_velocities[row, column] =
                result.states[row][body.angular_velocity_variable]
        end
    end
    final_angles = view(angles, size(angles, 1), :)
    final_velocities = view(angular_velocities,
        size(angular_velocities, 1), :)
    initial_angles = view(angles, 1, :)
    initial_velocities = view(angular_velocities, 1, :)
    energy(theta, omega) =
        dot(omega, reference.matrices.M * omega) / 2 +
        dot(theta, reference.matrices.K * theta) / 2
    initial_energy = energy(initial_angles, initial_velocities)
    final_energy = energy(final_angles, final_velocities)
    (; maximum_angle_error = maximum(abs,
           angles - reference.angles),
       maximum_angular_velocity_error = maximum(abs,
           angular_velocities - reference.angular_velocities),
       minimum_frequency = first(reference.frequencies),
       maximum_frequency = last(reference.frequencies),
       final_energy_ratio = final_energy / initial_energy)
end

function rotor_benchmark_case(rotor_count; duration, samples)
    source = rotor_train_toml(rotor_count;
        end_time = duration, output_samples = samples)
    common = benchmark_source(source)
    merge((; rotors = rotor_count), common,
        rotor_train_diagnostics(common.result))
end

function compact(value; digits = 4)
    string(round(value; sigdigits = digits))
end

function print_results(results; size_name = :links)
    println("| ", size_name, " | variables | states | Jacobian nnz | load s/MiB | run s/MiB | variables/wall s | GC % | accepted/rejected/corrector failures | residual evals | Jacobians/numeric LU/symbolic LU | Newton iterations | max order | max joint error |")
    println("| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in results
        println("| ", getproperty(row, size_name), " | ", row.variables,
            " | ", row.states,
            " | ", row.jacobian_nonzeros, " | ", compact(row.load_seconds),
            "/", compact(row.load_mebibytes), " | ",
            compact(row.run_seconds), "/", compact(row.run_mebibytes), " | ",
            compact(row.variables_per_wall_second), " | ",
            compact(row.run_gc_percent), " | ", row.accepted_steps, "/",
            row.rejected_steps, "/", row.corrector_failures, " | ",
            row.residual_evaluations, " | ", row.jacobian_evaluations, "/",
            row.factorizations, "/", row.symbolic_factorizations, " | ",
            row.newton_iterations, " | ",
            row.maximum_order, " | ", compact(row.maximum_joint_error), " |")
    end
end

function print_contact_results(results)
    println("| balls | variables | Jacobian nnz | run s/MiB | variables/wall s | accepted/rejected/failures | root evaluations | events/restarts | ms/event | max sampled penetration |")
    println("| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in results
        println("| ", row.balls, " | ", row.variables, " | ",
            row.jacobian_nonzeros, " | ", compact(row.run_seconds), "/",
            compact(row.run_mebibytes), " | ",
            compact(row.variables_per_wall_second), " | ",
            row.accepted_steps, "/", row.rejected_steps, "/",
            row.corrector_failures, " | ", row.root_evaluations, " | ",
            row.events_found, "/", row.history_restarts, " | ",
            compact(row.milliseconds_per_event), " | ",
            compact(row.maximum_sampled_penetration), " |")
    end
end

function print_rotor_results(results)
    println("| rotors | variables | states | Jacobian nnz | run s/MiB | variables/wall s | accepted/rejected/failures | Jacobians/numeric/symbolic | max angle error | max omega error | frequency range | final energy ratio |")
    println("| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in results
        println("| ", row.rotors, " | ", row.variables, " | ", row.states,
            " | ", row.jacobian_nonzeros, " | ",
            compact(row.run_seconds), "/", compact(row.run_mebibytes), " | ",
            compact(row.variables_per_wall_second), " | ",
            row.accepted_steps, "/", row.rejected_steps, "/",
            row.corrector_failures, " | ", row.jacobian_evaluations, "/",
            row.factorizations, "/", row.symbolic_factorizations, " | ",
            compact(row.maximum_angle_error), " | ",
            compact(row.maximum_angular_velocity_error), " | ",
            compact(row.minimum_frequency), "--",
            compact(row.maximum_frequency), " | ",
            compact(row.final_energy_ratio), " |")
    end
end

"""
Warm the model path, then benchmark the requested chain sizes once each.

The complete run timing includes a fresh TOML load. Compilation is excluded by
the two-link warm-up. Returned rows retain each result for further inspection.
"""
function benchmark_pendulum_chains(link_counts = (10, 25, 50);
        duration = 0.25, samples = 11)
    run_pendulum_chain(2; duration = 0.02, samples = 2)
    load_planar_model(IOBuffer(pendulum_chain_toml(2)))
    results = [benchmark_case(count; duration, samples)
        for count in link_counts]
    print_results(results)
    results
end

"""
Warm the model path, then benchmark connected closed-loop linkage chains.

Each cell is a parallelogram four-bar sharing one ground-pivoted rocker with
its neighbor. The complete mechanism therefore has one degree of freedom.
"""
function benchmark_parallelogram_chains(cell_counts = (10, 25, 50);
        duration = 0.25, samples = 11)
    run_parallelogram_chain(1; duration = 0.02, samples = 2)
    load_planar_model(IOBuffer(parallelogram_chain_toml(1)))
    results = [parallelogram_benchmark_case(count; duration, samples)
        for count in cell_counts]
    print_results(results; size_name = :cells)
    results
end

"""Warm the contact path, then benchmark staggered bouncing-ball banks."""
function benchmark_bouncing_ball_banks(ball_counts = (10, 25, 50);
        duration = 1.5, samples = 301)
    run_bouncing_ball_bank(1; duration = 1.5, samples = 5)
    load_planar_model(IOBuffer(bouncing_ball_bank_toml(1)))
    results = [contact_benchmark_case(count; duration, samples)
        for count in ball_counts]
    print_contact_results(results)
    results
end

"""Warm the smooth stiff path, then benchmark torsional rotor trains."""
function benchmark_rotor_trains(rotor_counts = (10, 25, 50);
        duration = 1.0, samples = 201)
    run_rotor_train(2; duration = 0.02, samples = 3)
    load_planar_model(IOBuffer(rotor_train_toml(2)))
    results = [rotor_benchmark_case(count; duration, samples)
        for count in rotor_counts]
    print_rotor_results(results)
    results
end

end
