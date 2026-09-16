using LinearAlgebra

if !isdefined(@__MODULE__, :DrivenPendulumSystem)
    include(joinpath(@__DIR__, "rotational_motion_generator_pendulum.jl"))
end
if !isdefined(@__MODULE__, :PlanarModeling)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarModeling.jl"))
end
using .PlanarModeling

struct SliderCrankParameters{T}
    crank_length::T
    rod_length::T
    masses::NTuple{2,T}
    inertias::NTuple{2,T}
    gravity::Vector{T}
end

function SliderCrankParameters(; crank_length = 0.5,
        rod_length = 2crank_length, masses = (1.0, 2.0),
        gravity = [0.0, -9.81])
    T = promote_type(typeof(crank_length), typeof(rod_length),
        eltype(masses), eltype(gravity))
    lengths = T.((crank_length, rod_length))
    inertias = ntuple(i -> T(masses[i]) * lengths[i]^2 / 12, 2)
    return SliderCrankParameters(T(crank_length), T(rod_length),
        T.(masses), inertias, T.(gravity))
end

function constant_speed_slider_crank_assembly(
        parameters = SliderCrankParameters();
        initial_angle = 0.0, angular_velocity = 2pi / 5)
    registrations = (
        crank = planar_body_registration(:crank),
        rod = planar_body_registration(:rod),
        ground_pin = revolute_joint_registration(:ground_pin),
        crank_pin = revolute_joint_registration(:crank_pin),
        slider = inplane_constraint_registration(:slider),
        motion = rotational_motion_registration(:motion),
    )
    builder = ModelLayoutBuilder()
    for registration in registrations
        allocate_component_variables!(builder, registration)
    end
    for body in (registrations.crank, registrations.rod)
        allocate_component_equation_block!(builder, body, :balance)
    end
    for joint in (registrations.ground_pin, registrations.crank_pin,
                  registrations.slider)
        for block in (:acceleration, :velocity, :position)
            allocate_component_equation_block!(builder, joint, block)
        end
    end
    for block in (:acceleration, :velocity, :position)
        allocate_component_equation_block!(builder, registrations.motion, block)
    end
    layout = finish_layout(builder)

    crank = allocated_planar_body(layout, :crank,
        parameters.masses[1], parameters.inertias[1])
    rod = allocated_planar_body(layout, :rod,
        parameters.masses[2], parameters.inertias[2])
    bodies = (crank, rod)
    gravities = (
        PlanarGravityComponent(:gravity_crank, crank, parameters.gravity),
        PlanarGravityComponent(:gravity_rod, rod, parameters.gravity),
    )
    r = parameters.crank_length
    ell = parameters.rod_length
    origin = PlanarGroundPointMarker([0.0, 0.0])
    crank_ground = planar_point_marker(crank, [-r / 2, 0.0])
    crank_end = planar_point_marker(crank, [r / 2, 0.0])
    rod_crank = planar_point_marker(rod, [-ell / 2, 0.0])
    rod_slider = planar_point_marker(rod, [ell / 2, 0.0])
    ground_pin = allocated_revolute_joint(layout, :ground_pin,
        crank, crank_ground, nothing, origin)
    crank_pin = allocated_revolute_joint(layout, :crank_pin,
        crank, crank_end, rod, rod_crank)
    slider = allocated_inplane_constraint(layout, :slider,
        rod, rod_slider, nothing, origin,
        PlanarGroundOrientationMarker(0.0))

    orientation = PlanarBodyOrientationMarker(crank.orientation_variable,
        crank.angular_velocity_variable, crank.balance_equations[3], 0.0)
    unwrapped_angle(t) = initial_angle + angular_velocity * t
    prescribed_angle(t) = atan(sin(unwrapped_angle(t)), cos(unwrapped_angle(t)))
    prescribed_omega(t) = angular_velocity
    prescribed_alpha(t) = zero(t)
    motion = allocated_rotational_motion_generator(layout, :motion,
        crank, orientation, nothing, PlanarGroundOrientationMarker(0.0),
        prescribed_angle, prescribed_omega, prescribed_alpha)
    model = assemble_planar_model(layout,
        (bodies..., ground_pin, crank_pin, slider, motion),
        (gravities..., ground_pin, crank_pin, slider, motion))
    return (; bodies, crank, rod, ground_pin, crank_pin, slider, motion,
        gravities, layout, model)
end

function slider_crank_position_seed(parameters, crank_angle; branch = 1)
    branch in (-1, 1) || throw(ArgumentError("branch must be -1 or 1"))
    r = parameters.crank_length
    ell = parameters.rod_length
    crank_joint = r .* [cos(crank_angle), sin(crank_angle)]
    horizontal_span = sqrt(ell^2 - crank_joint[2]^2)
    slider = [crank_joint[1] + branch * horizontal_span, 0.0]
    return (
        crank = (center = crank_joint / 2, angle = crank_angle),
        rod = (center = (crank_joint + slider) / 2,
               angle = atan(slider[2] - crank_joint[2],
                            slider[1] - crank_joint[1])),
        joints = ([0.0, 0.0], crank_joint, slider),
    )
end

function seed_slider_crank!(z, assembly, parameters, t; branch = 1)
    geometry = slider_crank_position_seed(
        parameters, assembly.motion.motion(t); branch)
    for (body, pose) in zip(assembly.bodies, (geometry.crank, geometry.rod))
        z[body.position_variables] .= pose.center
        z[body.orientation_variable] = pose.angle
    end
    z[assembly.motion.angle_variable] = assembly.motion.motion(t)
    return geometry
end

function analyze_slider_crank!(z, assembly, t)
    return (
        position = solve_planar_analysis!(z, assembly,
            KinematicPosition(), t),
        velocity = solve_planar_analysis!(z, assembly,
            KinematicVelocity(), t),
        acceleration = solve_planar_analysis!(z, assembly,
            KinematicAcceleration(), t),
        forces = solve_planar_analysis!(z, assembly,
            KinematicForces(), t),
    )
end

function predict_slider_crank_position!(z, assembly, step)
    predict_planar_configuration!(z, assembly.bodies, step)
    z[assembly.motion.angle_variable] +=
        step * z[assembly.motion.angular_velocity_variable] +
        (step^2 / 2) * z[assembly.motion.angular_acceleration_variable]
    return z
end

function constant_speed_slider_crank_history(times;
        parameters = SliderCrankParameters(), branch = 1, kwargs...)
    isempty(times) && throw(ArgumentError("at least one time is required"))
    issorted(times) || throw(ArgumentError("times must be sorted"))
    assembly = constant_speed_slider_crank_assembly(parameters; kwargs...)
    z = zeros(Float64, length(assembly.layout.catalog.variables))
    seed_slider_crank!(z, assembly, parameters, first(times); branch)
    states = Vector{Vector{Float64}}(undef, length(times))
    corrections = Vector{NamedTuple}(undef, length(times))
    for (sample, time) in enumerate(times)
        sample > 1 && predict_slider_crank_position!(
            z, assembly, time - times[sample - 1])
        corrections[sample] = analyze_slider_crank!(z, assembly, time)
        states[sample] = copy(z)
    end
    return (; times = collect(times), states, corrections, assembly, parameters)
end

function slider_crank_diagnostics(history)
    assembly = history.assembly
    errors = Dict(name => Float64[] for name in
        (:position, :velocity, :acceleration, :forces))
    conditions = Dict(name => Float64[] for name in keys(errors))
    policies = ((:position, KinematicPosition()),
                (:velocity, KinematicVelocity()),
                (:acceleration, KinematicAcceleration()),
                (:forces, KinematicForces()))
    for (t, z) in zip(history.times, history.states)
        for (name, policy) in policies
            selection = select_analysis(assembly.layout.catalog, policy)
            equations = zeros(length(selection.equation_indices))
            derivative = zeros(length(z))
            evaluate_analysis_equations!(equations, assembly.model,
                selection, t, z, derivative)
            jacobian = evaluate_analysis_jacobian(assembly.model,
                selection, t, z, derivative, 0.0)
            push!(errors[name], norm(equations, Inf))
            push!(conditions[name], cond(jacobian))
        end
    end
    return (
        maximum_errors = Dict(name => maximum(values)
            for (name, values) in errors),
        maximum_conditions = Dict(name => maximum(values)
            for (name, values) in conditions),
        maximum_position_corrections =
            maximum(c.position for c in history.corrections),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    history = constant_speed_slider_crank_history(
        range(0.0, 5.0; length = 201))
    println("Constant-speed slider-crank")
    println("  simultaneous variables: ", length(first(history.states)))
    for (name, value) in pairs(slider_crank_diagnostics(history))
        println("  ", name, ": ", value)
    end
end
