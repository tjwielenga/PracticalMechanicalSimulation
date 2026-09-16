using LinearAlgebra

if !isdefined(@__MODULE__, :DrivenPendulumSystem)
    include(joinpath(@__DIR__, "rotational_motion_generator_pendulum.jl"))
end
if !isdefined(@__MODULE__, :PlanarModeling)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarModeling.jl"))
end
using .PlanarModeling

struct FourBarParameters{T}
    crank_length::T
    coupler_length::T
    rocker_length::T
    ground_length::T
    masses::NTuple{3,T}
    inertias::NTuple{3,T}
    gravity::Vector{T}
end

function FourBarParameters(; crank_length = 0.6, coupler_length = 1.4,
        rocker_length = 1.0, ground_length = 1.5,
        masses = (1.0, 1.5, 1.0), gravity = [0.0, -9.81])
    T = promote_type(typeof(crank_length), typeof(coupler_length),
        typeof(rocker_length), typeof(ground_length), eltype(masses),
        eltype(gravity))
    lengths = T.((crank_length, coupler_length, rocker_length))
    inertias = ntuple(i -> T(masses[i]) * lengths[i]^2 / 12, 3)
    return FourBarParameters(T(crank_length), T(coupler_length),
        T(rocker_length), T(ground_length), T.(masses), inertias,
        T.(gravity))
end

struct DrivenFourBarSystem{P,A}
    parameters::P
    assembly::A
end

function driven_four_bar_assembly(parameters = FourBarParameters();
        initial_angle = deg2rad(30.0), angular_velocity = 2pi / 5)
    body_names = (:crank, :coupler, :rocker)
    pin_names = (:ground_crank, :crank_coupler,
                 :coupler_rocker, :rocker_ground)
    registrations = Dict{Symbol,ComponentRegistration}()
    for name in body_names
        registrations[name] = planar_body_registration(name)
    end
    for name in pin_names
        registrations[name] = revolute_joint_registration(name)
    end
    registrations[:motion] = rotational_motion_registration(:motion)

    builder = ModelLayoutBuilder()
    for name in (body_names..., pin_names..., :motion)
        allocate_component_variables!(builder, registrations[name])
    end
    for name in body_names
        allocate_component_equation_block!(builder, registrations[name], :balance)
    end
    for name in pin_names
        for block in (:acceleration, :velocity, :position)
            allocate_component_equation_block!(builder, registrations[name], block)
        end
    end
    for block in (:acceleration, :velocity, :position)
        allocate_component_equation_block!(builder, registrations[:motion], block)
    end
    layout = finish_layout(builder)

    crank = allocated_planar_body(layout, :crank,
        parameters.masses[1], parameters.inertias[1])
    coupler = allocated_planar_body(layout, :coupler,
        parameters.masses[2], parameters.inertias[2])
    rocker = allocated_planar_body(layout, :rocker,
        parameters.masses[3], parameters.inertias[3])
    bodies = (crank, coupler, rocker)
    gravities = ntuple(i -> PlanarGravityComponent(
        Symbol(:gravity_, body_names[i]), bodies[i], parameters.gravity), 3)

    a = parameters.crank_length
    b = parameters.coupler_length
    c = parameters.rocker_length
    left_ground = PlanarGroundPointMarker([0.0, 0.0])
    right_ground = PlanarGroundPointMarker([parameters.ground_length, 0.0])
    crank_left = planar_point_marker(crank, [-a / 2, 0.0])
    crank_right = planar_point_marker(crank, [a / 2, 0.0])
    coupler_left = planar_point_marker(coupler, [-b / 2, 0.0])
    coupler_right = planar_point_marker(coupler, [b / 2, 0.0])
    rocker_left = planar_point_marker(rocker, [-c / 2, 0.0])
    rocker_right = planar_point_marker(rocker, [c / 2, 0.0])
    pins = (
        allocated_revolute_joint(layout, :ground_crank, crank, crank_left,
            nothing, left_ground),
        allocated_revolute_joint(layout, :crank_coupler, crank, crank_right,
            coupler, coupler_left),
        allocated_revolute_joint(layout, :coupler_rocker, coupler, coupler_right,
            rocker, rocker_left),
        allocated_revolute_joint(layout, :rocker_ground, rocker, rocker_right,
            nothing, right_ground),
    )

    crank_orientation = PlanarBodyOrientationMarker(
        crank.orientation_variable, crank.angular_velocity_variable,
        crank.balance_equations[3], 0.0)
    ground_orientation = PlanarGroundOrientationMarker(0.0)
    unwrapped_angle(t) = initial_angle + angular_velocity * t
    prescribed_angle(t) = atan(sin(unwrapped_angle(t)), cos(unwrapped_angle(t)))
    prescribed_omega(t) = angular_velocity
    prescribed_alpha(t) = zero(t)
    motion = allocated_rotational_motion_generator(layout, :motion,
        crank, crank_orientation, nothing, ground_orientation,
        prescribed_angle, prescribed_omega, prescribed_alpha)
    model = assemble_planar_model(layout, (bodies..., pins..., motion),
        (gravities..., pins..., motion))
    return (; bodies, crank, coupler, rocker, pins, motion, gravities,
        layout, model)
end

function four_bar_position_seed(parameters, crank_angle; branch = 1)
    branch in (-1, 1) || throw(ArgumentError("branch must be -1 or 1"))
    a = parameters.crank_length
    b = parameters.coupler_length
    c = parameters.rocker_length
    A = [0.0, 0.0]
    D = [parameters.ground_length, 0.0]
    B = A + a .* [cos(crank_angle), sin(crank_angle)]
    delta = D - B
    distance = norm(delta)
    abs(b - c) < distance < b + c ||
        throw(ArgumentError("the prescribed crank angle has no four-bar assembly"))
    direction = delta / distance
    along = (b^2 - c^2 + distance^2) / (2distance)
    height = sqrt(max(zero(along), b^2 - along^2))
    normal = [-direction[2], direction[1]]
    C = B + along .* direction + branch * height .* normal
    return (
        crank = (center = (A + B) / 2, angle = atan(B[2] - A[2], B[1] - A[1])),
        coupler = (center = (B + C) / 2, angle = atan(C[2] - B[2], C[1] - B[1])),
        rocker = (center = (C + D) / 2, angle = atan(D[2] - C[2], D[1] - C[1])),
        joints = (A, B, C, D),
    )
end

function seed_four_bar_position!(canonical, assembly, parameters, t;
                                 branch = 1)
    geometry = four_bar_position_seed(parameters, assembly.motion.motion(t);
        branch)
    for (body, pose) in zip(assembly.bodies,
            (geometry.crank, geometry.coupler, geometry.rocker))
        canonical[body.position_variables] .= pose.center
        canonical[body.orientation_variable] = pose.angle
    end
    canonical[assembly.motion.angle_variable] = assembly.motion.motion(t)
    return geometry
end

function analyze_four_bar!(canonical, assembly, t)
    return (
        position = solve_planar_analysis!(canonical, assembly,
            KinematicPosition(), t),
        velocity = solve_planar_analysis!(canonical, assembly,
            KinematicVelocity(), t),
        acceleration = solve_planar_analysis!(canonical, assembly,
            KinematicAcceleration(), t),
        forces = solve_planar_analysis!(canonical, assembly,
            KinematicForces(), t),
    )
end

function predict_four_bar_position!(canonical, assembly, step)
    predict_planar_configuration!(canonical, assembly.bodies, step)
    canonical[assembly.motion.angle_variable] +=
        step * canonical[assembly.motion.angular_velocity_variable] +
        (step^2 / 2) * canonical[assembly.motion.angular_acceleration_variable]
    return canonical
end

function driven_four_bar_state(t; parameters = FourBarParameters(),
        branch = 1, kwargs...)
    assembly = driven_four_bar_assembly(parameters; kwargs...)
    canonical = zeros(Float64, length(assembly.layout.catalog.variables))
    seed_four_bar_position!(canonical, assembly, parameters, t; branch)
    corrections = analyze_four_bar!(canonical, assembly, t)
    return canonical, DrivenFourBarSystem(parameters, assembly), corrections
end

function driven_four_bar_history(times; parameters = FourBarParameters(),
        branch = 1, kwargs...)
    isempty(times) && throw(ArgumentError("at least one time is required"))
    issorted(times) || throw(ArgumentError("times must be sorted"))
    assembly = driven_four_bar_assembly(parameters; kwargs...)
    canonical = zeros(Float64, length(assembly.layout.catalog.variables))
    seed_four_bar_position!(canonical, assembly, parameters, first(times); branch)
    states = Vector{Vector{Float64}}(undef, length(times))
    corrections = Vector{NamedTuple}(undef, length(times))
    for (sample, time) in enumerate(times)
        sample > 1 && predict_four_bar_position!(canonical, assembly,
            time - times[sample - 1])
        corrections[sample] = analyze_four_bar!(canonical, assembly, time)
        states[sample] = copy(canonical)
    end
    return (; times = collect(times), states, corrections,
        system = DrivenFourBarSystem(parameters, assembly))
end

function driven_four_bar_diagnostics(t; kwargs...)
    state, system, corrections = driven_four_bar_state(t; kwargs...)
    assembly = system.assembly
    errors = Dict{Symbol,Float64}()
    conditions = Dict{Symbol,Float64}()
    for (name, policy) in ((:position, KinematicPosition()),
                           (:velocity, KinematicVelocity()),
                           (:acceleration, KinematicAcceleration()),
                           (:forces, KinematicForces()))
        selection = select_analysis(assembly.layout.catalog, policy)
        equations = zeros(length(selection.equation_indices))
        derivative = zeros(length(state))
        evaluate_analysis_equations!(equations, assembly.model, selection,
            t, state, derivative)
        jacobian = evaluate_analysis_jacobian(assembly.model, selection,
            t, state, derivative, 0.0)
        errors[name] = norm(equations, Inf)
        conditions[name] = cond(jacobian)
    end
    return (; state, system, corrections, errors, conditions,
        driving_torque = state[assembly.motion.torque_variable])
end

if abspath(PROGRAM_FILE) == @__FILE__
    result = driven_four_bar_diagnostics(0.7)
    println("Driven planar four-bar")
    println("  simultaneous variables: ", length(result.state))
    println("  position/velocity/acceleration/force errors: ", result.errors)
    println("  analysis Jacobian condition estimates: ", result.conditions)
    println("  driving torque: ", result.driving_torque)
end
