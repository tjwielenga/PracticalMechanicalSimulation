using LinearAlgebra
using SparseArrays

if !isdefined(@__MODULE__, :FourBarParameters)
    include(joinpath(@__DIR__, "driven_planar_four_bar.jl"))
end

function torque_driven_four_bar_assembly(parameters = FourBarParameters();
        applied_torque = 10.0)
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
    builder = ModelLayoutBuilder()
    for name in (body_names..., pin_names...)
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
    allocate_component_equation_block!(builder,
        registrations[:crank], :selected_state)
    layout = finish_layout(builder)

    crank = allocated_planar_body(layout, :crank,
        parameters.masses[1], parameters.inertias[1]; selected_state = true)
    coupler = allocated_planar_body(layout, :coupler,
        parameters.masses[2], parameters.inertias[2])
    rocker = allocated_planar_body(layout, :rocker,
        parameters.masses[3], parameters.inertias[3])
    bodies = (crank, coupler, rocker)
    gravities = ntuple(i -> PlanarGravityComponent(
        Symbol(:gravity_, body_names[i]), bodies[i], parameters.gravity), 3)

    a, b, c = (parameters.crank_length, parameters.coupler_length,
               parameters.rocker_length)
    left_ground = PlanarGroundPointMarker([0.0, 0.0])
    right_ground = PlanarGroundPointMarker([parameters.ground_length, 0.0])
    markers = (
        planar_point_marker(crank, [-a / 2, 0.0]),
        planar_point_marker(crank, [a / 2, 0.0]),
        planar_point_marker(coupler, [-b / 2, 0.0]),
        planar_point_marker(coupler, [b / 2, 0.0]),
        planar_point_marker(rocker, [-c / 2, 0.0]),
        planar_point_marker(rocker, [c / 2, 0.0]),
    )
    pins = (
        allocated_revolute_joint(layout, :ground_crank, crank, markers[1],
            nothing, left_ground),
        allocated_revolute_joint(layout, :crank_coupler, crank, markers[2],
            coupler, markers[3]),
        allocated_revolute_joint(layout, :coupler_rocker, coupler, markers[4],
            rocker, markers[5]),
        allocated_revolute_joint(layout, :rocker_ground, rocker, markers[6],
            nothing, right_ground),
    )
    crank_orientation = PlanarBodyOrientationMarker(
        crank.orientation_variable, crank.angular_velocity_variable,
        crank.balance_equations[3], 0.0)
    torque = PlanarConstantTorqueComponent(:applied_torque,
        crank_orientation, PlanarGroundOrientationMarker(0.0), applied_torque)

    model = assemble_planar_model(layout, (bodies..., pins...),
        (gravities..., pins..., torque))
    return (; bodies, crank, coupler, rocker, pins, torque, gravities,
        layout, model)
end

function torque_four_bar_initial_conditions(assembly, parameters;
        theta = deg2rad(30.0), omega = 0.0, branch = 1)
    z = zeros(Float64, length(assembly.layout.catalog.variables))
    geometry = four_bar_position_seed(parameters, theta; branch)
    for (body, pose) in zip(assembly.bodies,
            (geometry.crank, geometry.coupler, geometry.rocker))
        z[body.position_variables] .= pose.center
        z[body.orientation_variable] = pose.angle
    end
    z[assembly.crank.angular_velocity_variable] = omega

    position_variables = Int[]
    velocity_variables = Int[]
    for body in assembly.bodies
        append!(position_variables, body.position_variables)
        push!(position_variables, body.orientation_variable)
        append!(velocity_variables, body.velocity_variables)
        push!(velocity_variables, body.angular_velocity_variable)
    end
    position_variables = setdiff(position_variables,
        [assembly.crank.orientation_variable])
    velocity_variables = setdiff(velocity_variables,
        [assembly.crank.angular_velocity_variable])
    position_equations = vcat((collect(pin.position_equations)
        for pin in assembly.pins)...)
    velocity_equations = vcat((collect(pin.velocity_equations)
        for pin in assembly.pins)...)
    solve_planar_analysis!(z, assembly.model,
        AnalysisSelection(KinematicPosition(), position_variables,
            position_equations), 0.0)
    solve_planar_analysis!(z, assembly.model,
        AnalysisSelection(KinematicVelocity(), velocity_variables,
            velocity_equations), 0.0)

    acceleration_variables = Int[]
    acceleration_equations = Int[]
    for body in assembly.bodies
        append!(acceleration_variables, body.acceleration_variables)
        push!(acceleration_variables, body.angular_acceleration_variable)
        append!(acceleration_equations, body.balance_equations)
    end
    for pin in assembly.pins
        append!(acceleration_variables, pin.reaction_variables)
        append!(acceleration_equations, pin.acceleration_equations)
    end
    solve_planar_analysis!(z, assembly.model,
        AnalysisSelection(AccelerationIC(), acceleration_variables,
            acceleration_equations), 0.0)
    zdot = zeros(length(z))
    zdot[assembly.crank.angular_velocity_variable] =
        z[assembly.crank.angular_acceleration_variable]
    zdot[assembly.crank.orientation_variable] =
        z[assembly.crank.angular_velocity_variable]
    return z, zdot
end

function torque_driven_four_bar_equations!(equations, t, z, zdot, assembly)
    selection = select_analysis(assembly.layout.catalog, Dynamics())
    evaluate_analysis_equations!(equations, assembly.model, selection,
        t, z, zdot)
    return nothing
end

function torque_driven_four_bar_jacobian(t, z, zdot, coefficient, assembly)
    selection = select_analysis(assembly.layout.catalog, Dynamics())
    evaluate_analysis_sparse_jacobian(assembly.model, selection,
        t, z, zdot, coefficient)
end

function torque_driven_four_bar_jacobian!(matrix::SparseMatrixCSC,
        t, z, zdot, coefficient, assembly)
    selection = select_analysis(assembly.layout.catalog, Dynamics())
    evaluate_analysis_sparse_jacobian!(matrix, assembly.model, selection,
        t, z, zdot, coefficient)
    return nothing
end

function run_torque_driven_four_bar(; applied_torque = 10.0,
        theta0 = deg2rad(30.0), omega0 = 0.0, branch = 1,
        parameters = FourBarParameters(), tspan = (0.0, 1.0),
        atol = 1.0e-9, rtol = 1.0e-7, dt = 1.0e-6, dtmax = 0.005)
    assembly = torque_driven_four_bar_assembly(parameters; applied_torque)
    z0, zdot0 = torque_four_bar_initial_conditions(
        assembly, parameters; theta = theta0, omega = omega0, branch)
    prototype = torque_driven_four_bar_jacobian(
        first(tspan), z0, zdot0, 1.0, assembly)
    differential = falses(length(z0))
    differential[assembly.crank.angular_velocity_variable] = true
    differential[assembly.crank.orientation_variable] = true
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol, rtol = rtol, initial_step = dt, maximum_step = dtmax)
    solution = HistoricalDDASSL.dassl(
        torque_driven_four_bar_equations!, z0, zdot0, tspan;
        parameter = assembly, jacobian! = torque_driven_four_bar_jacobian!,
        jacobian_prototype = prototype, options,
        variable_levels = getfield.(assembly.layout.catalog.variables, :level),
        equation_levels = getfield.(assembly.layout.catalog.equations, :level),
        differential_vars = differential, error_control = differential,
        deficit = 0)
    return solution, assembly, parameters
end

function four_bar_mechanical_energy(z, assembly, parameters)
    energy = zero(eltype(z))
    for body in assembly.bodies
        velocity = z[body.velocity_variables]
        omega = z[body.angular_velocity_variable]
        position = z[body.position_variables]
        energy += 0.5 * body.mass * dot(velocity, velocity) +
            0.5 * body.inertia * omega^2 -
            body.mass * dot(parameters.gravity, position)
    end
    return energy
end

function torque_driven_four_bar_diagnostics(solution, assembly, parameters)
    equation_errors = Float64[]
    position_errors = Float64[]
    energy_balance_errors = Float64[]
    initial_energy = four_bar_mechanical_energy(solution.u[1], assembly, parameters)
    initial_angle = solution.u[1][assembly.crank.orientation_variable]
    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(length(z))
        torque_driven_four_bar_equations!(equations, t, z, zdot, assembly)
        push!(equation_errors, norm(equations, Inf))
        rows = vcat((collect(pin.position_equations) for pin in assembly.pins)...)
        push!(position_errors, norm(equations[rows], Inf))
        work = assembly.torque.torque *
            (z[assembly.crank.orientation_variable] - initial_angle)
        push!(energy_balance_errors, abs(
            four_bar_mechanical_energy(z, assembly, parameters) -
            initial_energy - work))
    end
    return (
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_position_constraint_error = maximum(position_errors),
        maximum_energy_balance_error = maximum(energy_balance_errors),
        final_crank_angle = solution.u[end][assembly.crank.orientation_variable],
        final_crank_speed = solution.u[end][assembly.crank.angular_velocity_variable],
        accepted_steps = solution.stats.accepted_steps,
        rejected_steps = solution.stats.rejected_steps,
        maximum_order = maximum(solution.orders),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution, assembly, parameters = run_torque_driven_four_bar()
    println("Torque-driven planar four-bar")
    for (name, value) in pairs(
            torque_driven_four_bar_diagnostics(solution, assembly, parameters))
        println("  ", name, ": ", value)
    end
end
