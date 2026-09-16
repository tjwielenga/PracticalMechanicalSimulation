using SparseArrays

if !isdefined(@__MODULE__, :ModularTwoBodySystem)
    include(joinpath(@__DIR__, "modular_sparse_two_body_pendulum.jl"))
end
if !isdefined(@__MODULE__, :AutomaticAnalysis)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "AutomaticAnalysis.jl"))
end
if !isdefined(@__MODULE__, :PlanarAppliedForces)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarAppliedForces.jl"))
end
if !isdefined(@__MODULE__, :PlanarComponentAssembly)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarComponentAssembly.jl"))
end
using .AutomaticAnalysis
using .PlanarAppliedForces
using .PlanarComponentAssembly

function allocated_body(name, physical, layout)
    variables = component_variable_indices(layout, name)
    balance = component_equation_indices(layout, name, :balance)
    states = component_equation_indices(layout, name, :selected_state)
    return PlanarRigidBodyComponent(name, physical.mass, physical.inertia,
        variables[1:2], variables[3], variables[4:5], variables[6],
        variables[7:8], variables[9], balance, states)
end

function automatic_two_body_assembly(system::ModularTwoBodySystem)
    registrations = (
        body_1 = planar_body_registration(:body_1),
        body_2 = planar_body_registration(:body_2),
        pin_1 = revolute_joint_registration(:pin_1),
        pin_2 = revolute_joint_registration(:pin_2),
    )
    builder = ModelLayoutBuilder()
    for registration in registrations
        allocate_component_variables!(builder, registration)
    end
    allocate_component_equation_block!(builder, registrations.body_1, :balance)
    allocate_component_equation_block!(builder, registrations.body_2, :balance)
    for block in (:acceleration, :velocity, :position)
        allocate_component_equation_block!(builder, registrations.pin_1, block)
    end
    for block in (:acceleration, :velocity, :position)
        allocate_component_equation_block!(builder, registrations.pin_2, block)
    end
    allocate_component_equation_block!(builder,
        registrations.body_1, :selected_state)
    allocate_component_equation_block!(builder,
        registrations.body_2, :selected_state)
    layout = finish_layout(builder)

    old_body_1, old_body_2 = system.bodies
    body_1 = allocated_body(:body_1, old_body_1, layout)
    body_2 = allocated_body(:body_2, old_body_2, layout)
    gravity_1 = PlanarGravityComponent(:gravity_1, body_1,
        collect(system.gravity))
    gravity_2 = PlanarGravityComponent(:gravity_2, body_2,
        collect(system.gravity))

    old_pin_1, old_pin_2 = system.joints
    upper_1 = PlanarBodyPointMarker(body_1.position_variables,
        body_1.orientation_variable, body_1.velocity_variables,
        body_1.angular_velocity_variable, body_1.balance_equations[1:2],
        body_1.balance_equations[3],
        collect(old_pin_1.marker_a.r_body))
    ground = PlanarGroundPointMarker(collect(old_pin_1.marker_b.position))
    pin_1 = PlanarRevoluteJointComponent(:pin_1,
        body_1, upper_1, nothing, ground,
        component_variable_indices(layout, :pin_1),
        component_equation_indices(layout, :pin_1, :acceleration),
        component_equation_indices(layout, :pin_1, :velocity),
        component_equation_indices(layout, :pin_1, :position))

    lower_1 = PlanarBodyPointMarker(body_1.position_variables,
        body_1.orientation_variable, body_1.velocity_variables,
        body_1.angular_velocity_variable, body_1.balance_equations[1:2],
        body_1.balance_equations[3],
        collect(old_pin_2.marker_a.r_body))
    upper_2 = PlanarBodyPointMarker(body_2.position_variables,
        body_2.orientation_variable, body_2.velocity_variables,
        body_2.angular_velocity_variable, body_2.balance_equations[1:2],
        body_2.balance_equations[3],
        collect(old_pin_2.marker_b.r_body))
    pin_2 = PlanarRevoluteJointComponent(:pin_2,
        body_1, lower_1, body_2, upper_2,
        component_variable_indices(layout, :pin_2),
        component_equation_indices(layout, :pin_2, :acceleration),
        component_equation_indices(layout, :pin_2, :velocity),
        component_equation_indices(layout, :pin_2, :position))
    components = (; body_1, body_2, gravity_1, gravity_2, pin_1, pin_2)
    return (; components, layout)
end

automatic_two_body_components(system) =
    automatic_two_body_assembly(system).components
automatic_two_body_catalog(system) =
    automatic_two_body_assembly(system).layout.catalog

function automatic_two_body_executable_model(system)
    assembly = automatic_two_body_assembly(system)
    components = assembly.components
    blocks = ExecutableEquationBlock[]
    for component in (components.body_1, components.body_2,
                      components.pin_1, components.pin_2)
        append!(blocks, executable_blocks(component))
    end
    contributions = EquationContribution[]
    for component in (components.gravity_1, components.gravity_2,
                      components.pin_1, components.pin_2)
        append!(contributions, equation_contributions(component))
    end
    return ExecutableAnalysisModel(assembly.layout.catalog,
        blocks, contributions)
end

function automatic_two_body_equations!(equations, t, z, zdot, model)
    selection = select_analysis(model.catalog, Dynamics())
    evaluate_analysis_equations!(equations, model, selection, t, z, zdot)
    return nothing
end

function automatic_two_body_jacobian(t, z, zdot, coefficient, model)
    selection = select_analysis(model.catalog, Dynamics())
    return evaluate_analysis_sparse_jacobian(model, selection, t, z, zdot,
        coefficient)
end

function automatic_two_body_sparse_jacobian!(matrix::SparseMatrixCSC,
        t, z, zdot, coefficient, model)
    selection = select_analysis(model.catalog, Dynamics())
    evaluate_analysis_sparse_jacobian!(matrix, model, selection,
        t, z, zdot, coefficient)
    return nothing
end

function run_automatic_two_body_component_model(;
        theta_1 = deg2rad(35.0), omega_1 = 0.0,
        theta_2 = deg2rad(-20.0), omega_2 = 0.0,
        tspan = (0.0, 1.0), atol = 1.0e-7, rtol = 1.0e-5,
        dt = 1.0e-6, dtmax = 0.02)
    system = ModularTwoBodySystem()
    model = automatic_two_body_executable_model(system)
    z0, zdot0 = modular_two_body_initial_conditions(
        theta_1, omega_1, theta_2, omega_2, system)
    prototype = automatic_two_body_jacobian(
        first(tspan), z0, zdot0, 1.0, model)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol, rtol = rtol, initial_step = dt, maximum_step = dtmax)
    solution = HistoricalDDASSL.dassl(
        automatic_two_body_equations!, z0, zdot0, tspan;
        parameter = model,
        jacobian! = automatic_two_body_sparse_jacobian!,
        jacobian_prototype = prototype,
        options,
        variable_levels = MODULAR_TWO_BODY_VARIABLE_LEVELS,
        equation_levels = MODULAR_TWO_BODY_EQUATION_LEVELS,
        differential_vars = MODULAR_TWO_BODY_DIFFERENTIAL_VARS,
        error_control = MODULAR_TWO_BODY_DIFFERENTIAL_VARS,
        deficit = 0)
    return solution, system, model
end
