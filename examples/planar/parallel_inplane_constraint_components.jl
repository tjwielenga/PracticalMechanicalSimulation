if !isdefined(@__MODULE__, :AutomaticAnalysis)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "AutomaticAnalysis.jl"))
end
if !isdefined(@__MODULE__, :PlanarAppliedForces)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarAppliedForces.jl"))
end
if !isdefined(@__MODULE__, :PlanarDirectedDistances)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarDirectedDistances.jl"))
end
if !isdefined(@__MODULE__, :PlanarComponentAssembly)
    include(joinpath(@__DIR__, "..", "..", "src", "planar", "PlanarComponentAssembly.jl"))
end
using .AutomaticAnalysis
using .PlanarAppliedForces
using .PlanarDirectedDistances
using .PlanarComponentAssembly

function parallel_inplane_component_assembly()
    body_registration = planar_body_registration(:body)
    guide_1_registration = inplane_constraint_registration(:guide_1)
    guide_2_registration = inplane_constraint_registration(:guide_2)
    builder = ModelLayoutBuilder()
    for registration in (body_registration, guide_1_registration,
                         guide_2_registration)
        allocate_component_variables!(builder, registration)
    end
    allocate_component_equation_block!(builder, body_registration, :balance)
    for guide in (guide_1_registration, guide_2_registration)
        for block in (:acceleration, :velocity, :position)
            allocate_component_equation_block!(builder, guide, block)
        end
    end
    layout = finish_layout(builder)

    variables = component_variable_indices(layout, :body)
    body = PlanarRigidBodyComponent(:body, 1.0, 1.0 / 12.0,
        variables[1:2], variables[3], variables[4:5], variables[6],
        variables[7:8], variables[9],
        component_equation_indices(layout, :body, :balance), 1:0)
    gravity = PlanarGravityComponent(:gravity, body, [0.0, -9.81])

    function make_guide(name, r_body, ground_position)
        body_marker = PlanarBodyPointMarker(body.position_variables,
            body.orientation_variable, body.velocity_variables,
            body.angular_velocity_variable, body.balance_equations[1:2],
            body.balance_equations[3], collect(r_body))
        ground_marker = PlanarGroundPointMarker(collect(ground_position))
        axis = PlanarDirectedAxis(nothing,
            PlanarGroundOrientationMarker(-pi / 2))
        geometry = PlanarDirectedDistance(
            body, body_marker, nothing, ground_marker, axis)
        return PlanarInplaneConstraint(name, geometry,
            only(component_variable_indices(layout, name)),
            only(component_equation_indices(layout, name, :acceleration)),
            only(component_equation_indices(layout, name, :velocity)),
            only(component_equation_indices(layout, name, :position)))
    end
    guide_1 = make_guide(:guide_1, [0.0, 0.5], [0.0, 0.5])
    guide_2 = make_guide(:guide_2, [0.0, -0.5], [0.0, -0.5])

    blocks = ExecutableEquationBlock[]
    append!(blocks, executable_blocks(body)[1:1])
    append!(blocks, executable_blocks(guide_1))
    append!(blocks, executable_blocks(guide_2))
    contributions = EquationContribution[]
    for component in (gravity, guide_1, guide_2)
        append!(contributions, equation_contributions(component))
    end
    model = ExecutableAnalysisModel(layout.catalog, blocks, contributions)
    return (; body, guide_1, guide_2, layout, model)
end
