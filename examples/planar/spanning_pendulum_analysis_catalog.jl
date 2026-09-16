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

function spanning_pendulum_components(system)
    mechanical = system.mechanical
    body = PlanarRigidBodyComponent(:body, mechanical.mass,
        mechanical.inertia, 1:2, 3, 4:5, 6, 7:8, 9, 1:3, 10:11)
    gravity = PlanarGravityComponent(:gravity, body,
        collect(mechanical.gravity))
    pin_marker = PlanarBodyPointMarker(7:8, 9, 4:5, 6, 1:2, 3,
        collect(mechanical.r_body))
    ground_marker = PlanarGroundPointMarker(collect(mechanical.pin))
    pin = PlanarRevoluteJointComponent(:pin, body, pin_marker,
        nothing, ground_marker, 10:11, 4:5, 6:7, 8:9)
    spring = PlanarSpanningSpringComponent(:spring, system.spring_damper)
    return (; body, gravity, pin, spring)
end

planar_body_analysis_registration(system) =
    component_registration(spanning_pendulum_components(system).body)
revolute_pin_analysis_registration(system) =
    component_registration(spanning_pendulum_components(system).pin)
spanning_spring_analysis_registration(system) =
    component_registration(spanning_pendulum_components(system).spring)

function spanning_pendulum_analysis_catalog(system)
    components = spanning_pendulum_components(system)
    body = component_registration(components.body)
    pin = component_registration(components.pin)
    spring = component_registration(components.spring)
    builder = AnalysisCatalogBuilder()

    register_component_variables!(builder, body)
    register_component_variables!(builder, pin)
    register_component_variables!(builder, spring)

    register_component_equation_block!(builder, body, :balance)
    register_component_equation_block!(builder, pin, :acceleration)
    register_component_equation_block!(builder, pin, :velocity)
    register_component_equation_block!(builder, pin, :position)
    register_component_equation_block!(builder, body, :selected_state)
    register_component_equation_block!(builder, spring, :geometry)
    register_component_equation_block!(builder, spring, :rate)
    register_component_equation_block!(builder, spring, :load)
    return finish_catalog(builder)
end

planar_body_analysis_registration() =
    planar_body_analysis_registration(SpanningSpringPendulumSystem())
revolute_pin_analysis_registration() =
    revolute_pin_analysis_registration(SpanningSpringPendulumSystem())
spanning_spring_analysis_registration() =
    spanning_spring_analysis_registration(SpanningSpringPendulumSystem())
spanning_pendulum_analysis_catalog() =
    spanning_pendulum_analysis_catalog(SpanningSpringPendulumSystem())
