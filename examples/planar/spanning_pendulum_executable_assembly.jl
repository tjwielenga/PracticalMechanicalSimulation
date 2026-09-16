if !isdefined(@__MODULE__, :SpanningSpringPendulumSystem)
    include(joinpath(@__DIR__, "spanning_spring_damper_pendulum.jl"))
end
if !isdefined(@__MODULE__, :spanning_pendulum_components)
    include(joinpath(@__DIR__, "spanning_pendulum_analysis_catalog.jl"))
end

function spanning_pendulum_executable_model(system)
    components = spanning_pendulum_components(system)
    catalog = spanning_pendulum_analysis_catalog(system)
    blocks = ExecutableEquationBlock[]
    append!(blocks, executable_blocks(components.body))
    append!(blocks, executable_blocks(components.pin))
    append!(blocks, executable_blocks(components.spring))
    contributions = EquationContribution[]
    append!(contributions, equation_contributions(components.gravity))
    append!(contributions, equation_contributions(components.pin))
    append!(contributions, equation_contributions(components.spring))
    return ExecutableAnalysisModel(catalog, blocks, contributions)
end
