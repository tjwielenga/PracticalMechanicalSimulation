using PracticalMechanicalSimulation
include("PendulumChainBenchmark.jl")
using .PendulumChainBenchmark

function usage(io)
    println(io,
        "usage: run_parallelogram_chain.jl CELLS [OUTPUT.simp] [DURATION] [SAMPLES]")
end

if abspath(PROGRAM_FILE) == @__FILE__
    if isempty(ARGS) || length(ARGS) > 4
        usage(stderr)
        exit(2)
    end
    cell_count = parse(Int, ARGS[1])
    output = length(ARGS) >= 2 ? ARGS[2] :
        joinpath(@__DIR__, "..", "results", "benchmarks",
            "parallelogram-chain-$cell_count.simp")
    duration = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 0.25
    samples = length(ARGS) >= 4 ? parse(Int, ARGS[4]) : 51
    result = run_parallelogram_chain(cell_count; duration, samples)
    write_result(output, result; overwrite = true)
    println("Wrote $output with ",
        "$(length(result.loaded.layout.catalog.variables)) variables, ",
        "$(result.loaded.analysis.degrees_of_freedom) states, and ",
        "$(length(result.times)) samples.")
end
