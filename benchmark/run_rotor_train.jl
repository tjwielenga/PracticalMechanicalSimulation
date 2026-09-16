using PracticalMechanicalSimulation
include("PendulumChainBenchmark.jl")
using .PendulumChainBenchmark

function usage(io)
    println(io,
        "usage: run_rotor_train.jl ROTORS [OUTPUT.simp] [DURATION] [SAMPLES]")
end

if abspath(PROGRAM_FILE) == @__FILE__
    if isempty(ARGS) || length(ARGS) > 4
        usage(stderr)
        exit(2)
    end
    rotor_count = parse(Int, ARGS[1])
    output = length(ARGS) >= 2 ? ARGS[2] : joinpath(@__DIR__, "..", "results",
        "benchmarks", "rotor-train-$rotor_count.simp")
    duration = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 1.0
    samples = length(ARGS) >= 4 ? parse(Int, ARGS[4]) : 201
    result = run_rotor_train(rotor_count; duration, samples)
    write_result(output, result; overwrite = true)
    println("Wrote $output with ",
        "$(length(result.loaded.layout.catalog.variables)) variables, ",
        "$(result.loaded.analysis.degrees_of_freedom) states, and ",
        "$(length(result.times)) samples.")
end
