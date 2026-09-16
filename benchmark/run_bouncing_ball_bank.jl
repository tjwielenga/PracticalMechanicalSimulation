using PracticalMechanicalSimulation
include("PendulumChainBenchmark.jl")
using .PendulumChainBenchmark

function usage(io)
    println(io,
        "usage: run_bouncing_ball_bank.jl BALLS [OUTPUT.simp] [DURATION] [SAMPLES]")
end

if abspath(PROGRAM_FILE) == @__FILE__
    if isempty(ARGS) || length(ARGS) > 4
        usage(stderr)
        exit(2)
    end
    ball_count = parse(Int, ARGS[1])
    output = length(ARGS) >= 2 ? ARGS[2] :
        joinpath(@__DIR__, "..", "results", "benchmarks",
            "bouncing-ball-bank-$ball_count.simp")
    duration = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 1.5
    samples = length(ARGS) >= 4 ? parse(Int, ARGS[4]) : 301
    result = run_bouncing_ball_bank(ball_count; duration, samples)
    write_result(output, result; overwrite = true)
    println("Wrote $output with ",
        "$(length(result.loaded.layout.catalog.variables)) variables, ",
        "$(result.loaded.analysis.degrees_of_freedom) states, ",
        "$(result.solution.stats.events_found) events, and ",
        "$(length(result.times)) samples.")
end
