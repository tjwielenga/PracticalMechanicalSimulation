include("PendulumChainBenchmark.jl")
using .PendulumChainBenchmark

if abspath(PROGRAM_FILE) == @__FILE__
    counts = isempty(ARGS) ? [10, 25, 50] : parse.(Int, ARGS)
    benchmark_pendulum_chains(counts)
end
