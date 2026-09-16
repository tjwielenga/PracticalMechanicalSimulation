project = @__DIR__
julia = Base.julia_cmd()

for suite in ("model_development_tests.jl", "ddassl_formulation_tests.jl")
    command = `$julia --project=$project $(joinpath(@__DIR__, suite))`
    println("Running paper verification: ", suite)
    run(command)
end
