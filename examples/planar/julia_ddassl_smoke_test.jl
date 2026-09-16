include(joinpath(@__DIR__, "..", "..", "src", "common", "HistoricalDDASSL.jl"))
using .HistoricalDDASSL
using SciMLBase

function exponential_residual!(out, t, y, yprime, parameter)
    out[1] = yprime[1] + y[1]
    return nothing
end

function exponential_jacobian!(matrix, t, y, yprime, cj, parameter)
    matrix[1, 1] = 1 + cj
    return nothing
end

function algebraic_residual!(out, t, y, yprime, parameter)
    out[1] = yprime[1] + y[1]
    out[2] = y[2] - y[1]^2
    return nothing
end

options = DASSLOptions{Float64}(
    rtol = 1.0e-7,
    atol = 1.0e-9,
    initial_step = 1.0e-4,
    maximum_step = 0.05,
)

numerical = dassl(exponential_residual!, [1.0], [-1.0], (0.0, 1.0);
    options)
analytical = dassl(exponential_residual!, [1.0], [-1.0], (0.0, 1.0);
    options, jacobian! = exponential_jacobian!)
dae = dassl(algebraic_residual!, [1.0, 1.0], [-1.0, 0.0], (0.0, 1.0);
    options, error_control = Bool[true, false])

exact = exp(-1.0)
println("Julia DDASSL reconstruction")
println("  numerical Jacobian retcode: ", numerical.retcode)
println("  numerical Jacobian error:   ", abs(numerical.y[end][1] - exact))
println("  numerical accepted/rejected: ",
    numerical.stats.accepted_steps, "/", numerical.stats.rejected_steps)
println("  analytical Jacobian error:  ", abs(analytical.y[end][1] - exact))
println("  algebraic state error:       ", abs(dae.y[end][2] - exact^2))
println("  algebraic equation error:    ",
    abs(dae.y[end][2] - dae.y[end][1]^2))
println("  interpolated y(0.5):         ", numerical(0.5)[1])
println("  interpolated yprime(0.5):    ", numerical(0.5, Val{1})[1])

@assert SciMLBase.successful_retcode(numerical.retcode)
@assert SciMLBase.successful_retcode(analytical.retcode)
@assert SciMLBase.successful_retcode(dae.retcode)
@assert abs(numerical.y[end][1] - exact) < 2.0e-5
@assert abs(analytical.y[end][1] - exact) < 2.0e-5
@assert abs(dae.y[end][2] - dae.y[end][1]^2) < 1.0e-8
