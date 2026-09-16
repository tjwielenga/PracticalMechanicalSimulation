using LinearAlgebra
using OrdinaryDiffEq

"""
Physical data for the planar rigid-body pendulum.

r_body points from the center of mass to the pin, expressed in the body
frame. gravity and pin are expressed in the global frame.
"""
struct PendulumParameters{T}
    mass::T
    inertia::T
    r_body::Vector{T}
    gravity::Vector{T}
    pin::Vector{T}
end

function PendulumParameters(; mass = 1.0,
                              inertia = 1.0 / 12.0,
                              r_body = [0.0, 0.5],
                              gravity = [0.0, -9.81],
                              pin = [0.0, 0.0])
    T = promote_type(typeof(mass), typeof(inertia),
                     eltype(r_body), eltype(gravity), eltype(pin))
    return PendulumParameters(
        T(mass),
        T(inertia),
        T.(r_body),
        T.(gravity),
        T.(pin),
    )
end

"""Rotation from body components to global components."""
function rotation(theta)
    c = cos(theta)
    s = sin(theta)
    return [c -s; s c]
end

"""
Return the current center-of-mass-to-pin vector c_global and its
infinitesimal rotational direction d_global.
"""
function marker_vectors(theta, p::PendulumParameters)
    A = rotation(theta)
    S = [zero(theta) -one(theta); one(theta) zero(theta)]
    c_global = A * p.r_body
    d_global = A * S * p.r_body
    return c_global, d_global
end

"""Position constraint Phi(q) for q = [R_x, R_y, theta]."""
function position_constraint(q, p::PendulumParameters)
    c_global, _ = marker_vectors(q[3], p)
    return q[1:2] + c_global - p.pin
end

"""Constraint partial D(q) = [I d] for q = [R_x, R_y, theta]."""
function constraint_partial(q, p::PendulumParameters)
    _, d_global = marker_vectors(q[3], p)
    T = promote_type(eltype(q), eltype(d_global))
    return T[1 0 d_global[1];
             0 1 d_global[2]]
end

"""
Project a user-supplied position onto Phi(q) = 0 using a weighted
minimum-distance correction.
"""
function consistent_position(q_user, p::PendulumParameters;
                             weights = Diagonal([p.mass, p.mass, p.inertia]),
                             abstol = 1.0e-12,
                             maxiters = 30)
    q = copy(q_user)
    W = Matrix(weights)
    multiplier = zeros(eltype(q), 2)

    for _ in 1:maxiters
        phi = position_constraint(q, p)
        D = constraint_partial(q, p)
        stationarity = W * (q - q_user) - D' * multiplier
        max(norm(phi, Inf), norm(stationarity, Inf)) <= abstol && return q

        # The exact KKT Hessian differs from the acceleration-like weight
        # matrix only in the scalar rotational entry for this pendulum.
        c_global, _ = marker_vectors(q[3], p)
        H = copy(W)
        H[3, 3] += dot(multiplier, c_global)
        K = [H -D'; D zeros(eltype(W), 2, 2)]
        rhs = vcat(-stationarity, -phi)
        correction = K \ rhs
        q .+= correction[1:3]
        multiplier .+= correction[4:5]
    end

    error("Position projection did not converge in $maxiters iterations")
end

"""
Project a user-supplied body velocity [V_x, V_y, omega] onto D(q)V_b = 0.
"""
function consistent_velocity(v_user, q, p::PendulumParameters;
                             weights = Diagonal([p.mass, p.mass, p.inertia]))
    v = copy(v_user)
    W = Matrix(weights)
    D = constraint_partial(q, p)
    K = [W -D'; D zeros(eltype(W), 2, 2)]
    rhs = vcat(-W * (v - v_user), -D * v)
    correction = K \ rhs
    v .+= correction[1:3]
    return v
end

"""
Solve the five Cartesian acceleration equations.

The six-component state is [R_x, R_y, theta, V_x, V_y, omega].
The result is (a_global, alpha, lambda_global).
"""
function acceleration_solution(u, p::PendulumParameters)
    theta = u[3]
    omega = u[6]
    c_global, d_global = marker_vectors(theta, p)

    T = promote_type(eltype(u), typeof(p.mass))
    K = zeros(T, 5, 5)
    rhs = zeros(T, 5)

    # Sum of forces: m*a - lambda = m*g.
    K[1, 1] = p.mass
    K[2, 2] = p.mass
    K[1, 4] = -1
    K[2, 5] = -1
    rhs[1:2] .= p.mass .* p.gravity

    # Sum of torques: J*alpha - d'lambda = 0.
    K[3, 3] = p.inertia
    K[3, 4:5] .= -d_global

    # Pin acceleration constraint: a + d*alpha = c*omega^2.
    K[4, 1] = 1
    K[5, 2] = 1
    K[4:5, 3] .= d_global
    rhs[4:5] .= c_global .* omega^2

    z = K \ rhs
    return z[1:2], z[3], z[4:5]
end

"""Six-state first-order right-hand side for SciML."""
function pendulum_rhs!(du, u, p::PendulumParameters, t)
    a_global, alpha, _ = acceleration_solution(u, p)
    du[1:2] .= u[4:5]
    du[3] = u[6]
    du[4:5] .= a_global
    du[6] = alpha
    return nothing
end

function velocity_constraint(u, p::PendulumParameters)
    q = u[1:3]
    v_body = u[4:6]
    return constraint_partial(q, p) * v_body
end

function total_energy(u, p::PendulumParameters)
    kinetic = 0.5 * p.mass * dot(u[4:5], u[4:5]) +
              0.5 * p.inertia * u[6]^2
    potential = -p.mass * dot(p.gravity, u[1:2])
    return kinetic + potential
end

"""
Run a demonstrative simulation and return the SciML solution and diagnostics.
"""
function run_example(; theta0 = deg2rad(45.0),
                       omega0 = 0.0,
                       tspan = (0.0, 5.0),
                       abstol = 1.0e-10,
                       reltol = 1.0e-10)
    p = PendulumParameters()

    # Deliberately start from an approximate Cartesian position. The
    # projection enforces the pin position constraint.
    q_guess = [0.01, -0.01, theta0]
    q0 = consistent_position(q_guess, p)

    # The projection also handles a requested nonzero angular velocity.
    v_guess = [0.0, 0.0, omega0]
    v0 = consistent_velocity(v_guess, q0, p)
    u0 = vcat(q0, v0)

    problem = ODEProblem(pendulum_rhs!, u0, tspan, p)
    solution = solve(problem, Tsit5(); abstol, reltol)

    position_errors = [norm(position_constraint(u[1:3], p)) for u in solution.u]
    velocity_errors = [norm(velocity_constraint(u, p)) for u in solution.u]
    energies = [total_energy(u, p) for u in solution.u]
    energy_errors = energies .- first(energies)

    diagnostics = (
        initial_position_error = first(position_errors),
        initial_velocity_error = first(velocity_errors),
        maximum_position_error = maximum(position_errors),
        maximum_velocity_error = maximum(velocity_errors),
        maximum_energy_error = maximum(abs, energy_errors),
    )

    return solution, diagnostics
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution, diagnostics = run_example()
    println("Minimal Cartesian pendulum")
    println("  accepted time points: ", length(solution.t))
    for (name, value) in pairs(diagnostics)
        println("  ", name, ": ", value)
    end
end
