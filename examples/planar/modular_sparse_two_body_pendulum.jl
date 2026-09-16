using LinearAlgebra
using SparseArrays
using OrdinaryDiffEq

if !isdefined(@__MODULE__, :HistoricalDDASSL)
    include(joinpath(@__DIR__, "..", "..", "src", "common", "HistoricalDDASSL.jl"))
end
if !isdefined(@__MODULE__, :rotation)
    include(joinpath(@__DIR__, "minimal_cartesian_pendulum.jl"))
end

struct PlanarBodyBlock{T}
    mass::T
    inertia::T
    variables::UnitRange{Int}
    equations::UnitRange{Int}
end

struct PlanarBodyMarker{T}
    body::Int
    r_body::Vector{T}
end

struct PlanarGroundMarker{T}
    position::Vector{T}
end

struct PlanarRevoluteJoint{A,B}
    marker_a::A
    marker_b::B
    reaction_variables::UnitRange{Int}
    equations::UnitRange{Int}
end

struct ModularTwoBodySystem{T}
    bodies::Vector{PlanarBodyBlock{T}}
    joints::Vector{PlanarRevoluteJoint}
    gravity::Vector{T}
end

body_indices(body::PlanarBodyBlock) = (
    acceleration = body.variables[1:2],
    alpha = body.variables[3],
    velocity = body.variables[4:5],
    omega = body.variables[6],
    position = body.variables[7:8],
    theta = body.variables[9],
)

function ModularTwoBodySystem(; mass_1 = 1.0, mass_2 = 1.0,
        length_1 = 1.0, length_2 = 1.0,
        inertia_1 = mass_1 * length_1^2 / 12,
        inertia_2 = mass_2 * length_2^2 / 12,
        gravity = [0.0, -9.81])
    T = promote_type(typeof(mass_1), typeof(mass_2), typeof(length_1),
        typeof(length_2), typeof(inertia_1), typeof(inertia_2),
        eltype(gravity))
    bodies = PlanarBodyBlock{T}[
        PlanarBodyBlock(T(mass_1), T(inertia_1), 1:9, 1:3),
        PlanarBodyBlock(T(mass_2), T(inertia_2), 10:18, 4:6),
    ]
    ground = PlanarGroundMarker(T[0, 0])
    upper_1 = PlanarBodyMarker(1, T[0, length_1 / 2])
    lower_1 = PlanarBodyMarker(1, T[0, -length_1 / 2])
    upper_2 = PlanarBodyMarker(2, T[0, length_2 / 2])
    joints = PlanarRevoluteJoint[
        PlanarRevoluteJoint(upper_1, ground, 19:20, 7:12),
        PlanarRevoluteJoint(lower_1, upper_2, 21:22, 13:18),
    ]
    return ModularTwoBodySystem(bodies, joints, T.(gravity))
end

function marker_kinematics(marker::PlanarBodyMarker, z, system)
    indices = body_indices(system.bodies[marker.body])
    theta = z[indices.theta]
    omega = z[indices.omega]
    alpha = z[indices.alpha]
    r = rotation(theta) * marker.r_body
    S = [zero(theta) -one(theta); one(theta) zero(theta)]
    d = rotation(theta) * S * marker.r_body
    return (
        position = z[indices.position] + r,
        velocity = z[indices.velocity] + d * omega,
        acceleration = z[indices.acceleration] + d * alpha - r * omega^2,
        r = r,
        d = d,
        indices = indices,
    )
end

function marker_kinematics(marker::PlanarGroundMarker, z, system)
    T = eltype(z)
    return (
        position = T.(marker.position),
        velocity = zeros(T, 2),
        acceleration = zeros(T, 2),
        r = zeros(T, 2),
        d = zeros(T, 2),
        indices = nothing,
    )
end

"""Assemble the 22 permanent and selected implicit equations."""
function modular_two_body_equations!(equations, t, z, zdot,
                                     system::ModularTwoBodySystem)
    fill!(equations, zero(eltype(equations)))

    for body in system.bodies
        indices = body_indices(body)
        rows = body.equations
        equations[rows[1:2]] .+= body.mass .* z[indices.acceleration] .-
            body.mass .* system.gravity
        equations[rows[3]] += body.inertia * z[indices.alpha]
    end

    for joint in system.joints
        marker_a = marker_kinematics(joint.marker_a, z, system)
        marker_b = marker_kinematics(joint.marker_b, z, system)
        rows = joint.equations
        equations[rows[1:2]] .= marker_a.acceleration - marker_b.acceleration
        equations[rows[3:4]] .= marker_a.velocity - marker_b.velocity
        equations[rows[5:6]] .= marker_a.position - marker_b.position

        reaction = z[joint.reaction_variables]
        for (marker, sign) in ((joint.marker_a, 1), (joint.marker_b, -1))
            marker isa PlanarGroundMarker && continue
            kinematics = marker_kinematics(marker, z, system)
            body = system.bodies[marker.body]
            equations[body.equations[1:2]] .-= sign .* reaction
            equations[body.equations[3]] -=
                sign * dot(kinematics.d, reaction)
        end
    end

    indices_1 = body_indices(system.bodies[1])
    indices_2 = body_indices(system.bodies[2])
    equations[19] = z[indices_1.alpha] - zdot[indices_1.omega]
    equations[20] = z[indices_1.omega] - zdot[indices_1.theta]
    equations[21] = z[indices_2.alpha] - zdot[indices_2.omega]
    equations[22] = z[indices_2.omega] - zdot[indices_2.theta]
    return nothing
end

add_entry!(rows, columns, values, row, column, value) =
    (push!(rows, row); push!(columns, column); push!(values, value); nothing)

function add_body_jacobian!(rows, columns, values, body)
    indices = body_indices(body)
    add_entry!(rows, columns, values, body.equations[1],
        indices.acceleration[1], body.mass)
    add_entry!(rows, columns, values, body.equations[2],
        indices.acceleration[2], body.mass)
    add_entry!(rows, columns, values, body.equations[3],
        indices.alpha, body.inertia)
end

function add_marker_constraint_jacobian!(rows, columns, values,
        equation_rows, marker::PlanarBodyMarker, sign, z, system)
    kinematics = marker_kinematics(marker, z, system)
    indices = kinematics.indices
    alpha = z[indices.alpha]
    omega = z[indices.omega]
    r = kinematics.r
    d = kinematics.d

    for component in 1:2
        add_entry!(rows, columns, values, equation_rows[component],
            indices.acceleration[component], sign)
        add_entry!(rows, columns, values, equation_rows[component],
            indices.alpha, sign * d[component])
        add_entry!(rows, columns, values, equation_rows[component],
            indices.omega, sign * (-2r[component] * omega))
        add_entry!(rows, columns, values, equation_rows[component],
            indices.theta,
            sign * (-r[component] * alpha - d[component] * omega^2))

        add_entry!(rows, columns, values, equation_rows[2 + component],
            indices.velocity[component], sign)
        add_entry!(rows, columns, values, equation_rows[2 + component],
            indices.omega, sign * d[component])
        add_entry!(rows, columns, values, equation_rows[2 + component],
            indices.theta, sign * (-r[component] * omega))

        add_entry!(rows, columns, values, equation_rows[4 + component],
            indices.position[component], sign)
        add_entry!(rows, columns, values, equation_rows[4 + component],
            indices.theta, sign * d[component])
    end
end

add_marker_constraint_jacobian!(rows, columns, values, equation_rows,
    marker::PlanarGroundMarker, sign, z, system) = nothing

function add_joint_jacobian!(rows, columns, values, joint, z, system)
    add_marker_constraint_jacobian!(rows, columns, values,
        joint.equations, joint.marker_a, 1, z, system)
    add_marker_constraint_jacobian!(rows, columns, values,
        joint.equations, joint.marker_b, -1, z, system)

    reaction = z[joint.reaction_variables]
    for (marker, sign) in ((joint.marker_a, 1), (joint.marker_b, -1))
        marker isa PlanarGroundMarker && continue
        kinematics = marker_kinematics(marker, z, system)
        body = system.bodies[marker.body]
        indices = kinematics.indices
        for component in 1:2
            add_entry!(rows, columns, values, body.equations[component],
                joint.reaction_variables[component], -sign)
            add_entry!(rows, columns, values, body.equations[3],
                joint.reaction_variables[component],
                -sign * kinematics.d[component])
        end
        add_entry!(rows, columns, values, body.equations[3], indices.theta,
            sign * dot(kinematics.r, reaction))
    end
end

"""Construct the analytical sparse matrix F_z + coefficient*F_zdot."""
function modular_two_body_sparse_jacobian(t, z, zdot, coefficient, system)
    rows = Int[]
    columns = Int[]
    values = eltype(z)[]

    for body in system.bodies
        add_body_jacobian!(rows, columns, values, body)
    end
    for joint in system.joints
        add_joint_jacobian!(rows, columns, values, joint, z, system)
    end

    for (row, body) in ((19, system.bodies[1]), (21, system.bodies[2]))
        indices = body_indices(body)
        add_entry!(rows, columns, values, row, indices.alpha, one(eltype(z)))
        add_entry!(rows, columns, values, row, indices.omega, -coefficient)
    end
    for (row, body) in ((20, system.bodies[1]), (22, system.bodies[2]))
        indices = body_indices(body)
        add_entry!(rows, columns, values, row, indices.omega, one(eltype(z)))
        add_entry!(rows, columns, values, row, indices.theta, -coefficient)
    end

    return sparse(rows, columns, values, 22, 22)
end

function modular_two_body_sparse_jacobian!(matrix::SparseMatrixCSC,
        t, z, zdot, coefficient, system)
    assembled = modular_two_body_sparse_jacobian(
        t, z, zdot, coefficient, system)
    matrix.colptr == assembled.colptr && matrix.rowval == assembled.rowval ||
        error("The modular Jacobian sparsity pattern changed")
    matrix.nzval .= assembled.nzval
    return nothing
end

function reconstruct_two_body_state(theta_1, omega_1, theta_2, omega_2,
                                    system)
    z = zeros(promote_type(typeof(theta_1), typeof(omega_1),
        typeof(theta_2), typeof(omega_2)), 22)
    indices_1 = body_indices(system.bodies[1])
    indices_2 = body_indices(system.bodies[2])
    upper_1 = system.joints[1].marker_a
    lower_1 = system.joints[2].marker_a
    upper_2 = system.joints[2].marker_b

    z[indices_1.theta] = theta_1
    z[indices_1.omega] = omega_1
    z[indices_2.theta] = theta_2
    z[indices_2.omega] = omega_2

    r_upper_1 = rotation(theta_1) * upper_1.r_body
    r_lower_1 = rotation(theta_1) * lower_1.r_body
    r_upper_2 = rotation(theta_2) * upper_2.r_body
    S = [0.0 -1.0; 1.0 0.0]
    d_upper_1 = rotation(theta_1) * S * upper_1.r_body
    d_lower_1 = rotation(theta_1) * S * lower_1.r_body
    d_upper_2 = rotation(theta_2) * S * upper_2.r_body

    z[indices_1.position] .= -r_upper_1
    z[indices_1.velocity] .= -d_upper_1 .* omega_1
    z[indices_2.position] .=
        z[indices_1.position] + r_lower_1 - r_upper_2
    z[indices_2.velocity] .= z[indices_1.velocity] +
        d_lower_1 .* omega_1 - d_upper_2 .* omega_2

    # Dynamics and acceleration constraints are linear in accelerations and
    # reactions for fixed positions and velocities.
    unknowns = [1, 2, 3, 10, 11, 12, 19, 20, 21, 22]
    equations_used = [1, 2, 3, 4, 5, 6, 7, 8, 13, 14]
    zdot = zeros(eltype(z), 22)
    base = zeros(eltype(z), 22)
    modular_two_body_equations!(base, 0.0, z, zdot, system)
    K = zeros(eltype(z), 10, 10)
    trial = similar(base)
    for (column, variable) in enumerate(unknowns)
        z[variable] += one(eltype(z))
        modular_two_body_equations!(trial, 0.0, z, zdot, system)
        K[:, column] .= trial[equations_used] - base[equations_used]
        z[variable] -= one(eltype(z))
    end
    z[unknowns] .= K \ (-base[equations_used])
    return z
end

function modular_two_body_initial_conditions(theta_1, omega_1,
        theta_2, omega_2, system)
    z = reconstruct_two_body_state(theta_1, omega_1, theta_2, omega_2, system)
    zdot = zeros(eltype(z), 22)
    indices_1 = body_indices(system.bodies[1])
    indices_2 = body_indices(system.bodies[2])
    zdot[indices_1.omega] = z[indices_1.alpha]
    zdot[indices_1.theta] = z[indices_1.omega]
    zdot[indices_2.omega] = z[indices_2.alpha]
    zdot[indices_2.theta] = z[indices_2.omega]
    return z, zdot
end

const MODULAR_TWO_BODY_VARIABLE_LEVELS =
    [2, 2, 2, 1, 1, 1, 0, 0, 0,
     2, 2, 2, 1, 1, 1, 0, 0, 0, 2, 2, 2, 2]
const MODULAR_TWO_BODY_EQUATION_LEVELS =
    [2, 2, 2, 2, 2, 2,
     2, 2, 1, 1, 0, 0, 2, 2, 1, 1, 0, 0,
     2, 1, 2, 1]
const MODULAR_TWO_BODY_DIFFERENTIAL_VARS =
    BitVector([i in (6, 9, 15, 18) for i in 1:22])

function run_modular_sparse_two_body_pendulum(;
        theta_1 = deg2rad(35.0), omega_1 = 0.0,
        theta_2 = deg2rad(-20.0), omega_2 = 0.0,
        tspan = (0.0, 1.0), atol = 1.0e-7, rtol = 1.0e-5,
        dt = 1.0e-6, dtmax = 0.02)
    system = ModularTwoBodySystem()
    z0, zdot0 = modular_two_body_initial_conditions(
        theta_1, omega_1, theta_2, omega_2, system)
    prototype = modular_two_body_sparse_jacobian(
        first(tspan), z0, zdot0, 1.0, system)
    options = HistoricalDDASSL.DASSLOptions{Float64}(
        atol = atol, rtol = rtol, initial_step = dt, maximum_step = dtmax)
    solution = HistoricalDDASSL.dassl(
        modular_two_body_equations!, z0, zdot0, tspan;
        parameter = system,
        jacobian! = modular_two_body_sparse_jacobian!,
        jacobian_prototype = prototype,
        options,
        variable_levels = MODULAR_TWO_BODY_VARIABLE_LEVELS,
        equation_levels = MODULAR_TWO_BODY_EQUATION_LEVELS,
        differential_vars = MODULAR_TWO_BODY_DIFFERENTIAL_VARS,
        error_control = MODULAR_TWO_BODY_DIFFERENTIAL_VARS,
        deficit = 0,
    )
    return solution, system
end

function reduced_two_body_rhs!(du, u, system, t)
    z = reconstruct_two_body_state(u[1], u[2], u[3], u[4], system)
    du[1] = u[2]
    du[2] = z[3]
    du[3] = u[4]
    du[4] = z[12]
    return nothing
end

"""Total kinetic plus gravitational potential energy of both bodies."""
function modular_two_body_energy(z, system)
    energy = zero(eltype(z))
    for body in system.bodies
        indices = body_indices(body)
        kinetic = 0.5 * body.mass * dot(
            z[indices.velocity], z[indices.velocity]) +
            0.5 * body.inertia * z[indices.omega]^2
        potential = -body.mass * dot(system.gravity, z[indices.position])
        energy += kinetic + potential
    end
    return energy
end

function modular_two_body_diagnostics(solution, system)
    initial = solution.u[1]
    reference_problem = ODEProblem(reduced_two_body_rhs!,
        [initial[9], initial[6], initial[18], initial[15]],
        (first(solution.t), last(solution.t)), system)
    reference = solve(reference_problem, Tsit5();
        abstol = 1.0e-11, reltol = 1.0e-11)
    equation_errors = Float64[]
    state_errors = Float64[]
    constraint_errors = Float64[]
    implicit_energies = Float64[]
    reference_energies = Float64[]
    for (t, z, zdot) in zip(solution.t, solution.u, solution.du)
        equations = zeros(22)
        modular_two_body_equations!(equations, t, z, zdot, system)
        push!(equation_errors, norm(equations, Inf))
        reference_state = reference(t)
        reference_z = reconstruct_two_body_state(
            reference_state[1], reference_state[2],
            reference_state[3], reference_state[4], system)
        push!(state_errors, norm(
            [z[9], z[6], z[18], z[15]] - reference_state, Inf))
        push!(constraint_errors,
            norm(equations[[7, 8, 9, 10, 11, 12,
                            13, 14, 15, 16, 17, 18]], Inf))
        push!(implicit_energies, modular_two_body_energy(z, system))
        push!(reference_energies, modular_two_body_energy(reference_z, system))
    end
    prototype = modular_two_body_sparse_jacobian(
        first(solution.t), first(solution.u), first(solution.du), 1.0, system)
    return (
        return_code = solution.retcode,
        simultaneous_variables = 22,
        controlled_states = count(solution.error_control),
        jacobian_stored_entries = nnz(prototype),
        jacobian_density = nnz(prototype) / length(prototype),
        maximum_implicit_equation_error = maximum(equation_errors),
        maximum_constraint_equation_error = maximum(constraint_errors),
        maximum_selected_state_difference = maximum(state_errors),
        maximum_bdf_energy_drift = maximum(abs,
            implicit_energies .- first(implicit_energies)),
        final_bdf_energy_drift =
            last(implicit_energies) - first(implicit_energies),
        maximum_reference_energy_drift = maximum(abs,
            reference_energies .- first(reference_energies)),
        maximum_energy_difference = maximum(abs,
            implicit_energies .- reference_energies),
    )
end

if abspath(PROGRAM_FILE) == @__FILE__
    solution, system = run_modular_sparse_two_body_pendulum()
    println("Modular sparse two-body pendulum")
    for (name, value) in pairs(modular_two_body_diagnostics(solution, system))
        println("  ", name, ": ", value)
    end
    println("  accepted steps: ", solution.stats.accepted_steps)
    println("  rejected steps: ", solution.stats.rejected_steps)
    println("  maximum BDF order: ", maximum(solution.orders))
end
