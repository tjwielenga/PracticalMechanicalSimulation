"""Linear constraints among spatial relative coordinates."""
module SpatialCoordinateCouplers

using LinearAlgebra
using ..AutomaticAnalysis
using ..SpatialComponentAssembly
using ..SpatialModeling
using ..SpatialConstraints
using ..SpatialConstraints: body_torque_rows,
    directed_distance_reaction_rows, add_directed_distance_reaction!,
    add_directed_distance_reaction_jacobian!,
    add_perp_reaction_to_body!, add_perp_reaction_jacobian!,
    marker_axis_kinematics

import ..SpatialComponentAssembly: component_registration,
    executable_blocks, equation_contributions

export SpatialCoordinateCoupler, spatial_coordinate_coupler_registration,
       allocated_spatial_coordinate_coupler, spatial_coordinate_variables

"""One ideal linear relation among hinge rotation and inline distance coordinates."""
struct SpatialCoordinateCoupler{C,T}
    name::Symbol
    coordinates::Vector{C}
    coefficients::Vector{T}
    offset::T
    coordinate_kind::Symbol
    reaction_variable::Int
    acceleration_equation::Int
    velocity_equation::Int
    position_equation::Int
end

function spatial_coordinate_coupler_registration(name::Symbol)
    variables = VariableDeclaration[
        VariableDeclaration(:lambda, :reaction, 2),
    ]
    blocks = EquationBlockDeclaration[
        EquationBlockDeclaration(:acceleration, [
            EquationDeclaration(:Phi_ddot, :constraint, 2,
                :coordinate_coupler)]),
        EquationBlockDeclaration(:velocity, [
            EquationDeclaration(:Phi_dot, :constraint, 1,
                :coordinate_coupler)]),
        EquationBlockDeclaration(:position, [
            EquationDeclaration(:Phi, :constraint, 0,
                :coordinate_coupler)]),
    ]
    families = ConstraintFamilyDeclaration[
        ConstraintFamilyDeclaration(:coordinate_coupler, :lambda, :Phi,
            :Phi_dot, :Phi_ddot),
    ]
    ComponentRegistration(name, variables, blocks, families)
end

component_registration(coupler::SpatialCoordinateCoupler) =
    spatial_coordinate_coupler_registration(coupler.name)

function allocated_spatial_coordinate_coupler(layout, name, coordinates,
        coefficients, offset, coordinate_kind)
    SpatialCoordinateCoupler(name, collect(coordinates),
        collect(coefficients), offset, coordinate_kind,
        only(component_variable_indices(layout, name)),
        only(component_equation_indices(layout, name, :acceleration)),
        only(component_equation_indices(layout, name, :velocity)),
        only(component_equation_indices(layout, name, :position)))
end

spatial_coordinate_variables(hinge::SpatialHingeConstraint) =
    (hinge.rotation_variables[3], hinge.rotation_variables[2],
     hinge.rotation_variables[1])
spatial_coordinate_variables(inline::SpatialInlineConstraint) =
    (inline.translation_variables[3], inline.translation_variables[2],
     inline.translation_variables[1])

function executable_blocks(coupler::SpatialCoordinateCoupler)
    rows = (coupler.position_equation, coupler.velocity_equation,
        coupler.acceleration_equation)
    groups = spatial_coordinate_variables.(coupler.coordinates)
    columns = ([group[1] for group in groups],
        [group[2] for group in groups], [group[3] for group in groups])
    offsets = (coupler.offset, zero(coupler.offset), zero(coupler.offset))
    names = (:position, :velocity, :acceleration)
    blocks = ExecutableEquationBlock[]
    for (name, row, variables, offset) in zip(names, rows, columns, offsets)
        residual! = function (equations, t, z, zdot)
            equations[row] = dot(coupler.coefficients, z[variables]) - offset
        end
        jacobian! = function (jacobian, t, z, zdot, coefficient)
            for (variable, factor) in zip(variables, coupler.coefficients)
                jacobian[row, variable] += factor
            end
        end
        push!(blocks, ExecutableEquationBlock(coupler.name, name, [row],
            residual!, jacobian!))
    end
    blocks
end

coordinate_target_rows(hinge::SpatialHingeConstraint) =
    unique!([body_torque_rows(hinge.marker_i);
             body_torque_rows(hinge.marker_j)])
coordinate_target_rows(inline::SpatialInlineConstraint) =
    directed_distance_reaction_rows(inline.axial_geometry)

function add_coordinate_reaction!(equations, z,
        hinge::SpatialHingeConstraint, coupler, factor)
    axis = marker_axis_kinematics(hinge.marker_j, z, 3)
    torque = factor * z[coupler.reaction_variable] .* axis.direction
    add_perp_reaction_to_body!(equations, z, hinge.marker_i, torque, 1)
    add_perp_reaction_to_body!(equations, z, hinge.marker_j, torque, -1)
end

function add_coordinate_reaction!(equations, z,
        inline::SpatialInlineConstraint, coupler, factor)
    add_directed_distance_reaction!(equations, z, inline.axial_geometry,
        coupler.reaction_variable, factor)
end

function add_coordinate_reaction_jacobian!(jacobian, z,
        hinge::SpatialHingeConstraint, coupler, factor)
    axis = marker_axis_kinematics(hinge.marker_j, z, 3)
    normal_derivatives = Tuple{Any,Any}[]
    if !isnothing(axis.body)
        push!(normal_derivatives,
            (axis.body, factor .* axis.direction_parameters))
    end
    normal = factor .* axis.direction
    add_perp_reaction_jacobian!(jacobian, z, coupler, hinge.marker_i, 1,
        normal, normal_derivatives)
    add_perp_reaction_jacobian!(jacobian, z, coupler, hinge.marker_j, -1,
        normal, normal_derivatives)
end

function add_coordinate_reaction_jacobian!(jacobian, z,
        inline::SpatialInlineConstraint, coupler, factor)
    add_directed_distance_reaction_jacobian!(jacobian, z,
        inline.axial_geometry, coupler.reaction_variable, factor)
end

function equation_contributions(coupler::SpatialCoordinateCoupler)
    rows = unique!(collect(Iterators.flatten(
        coordinate_target_rows(coordinate)
        for coordinate in coupler.coordinates)))
    residual! = function (equations, t, z, zdot)
        for (coordinate, factor) in
                zip(coupler.coordinates, coupler.coefficients)
            add_coordinate_reaction!(equations, z, coordinate, coupler,
                factor)
        end
    end
    jacobian! = function (jacobian, t, z, zdot, coefficient)
        for (coordinate, factor) in
                zip(coupler.coordinates, coupler.coefficients)
            add_coordinate_reaction_jacobian!(jacobian, z, coordinate,
                coupler, factor)
        end
    end
    [EquationContribution(coupler.name, :reaction_to_coordinates, rows,
        residual!, jacobian!)]
end

end
