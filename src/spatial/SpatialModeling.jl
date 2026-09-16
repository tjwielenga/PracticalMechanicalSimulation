"""Geometry and allocation helpers for the spatial modeler."""
module SpatialModeling

using LinearAlgebra
using ..SpatialComponentAssembly

export SpatialGroundMarker, SpatialBodyMarker, SpatialFloatingMarker,
       spatial_marker_position, spatial_marker_orientation,
       spatial_marker_velocity, spatial_marker_acceleration,
       allocated_spatial_body

"""Oriented marker fixed in the global frame."""
struct SpatialGroundMarker{T}
    name::Symbol
    position::Vector{T}
    orientation::Matrix{T}
end

"""
Oriented marker fixed in a body's reference frame.

Both `position_body` and `orientation_body` are expressed from that reference
frame, not necessarily from the body's center of mass. Body-reference to CM
offsets are handled when the model is loaded.
"""
struct SpatialBodyMarker{B,T}
    name::Symbol
    body::B
    position_body::Vector{T}
    orientation_body::Matrix{T}
end

"""
Point owned by one body while following another marker's global point.

Its position, velocity, and acceleration follow `follower`, but applied force
and moment contributions go to `body`. Gear, rack, and belt elements use this
to place a contact load on a moving body without adding a kinematic constraint.
"""
struct SpatialFloatingMarker{B,F}
    name::Symbol
    body::B
    follower::F
end

function spatial_marker_position(marker::SpatialGroundMarker, z)
    marker.position
end

function spatial_marker_position(marker::SpatialBodyMarker, z)
    parameters = @view z[marker.body.euler_parameter_variables]
    z[marker.body.position_variables] .+
        rotation_matrix(parameters) * marker.position_body
end

spatial_marker_position(marker::SpatialFloatingMarker, z) =
    spatial_marker_position(marker.follower, z)

spatial_marker_velocity(::SpatialGroundMarker, z) = zeros(eltype(z), 3)

function spatial_marker_velocity(marker::SpatialBodyMarker, z)
    body = marker.body
    parameters = @view z[body.euler_parameter_variables]
    omega = @view z[body.angular_velocity_variables]
    z[body.velocity_variables] .+
        rotation_matrix(parameters) * cross(omega, marker.position_body)
end

spatial_marker_velocity(marker::SpatialFloatingMarker, z) =
    spatial_marker_velocity(marker.follower, z)

spatial_marker_acceleration(::SpatialGroundMarker, z) = zeros(eltype(z), 3)

function spatial_marker_acceleration(marker::SpatialBodyMarker, z)
    body = marker.body
    parameters = @view z[body.euler_parameter_variables]
    omega = @view z[body.angular_velocity_variables]
    alpha = @view z[body.angular_acceleration_variables]
    local_acceleration = cross(alpha, marker.position_body) .+
        cross(omega, cross(omega, marker.position_body))
    z[body.acceleration_variables] .+
        rotation_matrix(parameters) * local_acceleration
end

spatial_marker_acceleration(marker::SpatialFloatingMarker, z) =
    spatial_marker_acceleration(marker.follower, z)

spatial_marker_orientation(marker::SpatialGroundMarker, z) = marker.orientation

function spatial_marker_orientation(marker::SpatialBodyMarker, z)
    parameters = @view z[marker.body.euler_parameter_variables]
    rotation_matrix(parameters) * marker.orientation_body
end

function spatial_marker_orientation(marker::SpatialFloatingMarker, z)
    parameters = @view z[marker.body.euler_parameter_variables]
    rotation_matrix(parameters)
end

end
