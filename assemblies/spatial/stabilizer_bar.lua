-- Segmented stabilizer bar converted from the author's earlier model.
-- Geometry inputs are initial global points and all units are SI.

local sim3d = require "sim3d"
local rigid_body, marker, revolute, spherical, perp, bushing =
    sim3d.rigid_body, sim3d.marker, sim3d.revolute,
    sim3d.spherical, sim3d.perp, sim3d.bushing
local vector, dot, cross, norm, unit, frame, link_frame =
    sim3d.vector, sim3d.dot, sim3d.cross, sim3d.norm,
    sim3d.unit, sim3d.frame, sim3d.link_frame
local local_point, local_frame = sim3d.local_point, sim3d.local_frame
local required = sim3d.required
local context = "stabilizer_bar"

local function frame_with_z(z_direction, x_hint)
    local z = unit(z_direction)
    local x = unit(x_hint-dot(x_hint, z)*z)
    return frame(x, cross(z, x), z)
end

local function universal_frame(standoff_direction, arm_direction)
    local x = unit(cross(standoff_direction, arm_direction))
    local y = unit(arm_direction-dot(arm_direction, x)*x)
    return frame(x, y, cross(x, y))
end

local function cylinder_properties(point_a, point_b, diameter, density, side)
    local length = norm(point_b-point_a)
    if length <= 1.0e-12 then
        error("stabilizer_bar cylinder endpoints must be separated")
    end
    local radius = diameter/2
    local mass = density*math.pi*radius^2*length
    local axial_inertia = 0.5*mass*radius^2
    local transverse_inertia = mass*(3*radius^2+length^2)/12
    return {
        center = 0.5*(point_a+point_b),
        orientation = link_frame(point_a, point_b, side),
        mass = mass,
        inertia = {axial_inertia, transverse_inertia, transverse_inertia}
    }
end

local function stabilizer_bar(p)
    local name = required(p, "name", context)
    local center_body = required(p, "center_body", context)
    local left_body = required(p, "left_body", context)
    local right_body = required(p, "right_body", context)
    local left_center_point = vector(required(
        p, "left_center_point", context))
    local right_center_point = vector(required(
        p, "right_center_point", context))
    local left_body_point = vector(required(p, "left_body_point", context))
    local left_offset_point = vector(required(
        p, "left_offset_point", context))
    local right_body_point = vector(required(
        p, "right_body_point", context))
    local right_offset_point = vector(required(
        p, "right_offset_point", context))
    local torsional_stiffness = required(
        p, "torsional_stiffness", context)
    local diameter = required(p, "diameter", context)
    local density = p.density or 7850.0
    local damping_time_scale = p.damping_time_scale or 0.0
    local up = vector(p.up or {0.0, 0.0, 1.0})
    local bar_color = p.bar_color or "steelblue"
    local link_color = p.link_color or "darkorange"

    if torsional_stiffness < 0 then
        error("stabilizer_bar torsional_stiffness must be nonnegative")
    end
    if diameter <= 0 or density <= 0 then
        error("stabilizer_bar diameter and density must be positive")
    end
    if damping_time_scale < 0 then
        error("stabilizer_bar damping_time_scale must be nonnegative")
    end

    local center = 0.5*(left_center_point+right_center_point)
    local center_axis = unit(right_center_point-left_center_point)
    local torsion_frame = frame_with_z(center_axis, up)
    local left_hinge_frame = frame_with_z(left_center_point-center, up)
    local right_hinge_frame = frame_with_z(right_center_point-center, up)
    local left_universal_frame = universal_frame(
        left_offset_point-left_body_point,
        left_offset_point-left_center_point)
    local right_universal_frame = universal_frame(
        right_offset_point-right_body_point,
        right_offset_point-right_center_point)

    -- The historical model assigned cylinder mass properties to the straight
    -- center halves and the two stand-offs. The bent arms were graphical parts
    -- of the center-half bodies and did not add separate mass properties.
    local left_center_properties = cylinder_properties(
        left_center_point, center, diameter, density, up)
    local right_center_properties = cylinder_properties(
        right_center_point, center, diameter, density, up)
    local left_standoff_properties = cylinder_properties(
        left_offset_point, left_body_point, diameter, density, center_axis)
    local right_standoff_properties = cylinder_properties(
        right_offset_point, right_body_point, diameter, density, center_axis)

    local root = name .. "."
    local left_center = root .. "left_center"
    local right_center = root .. "right_center"
    local left_standoff = root .. "left_standoff"
    local right_standoff = root .. "right_standoff"
    local carrier_root = center_body .. "." .. name .. "."
    local left_body_root = left_body .. "." .. name .. "."
    local right_body_root = right_body .. "." .. name .. "."

    local function add_body(body_name, properties, color)
        rigid_body {
            name = body_name,
            mass = properties.mass,
            inertia = properties.inertia,
            center_of_mass = body_name .. ".cm",
            position = properties.center,
            graphics = {show_default = false, color = color}
        }
        marker {
            name = body_name .. ".cm",
            orientation = properties.orientation
        }
    end

    add_body(left_center, left_center_properties, bar_color)
    add_body(right_center, right_center_properties, bar_color)
    add_body(left_standoff, left_standoff_properties, link_color)
    add_body(right_standoff, right_standoff_properties, link_color)

    local function body_marker(body_name, role, point, orientation)
        marker {
            name = body_name .. "." .. role,
            position = point,
            orientation = orientation
        }
        return body_name .. "." .. role
    end

    local left_center_origin = left_center_properties.center
    local right_center_origin = right_center_properties.center
    local left_standoff_origin = left_standoff_properties.center
    local right_standoff_origin = right_standoff_properties.center

    local left_pivot = body_marker(left_center, "pivot",
        left_center_point-left_center_origin, left_hinge_frame)
    local left_center_end = body_marker(left_center, "center_end",
        center-left_center_origin, torsion_frame)
    local left_offset = body_marker(left_center, "offset",
        left_offset_point-left_center_origin, left_universal_frame)
    local right_pivot = body_marker(right_center, "pivot",
        right_center_point-right_center_origin, right_hinge_frame)
    local right_center_end = body_marker(right_center, "center_end",
        center-right_center_origin, torsion_frame)
    local right_offset = body_marker(right_center, "offset",
        right_offset_point-right_center_origin, right_universal_frame)

    local left_standoff_top = body_marker(left_standoff, "bar_end",
        left_offset_point-left_standoff_origin, left_universal_frame)
    local left_standoff_bottom = body_marker(left_standoff, "body_end",
        left_body_point-left_standoff_origin, left_universal_frame)
    local right_standoff_top = body_marker(right_standoff, "bar_end",
        right_offset_point-right_standoff_origin, right_universal_frame)
    local right_standoff_bottom = body_marker(right_standoff, "body_end",
        right_body_point-right_standoff_origin, right_universal_frame)

    marker {
        name = carrier_root .. "left_pivot",
        position = local_point(center_body, left_center_point),
        orientation = local_frame(center_body, left_hinge_frame)
    }
    marker {
        name = carrier_root .. "right_pivot",
        position = local_point(center_body, right_center_point),
        orientation = local_frame(center_body, right_hinge_frame)
    }
    marker {
        name = left_body_root .. "connection",
        position = local_point(left_body, left_body_point)
    }
    marker {
        name = right_body_root .. "connection",
        position = local_point(right_body, right_body_point)
    }

    revolute {
        name = root .. "left_pivot",
        markers = {left_pivot, carrier_root .. "left_pivot"},
        rotation_coordinates = true
    }
    revolute {
        name = root .. "right_pivot",
        markers = {right_pivot, carrier_root .. "right_pivot"},
        rotation_coordinates = true
    }

    spherical {
        name = root .. "left_universal.spherical",
        markers = {left_offset, left_standoff_top}
    }
    perp {
        name = root .. "left_universal.perp",
        markers = {left_offset, left_standoff_top}
    }
    spherical {
        name = root .. "right_universal.spherical",
        markers = {right_offset, right_standoff_top}
    }
    perp {
        name = root .. "right_universal.perp",
        markers = {right_offset, right_standoff_top}
    }
    spherical {
        name = root .. "left_ball",
        markers = {left_standoff_bottom, left_body_root .. "connection"}
    }
    spherical {
        name = root .. "right_ball",
        markers = {right_standoff_bottom, right_body_root .. "connection"}
    }

    bushing {
        name = root .. "torsion",
        markers = {left_center_end, right_center_end},
        translational_stiffness = {0.0, 0.0, 0.0},
        rotational_stiffness = {0.0, 0.0, torsional_stiffness},
        damping_time_scale = damping_time_scale
    }

    local radius = diameter/2
    local link_radius = diameter/4
    local function add_cylinder(body_name, graphic_name, first, second,
            cylinder_radius, color)
        sim3d.model_table {
            name = body_name .. ".graphics." .. graphic_name,
            shape = "cylinder",
            markers = {first, second},
            radius = cylinder_radius,
            color = color
        }
    end

    add_cylinder(left_center, "center_half", left_pivot, left_center_end,
        radius, bar_color)
    add_cylinder(left_center, "arm", left_pivot, left_offset,
        radius, bar_color)
    add_cylinder(right_center, "center_half", right_center_end, right_pivot,
        radius, bar_color)
    add_cylinder(right_center, "arm", right_pivot, right_offset,
        radius, bar_color)
    add_cylinder(left_standoff, "link", left_standoff_top,
        left_standoff_bottom, link_radius, link_color)
    add_cylinder(right_standoff, "link", right_standoff_top,
        right_standoff_bottom, link_radius, link_color)

    return {
        left_center = left_center,
        right_center = right_center,
        left_standoff = left_standoff,
        right_standoff = right_standoff,
        bodies = {left_center, right_center, left_standoff, right_standoff}
    }
end

return stabilizer_bar
