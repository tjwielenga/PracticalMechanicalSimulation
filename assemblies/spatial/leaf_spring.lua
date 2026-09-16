-- Reusable segmented leaf spring. Geometry inputs are initial global points.

local sim3d = require "sim3d"
local rigid_body, marker, spherical, perp, bushing, model_table =
    sim3d.rigid_body, sim3d.marker, sim3d.spherical, sim3d.perp,
    sim3d.bushing, sim3d.model_table
local vector, dot, cross, norm, unit, link_frame =
    sim3d.vector, sim3d.dot, sim3d.cross, sim3d.norm, sim3d.unit,
    sim3d.link_frame
local frame, rotation, compose, box_inertia =
    sim3d.frame, sim3d.rotation, sim3d.compose, sim3d.box_inertia
local local_point, local_frame = sim3d.local_point, sim3d.local_frame
local required = sim3d.required
local context = "leaf_spring"

local function leaf_spring(p)
    local name = required(p, "name", context)
    local frame_body = required(p, "frame", context)
    local leaf_body = required(p, "leaf", context)
    local front_eye = vector(required(p, "front_eye", context))
    local leaf_center = vector(required(p, "leaf_center", context))
    local rear_shackle_eye = vector(required(
        p, "rear_shackle_eye", context))
    local rear_frame_eye = vector(required(p, "rear_frame_eye", context))
    local up = vector(p.up or {0.0, 0.0, 1.0})
    local leaf_stiffness = required(p, "leaf_stiffness", context)
    local twist_stiffness = p.twist_stiffness or 0.0
    local width = required(p, "width", context)
    local thickness = p.thickness or width/5
    local mount_stiffness = required(p, "mount_stiffness", context)
    local mount_rotational_stiffness =
        required(p, "mount_rotational_stiffness", context)
    local damping_time_scale = p.damping_time_scale or 0.006
    local nominal_load = p.nominal_load or 0.0
    local density = p.density or 7850.0
    local leaf_color = p.leaf_color or "steelblue"
    local shackle_color = p.shackle_color or "darkorange"

    local spring_axis = unit(front_eye-rear_shackle_eye)
    local up_axis = unit(up-dot(up, spring_axis)*spring_axis)
    local side_axis = unit(cross(spring_axis, up_axis))
    local spring_frame = frame(spring_axis, up_axis, side_axis)
    local universal_link_frame = frame(spring_axis, side_axis, -up_axis)
    local shackle_frame = link_frame(
        rear_shackle_eye, rear_frame_eye, side_axis)

    local front_length = math.abs(dot(front_eye-leaf_center, spring_axis))
    local rear_length = math.abs(dot(rear_shackle_eye-leaf_center, spring_axis))
    local total_length = front_length+rear_length
    local front_inner = leaf_center+0.25*front_length*spring_axis
    local rear_inner = leaf_center-0.25*rear_length*spring_axis

    local front_bending_stiffness = leaf_stiffness*rear_length/
        total_length*(0.75*front_length)^2
    local rear_bending_stiffness = leaf_stiffness*front_length/
        total_length*(0.75*rear_length)^2
    local front_nominal_angle =
        -(0.5*nominal_load*0.75*front_length)/front_bending_stiffness
    local rear_nominal_angle =
        -(-0.5*nominal_load*0.75*rear_length)/rear_bending_stiffness

    local front_center = 0.5*(front_eye+front_inner)
    local rear_center = 0.5*(rear_inner+rear_shackle_eye)
    local shackle_center = 0.5*(rear_shackle_eye+rear_frame_eye)
    local front_span = norm(front_inner-front_eye)
    local rear_span = norm(rear_shackle_eye-rear_inner)
    local shackle_span = norm(rear_frame_eye-rear_shackle_eye)
    local center_span = norm(rear_inner-front_inner)

    local front_size = {front_span, thickness, width}
    local rear_size = {rear_span, thickness, width}
    local shackle_size = {shackle_span, thickness, width}
    local center_size = {center_span, thickness, width}
    local front_mass = density*front_span*thickness*width
    local rear_mass = density*rear_span*thickness*width
    local shackle_mass = density*shackle_span*thickness*width

    local root = name .. "."
    local front_link = root .. "front_link"
    local rear_link = root .. "rear_link"
    local shackle = root .. "shackle"
    local leaf_root = leaf_body .. "." .. name .. "."
    local frame_root = frame_body .. "." .. name .. "."

    rigid_body {
        name = front_link,
        mass = front_mass,
        inertia = box_inertia(front_mass, front_size),
        center_of_mass = front_link .. ".cm",
        position = front_center,
        graphics = {
            shape = "box", marker = front_link .. ".cm",
            size = front_size, color = leaf_color
        }
    }
    marker {name = front_link .. ".cm", orientation = spring_frame}
    marker {
        name = front_link .. ".inner_joint",
        position = front_inner-front_center,
        orientation = universal_link_frame
    }
    marker {
        name = front_link .. ".inner_bushing",
        position = front_inner-front_center,
        orientation = compose(spring_frame,
            rotation(front_nominal_angle, {0.0, 0.0, 1.0}))
    }
    marker {
        name = front_link .. ".frame_eye",
        position = front_eye-front_center,
        orientation = spring_frame
    }

    rigid_body {
        name = rear_link,
        mass = rear_mass,
        inertia = box_inertia(rear_mass, rear_size),
        center_of_mass = rear_link .. ".cm",
        position = rear_center,
        graphics = {
            shape = "box", marker = rear_link .. ".cm",
            size = rear_size, color = leaf_color
        }
    }
    marker {name = rear_link .. ".cm", orientation = spring_frame}
    marker {
        name = rear_link .. ".inner_joint",
        position = rear_inner-rear_center,
        orientation = universal_link_frame
    }
    marker {
        name = rear_link .. ".inner_bushing",
        position = rear_inner-rear_center,
        orientation = compose(spring_frame,
            rotation(rear_nominal_angle, {0.0, 0.0, 1.0}))
    }
    marker {
        name = rear_link .. ".shackle_eye",
        position = rear_shackle_eye-rear_center,
        orientation = spring_frame
    }

    rigid_body {
        name = shackle,
        mass = shackle_mass,
        inertia = box_inertia(shackle_mass, shackle_size),
        center_of_mass = shackle .. ".cm",
        position = shackle_center,
        orientation = shackle_frame,
        graphics = {
            shape = "box", marker = shackle .. ".cm",
            size = shackle_size, color = shackle_color
        }
    }
    marker {name = shackle .. ".cm"}
    marker {
        name = shackle .. ".leaf_eye",
        position = local_point(shackle, rear_shackle_eye),
        orientation = local_frame(shackle, spring_frame)
    }
    marker {
        name = shackle .. ".frame_eye",
        position = local_point(shackle, rear_frame_eye),
        orientation = local_frame(shackle, spring_frame)
    }

    marker {
        name = leaf_root .. "center",
        position = local_point(leaf_body, leaf_center),
        orientation = local_frame(leaf_body, spring_frame)
    }
    marker {
        name = leaf_root .. "front_joint",
        position = local_point(leaf_body, front_inner),
        orientation = local_frame(leaf_body, spring_frame)
    }
    marker {
        name = leaf_root .. "rear_joint",
        position = local_point(leaf_body, rear_inner),
        orientation = local_frame(leaf_body, spring_frame)
    }
    marker {
        name = leaf_root .. "front_bushing",
        position = local_point(leaf_body, front_inner),
        orientation = local_frame(leaf_body, spring_frame)
    }
    marker {
        name = leaf_root .. "rear_bushing",
        position = local_point(leaf_body, rear_inner),
        orientation = local_frame(leaf_body, spring_frame)
    }
    model_table {
        name = leaf_body .. ".graphics." .. name .. ".central_leaf",
        shape = "box", marker = leaf_root .. "center",
        size = center_size, color = leaf_color
    }

    marker {
        name = frame_root .. "front_eye",
        position = local_point(frame_body, front_eye),
        orientation = local_frame(frame_body, spring_frame)
    }
    marker {
        name = frame_root .. "shackle_eye",
        position = local_point(frame_body, rear_frame_eye),
        orientation = local_frame(frame_body, spring_frame)
    }

    spherical {
        name = root .. "front_universal.spherical",
        markers = {leaf_root .. "front_joint", front_link .. ".inner_joint"}
    }
    perp {
        name = root .. "front_universal.perp",
        markers = {leaf_root .. "front_joint", front_link .. ".inner_joint"}
    }
    spherical {
        name = root .. "rear_universal.spherical",
        markers = {leaf_root .. "rear_joint", rear_link .. ".inner_joint"}
    }
    perp {
        name = root .. "rear_universal.perp",
        markers = {leaf_root .. "rear_joint", rear_link .. ".inner_joint"}
    }

    bushing {
        name = root .. "front_bending",
        markers = {front_link .. ".inner_bushing", leaf_root .. "front_bushing"},
        translational_stiffness = {0.0, 0.0, 0.0},
        rotational_stiffness = {
            twist_stiffness, twist_stiffness, front_bending_stiffness
        },
        damping_time_scale = damping_time_scale
    }
    bushing {
        name = root .. "rear_bending",
        markers = {rear_link .. ".inner_bushing", leaf_root .. "rear_bushing"},
        translational_stiffness = {0.0, 0.0, 0.0},
        rotational_stiffness = {
            twist_stiffness, twist_stiffness, rear_bending_stiffness
        },
        damping_time_scale = damping_time_scale
    }
    bushing {
        name = root .. "front_mount",
        markers = {front_link .. ".frame_eye", frame_root .. "front_eye"},
        translational_stiffness = mount_stiffness,
        rotational_stiffness = mount_rotational_stiffness,
        damping_time_scale = damping_time_scale
    }
    bushing {
        name = root .. "rear_mount",
        markers = {rear_link .. ".shackle_eye", shackle .. ".leaf_eye"},
        translational_stiffness = mount_stiffness,
        rotational_stiffness = mount_rotational_stiffness,
        damping_time_scale = damping_time_scale
    }
    bushing {
        name = root .. "shackle_mount",
        markers = {shackle .. ".frame_eye", frame_root .. "shackle_eye"},
        translational_stiffness = mount_stiffness,
        rotational_stiffness = mount_rotational_stiffness,
        damping_time_scale = damping_time_scale
    }

    return {
        front_link = front_link,
        rear_link = rear_link,
        shackle = shackle,
        bodies = {front_link, rear_link, shackle}
    }
end

return leaf_spring
