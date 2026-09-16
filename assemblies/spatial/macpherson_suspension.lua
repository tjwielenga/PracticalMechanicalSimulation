-- Independent MacPherson front suspension built from ordinary Sim3D elements.
-- Geometry inputs are initial global points and all units are SI.

local sim3d = require "sim3d"
local rigid_body, marker, revolute, spherical, inline, spanning_motion,
    spanning_force, model_table =
    sim3d.rigid_body, sim3d.marker, sim3d.revolute, sim3d.spherical,
    sim3d.inline, sim3d.spanning_motion, sim3d.spanning_force,
    sim3d.model_table
local vector, cross, norm, unit, frame, box_inertia =
    sim3d.vector, sim3d.cross, sim3d.norm, sim3d.unit, sim3d.frame,
    sim3d.box_inertia
local local_point, local_frame = sim3d.local_point, sim3d.local_frame
local required, positive = sim3d.required, sim3d.positive
local context = "macpherson_suspension"

local function frame_with_z(direction, hint)
    local z = unit(direction)
    local x = hint-z*(hint[1]*z[1]+hint[2]*z[2]+hint[3]*z[3])
    if norm(x) <= 1.0e-12 then
        error(context .. " orientation hint is parallel to its axis")
    end
    x = unit(x)
    return frame(x, cross(z, x), z)
end

local function body_size(points, diameter)
    local low = {points[1][1], points[1][2], points[1][3]}
    local high = {points[1][1], points[1][2], points[1][3]}
    for _, point in ipairs(points) do
        for axis = 1, 3 do
            low[axis] = math.min(low[axis], point[axis])
            high[axis] = math.max(high[axis], point[axis])
        end
    end
    return {
        math.max(high[1]-low[1], diameter),
        math.max(high[2]-low[2], diameter),
        math.max(high[3]-low[3], diameter)
    }
end

local function macpherson_suspension(p)
    local name = required(p, "name", context)
    local frame_body = required(p, "frame", context)
    local lower_front = vector(required(p, "lower_front", context))
    local lower_rear = vector(required(p, "lower_rear", context))
    local lower_ball = vector(required(p, "lower_ball", context))
    local strut_lower = vector(required(p, "strut_lower", context))
    local strut_top = vector(required(p, "strut_top", context))
    local tie_rod_inner = vector(required(p, "tie_rod_inner", context))
    local tie_rod_outer = vector(required(p, "tie_rod_outer", context))
    local lower_arm_mass = positive(p, "lower_arm_mass", context)
    local spindle_mass = positive(p, "spindle_mass", context)
    local strut_stiffness = positive(p, "strut_stiffness", context)
    local damping_time_scale = p.damping_time_scale or 0.03
    local nominal_load = p.nominal_load or 0.0
    local arm_diameter = p.arm_diameter or 0.035
    local spindle_diameter = p.spindle_diameter or 0.055
    local arm_color = p.arm_color or "steelblue"
    local spindle_color = p.spindle_color or "darkorange"
    local strut_color = p.strut_color or "gray35"
    local forward = vector(p.forward_direction or {1.0, 0.0, 0.0})
    local initial_velocity = forward*(p.forward_speed or 0.0)

    if damping_time_scale < 0 or nominal_load < 0 then
        error(context .. " damping_time_scale and nominal_load " ..
            "must be nonnegative")
    end
    if arm_diameter <= 0 or spindle_diameter <= 0 then
        error(context .. " graphic diameters must be positive")
    end
    local strut_length = norm(strut_top-strut_lower)
    -- A positive nominal load represents compression. The unloaded spring is
    -- longer than its installed length so it pushes the spindle away from the
    -- upper mount.
    local free_length = p.free_length or
        strut_length+nominal_load/strut_stiffness
    if free_length < 0 then
        error(context .. " free_length must be nonnegative")
    end

    local root = name .. "."
    local frame_root = frame_body .. "." .. name .. "."
    local lower_body = root .. "lower_control_arm"
    local spindle_body = root .. "spindle"

    local lower_center = (lower_front+lower_rear+lower_ball)/3
    local lower_size = body_size(
        {lower_front, lower_rear, lower_ball}, arm_diameter)
    rigid_body {
        name = lower_body,
        mass = lower_arm_mass,
        inertia = box_inertia(lower_arm_mass, lower_size),
        position = lower_center,
        velocity = initial_velocity,
        graphics = {show_default = false, color = arm_color}
    }
    marker {name = lower_body .. ".front", position = lower_front-lower_center}
    marker {name = lower_body .. ".rear", position = lower_rear-lower_center}
    marker {name = lower_body .. ".ball", position = lower_ball-lower_center}
    for index, edge in ipairs {{"front", "rear"}, {"rear", "ball"},
            {"ball", "front"}} do
        model_table {
            name = lower_body .. ".graphics.edge" .. index,
            shape = "cylinder",
            markers = {lower_body .. "." .. edge[1],
                lower_body .. "." .. edge[2]},
            radius = arm_diameter/2,
            color = arm_color
        }
    end

    local lower_mount = 0.5*(lower_front+lower_rear)
    local lower_axis = frame_with_z(lower_rear-lower_front,
        strut_top-strut_lower)
    marker {
        name = lower_body .. ".mount",
        position = lower_mount-lower_center,
        orientation = lower_axis
    }
    marker {
        name = frame_root .. "lower_mount",
        position = local_point(frame_body, lower_mount),
        orientation = local_frame(frame_body, lower_axis)
    }
    revolute {
        name = root .. "lower_arm_hinge",
        markers = {lower_body .. ".mount", frame_root .. "lower_mount"},
        rotation_coordinates = true
    }

    local spindle_center = (lower_ball+strut_lower+tie_rod_outer)/3
    local spindle_size = body_size(
        {lower_ball, strut_lower, tie_rod_outer}, spindle_diameter)
    rigid_body {
        name = spindle_body,
        mass = spindle_mass,
        inertia = box_inertia(spindle_mass, spindle_size),
        position = spindle_center,
        velocity = initial_velocity,
        graphics = {show_default = false, color = spindle_color}
    }
    marker {
        name = spindle_body .. ".lower_ball",
        position = lower_ball-spindle_center
    }
    marker {
        name = spindle_body .. ".strut_lower",
        position = strut_lower-spindle_center,
        orientation = frame_with_z(strut_top-strut_lower, forward)
    }
    marker {
        name = spindle_body .. ".tie_rod_outer",
        position = tie_rod_outer-spindle_center
    }
    model_table {
        name = spindle_body .. ".graphics.upright",
        shape = "cylinder",
        markers = {spindle_body .. ".lower_ball",
            spindle_body .. ".strut_lower"},
        radius = spindle_diameter/2,
        color = spindle_color
    }
    model_table {
        name = spindle_body .. ".graphics.steering_arm",
        shape = "cylinder",
        markers = {spindle_body .. ".lower_ball",
            spindle_body .. ".tie_rod_outer"},
        radius = 0.7*spindle_diameter/2,
        color = spindle_color
    }

    marker {
        name = frame_root .. "strut_top",
        position = local_point(frame_body, strut_top)
    }
    marker {
        name = frame_root .. "tie_rod_inner",
        position = local_point(frame_body, tie_rod_inner)
    }

    spherical {
        name = root .. "lower_ball_joint",
        markers = {spindle_body .. ".lower_ball", lower_body .. ".ball"}
    }
    inline {
        name = root .. "strut_guide",
        markers = {frame_root .. "strut_top", spindle_body .. ".strut_lower"},
        translation_coordinates = true
    }
    spanning_motion {
        name = root .. "toe_link",
        markers = {spindle_body .. ".tie_rod_outer",
            frame_root .. "tie_rod_inner"},
        distance = norm(tie_rod_outer-tie_rod_inner)
    }
    spanning_force {
        name = root .. "strut_spring",
        markers = {spindle_body .. ".strut_lower", frame_root .. "strut_top"},
        stiffness = strut_stiffness,
        damping = strut_stiffness*damping_time_scale,
        free_length = free_length
    }

    -- A fixed-length lower tube makes the telescoping strut visible while the
    -- force connector shows the changing distance to the upper mount.
    local tube_tip = strut_lower+
        unit(strut_top-strut_lower)*math.min(0.35, 0.55*strut_length)
    marker {
        name = spindle_body .. ".strut_tube_tip",
        position = tube_tip-spindle_center
    }
    model_table {
        name = spindle_body .. ".graphics.strut_tube",
        shape = "cylinder",
        markers = {spindle_body .. ".strut_lower",
            spindle_body .. ".strut_tube_tip"},
        radius = 0.6*spindle_diameter,
        color = strut_color
    }

    return {
        lower_control_arm = lower_body,
        spindle = spindle_body,
        strut = root .. "strut_spring",
        toe_link = root .. "toe_link",
        bodies = {lower_body, spindle_body}
    }
end

return macpherson_suspension
