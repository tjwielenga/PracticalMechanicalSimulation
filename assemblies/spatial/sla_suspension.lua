-- Short-long-arm independent suspension converted from historical suspSLA.py.
-- Geometry inputs are initial global points and all units are SI.

local sim3d = require "sim3d"
local rigid_body, marker, revolute, spherical, bushing, model_table =
    sim3d.rigid_body, sim3d.marker, sim3d.revolute,
    sim3d.spherical, sim3d.bushing, sim3d.model_table
local vector, cross, norm, unit, frame, link_frame, box_inertia =
    sim3d.vector, sim3d.cross, sim3d.norm, sim3d.unit,
    sim3d.frame, sim3d.link_frame, sim3d.box_inertia
local local_point, local_frame = sim3d.local_point, sim3d.local_frame
local required = sim3d.required
local context = "sla_suspension"

local function frame_with_z(direction, hint)
    local z = unit(direction)
    local x = hint-z*(hint[1]*z[1]+hint[2]*z[2]+hint[3]*z[3])
    if norm(x) <= 1.0e-12 then
        x = vector {1.0, 0.0, 0.0}-z*z[1]
    end
    x = unit(x)
    return frame(x, cross(z, x), z)
end

local function cylinder_inertia(mass, length, radius)
    local axial = 0.5*mass*radius^2
    local transverse = mass*(3*radius^2+length^2)/12
    return {axial, transverse, transverse}
end

local function sla_suspension(p)
    local name = required(p, "name", context)
    local frame_body = required(p, "frame", context)
    local spindle_mass = required(p, "spindle_mass", context)
    local upper_front = vector(required(p, "upper_front", context))
    local upper_rear = vector(required(p, "upper_rear", context))
    local upper_ball = vector(required(p, "upper_ball", context))
    local lower_front = vector(required(p, "lower_front", context))
    local lower_rear = vector(required(p, "lower_rear", context))
    local lower_ball = vector(required(p, "lower_ball", context))
    local upper_mount_stiffness = p.upper_mount_stiffness
    local lower_mount_stiffness = p.lower_mount_stiffness
    local damping_time_scale = p.damping_time_scale or 0.01
    local spindle_diameter = p.spindle_diameter or 0.03
    local arm_diameter = p.arm_diameter or spindle_diameter
    local density_color = p.arm_color or "steelblue"
    local spindle_color = p.spindle_color or "darkorange"
    local up = vector(p.up or {0.0, 0.0, 1.0})

    if spindle_mass <= 0 then
        error("sla_suspension spindle_mass must be positive")
    end
    if spindle_diameter <= 0 or arm_diameter <= 0 then
        error("sla_suspension diameters must be positive")
    end

    local root = name .. "."
    local upper_body = root .. "upper_control_arm"
    local lower_body = root .. "lower_control_arm"
    local spindle_body = root .. "spindle"
    local frame_root = frame_body .. "." .. name .. "."

    local function make_arm(body, points, mass, color)
        local center = (points[1]+points[2]+points[3])/3
        local span = points[1]-points[3]
        local size = {
            math.max(math.abs(span[1]), arm_diameter),
            math.max(math.abs(span[2]), arm_diameter),
            math.max(math.abs(span[3]), arm_diameter)
        }
        rigid_body {
            name = body,
            mass = mass,
            inertia = box_inertia(mass, size),
            center_of_mass = body .. ".cm",
            position = center,
            graphics = {show_default = false, color = color}
        }
        marker {name = body .. ".cm"}
        local point_names = {}
        for index, point in ipairs(points) do
            local point_name = body .. ".point" .. index
            marker {name = point_name, position = point-center}
            point_names[index] = point_name
        end
        for index, edge in ipairs {{1, 2}, {2, 3}, {3, 1}} do
            model_table {
                name = body .. ".graphics.edge" .. index,
                shape = "cylinder",
                markers = {point_names[edge[1]], point_names[edge[2]]},
                radius = arm_diameter/2,
                color = color
            }
        end
        return center, point_names
    end

    local upper_center, upper_points = make_arm(upper_body,
        {upper_front, upper_rear, upper_ball}, spindle_mass/4, density_color)
    local lower_center, lower_points = make_arm(lower_body,
        {lower_front, lower_rear, lower_ball}, spindle_mass/4, density_color)

    local spindle_center = 0.5*(lower_ball+upper_ball)
    local spindle_axis = upper_ball-lower_ball
    local spindle_length = norm(spindle_axis)
    local spindle_radius = spindle_diameter/2
    local spindle_orientation = link_frame(lower_ball, upper_ball, up)
    rigid_body {
        name = spindle_body,
        mass = spindle_mass/2,
        inertia = cylinder_inertia(spindle_mass/2,
            spindle_length, spindle_radius),
        center_of_mass = spindle_body .. ".cm",
        position = spindle_center,
        orientation = spindle_orientation,
        graphics = {
            shape = "cylinder", marker = spindle_body .. ".cm",
            axis = "x", length = spindle_length, radius = spindle_radius,
            color = spindle_color
        }
    }
    marker {name = spindle_body .. ".cm"}
    marker {
        name = spindle_body .. ".upper_ball",
        position = local_point(spindle_body, upper_ball)
    }
    marker {
        name = spindle_body .. ".lower_ball",
        position = local_point(spindle_body, lower_ball)
    }

    local function arm_mount(label, body, body_center, front, rear,
            stiffness, point_names)
        if stiffness == nil then
            local center = 0.5*(front+rear)
            local orientation = frame_with_z(front-rear, up)
            marker {
                name = body .. ".mount",
                position = center-body_center,
                orientation = orientation
            }
            marker {
                name = frame_root .. label .. "_mount",
                position = local_point(frame_body, center),
                orientation = local_frame(frame_body, orientation)
            }
            revolute {
                name = root .. label .. "_hinge",
                markers = {body .. ".mount", frame_root .. label .. "_mount"},
                rotation_coordinates = true
            }
        else
            for index, item in ipairs {
                {"front", front, point_names[1]},
                {"rear", rear, point_names[2]}
            } do
                local role, point, body_marker = item[1], item[2], item[3]
                marker {
                    name = frame_root .. label .. "_" .. role,
                    position = local_point(frame_body, point)
                }
                bushing {
                    name = root .. label .. "_" .. role .. "_bushing",
                    markers = {body_marker,
                        frame_root .. label .. "_" .. role},
                    translational_stiffness = stiffness,
                    rotational_stiffness = p.mount_rotational_stiffness or
                        {0.0, 0.0, 0.0},
                    damping_time_scale = damping_time_scale
                }
            end
        end
    end

    arm_mount("upper", upper_body, upper_center, upper_front, upper_rear,
        upper_mount_stiffness, upper_points)
    arm_mount("lower", lower_body, lower_center, lower_front, lower_rear,
        lower_mount_stiffness, lower_points)

    spherical {
        name = root .. "upper_ball_joint",
        markers = {spindle_body .. ".upper_ball", upper_points[3]}
    }
    spherical {
        name = root .. "lower_ball_joint",
        markers = {spindle_body .. ".lower_ball", lower_points[3]}
    }

    return {
        upper_control_arm = upper_body,
        lower_control_arm = lower_body,
        spindle = spindle_body,
        bodies = {upper_body, lower_body, spindle_body}
    }
end

return sla_suspension
