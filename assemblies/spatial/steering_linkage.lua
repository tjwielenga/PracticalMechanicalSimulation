-- Parallelogram steering linkage converted from historical normalSteer.py,
-- with its steering gear completed from historical normalSteer2.pl.

local sim3d = require "sim3d"
local rigid_body, marker, revolute, spherical, perp, bushing, coupler,
    applied_torque, rotational_motion, model_table =
    sim3d.rigid_body, sim3d.marker, sim3d.revolute,
    sim3d.spherical, sim3d.perp, sim3d.bushing, sim3d.coupler,
    sim3d.applied_torque, sim3d.rotational_motion, sim3d.model_table
local vector, dot, cross, norm, unit, frame, link_frame =
    sim3d.vector, sim3d.dot, sim3d.cross, sim3d.norm, sim3d.unit,
    sim3d.frame, sim3d.link_frame
local local_point, local_frame = sim3d.local_point, sim3d.local_frame
local required = sim3d.required
local context = "steering_linkage"

local function frame_with_z(direction, hint)
    local z = unit(direction)
    local x = hint-z*(hint[1]*z[1]+hint[2]*z[2]+hint[3]*z[3])
    if norm(x) <= 1.0e-12 then
        x = vector {1.0, 0.0, 0.0}-z*z[1]
    end
    x = unit(x)
    return frame(x, cross(z, x), z)
end

local function cylinder_properties(first, second, diameter, density, side)
    local length = norm(second-first)
    local radius = diameter/2
    local mass = density*math.pi*radius^2*length
    local axial = 0.5*mass*radius^2
    local transverse = mass*(3*radius^2+length^2)/12
    return {
        center = 0.5*(first+second),
        orientation = link_frame(first, second, side),
        length = length,
        radius = radius,
        mass = mass,
        inertia = {axial, transverse, transverse}
    }
end

local function number_text(value)
    return string.format("%.17g", value)
end

local function steering_wheel_ring(name, marker_name, radius, tube_radius,
        color)
    local around_segments = 32
    local tube_segments = 8
    local vertices = {}
    local faces = {}
    for around = 0, around_segments-1 do
        local phi = 2*math.pi*around/around_segments
        for tube = 0, tube_segments-1 do
            local theta = 2*math.pi*tube/tube_segments
            local radial = radius+tube_radius*math.cos(theta)
            table.insert(vertices, {
                radial*math.cos(phi),
                radial*math.sin(phi),
                tube_radius*math.sin(theta)
            })
        end
    end
    local function index(around, tube)
        return (around%around_segments)*tube_segments+
            (tube%tube_segments)+1
    end
    for around = 0, around_segments-1 do
        for tube = 0, tube_segments-1 do
            table.insert(faces, {
                index(around, tube), index(around+1, tube),
                index(around+1, tube+1), index(around, tube+1)
            })
        end
    end
    model_table {
        name = name,
        shape = "surface",
        marker = marker_name,
        vertices = vertices,
        draw_edges = false,
        patches = {
            rim = {color = color, opacity = 1.0, faces = faces}
        }
    }
end

local function motion_text(value, field)
    if type(value) == "number" then return number_text(value) end
    if type(value) == "string" then return value end
    error("steering_linkage " .. field .. " must be a number or expression")
end

local function steering_linkage(p)
    local name = required(p, "name", context)
    local frame_body = required(p, "frame", context)
    local left_spindle = required(p, "left_spindle", context)
    local right_spindle = required(p, "right_spindle", context)
    local pitman_pivot = vector(required(p, "pitman_pivot", context))
    local pitman_axis_point = vector(required(
        p, "pitman_axis_point", context))
    local center_to_pitman = vector(required(
        p, "center_to_pitman", context))
    local center_to_idler = vector(required(
        p, "center_to_idler", context))
    local idler_pivot = vector(required(p, "idler_pivot", context))
    local idler_axis_point = vector(required(
        p, "idler_axis_point", context))
    local left_inner_tie = vector(required(p, "left_inner_tie", context))
    local left_knuckle = vector(required(p, "left_knuckle", context))
    local right_inner_tie = vector(required(p, "right_inner_tie", context))
    local right_knuckle = vector(required(p, "right_knuckle", context))
    local steering_wheel = vector(required(p, "steering_wheel", context))
    local steering_column_end = vector(required(
        p, "steering_column_end", context))
    local diameter = required(p, "diameter", context)
    local wheel_diameter = p.wheel_diameter or 0.30
    local steel_density = p.steel_density or 7850.0
    local plastic_density = p.plastic_density or 1200.0
    local up = vector(p.up or {0.0, 0.0, 1.0})
    local fore_aft_hint = vector(p.fore_aft or {1.0, 0.0, 0.0})
    local link_color = p.link_color or "gray35"
    local tie_color = p.tie_color or "steelblue"
    local wheel_color = p.wheel_color or "black"
    local gear_ratio = p.gear_ratio or 1.0
    local windup_stiffness = p.windup_stiffness or 0.0
    local windup_damping = p.windup_damping or 0.0
    local free_play = p.free_play or 0.0
    local rotational_damping = p.rotational_damping or 0.0
    local pitman_motion_angle = p.pitman_motion_angle
    local wheel_motion_angle = p.wheel_motion_angle
    local root = name .. "."
    local frame_root = frame_body .. "." .. name .. "."

    if math.abs(gear_ratio) <= 1.0e-12 then
        error("steering_linkage gear_ratio must be nonzero")
    end
    if pitman_motion_angle ~= nil and wheel_motion_angle ~= nil then
        error("steering_linkage cannot prescribe both pitman_motion_angle " ..
            "and wheel_motion_angle")
    end
    if windup_stiffness < 0 or windup_damping < 0 or
            free_play < 0 or rotational_damping < 0 then
        error("steering_linkage stiffness, damping, and free play must be nonnegative")
    end

    local function make_rod(role, first, second, rod_diameter, color)
        local body = root .. role
        local properties = cylinder_properties(
            first, second, rod_diameter, steel_density, up)
        rigid_body {
            name = body,
            mass = properties.mass,
            inertia = properties.inertia,
            center_of_mass = body .. ".cm",
            position = properties.center,
            orientation = properties.orientation,
            graphics = {
                shape = "cylinder", marker = body .. ".cm", axis = "x",
                length = properties.length, radius = properties.radius,
                color = color
            }
        }
        marker {name = body .. ".cm"}
        marker {name = body .. ".first", position = local_point(body, first)}
        marker {name = body .. ".second", position = local_point(body, second)}
        return body
    end

    local left_tie = make_rod("left_tie_rod", left_inner_tie,
        left_knuckle, diameter, tie_color)
    local right_tie = make_rod("right_tie_rod", right_inner_tie,
        right_knuckle, diameter, tie_color)
    local center_link = make_rod("center_link", left_inner_tie,
        right_inner_tie, diameter, link_color)
    local pitman = make_rod("pitman_arm", pitman_pivot,
        center_to_pitman, diameter, link_color)
    local idler = make_rod("idler_arm", idler_pivot,
        center_to_idler, diameter, link_color)

    local function add_body_marker(body, role, point, orientation)
        marker {
            name = body .. "." .. role,
            position = local_point(body, point),
            orientation = orientation and local_frame(body, orientation) or nil
        }
        return body .. "." .. role
    end
    local function add_external_marker(body, role, point, orientation)
        marker {
            name = body .. "." .. role,
            position = local_point(body, point),
            orientation = orientation and local_frame(body, orientation) or nil
        }
        return body .. "." .. role
    end

    local pitman_frame = frame_with_z(pitman_axis_point-pitman_pivot, up)
    local idler_frame = frame_with_z(idler_axis_point-idler_pivot, up)
    local pitman_body_pivot = add_body_marker(
        pitman, "pivot", pitman_pivot, pitman_frame)
    local pitman_frame_pivot = add_external_marker(
        frame_body, name .. ".pitman_pivot", pitman_pivot, pitman_frame)
    local idler_body_pivot = add_body_marker(
        idler, "pivot", idler_pivot, idler_frame)
    local idler_frame_pivot = add_external_marker(
        frame_body, name .. ".idler_pivot", idler_pivot, idler_frame)
    revolute {
        name = root .. "pitman_pivot",
        markers = {pitman_body_pivot, pitman_frame_pivot},
        rotation_coordinates = true
    }
    revolute {
        name = root .. "idler_pivot",
        markers = {idler_body_pivot, idler_frame_pivot},
        rotation_coordinates = true
    }

    local function ball(name_suffix, body_a, point_a, body_b, point_b,
            orientation_a, orientation_b)
        local marker_a = add_body_marker(body_a, name_suffix .. "_a",
            point_a, orientation_a)
        local marker_b = add_external_marker(body_b,
            name .. "." .. name_suffix .. "_b", point_b, orientation_b)
        spherical {
            name = root .. name_suffix,
            markers = {marker_a, marker_b}
        }
        return marker_a, marker_b
    end
    local pitman_arm_axis = unit(center_to_pitman-pitman_pivot)
    local center_link_fore_aft = fore_aft_hint-
        pitman_arm_axis*dot(fore_aft_hint, pitman_arm_axis)
    if norm(center_link_fore_aft) <= 1.0e-12 then
        error("steering_linkage fore_aft direction must not be parallel " ..
            "to the pitman arm")
    end
    center_link_fore_aft = unit(center_link_fore_aft)
    local pitman_center_frame = frame(pitman_arm_axis,
        center_link_fore_aft,
        unit(cross(pitman_arm_axis, center_link_fore_aft)))
    local pitman_center_marker, center_pitman_marker = ball(
        "pitman_to_center", pitman, center_to_pitman,
        center_link, center_to_pitman,
        pitman_center_frame, pitman_center_frame)
    perp {
        name = root .. "pitman_to_center_perp",
        markers = {pitman_center_marker, center_pitman_marker}
    }
    ball("idler_to_center", idler, center_to_idler,
        center_link, center_to_idler)
    ball("left_inner_tie", left_tie, left_inner_tie,
        center_link, left_inner_tie)
    ball("right_inner_tie", right_tie, right_inner_tie,
        center_link, right_inner_tie)
    ball("left_outer_tie", left_tie, left_knuckle,
        left_spindle, left_knuckle)
    ball("right_outer_tie", right_tie, right_knuckle,
        right_spindle, right_knuckle)

    local pitman_joint = root .. "pitman_pivot"
    local wheel_body = nil
    local windup_body = nil
    local windup_joint = nil
    local steering_coupler = nil
    local wheel_motion = nil

    if pitman_motion_angle ~= nil then
        -- This direct steering option removes the steering column, gear
        -- coupler, windup compliance, and pitman damping. It is useful when
        -- the steering linkage itself is being assembled or diagnosed.
        rotational_motion {
            name = root .. "pitman_motion",
            joint = pitman_joint,
            angle = motion_text(pitman_motion_angle, "pitman_motion_angle")
        }
    else
        -- The Python source stops after constructing this steering-wheel
        -- body. The coupler, windup shaft, compliance, free play, and pitman
        -- damping below follow the older normalSteer2.pl formulation.
        local column = cylinder_properties(steering_wheel,
            steering_column_end, diameter, steel_density, up)
        local torus_radius = wheel_diameter/2
        local tube_radius = diameter/2
        local torus_mass = plastic_density*2*math.pi^2*torus_radius*tube_radius^2
        wheel_body = root .. "steering_wheel"
        rigid_body {
            name = wheel_body,
            mass = column.mass+torus_mass,
            inertia = {
                column.inertia[1]+torus_mass*torus_radius^2,
                column.inertia[2]+0.5*torus_mass*torus_radius^2,
                column.inertia[3]+0.5*torus_mass*torus_radius^2
            },
            center_of_mass = wheel_body .. ".cm",
            position = column.center,
            orientation = column.orientation,
            graphics = {show_default = false, color = wheel_color}
        }
        marker {name = wheel_body .. ".cm"}
        marker {
            name = wheel_body .. ".wheel_center",
            position = local_point(wheel_body, steering_wheel),
            orientation = local_frame(wheel_body,
                frame_with_z(steering_wheel-steering_column_end, up))
        }
        marker {
            name = wheel_body .. ".column_mount",
            position = local_point(wheel_body, steering_column_end),
            orientation = local_frame(wheel_body,
                frame_with_z(steering_wheel-steering_column_end, up))
        }
        marker {
            name = frame_root .. "steering_column_mount",
            position = local_point(frame_body, steering_column_end),
            orientation = local_frame(frame_body,
                frame_with_z(steering_wheel-steering_column_end, up))
        }
        revolute {
            name = root .. "steering_column",
            markers = {wheel_body .. ".column_mount",
                frame_root .. "steering_column_mount"},
            rotation_coordinates = true
        }

        local steering_joint = root .. "steering_column"
        if wheel_motion_angle ~= nil then
            wheel_motion = root .. "steering_wheel_motion"
            rotational_motion {
                name = wheel_motion,
                joint = steering_joint,
                angle = motion_text(wheel_motion_angle,
                    "wheel_motion_angle")
            }
        end
        steering_coupler = root .. "steering_gear"
        if windup_stiffness > 0 then
            -- normalSteer2.pl inserts this massless rotating shaft between
            -- the rigid gear-ratio coupler and compliant sector shaft.
            windup_body = root .. "windup_shaft"
            windup_joint = root .. "windup_shaft_pivot"
            local tiny_mass = p.tiny_mass or 1.0e-6
            local tiny_inertia = p.tiny_inertia or {1.0e-8, 1.0e-8, 1.0e-8}
            rigid_body {
                name = windup_body,
                mass = tiny_mass,
                inertia = tiny_inertia,
                center_of_mass = windup_body .. ".cm",
                position = pitman_pivot,
                orientation = pitman_frame,
                graphics = {show_default = false}
            }
            marker {name = windup_body .. ".cm"}
            marker {name = windup_body .. ".pivot"}
            revolute {
                name = windup_joint,
                markers = {windup_body .. ".pivot", pitman_frame_pivot},
                rotation_coordinates = true
            }
            coupler {
                name = steering_coupler,
                coordinates = {
                    steering_joint .. ".rotation",
                    windup_joint .. ".rotation"
                },
                coefficients = {-1.0, gear_ratio},
                offset = "initial"
            }

            if free_play > 0 then
                -- The Perl BISTOP half-gap is the steering-wheel free play
                -- divided by twice the absolute gear ratio. Two opposite
                -- joint torques reproduce an internal dead-band torque.
                local half_gap = free_play/(2*math.abs(gear_ratio))
                local difference = "(" .. pitman_joint .. ".theta-" ..
                    windup_joint .. ".theta)"
                local deadband = "(max(" .. difference .. "-(" ..
                    number_text(half_gap) .. "),0)+min(" .. difference ..
                    "+(" .. number_text(half_gap) .. "),0))"
                local pitman_torque = "-" .. number_text(windup_stiffness) ..
                    "*" .. deadband
                applied_torque {
                    name = root .. "windup_on_pitman",
                    joint = pitman_joint,
                    expression = pitman_torque
                }
                applied_torque {
                    name = root .. "windup_on_shaft",
                    joint = windup_joint,
                    expression = "-(" .. pitman_torque .. ")"
                }
            else
                bushing {
                    name = root .. "windup_spring_damper",
                    markers = {windup_body .. ".pivot", pitman_body_pivot},
                    translational_stiffness = {0.0, 0.0, 0.0},
                    rotational_stiffness = {0.0, 0.0, windup_stiffness},
                    translational_damping = {0.0, 0.0, 0.0},
                    rotational_damping = {0.0, 0.0, windup_damping}
                }
            end
        else
            coupler {
                name = steering_coupler,
                coordinates = {
                    steering_joint .. ".rotation",
                    pitman_joint .. ".rotation"
                },
                coefficients = {-1.0, gear_ratio},
                offset = "initial"
            }
        end

        if rotational_damping > 0 then
            applied_torque {
                name = root .. "pitman_damper",
                joint = pitman_joint,
                stiffness = 0.0,
                damping = rotational_damping,
                free_angle = "initial"
            }
        end

        model_table {
            name = wheel_body .. ".graphics.column",
            shape = "cylinder", marker = wheel_body .. ".cm", axis = "x",
            length = column.length, radius = column.radius, color = wheel_color
        }
        steering_wheel_ring(wheel_body .. ".graphics.rim",
            wheel_body .. ".wheel_center", torus_radius, tube_radius,
            wheel_color)
    end

    local bodies = {left_tie, right_tie, center_link, pitman, idler}
    if wheel_body then table.insert(bodies, wheel_body) end
    if windup_body then table.insert(bodies, windup_body) end

    return {
        left_tie_rod = left_tie,
        right_tie_rod = right_tie,
        center_link = center_link,
        pitman_arm = pitman,
        idler_arm = idler,
        steering_wheel = wheel_body,
        steering_gear = {
            coupler = steering_coupler,
            pitman_motion = pitman_motion_angle ~= nil and
                root .. "pitman_motion" or nil,
            steering_wheel_motion = wheel_motion,
            windup_shaft = windup_body,
            windup_joint = windup_joint,
            gear_ratio = gear_ratio,
            windup_stiffness = windup_stiffness,
            windup_damping = windup_damping,
            free_play = free_play,
            rotational_damping = rotational_damping
        },
        bodies = bodies
    }
end

return steering_linkage
