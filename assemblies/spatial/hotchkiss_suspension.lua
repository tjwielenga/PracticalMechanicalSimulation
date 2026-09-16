-- Solid rear axle carried by two leaf springs, converted from suspHotchkiss.py.
-- Geometry inputs are initial global points and all units are SI.

local sim3d = require "sim3d"
local rigid_body, marker, model_table =
    sim3d.rigid_body, sim3d.marker, sim3d.model_table
local vector, dot, norm, frame, link_frame, local_point =
    sim3d.vector, sim3d.dot, sim3d.norm, sim3d.frame,
    sim3d.link_frame, sim3d.local_point
local required = sim3d.required
local leaf_spring = require "spatial.leaf_spring"
local context = "hotchkiss_suspension"

local function hotchkiss_suspension(p)
    local name = required(p, "name", context)
    local frame_body = required(p, "frame", context)
    local left_axle_end = vector(required(p, "left_axle_end", context))
    local right_axle_end = vector(required(p, "right_axle_end", context))
    local axle_mass = required(p, "axle_mass", context)
    local axle_inertia = required(p, "axle_inertia", context)
    local axle_diameter = required(p, "axle_diameter", context)
    local differential_diameter = p.differential_diameter or 2*axle_diameter
    local up = vector(p.up or {0.0, 0.0, 1.0})
    local axle_color = p.axle_color or "gray35"
    local axle = name .. ".axle"
    local axle_center = 0.5*(left_axle_end+right_axle_end)
    local axle_length = norm(right_axle_end-left_axle_end)
    local axle_frame = link_frame(left_axle_end, right_axle_end, up)

    rigid_body {
        name = axle,
        mass = axle_mass,
        inertia = axle_inertia,
        center_of_mass = axle .. ".cm",
        position = axle_center,
        orientation = axle_frame,
        graphics = {
            shape = "cylinder", marker = axle .. ".cm", axis = "x",
            length = axle_length, radius = axle_diameter/2,
            color = axle_color
        }
    }
    marker {name = axle .. ".cm"}
    marker {
        name = axle .. ".differential",
        orientation = frame(
            vector {0.0, 1.0, 0.0},
            vector {0.0, 0.0, 1.0},
            vector {1.0, 0.0, 0.0})
    }
    model_table {
        name = axle .. ".graphics.differential",
        shape = "frustum", marker = axle .. ".differential",
        length = math.max(1.5*axle_diameter, 0.18),
        radius_1 = differential_diameter/2,
        radius_2 = differential_diameter/2,
        color = axle_color
    }

    local common = {
        frame = frame_body,
        leaf = axle,
        up = up,
        leaf_stiffness = required(p, "leaf_stiffness", context),
        twist_stiffness = required(p, "twist_stiffness", context),
        width = required(p, "leaf_width", context),
        thickness = p.leaf_thickness,
        mount_stiffness = required(p, "mount_stiffness", context),
        mount_rotational_stiffness =
            required(p, "mount_rotational_stiffness", context),
        damping_time_scale = p.damping_time_scale or 0.01,
        nominal_load = p.nominal_load or 0.0,
        density = p.density or 7850.0,
        leaf_color = p.leaf_color,
        shackle_color = p.shackle_color
    }

    local function add_leaf(side, front_eye, leaf_center,
            rear_shackle_eye, rear_frame_eye)
        return leaf_spring {
            name = name .. "." .. side,
            frame = common.frame,
            leaf = common.leaf,
            front_eye = front_eye,
            leaf_center = leaf_center,
            rear_shackle_eye = rear_shackle_eye,
            rear_frame_eye = rear_frame_eye,
            up = common.up,
            leaf_stiffness = common.leaf_stiffness,
            twist_stiffness = common.twist_stiffness,
            width = common.width,
            thickness = common.thickness,
            mount_stiffness = common.mount_stiffness,
            mount_rotational_stiffness = common.mount_rotational_stiffness,
            damping_time_scale = common.damping_time_scale,
            nominal_load = common.nominal_load,
            density = common.density,
            leaf_color = common.leaf_color,
            shackle_color = common.shackle_color
        }
    end

    local left_leaf = add_leaf("left",
        required(p, "left_front_eye", context),
        required(p, "left_leaf_center", context),
        required(p, "left_rear_shackle_eye", context),
        required(p, "left_rear_frame_eye", context))
    local right_leaf = add_leaf("right",
        required(p, "right_front_eye", context),
        required(p, "right_leaf_center", context),
        required(p, "right_rear_shackle_eye", context),
        required(p, "right_rear_frame_eye", context))

    -- The old graphics added short vertical axle-to-leaf cylinders.  They do
    -- not carry mass or add another mechanical element.
    local function axle_drop(side, axle_end, leaf_point)
        local projection = leaf_point-up*dot(leaf_point-axle_end, up)
        marker {
            name = axle .. "." .. side .. "_spring_seat",
            position = local_point(axle, projection)
        }
        marker {
            name = axle .. "." .. side .. "_leaf_center",
            position = local_point(axle, leaf_point)
        }
        model_table {
            name = axle .. ".graphics." .. side .. "_spring_seat",
            shape = "cylinder",
            markers = {axle .. "." .. side .. "_spring_seat",
                axle .. "." .. side .. "_leaf_center"},
            radius = axle_diameter/2,
            color = axle_color
        }
    end
    axle_drop("left", left_axle_end,
        vector(required(p, "left_leaf_center", context)))
    axle_drop("right", right_axle_end,
        vector(required(p, "right_leaf_center", context)))

    return {
        axle = axle,
        left = left_leaf,
        right = right_leaf,
        bodies = {axle,
            left_leaf.bodies[1], left_leaf.bodies[2], left_leaf.bodies[3],
            right_leaf.bodies[1], right_leaf.bodies[2], right_leaf.bodies[3]}
    }
end

return hotchkiss_suspension
