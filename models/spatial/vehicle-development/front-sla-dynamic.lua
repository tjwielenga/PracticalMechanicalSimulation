local sim3d = require "sim3d"
local model, analysis, simulation, graphics, state_selection,
    initial_conditions,
    ground, rigid_body, marker, spanning_force, spanning_motion, gravity,
    model_table =
    sim3d.model, sim3d.analysis, sim3d.simulation, sim3d.graphics,
    sim3d.state_selection, sim3d.initial_conditions,
    sim3d.ground, sim3d.rigid_body,
    sim3d.marker, sim3d.spanning_force, sim3d.spanning_motion,
    sim3d.gravity, sim3d.model_table
local box_inertia, vector, norm, local_point =
    sim3d.box_inertia, sim3d.vector, sim3d.norm, sim3d.local_point
local sla_suspension = require "spatial.sla_suspension"
local tire = require "spatial.gylt_245_75_r16"

model {
    name = "front_sla_dynamic",
    title = "Vehicle with front SLA suspension — forward dynamics",
    dimension = "spatial"
}

analysis {mode = "dynamic"}

initial_conditions {
    result = "../../../results/examples/spatial/front-sla-static.simp",
    sample = "last",
    include_velocities = false
}

simulation {
    start_time = 0.0,
    end_time = 1.0,
    output_samples = 61,
    relative_tolerance = 1.0e-6,
    absolute_tolerance = 1.0e-8,
    initial_step = 1.0e-7,
    maximum_step = 0.005
}

state_selection {
    method = "preferred",
    preferred_velocities = {
        "vehicle.chassis.V_x", "vehicle.chassis.V_y",
        "vehicle.chassis.V_z", "vehicle.chassis.omega_x",
        "vehicle.chassis.omega_y", "vehicle.chassis.omega_z",
        "vehicle.left_front.lower_hinge.omega",
        "vehicle.right_front.lower_hinge.omega",
        "vehicle.left_front.tire.axle.omega",
        "vehicle.right_front.tire.axle.omega",
        "vehicle.left_rear_tire.axle.omega",
        "vehicle.right_rear_tire.axle.omega"
    },
    allow_fallback = true
}

graphics {background = "white", body_palette = "colorblind"}

ground {name = "ground"}
marker {
    name = "ground.road",
    graphics = {
        shape = "xy_frame", axis_length = 0.5, plane_size = 9.0,
        plane_color = "gray70", opacity = 0.12, label = "road"
    }
}

local speed = 2.0
local chassis_mass = 1576.0
local chassis_size = {3.8, 1.55, 0.55}
rigid_body {
    name = "vehicle.chassis",
    mass = chassis_mass,
    inertia = box_inertia(chassis_mass, chassis_size),
    position = {0.0, 0.0, 0.75},
    velocity = {speed, 0.0, 0.0},
    graphics = {
        shape = "box", marker = "vehicle.chassis.center",
        size = chassis_size, color = "gray65", opacity = 0.28
    }
}
marker {name = "vehicle.chassis.center"}

local front_geometry = {
    left = {
        wheel = {1.55, -0.82, 0.370},
        lower_front = {1.35, -0.46, 0.42},
        lower_rear = {1.72, -0.46, 0.42},
        lower_ball = {1.55, -0.72, 0.32},
        upper_front = {1.35, -0.48, 0.92},
        upper_rear = {1.72, -0.48, 0.92},
        upper_ball = {1.55, -0.70, 0.82},
        spring_lower = {1.55, -0.57, 0.39},
        spring_top = {1.55, -0.50, 1.20},
        tie_inner = {1.35, -0.40, 0.565},
        tie_outer = {1.38, -0.69, 0.47}
    },
    right = {
        wheel = {1.55, 0.82, 0.370},
        lower_front = {1.35, 0.46, 0.42},
        lower_rear = {1.72, 0.46, 0.42},
        lower_ball = {1.55, 0.72, 0.32},
        upper_front = {1.35, 0.48, 0.92},
        upper_rear = {1.72, 0.48, 0.92},
        upper_ball = {1.55, 0.70, 0.82},
        spring_lower = {1.55, 0.57, 0.39},
        spring_top = {1.55, 0.50, 1.20},
        tie_inner = {1.35, 0.40, 0.565},
        tie_outer = {1.38, 0.69, 0.47}
    }
}

local corners = {}
local tires = {}
for side, geometry in pairs(front_geometry) do
    local corner_name = "vehicle." .. side .. "_front"
    corners[side] = sla_suspension {
        name = corner_name,
        frame = "vehicle.chassis",
        upper_front = geometry.upper_front,
        upper_rear = geometry.upper_rear,
        upper_ball = geometry.upper_ball,
        lower_front = geometry.lower_front,
        lower_rear = geometry.lower_rear,
        lower_ball = geometry.lower_ball,
        spindle_mass = 32.0,
        damping_time_scale = 0.03,
        arm_color = "steelblue",
        spindle_color = "darkorange"
    }
    for _, body in ipairs(corners[side].bodies) do
        model_table {name = body, velocity = {speed, 0.0, 0.0}}
    end

    local spring_top = "vehicle.chassis." .. side .. "_front_spring_top"
    local spring_bottom = corners[side].lower_control_arm .. ".spring"
    marker {
        name = spring_top,
        position = local_point("vehicle.chassis", geometry.spring_top)
    }
    marker {
        name = spring_bottom,
        position = local_point(corners[side].lower_control_arm,
            geometry.spring_lower)
    }
    local spring_stiffness = 175000.0
    local installed_length = norm(
        vector(geometry.spring_top)-vector(geometry.spring_lower))
    spanning_force {
        name = corner_name .. ".spring",
        markers = {spring_bottom, spring_top},
        stiffness = spring_stiffness,
        damping = spring_stiffness*0.03,
        free_length = installed_length+4200.0/spring_stiffness
    }

    local tie_inner = "vehicle.chassis." .. side .. "_front_tie_inner"
    local tie_outer = corners[side].spindle .. ".tie_rod_outer"
    marker {
        name = tie_inner,
        position = local_point("vehicle.chassis", geometry.tie_inner)
    }
    marker {
        name = tie_outer,
        position = local_point(corners[side].spindle, geometry.tie_outer)
    }
    spanning_motion {
        name = corner_name .. ".toe_link",
        markers = {tie_outer, tie_inner},
        distance = norm(vector(geometry.tie_outer)-vector(geometry.tie_inner))
    }
    tires[side .. "_front"] = tire {
        name = corner_name .. ".tire",
        spindle = corners[side].spindle,
        center = geometry.wheel,
        road_marker = "ground.road",
        pressure_psi = 60,
        axle_direction = {0.0, 1.0, 0.0},
        forward_direction = {1.0, 0.0, 0.0},
        forward_speed = speed,
        normal_damping_time_scale = 0.01,
        show_default = side ~= "left",
        color = "gray20"
    }
end

for side, y in pairs {left = -0.78, right = 0.78} do
    tires[side .. "_rear"] = tire {
        name = "vehicle." .. side .. "_rear_tire",
        spindle = "vehicle.chassis",
        center = {-1.55, y, 0.370},
        road_marker = "ground.road",
        pressure_psi = 60,
        axle_direction = {0.0, 1.0, 0.0},
        forward_direction = {1.0, 0.0, 0.0},
        forward_speed = speed,
        normal_damping_time_scale = 0.01,
        color = "gray20"
    }
end

local bodies = {"vehicle.chassis"}
for _, corner in pairs(corners) do
    for _, body in ipairs(corner.bodies) do table.insert(bodies, body) end
end
for _, wheel in pairs(tires) do table.insert(bodies, wheel.body) end
gravity {
    name = "gravity",
    acceleration = {0.0, 0.0, -9.81},
    bodies = bodies
}
