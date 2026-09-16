local sim3d = require "sim3d"
local model, analysis, simulation, graphics, state_selection,
    ground, rigid_body, marker, bushing, spanning_force, spanning_motion,
    gravity =
    sim3d.model, sim3d.analysis, sim3d.simulation, sim3d.graphics,
    sim3d.state_selection, sim3d.ground, sim3d.rigid_body,
    sim3d.marker, sim3d.bushing, sim3d.spanning_force,
    sim3d.spanning_motion, sim3d.gravity
local box_inertia, vector, norm, local_point =
    sim3d.box_inertia, sim3d.vector, sim3d.norm, sim3d.local_point
local sla_suspension = require "spatial.sla_suspension"
local tire = require "spatial.gylt_245_75_r16"

model {
    name = "front_sla_static",
    title = "Vehicle with front SLA suspension — static equilibrium",
    dimension = "spatial"
}

analysis {
    mode = "static",
    static_method = "dynamic_relaxation",
    relaxation_duration = 0.5,
    relaxation_min_cycles = 2,
    relaxation_max_cycles = 20
}

simulation {
    start_time = 0.0,
    end_time = 0.0,
    output_samples = 1,
    relative_tolerance = 1.0e-6,
    absolute_tolerance = 1.0e-8
}

state_selection {
    method = "preferred",
    preferred_velocities = {
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
        shape = "xy_frame", axis_length = 0.5, plane_size = 7.0,
        plane_color = "gray70", opacity = 0.12, label = "road"
    }
}

local chassis_mass = 1576.0
local chassis_size = {3.8, 1.55, 0.55}
rigid_body {
    name = "vehicle.chassis",
    mass = chassis_mass,
    inertia = box_inertia(chassis_mass, chassis_size),
    position = {0.0, 0.0, 0.75},
    graphics = {
        shape = "box", marker = "vehicle.chassis.center",
        size = chassis_size, color = "gray65", opacity = 0.28
    }
}
marker {name = "vehicle.chassis.center"}
marker {name = "ground.chassis_hold", position = {0.0, 0.0, 0.75}}
bushing {
    name = "vehicle.static_hold",
    markers = {"vehicle.chassis.center", "ground.chassis_hold"},
    translational_stiffness = {5000.0, 5000.0, 0.0},
    rotational_stiffness = {0.0, 0.0, 5000.0},
    damping_time_scale = 0.10,
    active_during = "static"
}

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
        normal_damping_time_scale = 0.01,
        static_spin_stiffness = 1000.0,
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
        normal_damping_time_scale = 0.01,
        static_spin_stiffness = 1000.0,
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
