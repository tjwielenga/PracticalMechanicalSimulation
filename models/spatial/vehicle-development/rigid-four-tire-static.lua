local sim3d = require "sim3d"
local model, analysis, simulation, graphics, ground, rigid_body, marker,
    gravity =
    sim3d.model, sim3d.analysis, sim3d.simulation, sim3d.graphics,
    sim3d.ground, sim3d.rigid_body, sim3d.marker,
    sim3d.gravity
local box_inertia = sim3d.box_inertia
local gylt_245_75_r16 = require "spatial.gylt_245_75_r16"

model {
    name = "rigid_four_tire_static",
    title = "Rigid four-tire vehicle static equilibrium",
    dimension = "spatial"
}

analysis {
    mode = "static",
    static_method = "newton"
}

simulation {
    start_time = 0.0,
    end_time = 0.0,
    output_samples = 1,
    relative_tolerance = 1.0e-6,
    absolute_tolerance = 1.0e-8
}

graphics {
    background = "white",
    body_palette = "colorblind"
}

ground {name = "ground"}
marker {
    name = "ground.road",
    graphics = {
        shape = "xy_frame", axis_length = 0.5, plane_size = 6.0,
        plane_color = "gray70", opacity = 0.12, label = "road"
    }
}

local chassis_mass = 1640.0
local chassis_size = {3.8, 1.55, 0.55}
local chassis_center = {0.0, 0.0, 0.75}
rigid_body {
    name = "vehicle.chassis",
    mass = chassis_mass,
    inertia = box_inertia(chassis_mass, chassis_size),
    position = chassis_center,
    graphics = {
        shape = "box", marker = "vehicle.chassis.center",
        size = chassis_size, color = "gray65", opacity = 0.35
    }
}
marker {name = "vehicle.chassis.center"}

-- The tire rate is about 416 kN/m. With chassis and wheel mass included,
-- z = 0.370 m begins with approximately one quarter of the vehicle weight
-- at each tire and avoids an artificial impact at the start of static.
local wheel_centers = {
    left_front = { 1.55, -0.78, 0.370},
    right_front = {1.55,  0.78, 0.370},
    left_rear = {-1.55, -0.78, 0.370},
    right_rear = {-1.55,  0.78, 0.370}
}
local tires = {}
for role, center in pairs(wheel_centers) do
    tires[role] = gylt_245_75_r16 {
        name = "vehicle." .. role .. "_tire",
        spindle = "vehicle.chassis",
        center = center,
        road_marker = "ground.road",
        pressure_psi = 60,
        axle_direction = {0.0, 1.0, 0.0},
        forward_direction = {1.0, 0.0, 0.0},
        normal_damping_time_scale = 0.01,
        color = "gray20"
    }
end

gravity {
    name = "gravity",
    acceleration = {0.0, 0.0, -9.81},
    bodies = {
        "vehicle.chassis",
        tires.left_front.body,
        tires.right_front.body,
        tires.left_rear.body,
        tires.right_rear.body
    }
}
