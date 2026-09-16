local sim3d = require "sim3d"
local model, analysis, simulation, graphics, state_selection,
    initial_conditions,
    ground, marker, gravity =
    sim3d.model, sim3d.analysis, sim3d.simulation, sim3d.graphics,
    sim3d.state_selection, sim3d.initial_conditions,
    sim3d.ground, sim3d.marker, sim3d.gravity
local vector, cross, unit, frame =
    sim3d.vector, sim3d.cross, sim3d.unit, sim3d.frame
local gmc_rally_van = require "spatial.gmc_rally_van"
local ground_pad = require "spatial.ground_pad"

model {
    name = "gmc_rally_van",
    title = "Historical GMC rally-van suspension conversion",
    dimension = "spatial"
}

analysis {
    mode = "dynamic",
    -- These settings are used if Modal analysis is selected in SimpView.
    modes = 9,
    frequency_shift_hz = 2.0
}

initial_conditions {
    result = "../../results/examples/spatial/gmc-rally-van-static.simp",
    sample = "last",
    include_velocities = false
}

simulation {
    start_time = 0.0,
    end_time = 5.0,
    output_samples = 301,
    relative_tolerance = 1.0e-5,
    absolute_tolerance = 1.0e-7,
    initial_step = 1.0e-7,
    maximum_step = 0.1
}

-- Prefer coordinates with direct physical meaning. QR completes this partial
-- list with enough additional independent velocities for all 41 states.
state_selection {
    method = "preferred",
    preferred_velocities = {
        "van.left_front_tire.axle.omega",
        "van.right_front_tire.axle.omega",
        "van.left_rear_tire.axle.omega",
        "van.right_rear_tire.axle.omega",
        "van.left_front.upper_hinge.omega",
        "van.right_front.upper_hinge.omega",
        "van.rear.axle.V_z",
        "van.rear.axle.omega_x",
        "van.body.V_x", "van.body.V_y", "van.body.V_z",
        "van.body.omega_x", "van.body.omega_y", "van.body.omega_z"
    },
    allow_fallback = false
}

graphics {
    background = "white",
    body_palette = "colorblind"
}

ground {name = "ground"}
local road_z = unit(vector {-0.0010453444, -0.0020088528, 1.0})
local road_x = unit(cross(vector {0.0, 1.0, 0.0}, road_z))
local road_y = cross(road_z, road_x)
marker {
    name = "ground.road",
    -- The Rally Van hardpoints were recorded for a smaller P215 tire. Lowering
    -- the road approximately preserves the original suspension ride height
    -- when the LT245/75R16 tire is installed.
    -- The loaded tire deflections place the road approximately 30 mm below
    -- the historical coordinate-system origin with the larger LT245 tire.
    position = {0.0, 0.0, -0.0295190411},
    orientation = frame(road_x, road_y, road_z),
    graphics = {
        shape = "xy_frame", axis_length = 0.5, plane_size = 0.5,
        opacity = 0.0,
        label = "road"
    }
}
ground_pad {
    name = "asphalt_pad", marker = "ground.road",
    length = 100.0, width = 100.0, color = "gray25"
}

-- This reproduces the mechanical content of GMCRallyVan.py as far as its
-- historical dependency files define it. The steering gear is completed from
-- normalSteer2.pl and the wheels and tires use the GYLT245/75R16 data.
local van = gmc_rally_van {
    name = "van",
    road_marker = "ground.road",
    -- Sweep the steering wheel through a smooth periodic ±90 degree cycle.
    -- The 19.5:1 gear coupler gives approximately ±4.6 degrees at the
    -- pitman arm, with the historical windup compliance between them. This
    -- normalized two-sine waveform starts from the static model's zero angle
    -- with zero angular speed and acceleration.
    steering_wheel_motion_angle =
        "-(pi/2)*0.769800358919501*(sin(2*pi*t/5)-0.5*sin(4*pi*t/5))",
    tire_damping_time_scale = 0.01,
    -- The saved static configuration is loaded first. These declared values
    -- then establish a consistent 16 m/s translation and free-rolling spin.
    forward_speed = 16.0,
    colors = {
        body = "gray70",
        control_arm = "steelblue",
        spindle = "darkorange",
        stabilizer = "firebrick",
        stabilizer_link = "orange",
        steering_link = "gray35",
        tie_rod = "seagreen",
        steering_wheel = "black",
        axle = "gray35",
        leaf = "steelblue",
        shackle = "darkorange",
        tire = "gray20"
    }
}

gravity {
    name = "gravity",
    acceleration = {0.0, 0.0, -9.81},
    bodies = van.bodies
}
