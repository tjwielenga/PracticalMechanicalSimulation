-- Rectangular ground surface attached to an oriented marker.

local sim3d = require "sim3d"
local model_table = sim3d.model_table
local required, positive = sim3d.required, sim3d.positive

local function ground_pad(p)
    p = p or {}
    local context = "ground_pad"
    local marker = required(p, "marker", context)
    local name = p.name or "pad"
    local dimensions = {length = p.length or 100.0, width = p.width or 100.0}
    local length = positive(dimensions, "length", context)
    local width = positive(dimensions, "width", context)
    local half_length, half_width = length/2, width/2

    local graphic_name = marker .. ".graphics." .. name
    model_table {
        name = graphic_name,
        shape = "surface",
        vertices = {
            {-half_length, -half_width, 0.0},
            { half_length, -half_width, 0.0},
            { half_length,  half_width, 0.0},
            {-half_length,  half_width, 0.0}
        },
        faces = {{1, 2, 3, 4}},
        color = p.color or "gray25",
        opacity = p.opacity or 0.96,
        draw_edges = p.draw_edges == true,
        edge_color = p.edge_color or "gray15",
        edge_width = p.edge_width or 1.0,
        include_in_fit = p.include_in_fit == true
    }

    return {graphic = graphic_name, marker = marker}
end

return ground_pad
