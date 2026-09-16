"""Small unit-aware parsers for human-readable model input."""
module InputUnits

export angle_value

"""
Return an angle in radians. Numeric input is already in radians; a quoted
string may end in `°` or `deg` and is converted from degrees.
"""
function angle_value(value, label)
    radians = if value isa Number
        Float64(value)
    elseif value isa AbstractString
        text = strip(String(value))
        number_text = if endswith(text, "°")
            strip(chop(text; tail = 1))
        elseif endswith(lowercase(text), "deg")
            strip(chop(text; tail = 3))
        else
            throw(ArgumentError(
                "$label must be radians or a quoted value ending in ° or deg"))
        end
        degrees = tryparse(Float64, number_text)
        isnothing(degrees) && throw(ArgumentError(
            "$label has an invalid degree value '$value'"))
        deg2rad(degrees)
    else
        throw(ArgumentError(
            "$label must be radians or a quoted value ending in ° or deg"))
    end
    isfinite(radians) || throw(ArgumentError("$label must be finite"))
    radians
end

end
