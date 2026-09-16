"""One folder, signal, or time entry in the hierarchical plot browser."""
struct SignalBrowserEntry
    kind::Symbol
    name::String
    index::Int
end

function signal_browser_options(labels, path; include_time = false,
        time_label = "time (s)")
    depth = length(path)
    folders = Set{String}()
    variables = Pair{String,Int}[]
    for (index, label) in enumerate(labels)
        parts = String.(split(label, '.'))
        length(parts) > depth || continue
        all(parts[position] == path[position] for position in eachindex(path)) ||
            continue
        if length(parts) == depth + 1
            push!(variables, parts[end] => index)
        else
            push!(folders, parts[depth + 1])
        end
    end
    options = Pair{String,SignalBrowserEntry}[]
    include_time && isempty(path) && push!(options,
        time_label => SignalBrowserEntry(:time, time_label, 0))
    for folder in sort!(collect(folders))
        push!(options, "$folder /" =>
            SignalBrowserEntry(:folder, folder, 0))
    end
    for (variable, index) in sort!(variables; by = first)
        push!(options, variable => SignalBrowserEntry(:signal, variable, index))
    end
    options
end

function signal_component_path(labels, index)
    index == 0 && return String[]
    parts = String.(split(labels[index], '.'))
    length(parts) <= 1 ? String[] : parts[1:(end - 1)]
end

function compact_signal_label(label; limit = 48)
    length(label) <= limit && return label
    leading = div(limit - 1, 2)
    trailing = limit - 1 - leading
    first(label, leading) * "…" * last(label, trailing)
end
