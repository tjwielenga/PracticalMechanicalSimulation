module ViewerSignalBrowserTests

using Test

include("../../src/viewer/SignalBrowser.jl")

@testset "hierarchical viewer signal browser" begin
    labels = [
        "gravity.force",
        "van.body.R_x",
        "van.steering.idler.theta",
        "van.steering.pitman.omega",
        "van.steering.pitman.theta",
    ]

    root = signal_browser_options(labels, String[]; include_time = true)
    @test first.(root) == ["time (s)", "gravity /", "van /"]
    @test last.(root) == [
        SignalBrowserEntry(:time, "time (s)", 0),
        SignalBrowserEntry(:folder, "gravity", 0),
        SignalBrowserEntry(:folder, "van", 0),
    ]

    steering = signal_browser_options(labels, ["van", "steering"])
    @test first.(steering) == ["idler /", "pitman /"]
    pitman = signal_browser_options(labels,
        ["van", "steering", "pitman"])
    @test first.(pitman) == ["omega", "theta"]
    @test getproperty.(last.(pitman), :index) == [4, 5]

    @test signal_component_path(labels, 5) ==
        ["van", "steering", "pitman"]
    @test signal_component_path(labels, 0) == String[]
    @test compact_signal_label("short.name") == "short.name"
    @test length(compact_signal_label(repeat("x", 60))) == 48
end

end
