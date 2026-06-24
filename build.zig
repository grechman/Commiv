const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const commiv = b.addModule("commiv", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Library tests.
    const lib_tests = b.addTest(.{ .root_module = commiv });
    const run_lib_tests = b.addRunArtifact(lib_tests);
    const test_step = b.step("test", "Run library tests");
    test_step.dependOn(&run_lib_tests.step);

    // Embedded integration example (runs).
    const example = b.addExecutable(.{
        .name = "commiv-example",
        .root_module = b.createModule(.{
            .root_source_file = b.path("examples/basic.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{.{ .name = "commiv", .module = commiv }},
        }),
    });
    const example_step = b.step("example", "Run the embedded solver example");
    example_step.dependOn(&b.addRunArtifact(example).step);

    // Deterministic TSP benchmark (runs).
    const bench = b.addExecutable(.{
        .name = "commiv-bench",
        .root_module = b.createModule(.{
            .root_source_file = b.path("examples/bench.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{.{ .name = "commiv", .module = commiv }},
        }),
    });
    const bench_step = b.step("bench", "Run the deterministic TSP benchmark");
    bench_step.dependOn(&b.addRunArtifact(bench).step);

    // Gap benchmarks: build-only, each reads env vars and prints cost/gap/time.
    const benches = [_]struct { name: []const u8, src: []const u8, step: []const u8, desc: []const u8 }{
        .{ .name = "commiv-cvrpbench", .src = "examples/cvrpbench.zig", .step = "cvrpbench", .desc = "CVRP gap benchmark (Augerat / Uchoa X) vs optima" },
        .{ .name = "commiv-acvrpbench", .src = "examples/acvrpbench.zig", .step = "acvrpbench", .desc = "Asymmetric CVRP benchmark vs LKH-3 reference" },
        .{ .name = "commiv-atspbench", .src = "examples/atspbench.zig", .step = "atspbench", .desc = "ATSP gap benchmark (TSPLIB) vs proven optima" },
        .{ .name = "commiv-vrptwbench", .src = "examples/vrptwbench.zig", .step = "vrptwbench", .desc = "VRPTW gap benchmark (Solomon) vs SINTEF BKS" },
        .{ .name = "commiv-roadbench", .src = "examples/roadbench.zig", .step = "roadbench", .desc = "Real directed-road (OSRM Moscow) asymmetry + solve benchmark" },
    };
    for (benches) |x| {
        const exe = b.addExecutable(.{
            .name = x.name,
            .root_module = b.createModule(.{
                .root_source_file = b.path(x.src),
                .target = target,
                .optimize = optimize,
                .imports = &.{.{ .name = "commiv", .module = commiv }},
            }),
        });
        const step = b.step(x.step, x.desc);
        step.dependOn(&b.addInstallArtifact(exe, .{}).step);
    }
}
