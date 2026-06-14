const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const commiv = b.addModule("commiv", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    const lib_tests = b.addTest(.{
        .root_module = commiv,
    });
    const run_lib_tests = b.addRunArtifact(lib_tests);

    const test_step = b.step("test", "Run library tests");
    test_step.dependOn(&run_lib_tests.step);

    const example_module = b.createModule(.{
        .root_source_file = b.path("examples/basic.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "commiv", .module = commiv },
        },
    });
    const example = b.addExecutable(.{
        .name = "commiv-example",
        .root_module = example_module,
    });
    const run_example = b.addRunArtifact(example);

    const example_step = b.step("example", "Run embedded solver example");
    example_step.dependOn(&run_example.step);

    const bench_module = b.createModule(.{
        .root_source_file = b.path("examples/bench.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "commiv", .module = commiv },
        },
    });
    const bench = b.addExecutable(.{
        .name = "commiv-bench",
        .root_module = bench_module,
    });
    const run_bench = b.addRunArtifact(bench);

    const bench_step = b.step("bench", "Run deterministic solver benchmarks");
    bench_step.dependOn(&run_bench.step);
}
