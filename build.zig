const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const with_cgal = b.option(bool, "with-cgal", "Enable CGAL-backed Delaunay candidate generation") orelse false;

    const options = b.addOptions();
    options.addOption(bool, "with_cgal", with_cgal);

    const commiv = b.addModule("commiv", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });
    commiv.addOptions("build_options", options);
    if (with_cgal) addCgal(commiv, b);

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

    const cgal_probe = b.addExecutable(.{
        .name = "commiv-cgal-probe",
        .root_module = b.createModule(.{
            .root_source_file = b.path("examples/cgal_probe.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    addCgal(cgal_probe.root_module, b);
    const run_cgal_probe = b.addRunArtifact(cgal_probe);

    const cgal_probe_step = b.step("cgal-probe", "Build and run the CGAL Delaunay shim probe");
    cgal_probe_step.dependOn(&run_cgal_probe.step);
}

fn addCgal(module: *std.Build.Module, b: *std.Build) void {
    module.addCSourceFile(.{
        .file = b.path("src/cgal_delaunay.cpp"),
        .flags = &.{"-std=c++17"},
        .language = .cpp,
    });
    module.addIncludePath(b.path("src"));
    module.linkSystemLibrary("c++", .{});
    module.linkSystemLibrary("gmp", .{});
    module.linkSystemLibrary("mpfr", .{});
}
