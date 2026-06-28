const std = @import("std");

pub const CvrpInstance = struct {
    n: usize, // customer count (excludes depot)
    matrix: []const u32, // (n+1)*(n+1), directional, depot=0
    demand: []const u32, // length n+1, demand[0]=0
    capacity: u32,

    pub fn dim(self: CvrpInstance) usize {
        return self.n + 1;
    }
    pub fn d(self: CvrpInstance, a: usize, b: usize) u64 {
        return self.matrix[a * self.dim() + b];
    }
};

pub const CvrpResult = struct {
    allocator: std.mem.Allocator,
    routes: [][]usize, // each route: customer indices in visit order (depot implied at both ends)
    total_cost: u64,

    pub fn deinit(self: *CvrpResult) void {
        for (self.routes) |r| self.allocator.free(r);
        self.allocator.free(self.routes);
        self.* = undefined;
    }
};
