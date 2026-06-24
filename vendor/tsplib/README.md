# TSPLIB fixtures

Place symmetric TSPLIB `.tsp` fixtures here for `zig build bench`.

Expected files:

- `berlin52.tsp`, optimum `7542`
- `eil76.tsp`, optimum `538`
- `rat195.tsp`, optimum `2323`
- `lin318.tsp`, optimum `42029`
- `rat575.tsp`, optimum `6773`

The benchmark harness looks for these exact paths and reports missing files
instead of silently substituting synthetic data. The intended source is TSPLIB95
or a faithful mirror of its symmetric TSP data.
