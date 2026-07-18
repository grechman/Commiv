# commiv-serve as a from-scratch container: one static musl binary, no OS.
# Build the binary first (any machine with zig), then the image:
#   zig build serve -Dtarget=x86_64-linux-musl -Doptimize=ReleaseFast
#   docker build -t commiv-serve .
# Run:
#   docker run --rm -p 8080:8080 commiv-serve
# The server binds COMMIV_HOST (default 127.0.0.1 — inside a container you
# want 0.0.0.0, set below) and COMMIV_PORT (default 8080). See docs/rest.md.
FROM scratch
COPY zig-out/bin/commiv-serve /commiv-serve
ENV COMMIV_HOST=0.0.0.0
EXPOSE 8080
ENTRYPOINT ["/commiv-serve"]
