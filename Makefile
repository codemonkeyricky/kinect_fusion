COPTS := 
COPTS += --cxxopt=-std=gnu++17
COPTS += --copt=-O3
# COPTS += --copt=-DDYNAMIC_CHUNK 

BAZEL_OPT := -c dbg
BAZEL_OPT += --disk_cache=~/.cache/

all:
	bazel build ${BAZEL_OPT} ${COPTS} //...

dbg:
	gdb -tui ./bazel-bin/kinect_fusion

run:
	./bazel-bin/kinect_fusion

test:
	bazel test ${BAZEL_OPT} ${COPTS} //... 

perf: 
	perf record -g ./bazel-bin/kinect_fusion
