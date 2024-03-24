COPTS := 
COPTS += --cxxopt=-std=gnu++17
COPTS += --copt=-O0
COPTS += --copt=-DDYNAMIC_CHUNK 

BAZEL_OPT := -c dbg
BAZEL_OPT += --disk_cache=~/.cache/

all:
	bazel build ${BAZEL_OPT} ${COPTS} //...

dbg:
	gdb -tui ./bazel-bin/kinect_fusion

run:
	./bazel-bin/kinect_fusion

benchmark:
	bazel run ${BAZEL_OPT} ${COPTS} //src/icp/benchmark:benchmark

test:
	bazel test ${BAZEL_OPT} ${COPTS}  //src/models/test:volume_test

perf: 
	perf record -g ./bazel-bin/kinect_fusion


dbg_vuh:
	gdb -tui bazel-bin/src/vuh/example
