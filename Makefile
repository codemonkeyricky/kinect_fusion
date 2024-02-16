all:
	bazel build -c dbg --copt=-std=gnu++17 --copt=-O3 //:kinect_fusion --sandbox_debug

dbg:
	gdb -tui ./bazel-bin/kinect_fusion

run:
	./bazel-bin/kinect_fusion

test:
	bazel test -c dbg --cxxopt=-std=c++14 --test_output=all //src/helpers/test:octree_test

perf: 
	perf record -g ./bazel-bin/kinect_fusion
