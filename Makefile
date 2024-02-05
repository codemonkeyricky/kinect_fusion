all:
	bazel build -c dbg --copt=-std=gnu++17 --copt=-O3 //:kinect_fusion

dbg:
	gdb -tui ./bazel-bin/kinect_fusion

run:
	./bazel-bin/kinect_fusion
