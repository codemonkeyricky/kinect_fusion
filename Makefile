all:
	bazel build -c dbg --cxxopt=-std=gnu++17 --copt=-O3 //... 

dbg:
	gdb -tui ./bazel-bin/kinect_fusion

run:
	./bazel-bin/kinect_fusion

test:
	bazel test -c dbg --cxxopt=-std=gnu++17 --copt=-O3 //...

perf: 
	perf record -g ./bazel-bin/kinect_fusion
