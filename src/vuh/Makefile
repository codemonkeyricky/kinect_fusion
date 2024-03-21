

all: test/shaders/saxpy.spv
	g++ -g  \
		src/instance.cpp \
		src/device.cpp \
		src/error.cpp \
		src/utils.cpp \
		src/main.cpp \
		-lvulkan \
		-Isrc/include

test/shaders/saxpy.spv: test/shaders/saxpy.comp
	glslc test/shaders/saxpy.comp -o test/shaders/saxpy.spv
