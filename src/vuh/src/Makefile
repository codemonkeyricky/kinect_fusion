

all: test/shaders/saxpy.spv
	g++ \
		instance.cpp \
		device.cpp \
		error.cpp \
		utils.cpp \
		main.cpp \
		-lvulkan \
		-Iinclude

test/shaders/saxpy.spv: test/shaders/saxpy.comp
	glslc test/shaders/saxpy.comp -o test/shaders/saxpy.spv
