.PHONY: all clean run setup

BUILD_TYPE ?= Release

all: setup
	cmake --build build --config $(BUILD_TYPE)

setup:
	@if [ ! -d "build" ]; then \
		if command -v ninja >/dev/null 2>&1; then \
			cmake -B build -S . -G Ninja -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		else \
			cmake -B build -S . -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		fi \
	fi

clean:
	rm -rf build

run: all
	./plotter