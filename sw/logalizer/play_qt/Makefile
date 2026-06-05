.PHONY: all clean run setup

BUILD_TYPE ?= Release
BUILD_DIR ?= ../../../var/build_qt_play

all: setup
	cmake --build $(BUILD_DIR) --config $(BUILD_TYPE)

setup:
	@if [ ! -d "$(BUILD_DIR)" ]; then \
		if command -v ninja >/dev/null 2>&1; then \
			cmake -B $(BUILD_DIR) -S . -G Ninja -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		else \
			cmake -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		fi \
	fi

clean:
	rm -rf $(BUILD_DIR)
	rm -f play

run: all
	$(BUILD_DIR)/play
