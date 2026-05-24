.PHONY: all clean run setup build_ninja strip

BUILD_TYPE ?= Release
BUILD_DIR ?= ../../../var/build_qt

all: setup
	cmake --build $(BUILD_DIR) --config $(BUILD_TYPE) --parallel $$(nproc)

setup:
	@if [ ! -d "$(BUILD_DIR)" ]; then \
		if command -v ninja >/dev/null 2>&1; then \
			cmake -B $(BUILD_DIR) -S . -G Ninja -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		else \
			cmake -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		fi \
	fi

build: all

clean:
	rm -rf $(BUILD_DIR)
	rm -f gcs

strip: all
	strip -s $(BUILD_DIR)/gcs || true

run: all
	$(BUILD_DIR)/gcs
