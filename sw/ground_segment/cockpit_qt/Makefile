.PHONY: all clean run setup build_ninja strip

BUILD_TYPE ?= Release
BUILD_DIR ?= ../../../var/build_qt_super

all: setup
	cmake --build $(BUILD_DIR) --config $(BUILD_TYPE) --target equinoxgcs --parallel $$(nproc)

setup:
	@if [ ! -f "$(BUILD_DIR)/CMakeCache.txt" ]; then \
		if command -v ninja >/dev/null 2>&1; then \
			cmake -B $(BUILD_DIR) -S ../../../sw/qt_superbuild -G Ninja -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		else \
			cmake -B $(BUILD_DIR) -S ../../../sw/qt_superbuild -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
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
