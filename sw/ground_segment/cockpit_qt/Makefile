.PHONY: all clean run setup build_ninja
all: setup
	cmake --build build --parallel $$(nproc)

setup:
	@if [ ! -d "build" ]; then \
		if command -v ninja >/dev/null 2>&1; then \
			cmake -B build -S . -G Ninja -DCMAKE_BUILD_TYPE=Release; \
		else \
			cmake -B build -S . -DCMAKE_BUILD_TYPE=Release; \
		fi \
	fi

build: all

clean:
	rm -rf build

strip: all
	strip -s gcs || true

run: all
	./gcs
