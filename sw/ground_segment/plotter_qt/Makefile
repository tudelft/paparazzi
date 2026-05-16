.PHONY: all clean run setup build_ninja

all: setup
	cmake --build build --parallel $32

setup:
	@if [ ! -d "build" ]; then \
		if command -v ninja >/dev/null 2>&1; then \
			cmake -B build -S . -G Ninja; \
		else \
			cmake -B build -S .; \
		fi \
	fi

clean:
	rm -rf build

run: all
	./plotter_qt