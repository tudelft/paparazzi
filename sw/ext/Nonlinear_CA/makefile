# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects

make:
	@echo "Compiling..."
#	gcc -o Orangepi_PPZ_run *.c MATLAB_generated_files/*.c -Ofast -march=native -lm -lwiringPi -pthread -fopenmp
	gcc -o Orangepi_PPZ_run *.c MATLAB_generated_files/*.c -O3 -march=native -lm -lwiringPi -pthread -fopenmp
clean:
	@echo "Cleaning up..."
	rm -rvf Orangepi_PPZ_run
