# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects

CC = gcc

PAPARAZZI_SRC=../../..
UNAME = $(shell uname -s)

CFLAGS += -Wall -fPIC -g

# Paparazzi includes
INCLUDES += -I$(PAPARAZZI_SRC)/sw/airborne/ -I$(PAPARAZZI_SRC)/sw/include/ $(IVY_INC)
INCLUDES += -I$(PAPARAZZI_SRC)/sw/airborne/arch/linux/ 

make:
	@echo "Compiling..."
#	gcc -o Orangepi_PPZ_run *.c MATLAB_generated_files/*.c -Ofast -march=native -lm -lwiringPi -pthread -fopenmp
	gcc -o Orangepi_PPZ_run *.c MATLAB_generated_files/*.c -O3 -march=native -lm -lwiringPi -fopenmp $(INCLUDES) $(LIBRARYS) -lpthread $(GLIB_LDFLAGS) $(IVY_LDFLAGS) -include $(PAPARAZZI_SRC)/sw/ext/nonlinear_CA/am7x.h 
clean:
	@echo "Cleaning up..."
	rm -rvf Orangepi_PPZ_run