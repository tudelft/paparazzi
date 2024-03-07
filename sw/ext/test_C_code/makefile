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
	gcc -o test_code *.c -O2 -march=native -lm -fopenmp $(INCLUDES) $(LIBRARYS) -lpthread $(GLIB_LDFLAGS) $(IVY_LDFLAGS) 
clean:
	@echo "Cleaning up..."
	rm -rvf test_code