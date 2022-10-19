OPTIM = -O0 -fno-loop-optimize -fno-aggressive-loop-optimizations
DEBUG = -g
WARN_FLAGS = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align -Wcast-qual -Wshadow
DEFINES = -DTESTING -DCA_N_U=20 -DCA_N_V=6

CC = gcc
CC_FLAGS = -fstack-usage -fwrapv -fPIC $(WARN_FLAGS) $(DEBUG) $(OPTIM)
LINK_FLAGS = -lm
AR = ar
ARFLAGS = ruvs
PRODUCT = solveActiveSet.a
BIN_DIR = ./bin
INCLUDES = -Icommon/ -Ilib/
SOURCES = solveActiveSet.c solveActiveSet_chol.c solveActiveSet_qr.c solveActiveSet_pprz.c common/setup_wls.c lib/chol_math.c lib/qr_updates.c lib/qr_wrapper.c lib/qr_solve/qr_solve.c lib/qr_solve/r8lib_min.c lib/sparse_math.c

all: $(patsubst %, $(BIN_DIR)/%.o, $(basename $(SOURCES)))
	$(AR) $(ARFLAGS) $(BIN_DIR)/$(PRODUCT) $?
	$(CC) $(CC_FLAGS) -o $(BIN_DIR)/main.o main.c $(BIN_DIR)/$(PRODUCT) $(DEFINES) $(DEBUG) $(WARN_FLAGS) $(INCLUDES) $(LINK_FLAGS)

$(BIN_DIR)/%.o : %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CC_FLAGS) -o $@ $*.c $(DEFINES) $(DEBUG) $(WARN_FLAGS) $(INCLUDES)

clean:
	@rm -rf *.o
	@rm -rf *.a
	@rm -rf *.su
	@rm -rf bin
