CC ?= gcc
AR ?= ar
CFLAGS ?= -std=gnu99 -D_FILE_OFFSET_BITS=64 -Wall -Wextra -O3 -g -MMD
LDFLAGS ?=
LIBS ?= -lm
OUT_LIB ?= libdcadec/libdcadec.a
OUT_DEC ?= dcadec
OUT_CUT ?= dcacut

SRC_LIB = \
libdcadec/bitstream.c \
libdcadec/core_decoder.c \
libdcadec/dca_context.c \
libdcadec/dca_stream.c \
libdcadec/dca_waveout.c \
libdcadec/dmix_tables.c \
libdcadec/exss_parser.c \
libdcadec/idct_fixed.c \
libdcadec/interpolator.c \
libdcadec/interpolator_fixed.c \
libdcadec/interpolator_float.c \
libdcadec/ta.c \
libdcadec/xll_decoder.c
OBJ_LIB = $(SRC_LIB:.c=.o)
DEP_LIB = $(SRC_LIB:.c=.d)

SRC_DEC = dcadec.c
OBJ_DEC = $(SRC_DEC:.c=.o)
DEP_DEC = $(SRC_DEC:.c=.d)

SRC_CUT = dcacut.c
OBJ_CUT = $(SRC_CUT:.c=.o)
DEP_CUT = $(SRC_CUT:.c=.d)

all: $(OUT_LIB) $(OUT_DEC) $(OUT_CUT)

default: all

-include $(DEP_LIB) $(DEP_DEC) $(DEP_CUT)

$(OUT_LIB): $(OBJ_LIB)
	$(AR) crsu $@ $(OBJ_LIB)

$(OUT_DEC): $(OBJ_DEC) $(OUT_LIB)
	$(CC) $(LDFLAGS) -o $@ $(OBJ_DEC) $(OUT_LIB) $(LIBS)

$(OUT_CUT): $(OBJ_CUT) $(OUT_LIB)
	$(CC) $(LDFLAGS) -o $@ $(OBJ_CUT) $(OUT_LIB) $(LIBS)

clean:
	rm -f $(OUT_LIB) $(OBJ_LIB) $(DEP_LIB)
	rm -f $(OUT_DEC) $(OBJ_DEC) $(DEP_DEC)
	rm -f $(OUT_CUT) $(OBJ_CUT) $(DEP_CUT)
