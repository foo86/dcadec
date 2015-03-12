CC ?= gcc
CFLAGS ?= -std=gnu99 -D_FILE_OFFSET_BITS=64 -Wall -Wextra -O3 -g -MMD
LDFLAGS ?=
LIBS ?= -lm
OUT ?= dcadec

SRC = \
bitstream.c \
core_decoder.c \
dca_context.c \
dca_stream.c \
dca_waveout.c \
dmix_tables.c \
exss_parser.c \
idct_fixed.c \
interpolator.c \
interpolator_fixed.c \
interpolator_float.c \
main.c \
ta.c \
xll_decoder.c
OBJS = $(SRC:.c=.o)
DEPS = $(SRC:.c=.d)

all: $(OUT)

default: all

-include $(DEPS)

$(OUT): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

clean:
	rm -f $(OUT) $(OBJS) $(DEPS)
