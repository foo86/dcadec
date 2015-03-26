CC = gcc
AR = ar
override CFLAGS := -std=gnu99 -D_FILE_OFFSET_BITS=64 -Wall -Wextra -O3 -g -MMD $(CFLAGS)
LDFLAGS =

PREFIX ?= /usr/local
BINDIR ?= $(PREFIX)/bin
LIBDIR ?= $(PREFIX)/lib
INCLUDEDIR ?= $(PREFIX)/include
PKG_CONFIG_PATH ?= $(LIBDIR)/pkgconfig

-include .config

ifdef CONFIG_NDEBUG
    CFLAGS += -DNDEBUG
endif

ifdef CONFIG_WINDOWS
    EXESUF ?= .exe
    DLLSUF ?= .dll
    LIBSUF ?= .a
    LIBS ?=
else
    EXESUF ?=
    DLLSUF ?= .so
    LIBSUF ?= .a
    LIBS ?= -lm
endif

ifdef CONFIG_SHARED
    OUT_LIB ?= libdcadec/libdcadec$(DLLSUF)
else
    OUT_LIB ?= libdcadec/libdcadec$(LIBSUF)
endif

OUT_DEC ?= dcadec$(EXESUF)
OUT_CUT ?= dcacut$(EXESUF)

SRC_LIB = \
libdcadec/bitstream.c \
libdcadec/core_decoder.c \
libdcadec/dca_context.c \
libdcadec/dmix_tables.c \
libdcadec/exss_parser.c \
libdcadec/idct_fixed.c \
libdcadec/interpolator.c \
libdcadec/interpolator_fixed.c \
libdcadec/interpolator_float.c \
libdcadec/ta.c \
libdcadec/xll_decoder.c
INC_LIB = \
libdcadec/dca_context.h

ifndef CONFIG_SMALL
SRC_LIB += libdcadec/dca_stream.c
SRC_LIB += libdcadec/dca_waveout.c
INC_LIB += libdcadec/dca_stream.h
INC_LIB += libdcadec/dca_waveout.h
endif

OBJ_LIB = $(SRC_LIB:.c=.o)
DEP_LIB = $(SRC_LIB:.c=.d)

SRC_DEC = dcadec.c
OBJ_DEC = $(SRC_DEC:.c=.o)
DEP_DEC = $(SRC_DEC:.c=.d)

SRC_CUT = dcacut.c
OBJ_CUT = $(SRC_CUT:.c=.o)
DEP_CUT = $(SRC_CUT:.c=.d)

default: $(OUT_LIB) $(OUT_DEC)

all: $(OUT_LIB) $(OUT_DEC) $(OUT_CUT)

-include $(DEP_LIB) $(DEP_DEC) $(DEP_CUT)

ifdef CONFIG_SHARED
    CFLAGS_DLL = $(CFLAGS) -DDCADEC_SHARED -DDCADEC_INTERNAL
    LDFLAGS_DLL = $(LDFLAGS) -shared

    ifdef CONFIG_WINDOWS
        IMP_LIB = libdcadec/libdcadec$(DLLSUF)$(LIBSUF)
        IMP_DEF = libdcadec/libdcadec.def
        EXTRA_LIB = $(IMP_LIB) $(IMP_DEF)
        LDFLAGS_DLL += -static-libgcc
        LDFLAGS_DLL += -Wl,--nxcompat,--dynamicbase
        LDFLAGS_DLL += -Wl,--output-def,$(IMP_DEF)
        LDFLAGS_DLL += -Wl,--out-implib,$(IMP_LIB)
    else
        CFLAGS_DLL += -fPIC -fvisibility=hidden
        IMP_LIB = -Llibdcadec -ldcadec
    endif

libdcadec/%.o: libdcadec/%.c
	$(CC) -c $(CFLAGS_DLL) -o $@ $<

$(OUT_LIB): $(OBJ_LIB)
	$(CC) $(LDFLAGS_DLL) -o $@ $(OBJ_LIB) $(LIBS)

$(OUT_DEC): $(OBJ_DEC) $(OUT_LIB)
	$(CC) $(LDFLAGS) -o $@ $(OBJ_DEC) $(IMP_LIB) $(LIBS)

$(OUT_CUT): $(OBJ_CUT) $(OUT_LIB)
	$(CC) $(LDFLAGS) -o $@ $(OBJ_CUT) $(IMP_LIB) $(LIBS)

else

$(OUT_LIB): $(OBJ_LIB)
	$(AR) crsu $@ $(OBJ_LIB)

$(OUT_DEC): $(OBJ_DEC) $(OUT_LIB)
	$(CC) $(LDFLAGS) -o $@ $(OBJ_DEC) $(OUT_LIB) $(LIBS)

$(OUT_CUT): $(OBJ_CUT) $(OUT_LIB)
	$(CC) $(LDFLAGS) -o $@ $(OBJ_CUT) $(OUT_LIB) $(LIBS)

endif

clean:
	$(RM) $(OUT_LIB) $(OBJ_LIB) $(DEP_LIB) $(EXTRA_LIB)
	$(RM) $(OUT_DEC) $(OBJ_DEC) $(DEP_DEC)
	$(RM) $(OUT_CUT) $(OBJ_CUT) $(DEP_CUT)
	$(RM) dcadec.pc

.PHONY: dcadec.pc
dcadec.pc: dcadec.pc.in
	sed "s,%PREFIX%,$(PREFIX),;s,%LIBDIR%,$(LIBDIR),;s,%INCLUDEDIR%,$(INCLUDEDIR)," dcadec.pc.in > dcadec.pc

install: $(OUT_LIB) $(OUT_DEC) dcadec.pc
	install -d -m 755 $(DESTDIR)$(LIBDIR) $(DESTDIR)$(PKG_CONFIG_PATH) $(DESTDIR)$(INCLUDEDIR)/libdcadec $(DESTDIR)$(BINDIR)
	install -m 644 $(OUT_LIB) $(DESTDIR)$(LIBDIR)
	install -m 644 $(INC_LIB) $(DESTDIR)$(INCLUDEDIR)/libdcadec
	install -m 644 dcadec.pc $(DESTDIR)$(PKG_CONFIG_PATH)
	install -m 755 $(OUT_DEC) $(DESTDIR)$(BINDIR)
