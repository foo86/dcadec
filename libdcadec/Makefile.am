includelibdir = $(includedir)/libdcadec

lib_LTLIBRARIES = libdcadec.la

libdcadec_la_SOURCES = \
	bitstream.c \
	bitstream.h \
	common.h \
	compiler.h \
	core_decoder.c \
	core_decoder.h \
	core_huffman.h \
	core_tables.h \
	core_vectors.h \
	dca_context.c \
	dmix_tables.c \
	dmix_tables.h \
	exss_parser.c \
	exss_parser.h \
	fir_fixed.h \
	fir_float.h \
	fixed_math.h \
	huffman.h \
	idct.h \
	idct_fixed.c \
	idct_float.c \
	interpolator.c \
	interpolator.h \
	interpolator_fixed.c \
	interpolator_float.c \
	ta.c \
	ta.h \
	xll_decoder.c \
	xll_decoder.h \
	xll_tables.h

includelib_HEADERS = \
	dca_context.h

if !SMALL
libdcadec_la_SOURCES += \
	dca_frame.c \
	dca_stream.c \
	dca_waveout.c
includelib_HEADERS += \
	dca_frame.h \
	dca_stream.h \
	dca_waveout.h
endif

AM_CPPFLAGS = -D_FILE_OFFSET_BITS=64
AM_CFLAGS = -std=gnu99 -Wall -Wextra -O3 -ffast-math -g

libdcadec_la_LDFLAGS = -version-info 1:0:1
libdcadec_la_LIBADD = -lm
