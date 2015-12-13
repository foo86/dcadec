SUBDIRS = libdcadec
ACLOCAL_AMFLAGS = -I m4

pkgconfigdir = $(libdir)/pkgconfig

bin_PROGRAMS = dcadec
pkgconfig_DATA = dcadec.pc

dcadec_SOURCES = \
	dcadec.c

dcadec_LDADD = \
	libdcadec/libdcadec.la

