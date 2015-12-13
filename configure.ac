AC_INIT([dcadec], [0.2.0], [])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_MACRO_DIRS([m4])
AM_PROG_AR
AC_ARG_ENABLE([small],
  [  --enable-small    build reduced library],
  [case "${enableval}" in
    yes) small=true ;;
     no) small=false ;;
      *) AC_MSG_ERROR([bad value ${enableval} for --enable-small]) ;;
   esac],[small=false])
AM_CONDITIONAL([SMALL], [test x$small = xtrue])
LT_INIT
AC_CONFIG_FILES([
  Makefile
  dcadec.pc
  libdcadec/Makefile
])
AC_OUTPUT
