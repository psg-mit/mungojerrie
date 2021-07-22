#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([cudd], [3.1.0], [Fabio@Colorado.EDU])
AC_CONFIG_AUX_DIR([build-aux])
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST
AM_INIT_AUTOMAKE(
  [1.14 -Wall -Werror foreign subdir-objects color-tests silent-rules]
)
AC_REQUIRE_AUX_FILE([tap-driver.sh])
AC_ARG_ENABLE([reentrant],
  [AS_HELP_STRING([--disable-reentrant],[disable reentrancy sypport])],,
  [enable_reentrant="yes"])

AC_CONFIG_SRCDIR([epd.h])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
# Set our own default (instead of "-g -O2") unless CXXFLAGS is already defined.
: ${CXXFLAGS="-g -O3"}
# Do the same for CFLAGS.
: ${CFLAGS="-g -O3"}
AC_PROG_CXX
AC_PROG_CC
AC_C_BIGENDIAN
m4_ifdef([AM_PROG_AR],[AM_PROG_AR])

LT_INIT

AX_VALGRIND_DFLT([drd], [off])
AX_VALGRIND_DFLT([helgrind], [off])
AX_VALGRIND_DFLT([sgcheck], [off])
AX_VALGRIND_CHECK

# Checks for libraries.
AC_SEARCH_LIBS([m],[pow])

# Checks for header files.
AC_CHECK_HEADERS([float.h inttypes.h limits.h stddef.h stdlib.h string.h unistd.h math.h assert.h sys/times.h sys/time.h sys/resource.h sys/types.h errno.h])

echo "enable-reentrant: " x$enable_reentrant
AC_MSG_CHECKING([whether to enable reentrancy support])
if test "x$enable_reentrant" = "xyes" ; then
  AC_MSG_RESULT([yes])
  AC_MSG_CHECKING([for thread local storage class specifier])
  keywords="_Thread_local __thread __declspec(thread)"
  tls=none
  for tls_keyword in $keywords ; do
    AC_COMPILE_IFELSE([AC_LANG_SOURCE([
      static void foo(void) {
        static ] $tls_keyword [ int bar;
      }])], [tls=$tls_keyword; break], [])
  done
  AC_MSG_RESULT($tls)
  if test x$tls != xnone ; then
    AC_DEFINE_UNQUOTED([TLS], $tls, [Thread-local storage class specifier.])
  else
    AC_MSG_ERROR([Reentrancy needs compiler support for thread-local storage])
  fi
else
  AC_MSG_RESULT([no])
fi

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_CHECK_TYPES([ptrdiff_t])
AC_CHECK_SIZEOF([int])
AC_CHECK_SIZEOF([long])
AC_CHECK_SIZEOF([void *])

# Checks for library functions.
AC_CHECK_LIB([m],[pow])
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_CHECK_FUNCS(
  [sysconf memset pow powl sqrt ceil strchr strstr getrusage getrlimit]
)

# Check for a working implementation of IEEE 754 floating point
# Specifically, check for correct treatment of +Infinity
AC_MSG_CHECKING([for +Infinity (IEEE 754 floating point)])
AC_CACHE_VAL(ac_cv_have_ieee_754,
[ AC_TRY_RUN([
#include <math.h>
main()
{
  if (HUGE_VAL != HUGE_VAL * 3 || HUGE_VAL != HUGE_VAL / 3) return 1;
  return 0;
}
],ac_cv_have_ieee_754=yes,ac_cv_have_ieee_754=no,ac_cv_have_ieee_754=no)])
if test $ac_cv_have_ieee_754 = yes ; then
  AC_MSG_RESULT(working)
  AC_DEFINE([HAVE_IEEE_754], [1], [Define to 1 if you have working floating-point infinities])
else
  AC_MSG_RESULT(broken)
fi

AM_CXXFLAGS="-std=c++11 ${AM_CXXFLAGS}"

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
