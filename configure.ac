#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([Mungojerrie], [1.0])
AC_CONFIG_AUX_DIR([build-aux])
AC_CONFIG_MACRO_DIR([m4])
AC_CANONICAL_HOST
AM_INIT_AUTOMAKE(
  [1.14 -Wall -Werror -Wno-portability foreign subdir-objects color-tests silent-rules]
)
AC_REQUIRE_AUX_FILE([tap-driver.sh])
AC_CONFIG_SRCDIR([LTS/LTS.cc])
AC_CONFIG_HEADERS([config.h])

AC_ARG_WITH([or-tools],
  [AS_HELP_STRING([--with-or-tools@<:@=ARG@:>@],
    [use Google or-tools from a standard location (ARG=yes),
     from the specified location (ARG=<path>),
     or disable it (ARG=no)
        @<:@ARG=yes@:>@ ])],
  [
   AS_CASE([$withval],
     [no],[want_or_tools="no";or_tools_path=""],
     [yes],[want_or_tools="yes";or_tools_path=""],
     [want_or_tools="yes";or_tools_path="$withval"])
  ],
  [want_or_tools="yes"])

AC_ARG_WITH([epmc-mj],
  [AS_HELP_STRING([--with-epmc-mj@<:@=ARG@:>@],
    [use epmc-mj from a standard location (ARG=yes),
     from the specified location (ARG=<path>),
     or disable it (ARG=no)
        @<:@ARG=yes@:>@ ])],
  [
   AS_CASE([$withval],
     [no],[want_epmc_mj="no";epmc_mj_path=""],
     [yes],[want_epmc_mj="yes";epmc_mj_path=""],
     [want_epmc_mj="yes";epmc_mj_path="$withval"])
  ],
  [want_epmc_mj="yes"])

AC_ARG_WITH([spot],
  [AS_HELP_STRING([--with-spot@<:@=ARG@:>@],
    [use Spot from a standard location (ARG=yes),
     from the specified location (ARG=<path>),
     or disable it (ARG=no)
        @<:@ARG=yes@:>@ ])],
  [
   AS_CASE([$withval],
     [no],[want_spot="no";spot_path=""],
     [yes],[want_spot="yes";spot_path=""],
     [want_spot="yes";spot_path="$withval"])
  ],
  [want_spot="yes"])

AC_ARG_WITH([owl],
  [AS_HELP_STRING([--with-owl@<:@=ARG@:>@],
    [use Owl from a standard location (ARG=yes),
     from the specified location (ARG=<path>),
     or disable it (ARG=no)
        @<:@ARG=yes@:>@ ])],
  [
   AS_CASE([$withval],
     [no],[want_owl="no";owl_path=""],
     [yes],[want_owl="yes";owl_path=""],
     [want_owl="yes";owl_path="$withval"])
  ],
  [want_owl="yes"])

# Checks for programs.
# Set our own default (instead of "-g -O2") unless CXXFLAGS is already defined.
: ${CXXFLAGS="-g -O3"}
# Do the same for CFLAGS.
: ${CFLAGS="-g -O3"}
AC_PROG_CXX
AC_PROG_CC
m4_ifdef([AM_PROG_AR],[AM_PROG_AR])
AC_PROG_LEX
AC_CHECK_PROGS([YACC], [bison byacc], [yacc])
AC_CHECK_BISON_VERSION([3.0.4])

AC_LANG([C++])
: ${AM_CXXFLAGS="-Wall -Wextra -std=c++14 -pedantic"}
: ${AM_CFLAGS="-Wall -Wextra -std=c11 -pedantic"}

# Building documentation requires doxygen.
AC_CHECK_PROGS([DOXYGEN], [doxygen])
if test -z "$DOXYGEN"; then
  AC_MSG_WARN([Doxygen not found - continuing without Doxygen support])
fi
AM_CONDITIONAL([HAVE_DOXYGEN],[test -n "$DOXYGEN"])
AM_COND_IF([HAVE_DOXYGEN], [AC_CONFIG_FILES([Doxyfile])])

# Checks for libraries.
AX_BOOST_BASE(
  [1.64.0],,[AC_MSG_FAILURE([boost could not be located.])]
)
AX_BOOST_PROGRAM_OPTIONS
AX_BOOST_SERIALIZATION
AX_BOOST_SYSTEM
AX_BOOST_FILESYSTEM
AX_BOOST_PROCESS
if test "x$ax_cv_boost_process" = "xyes"; then
  : ${AM_LDFLAGS="-pthread"}
fi

AX_VALGRIND_DFLT([drd], [off])
AX_VALGRIND_DFLT([helgrind], [off])
AX_VALGRIND_DFLT([sgcheck], [off])
AX_VALGRIND_CHECK

# Check for spot's ltl2tgba.
AC_PATH_PROG([SPOT], [ltl2tgba],[],[${spot_path}/bin])
AM_CONDITIONAL([HAVE_SPOT],[test -n "$SPOT" && test "x$ax_cv_boost_process" = "xyes"])
AM_COND_IF([HAVE_SPOT], [AC_DEFINE([HAVE_SPOT], [1], [Define to 1 if ltl2tgba is present])
  AC_DEFINE_UNQUOTED([SPOT_PATH],"${SPOT}", [Path to ltl2tgba if present])
  AC_SUBST([HAVE_SPOT],[1])], [])

# Check for epmc-mj.
AC_CHECK_FILE([${epmc_mj_path}/epmc-mj-v16.jar],[SLIM=${epmc_mj_path}/epmc-mj-v16.jar],[])
AM_CONDITIONAL([HAVE_SLIM],[test -n "$SLIM" && test "x$HAVE_SPOT" = "x1"])
AM_COND_IF([HAVE_SLIM], [AC_DEFINE([HAVE_SLIM], [1], [Define to 1 if the epmc-mj jar file is present])
  AC_DEFINE_UNQUOTED([SLIM_PATH],"${SLIM}", [Path to epmc-mj jar file if present])
  AC_SUBST([HAVE_SLIM],[1])], [])

# Check for Owl's ltl2ldba.
AC_PATH_PROG([OWL], [ltl2ldba],[],[${owl_path}/bin])
AM_CONDITIONAL([HAVE_OWL],[test -n "$OWL" && test "x$ax_cv_boost_process" = "xyes"])
AM_COND_IF([HAVE_OWL], [AC_DEFINE([HAVE_OWL], [1], [Define to 1 if ltl2ldba is present])
  AC_DEFINE_UNQUOTED([OWL_PATH],"${OWL}", [Path to ltl2ldba if present])
  AC_SUBST([HAVE_OWL],[1])], [])

MJ_SEARCH_ORTOOLS

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_SIZE_T

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_CHECK_FUNCS([select])

AC_SUBST([AM_CPPFLAGS])
AC_SUBST([AM_CXXFLAGS])
AC_SUBST([AM_LDFLAGS])
AC_SUBST([AM_LFLAGS])
AC_SUBST([AM_YFLAGS])

AC_CONFIG_FILES([Makefile Pparser/Pparser.yy])
AC_CONFIG_SUBDIRS([cudd])
AC_OUTPUT
