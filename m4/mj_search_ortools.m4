#                                               -*- Autoconf -*-
# MJ_SEARCH_ORTOOLS
# -----------------
# Define OR_TOOLS_CPPFLAGS, OR_TOOLS_LDFLAGS, OR_TOOLS_LIB,
# allowing the user to specify the location of Google's
# or-tools library.
# Also controls the definitions of preprocessor tokens
# HAVE_LIBORTOOLS and HAVE_ORTOOLS_LINEAR_SOLVER_LINEAR_SOLVER_H.
AC_DEFUN([MJ_SEARCH_ORTOOLS],
[OR_TOOLS_CPPFLAGS=
OR_TOOLS_LDFLAGS=
OR_TOOLS_LIB=
if test "x${want_or_tools}" = xyes; then
  m4_ifval([], , [AH_CHECK_LIB([ortools])])dnl
  AS_VAR_PUSHDEF([ac_Lib], [ac_cv_lib_ortools_MPSolver])dnl
  AC_CACHE_CHECK([whether -lortools exports operations_research::MPSolver], [ac_Lib],
    [ac_check_lib_save_LIBS=$LIBS
    ac_check_lib_save_LDFLAGS=$LDFLAGS
    LIBS="-lortools $LIBS"
    AS_IF([test "x${or_tools_path}" != "x"],[
      LDFLAGS="-Wl,-rpath $or_tools_path/lib -L$or_tools_path/lib $LDFLAGS"],[])
    AC_LINK_IFELSE([AC_LANG_PROGRAM(
    [[#include <string>
namespace operations_research {
  struct MPSolver {
    enum OptimizationProblemType {GLOP_LINEAR_PROGRAMMING = 2};
    MPSolver(const std::string& name, OptimizationProblemType problem_type);
  };
}
using namespace operations_research;]],
    [[  MPSolver solver("LPExample", MPSolver::GLOP_LINEAR_PROGRAMMING)]])],
      [AS_VAR_SET([ac_Lib], [yes])],
      [AS_VAR_SET([ac_Lib], [no])])
    LIBS=$ac_check_lib_save_LIBS
    LDFLAGS=$ac_check_lib_save_LDFLAGS])
  AS_VAR_IF([ac_Lib], [yes],
    [AC_DEFINE_UNQUOTED(AS_TR_CPP(HAVE_LIBORTOOLS))
    OR_TOOLS_LIB="-lortools"
    AS_IF([test "x${or_tools_path}" != "x"],[
      OR_TOOLS_LDFLAGS="-Wl,-rpath $or_tools_path/lib -L$or_tools_path/lib"],[])
    have_or_tools=yes
    AC_SUBST([HAVE_LIBORTOOLS],[1])
  ],[])dnl
  AS_VAR_POPDEF([ac_Lib])
  AS_IF([test "x${have_or_tools}" = "xyes"],[
    AS_IF([test "x${or_tools_path}" != "x"],[
      ac_check_lib_save_CPPFLAGS=$CPPFLAGS
      CPPFLAGS="-std=c++14 -I$or_tools_path/include $CPPFLAGS"],[])
    AC_CHECK_HEADERS([ortools/linear_solver/linear_solver.h], [],
      [have_or_tools=no])],[])
    AS_IF([test "x${or_tools_path}" != "x"],[
      OR_TOOLS_CPPFLAGS="-I$or_tools_path/include"
      CPPFLAGS=$ac_check_lib_save_CPPFLAGS],[])
  if test "x${have_or_tools}" = xno; then
    AC_MSG_WARN([
----------------------------------
Unable to locate or-tools library.
----------------------------------])
  fi
fi

AC_SUBST([OR_TOOLS_CPPFLAGS])
AC_SUBST([OR_TOOLS_LDFLAGS])
AC_SUBST([OR_TOOLS_LIB])])
