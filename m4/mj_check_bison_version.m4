dnl                                             -*- Autoconf -*-
dnl
dnl Check Bison version
dnl AC_CHECK_BISON_VERSION([MIN_VERSION=3.0.4])
dnl


AC_DEFUN([AC_CHECK_BISON_VERSION], [
if test "x$1" = "x" ; then
  bison_required_version="3.0.4"
else
  bison_required_version="$1"
fi
bison_new_api_version="3.3"
AC_DEFINE_UNQUOTED([BISON_VERSION], [0.0], [Bison version if bison is not available])
AC_MSG_CHECKING([for bison version >= $bison_required_version])
bison_version=`$YACC --version | head -n 1 | cut '-d ' -f 4`
AC_DEFINE_UNQUOTED([BISON_VERSION], [$bison_version], [Defines bison version])
if test "$bison_version" \< "$bison_required_version" ; then
  BISON=:
  AC_MSG_RESULT([no])
  AC_MSG_ERROR([Bison version $bison_required_version or higher must be installed on the system!])
else
  AC_MSG_RESULT([yes])
fi
if test "$bison_version" \< "$bison_new_api_version" ; then
  AC_DEFINE_UNQUOTED([BISON_PARSER_CLASS], [parser_class_name], [old style api])
  AC_SUBST([BISON_PARSER_CLASS], [parser_class_name])
  AC_DEFINE_UNQUOTED([BISON_NEW_API], [0], [Bison does not accept api.parser.class])
else
  AC_DEFINE_UNQUOTED([BISON_PARSER_CLASS], [api.parser.class], [new style api])
  AC_SUBST([BISON_PARSER_CLASS], [api.parser.class])
  AC_DEFINE_UNQUOTED([BISON_NEW_API], [1], [Bison accepts api.parser.class])
fi
])
