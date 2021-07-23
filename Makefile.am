SUBDIRS = cudd
ACLOCAL_AMFLAGS = -I m4
bin_PROGRAMS = mungojerrie
lib_LTLIBRARIES = libmungojerrie.la

mungojerrie_SOURCES =
libmungojerrie_la_SOURCES =
BUILT_SOURCES =

include_HEADERS =

AM_CPPFLAGS = @AM_CPPFLAGS@
check_PROGRAMS =
check_SCRIPTS =
EXTRA_DIST = README.md examples images/logo.png
dist_check_DATA =
TESTS =
CLEANFILES =
DISTCLEANFILES =
TEST_LOG_DRIVER = env AM_TAP_AWK='$(AWK)' $(SHELL) \
  $(top_srcdir)/build-aux/tap-driver.sh

libmungojerrie_la_LIBADD = $(BOOST_LDFLAGS) $(BOOST_PROGRAM_OPTIONS_LIB) \
  cudd/libobj.la cudd/libcudd.la cudd/libepd.la cudd/libmtr.la \
  cudd/libst.la cudd/libutil.la \
  $(OR_TOOLS_LDFLAGS) $(OR_TOOLS_LIB) \
  $(BOOST_SERIALIZATION_LIB) \
  $(BOOST_FILESYSTEM_LIB) \
  $(BOOST_SYSTEM_LIB)

do_subst = sed \
  -e 's,[@]EXEEXT[@],$(EXEEXT),g' \
  -e 's,[@]srcdir[@],$(srcdir),g' \
  -e 's,[@]HAVE_SLIM[@],$(HAVE_SLIM),g' \
  -e 's,[@]HAVE_SPOT[@],$(HAVE_SPOT),g' \
  -e 's,[@]HAVE_OWL[@],$(HAVE_OWL),g' \
  -e 's,[@]HAVE_LIBORTOOLS[@],$(HAVE_LIBORTOOLS),g'

include $(top_srcdir)/Util/Util.make
include $(top_srcdir)/LTS/LTS.make
include $(top_srcdir)/Model/Model.make
include $(top_srcdir)/Pparser/Pparser.make
include $(top_srcdir)/Hparser/Hparser.make
include $(top_srcdir)/Gym/Gym.make
include $(top_srcdir)/Learners/Learners.make

mungojerrie_LDADD = libmungojerrie.la

include $(top_srcdir)/Main/Main.make

AM_CPPFLAGS += $(BOOST_CPPFLAGS) -I$(top_srcdir)/cudd -DUSE_GLOP $(OR_TOOLS_CPPFLAGS)
AM_CXXFLAGS += $(OPENMP_CXXFLAGS)

TESTS += $(check_SCRIPTS)
CLEANFILES += $(check_SCRIPTS)

dist-hook:
	rm -rf `find $(distdir) -name .git`
	rm -rf $(distdir)/_config.yml $(distdir)/examples/cav
	rm -rf $(distdir)/Pparser/Pparser.yy
	rm -rf `find $(distdir) -name '*~'`

if HAVE_DOXYGEN

html/index.html: Doxyfile README.md $(mungojerrie_SOURCES) $(libmungojerrie_la_SOURCES) $(testSet_SOURCES) $(testLTS_SOURCES) $(testModel_SOURCES) $(testProduct_SOURCES)
	@if $(AM_V_P); then dest='2>&1'; else dest='> /dev/null 2>&1'; fi; \
	eval "$(DOXYGEN) $< $${dest}"

DISTCLEANFILES += Doxyfile

clean-local:
	$(RM) -r html

else

html/index.html:

endif

@VALGRIND_CHECK_RULES@
VALGRIND_SUPPRESSIONS_FILES =
