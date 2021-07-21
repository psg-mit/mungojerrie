Main_sources = \
  Main/Verbosity.hh \
  Main/CommandLineOptions.hh \
  Main/CommandLineOptions.cc \
  Main/main.cc

mungojerrie_SOURCES += $(Main_sources)

AM_CPPFLAGS += -I$(srcdir)/Main -IMain

check_SCRIPTS += Main/test_mj.test

dist_check_DATA += examples/lda3-5.prism examples/lda.hoa \
  examples/counter.prism examples/counter.hoa

EXTRA_DIST += Main/test_mj.test.in

Main/test_mj.test: Main/test_mj.test.in Makefile
	$(do_subst) $< > $@
	chmod +x $@

CLEANFILES += Main/*.tst Main/differences
