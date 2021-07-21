LTS_sources = \
  LTS/LTS.hh \
  LTS/LTS.cc \
  LTS/Parity.hh \
  LTS/Parity.cc \
  LTS/Stree.hh \
  LTS/Stree.cc

mungojerrie_SOURCES += $(LTS_sources)

AM_CPPFLAGS += -I$(srcdir)/LTS -ILTS

check_PROGRAMS += testLTS
TESTS += testLTS

testLTS_SOURCES = \
  LTS/testLTS.cc \
  $(LTS_sources) \
  $(Util_sources)

testLTS_LDADD = cudd/libobj.a cudd/libcudd.a cudd/libepd.a cudd/libmtr.a \
  cudd/libst.a cudd/libutil.a
