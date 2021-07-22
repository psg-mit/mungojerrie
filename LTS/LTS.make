LTS_sources = \
  LTS/LTS.cc \
  LTS/Parity.cc \
  LTS/Stree.cc

LTS_headers = \
  LTS/LTS.hh \
  LTS/Parity.hh \
  LTS/Stree.hh

libmungojerrie_la_SOURCES += $(LTS_sources)
include_HEADERS += $(LTS_headers)

AM_CPPFLAGS += -I$(srcdir)/LTS -ILTS

check_PROGRAMS += testLTS
TESTS += testLTS

testLTS_SOURCES = \
  LTS/testLTS.cc

testLTS_LDADD = cudd/libobj.a cudd/libcudd.a cudd/libepd.a cudd/libmtr.a \
  cudd/libst.a cudd/libutil.a libmungojerrie.la
