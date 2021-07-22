Util_sources = \
  Util/Util.cc

Util_headers = \
  Util/Set.hh \
  Util/State.hh \
  Util/Util.hh

libmungojerrie_la_SOURCES += $(Util_sources)
include_HEADERS += $(Util_headers)

AM_CPPFLAGS += -I$(srcdir)/Util -IUtil

check_PROGRAMS += testSet testUtil
TESTS += testSet testUtil

testSet_SOURCES = \
  Util/testSet.cc

testSet_LDADD = libmungojerrie.la

testUtil_SOURCES = \
  Util/testUtil.cc

testUtil_LDADD = libmungojerrie.la
