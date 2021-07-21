Util_sources = \
  Util/Set.hh \
  Util/State.hh \
  Util/Util.hh \
  Util/Util.cc

mungojerrie_SOURCES += $(Util_sources)

AM_CPPFLAGS += -I$(srcdir)/Util -IUtil

check_PROGRAMS += testSet testUtil
TESTS += testSet testUtil

testSet_SOURCES = \
  Util/testSet.cc \
  Util/Set.hh

testSet_LDADD =

testUtil_SOURCES = \
  Util/testUtil.cc \
  Util/Util.hh \
  Util/Util.cc

testUtil_LDADD =
