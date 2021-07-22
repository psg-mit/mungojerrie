Model_sources = \
  Model/Model.cc \
  Model/ModelCheck.cc \
  Model/ModelGames.cc

Model_headers = \
  Model/Model.hh \
  Model/ModelOptions.hh

libmungojerrie_la_SOURCES += $(Model_sources)
include_HEADERS += $(Model_headers)

AM_CPPFLAGS += -I$(srcdir)/Model -IModel

check_PROGRAMS += testModel testProduct
TESTS += testModel testProduct

testModel_SOURCES = \
  Model/testModel.cc

testProduct_SOURCES = \
  Model/testProduct.cc

testModel_LDADD = libmungojerrie.la
testProduct_LDADD = libmungojerrie.la
