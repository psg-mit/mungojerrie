Model_sources = \
  Model/Model.hh \
  Model/Model.cc \
  Model/ModelCheck.cc \
  Model/ModelGames.cc \
  Model/ModelOptions.hh

mungojerrie_SOURCES += $(Model_sources)

AM_CPPFLAGS += -I$(srcdir)/Model -IModel

check_PROGRAMS += testModel testProduct
TESTS += testModel testProduct

testModel_SOURCES = \
  Model/testModel.cc \
  $(Model_sources) \
  $(LTS_sources) \
  $(Util_sources)

testProduct_SOURCES = \
  Model/testProduct.cc \
  $(Model_sources) \
  $(LTS_sources) \
  $(Util_sources)
