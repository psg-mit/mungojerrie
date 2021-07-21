Pparser_sources = \
  Pparser/Pwrapper.hh \
  Pparser/Pparser.yy \
  Pparser/Pscanner.ll \
  Pparser/Past.hh \
  Pparser/Pdriver.hh \
  Pparser/Pdriver.cc

mungojerrie_SOURCES += $(Pparser_sources)
BUILT_SOURCES += Pparser/Pparser.hh Pparser/Pparser.cc

testModel_SOURCES += $(Pparser_sources)
testProduct_SOURCES += $(Pparser_sources)

AM_LFLAGS = -o$(LEX_OUTPUT_ROOT).c
AM_CPPFLAGS += -I$(srcdir)/Pparser -IPparser

Pparser/Pdriver.$(OBJEXT) Pparser/Pscanner.cc: Pparser/Pparser.hh

Pparser/Pparser.hh: Pparser/Pparser.cc

Pparser/Pparser.cc: Pparser/Pparser.yy
	$(YACC) -o Pparser/Pparser.cc --name-prefix=pparser $(YFLAGS) Pparser/Pparser.yy

CLEANFILES += \
  Pparser/Pparser.yy \
  Pparser/Pparser.hh \
  Pparser/Pparser.cc \
  Pparser/Pscanner.cc \
  Pparser/location.hh \
  Pparser/stack.hh \
  Pparser/position.hh
