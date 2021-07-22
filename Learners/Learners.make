Learners_sources = \
  Learners/Learner.cc

Learners_headers = \
  Learners/Learner.hh

libmungojerrie_la_SOURCES += $(Learners_sources)
include_HEADERS += $(Learners_headers)

AM_CPPFLAGS += -I$(srcdir)/Learners -ILearners