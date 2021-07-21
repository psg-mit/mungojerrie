Learners_sources = \
  Learners/Learner.hh \
  Learners/Learner.cc

mungojerrie_SOURCES += $(Learners_sources)

AM_CPPFLAGS += -I$(srcdir)/Learners -ILearners