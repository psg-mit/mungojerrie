Gym_sources = \
  Gym/Gym.hh \
  Gym/Gym.cc

mungojerrie_SOURCES += $(Gym_sources)

AM_CPPFLAGS += -I$(srcdir)/Gym -IGym