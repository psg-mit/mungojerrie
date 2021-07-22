Gym_sources = \
  Gym/Gym.cc

Gym_headers = \
  Gym/Gym.hh

libmungojerrie_la_SOURCES += $(Gym_sources)
include_HEADERS += $(Gym_headers)

AM_CPPFLAGS += -I$(srcdir)/Gym -IGym