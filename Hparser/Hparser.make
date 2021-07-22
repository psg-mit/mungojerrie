Hparser_sources = \
  Hparser/Hparser.cc

Hparser_headers = \
  Hparser/Hparser.hh \
  Hparser/cpphoafparser/ast/atom_acceptance.hh \
  Hparser/cpphoafparser/ast/atom_label.hh \
  Hparser/cpphoafparser/ast/boolean_expression.hh \
  Hparser/cpphoafparser/consumer/hoa_consumer_exception.hh \
  Hparser/cpphoafparser/consumer/hoa_intermediate_check_validity.hh \
  Hparser/cpphoafparser/consumer/hoa_consumer.hh \
  Hparser/cpphoafparser/consumer/hoa_intermediate.hh \
  Hparser/cpphoafparser/consumer/hoa_consumer_null.hh \
  Hparser/cpphoafparser/consumer/hoa_intermediate_resolve_aliases.hh \
  Hparser/cpphoafparser/consumer/hoa_consumer_print.hh \
  Hparser/cpphoafparser/consumer/hoa_intermediate_trace.hh \
  Hparser/cpphoafparser/parser/hoa_lexer.hh \
  Hparser/cpphoafparser/parser/hoa_parser_exception.hh \
  Hparser/cpphoafparser/parser/hoa_parser_helper.hh \
  Hparser/cpphoafparser/parser/hoa_parser.hh \
  Hparser/cpphoafparser/util/acceptance_repository.hh \
  Hparser/cpphoafparser/util/dynamic_bitset.hh \
  Hparser/cpphoafparser/util/int_or_string.hh \
  Hparser/cpphoafparser/util/acceptance_repository_standard.hh \
  Hparser/cpphoafparser/util/implicit_edge_helper.hh

libmungojerrie_la_SOURCES += $(Hparser_sources)
include_HEADERS += $(Hparser_headers)

AM_CPPFLAGS += -I$(srcdir)/Hparser -IHparser
