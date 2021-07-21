// A Bison parser, made by GNU Bison 3.0.4.

// Skeleton implementation for Bison LALR(1) parsers in C++

// Copyright (C) 2002-2015 Free Software Foundation, Inc.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// As a special exception, you may create a larger work that contains
// part or all of the Bison parser skeleton and distribute that work
// under terms of your choice, so long as that work isn't itself a
// parser generator using the skeleton or a modified version thereof
// as a parser skeleton.  Alternatively, if you modify or redistribute
// the parser skeleton itself, you may (at your option) remove this
// special exception, which will cause the skeleton and the resulting
// Bison output files to be licensed under the GNU General Public
// License without this special exception.

// This special exception was added by the Free Software Foundation in
// version 2.2 of Bison.

// Take the name prefix into account.
#define yylex   pparserlex

// First part of user declarations.

#line 39 "Pparser/Pparser.cc" // lalr1.cc:404

# ifndef YY_NULLPTR
#  if defined __cplusplus && 201103L <= __cplusplus
#   define YY_NULLPTR nullptr
#  else
#   define YY_NULLPTR 0
#  endif
# endif

#include "Pparser.hh"

// User implementation prologue.

#line 53 "Pparser/Pparser.cc" // lalr1.cc:412
// Unqualified %code blocks.
#line 77 "Pparser/Pparser.yy" // lalr1.cc:413

#include "Pwrapper.hh"
  ast::LocationType astloc(pparser::p_parser::location_type const & l);

#line 60 "Pparser/Pparser.cc" // lalr1.cc:413


#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> // FIXME: INFRINGES ON USER NAME SPACE.
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

#define YYRHSLOC(Rhs, K) ((Rhs)[K].location)
/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

# ifndef YYLLOC_DEFAULT
#  define YYLLOC_DEFAULT(Current, Rhs, N)                               \
    do                                                                  \
      if (N)                                                            \
        {                                                               \
          (Current).begin  = YYRHSLOC (Rhs, 1).begin;                   \
          (Current).end    = YYRHSLOC (Rhs, N).end;                     \
        }                                                               \
      else                                                              \
        {                                                               \
          (Current).begin = (Current).end = YYRHSLOC (Rhs, 0).end;      \
        }                                                               \
    while (/*CONSTCOND*/ false)
# endif


// Suppress unused-variable warnings by "using" E.
#define YYUSE(E) ((void) (E))

// Enable debugging if requested.
#if YYDEBUG

// A pseudo ostream that takes yydebug_ into account.
# define YYCDEBUG if (yydebug_) (*yycdebug_)

# define YY_SYMBOL_PRINT(Title, Symbol)         \
  do {                                          \
    if (yydebug_)                               \
    {                                           \
      *yycdebug_ << Title << ' ';               \
      yy_print_ (*yycdebug_, Symbol);           \
      *yycdebug_ << std::endl;                  \
    }                                           \
  } while (false)

# define YY_REDUCE_PRINT(Rule)          \
  do {                                  \
    if (yydebug_)                       \
      yy_reduce_print_ (Rule);          \
  } while (false)

# define YY_STACK_PRINT()               \
  do {                                  \
    if (yydebug_)                       \
      yystack_print_ ();                \
  } while (false)

#else // !YYDEBUG

# define YYCDEBUG if (false) std::cerr
# define YY_SYMBOL_PRINT(Title, Symbol)  YYUSE(Symbol)
# define YY_REDUCE_PRINT(Rule)           static_cast<void>(0)
# define YY_STACK_PRINT()                static_cast<void>(0)

#endif // !YYDEBUG

#define yyerrok         (yyerrstatus_ = 0)
#define yyclearin       (yyla.clear ())

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab
#define YYRECOVERING()  (!!yyerrstatus_)


namespace pparser {
#line 146 "Pparser/Pparser.cc" // lalr1.cc:479

  /* Return YYSTR after stripping away unnecessary quotes and
     backslashes, so that it's suitable for yyerror.  The heuristic is
     that double-quoting is unnecessary unless the string contains an
     apostrophe, a comma, or backslash (other than backslash-backslash).
     YYSTR is taken from yytname.  */
  std::string
  p_parser::yytnamerr_ (const char *yystr)
  {
    if (*yystr == '"')
      {
        std::string yyr = "";
        char const *yyp = yystr;

        for (;;)
          switch (*++yyp)
            {
            case '\'':
            case ',':
              goto do_not_strip_quotes;

            case '\\':
              if (*++yyp != '\\')
                goto do_not_strip_quotes;
              // Fall through.
            default:
              yyr += *yyp;
              break;

            case '"':
              return yyr;
            }
      do_not_strip_quotes: ;
      }

    return yystr;
  }


  /// Build a parser object.
  p_parser::p_parser (p_driver& driver_yyarg)
    :
#if YYDEBUG
      yydebug_ (false),
      yycdebug_ (&std::cerr),
#endif
      driver (driver_yyarg)
  {}

  p_parser::~p_parser ()
  {}


  /*---------------.
  | Symbol types.  |
  `---------------*/

  inline
  p_parser::syntax_error::syntax_error (const location_type& l, const std::string& m)
    : std::runtime_error (m)
    , location (l)
  {}

  // basic_symbol.
  template <typename Base>
  inline
  p_parser::basic_symbol<Base>::basic_symbol ()
    : value ()
  {}

  template <typename Base>
  inline
  p_parser::basic_symbol<Base>::basic_symbol (const basic_symbol& other)
    : Base (other)
    , value ()
    , location (other.location)
  {
      switch (other.type_get ())
    {
      case 107: // module_command
        value.copy< ast::Command > (other.value);
        break;

      case 133: // control_specification
        value.copy< ast::ControlSpec > (other.value);
        break;

      case 103: // low_value
      case 104: // high_value
      case 105: // init_value
      case 109: // command_guard
      case 113: // next_state_expression
      case 114: // primed_state_expression
      case 118: // expression
      case 128: // reward_guard
      case 129: // reward_value
        value.copy< ast::Expression > (other.value);
        break;

      case 116: // formula_definition
        value.copy< ast::Formula > (other.value);
        break;

      case 122: // replacement
        value.copy< ast::NameReplacement > (other.value);
        break;

      case 124: // rewards_definition
        value.copy< ast::RewardSection > (other.value);
        break;

      case 127: // reward_specification
        value.copy< ast::RewardSpec > (other.value);
        break;

      case 112: // prob_transition
        value.copy< ast::Transition > (other.value);
        break;

      case 101: // variable_definition
      case 102: // global_variable_definition
        value.copy< ast::Variable > (other.value);
        break;

      case 71: // "floating-point number"
        value.copy< double > (other.value);
        break;

      case 70: // "integer number"
        value.copy< int > (other.value);
        break;

      case 106: // module_variables_and_commands
        value.copy< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > (other.value);
        break;

      case 69: // "identifier"
      case 100: // module_name
      case 108: // action_name
      case 117: // formula_name
      case 125: // optional_reward_name
      case 131: // player_name
      case 134: // variable_name
        value.copy< std::string > (other.value);
        break;

      case 132: // player_control_list
        value.copy< std::vector<ast::ControlSpec> > (other.value);
        break;

      case 119: // expression_list
        value.copy< std::vector<ast::Expression> > (other.value);
        break;

      case 121: // replacement_list
        value.copy< std::vector<ast::NameReplacement> > (other.value);
        break;

      case 130: // player_definition
        value.copy< std::vector<ast::Player> > (other.value);
        break;

      case 126: // reward_list
        value.copy< std::vector<ast::RewardSpec>  > (other.value);
        break;

      case 110: // transitions
      case 111: // transition_list
        value.copy< std::vector<ast::Transition>  > (other.value);
        break;

      default:
        break;
    }

  }


  template <typename Base>
  inline
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const semantic_type& v, const location_type& l)
    : Base (t)
    , value ()
    , location (l)
  {
    (void) v;
      switch (this->type_get ())
    {
      case 107: // module_command
        value.copy< ast::Command > (v);
        break;

      case 133: // control_specification
        value.copy< ast::ControlSpec > (v);
        break;

      case 103: // low_value
      case 104: // high_value
      case 105: // init_value
      case 109: // command_guard
      case 113: // next_state_expression
      case 114: // primed_state_expression
      case 118: // expression
      case 128: // reward_guard
      case 129: // reward_value
        value.copy< ast::Expression > (v);
        break;

      case 116: // formula_definition
        value.copy< ast::Formula > (v);
        break;

      case 122: // replacement
        value.copy< ast::NameReplacement > (v);
        break;

      case 124: // rewards_definition
        value.copy< ast::RewardSection > (v);
        break;

      case 127: // reward_specification
        value.copy< ast::RewardSpec > (v);
        break;

      case 112: // prob_transition
        value.copy< ast::Transition > (v);
        break;

      case 101: // variable_definition
      case 102: // global_variable_definition
        value.copy< ast::Variable > (v);
        break;

      case 71: // "floating-point number"
        value.copy< double > (v);
        break;

      case 70: // "integer number"
        value.copy< int > (v);
        break;

      case 106: // module_variables_and_commands
        value.copy< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > (v);
        break;

      case 69: // "identifier"
      case 100: // module_name
      case 108: // action_name
      case 117: // formula_name
      case 125: // optional_reward_name
      case 131: // player_name
      case 134: // variable_name
        value.copy< std::string > (v);
        break;

      case 132: // player_control_list
        value.copy< std::vector<ast::ControlSpec> > (v);
        break;

      case 119: // expression_list
        value.copy< std::vector<ast::Expression> > (v);
        break;

      case 121: // replacement_list
        value.copy< std::vector<ast::NameReplacement> > (v);
        break;

      case 130: // player_definition
        value.copy< std::vector<ast::Player> > (v);
        break;

      case 126: // reward_list
        value.copy< std::vector<ast::RewardSpec>  > (v);
        break;

      case 110: // transitions
      case 111: // transition_list
        value.copy< std::vector<ast::Transition>  > (v);
        break;

      default:
        break;
    }
}


  // Implementation of basic_symbol constructor for each type.

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const location_type& l)
    : Base (t)
    , value ()
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::Command v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::ControlSpec v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::Expression v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::Formula v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::NameReplacement v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::RewardSection v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::RewardSpec v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::Transition v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const ast::Variable v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const double v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const int v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const std::pair<std::vector<ast::Variable>, std::vector<ast::Command>> v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const std::string v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const std::vector<ast::ControlSpec> v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const std::vector<ast::Expression> v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const std::vector<ast::NameReplacement> v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const std::vector<ast::Player> v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const std::vector<ast::RewardSpec>  v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}

  template <typename Base>
  p_parser::basic_symbol<Base>::basic_symbol (typename Base::kind_type t, const std::vector<ast::Transition>  v, const location_type& l)
    : Base (t)
    , value (v)
    , location (l)
  {}


  template <typename Base>
  inline
  p_parser::basic_symbol<Base>::~basic_symbol ()
  {
    clear ();
  }

  template <typename Base>
  inline
  void
  p_parser::basic_symbol<Base>::clear ()
  {
    // User destructor.
    symbol_number_type yytype = this->type_get ();
    basic_symbol<Base>& yysym = *this;
    (void) yysym;
    switch (yytype)
    {
   default:
      break;
    }

    // Type destructor.
    switch (yytype)
    {
      case 107: // module_command
        value.template destroy< ast::Command > ();
        break;

      case 133: // control_specification
        value.template destroy< ast::ControlSpec > ();
        break;

      case 103: // low_value
      case 104: // high_value
      case 105: // init_value
      case 109: // command_guard
      case 113: // next_state_expression
      case 114: // primed_state_expression
      case 118: // expression
      case 128: // reward_guard
      case 129: // reward_value
        value.template destroy< ast::Expression > ();
        break;

      case 116: // formula_definition
        value.template destroy< ast::Formula > ();
        break;

      case 122: // replacement
        value.template destroy< ast::NameReplacement > ();
        break;

      case 124: // rewards_definition
        value.template destroy< ast::RewardSection > ();
        break;

      case 127: // reward_specification
        value.template destroy< ast::RewardSpec > ();
        break;

      case 112: // prob_transition
        value.template destroy< ast::Transition > ();
        break;

      case 101: // variable_definition
      case 102: // global_variable_definition
        value.template destroy< ast::Variable > ();
        break;

      case 71: // "floating-point number"
        value.template destroy< double > ();
        break;

      case 70: // "integer number"
        value.template destroy< int > ();
        break;

      case 106: // module_variables_and_commands
        value.template destroy< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > ();
        break;

      case 69: // "identifier"
      case 100: // module_name
      case 108: // action_name
      case 117: // formula_name
      case 125: // optional_reward_name
      case 131: // player_name
      case 134: // variable_name
        value.template destroy< std::string > ();
        break;

      case 132: // player_control_list
        value.template destroy< std::vector<ast::ControlSpec> > ();
        break;

      case 119: // expression_list
        value.template destroy< std::vector<ast::Expression> > ();
        break;

      case 121: // replacement_list
        value.template destroy< std::vector<ast::NameReplacement> > ();
        break;

      case 130: // player_definition
        value.template destroy< std::vector<ast::Player> > ();
        break;

      case 126: // reward_list
        value.template destroy< std::vector<ast::RewardSpec>  > ();
        break;

      case 110: // transitions
      case 111: // transition_list
        value.template destroy< std::vector<ast::Transition>  > ();
        break;

      default:
        break;
    }

    Base::clear ();
  }

  template <typename Base>
  inline
  bool
  p_parser::basic_symbol<Base>::empty () const
  {
    return Base::type_get () == empty_symbol;
  }

  template <typename Base>
  inline
  void
  p_parser::basic_symbol<Base>::move (basic_symbol& s)
  {
    super_type::move(s);
      switch (this->type_get ())
    {
      case 107: // module_command
        value.move< ast::Command > (s.value);
        break;

      case 133: // control_specification
        value.move< ast::ControlSpec > (s.value);
        break;

      case 103: // low_value
      case 104: // high_value
      case 105: // init_value
      case 109: // command_guard
      case 113: // next_state_expression
      case 114: // primed_state_expression
      case 118: // expression
      case 128: // reward_guard
      case 129: // reward_value
        value.move< ast::Expression > (s.value);
        break;

      case 116: // formula_definition
        value.move< ast::Formula > (s.value);
        break;

      case 122: // replacement
        value.move< ast::NameReplacement > (s.value);
        break;

      case 124: // rewards_definition
        value.move< ast::RewardSection > (s.value);
        break;

      case 127: // reward_specification
        value.move< ast::RewardSpec > (s.value);
        break;

      case 112: // prob_transition
        value.move< ast::Transition > (s.value);
        break;

      case 101: // variable_definition
      case 102: // global_variable_definition
        value.move< ast::Variable > (s.value);
        break;

      case 71: // "floating-point number"
        value.move< double > (s.value);
        break;

      case 70: // "integer number"
        value.move< int > (s.value);
        break;

      case 106: // module_variables_and_commands
        value.move< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > (s.value);
        break;

      case 69: // "identifier"
      case 100: // module_name
      case 108: // action_name
      case 117: // formula_name
      case 125: // optional_reward_name
      case 131: // player_name
      case 134: // variable_name
        value.move< std::string > (s.value);
        break;

      case 132: // player_control_list
        value.move< std::vector<ast::ControlSpec> > (s.value);
        break;

      case 119: // expression_list
        value.move< std::vector<ast::Expression> > (s.value);
        break;

      case 121: // replacement_list
        value.move< std::vector<ast::NameReplacement> > (s.value);
        break;

      case 130: // player_definition
        value.move< std::vector<ast::Player> > (s.value);
        break;

      case 126: // reward_list
        value.move< std::vector<ast::RewardSpec>  > (s.value);
        break;

      case 110: // transitions
      case 111: // transition_list
        value.move< std::vector<ast::Transition>  > (s.value);
        break;

      default:
        break;
    }

    location = s.location;
  }

  // by_type.
  inline
  p_parser::by_type::by_type ()
    : type (empty_symbol)
  {}

  inline
  p_parser::by_type::by_type (const by_type& other)
    : type (other.type)
  {}

  inline
  p_parser::by_type::by_type (token_type t)
    : type (yytranslate_ (t))
  {}

  inline
  void
  p_parser::by_type::clear ()
  {
    type = empty_symbol;
  }

  inline
  void
  p_parser::by_type::move (by_type& that)
  {
    type = that.type;
    that.clear ();
  }

  inline
  int
  p_parser::by_type::type_get () const
  {
    return type;
  }
  // Implementation of make_symbol for each symbol type.
  p_parser::symbol_type
  p_parser::make_END (const location_type& l)
  {
    return symbol_type (token::END, l);
  }

  p_parser::symbol_type
  p_parser::make_NOTEQ (const location_type& l)
  {
    return symbol_type (token::NOTEQ, l);
  }

  p_parser::symbol_type
  p_parser::make_ARROW (const location_type& l)
  {
    return symbol_type (token::ARROW, l);
  }

  p_parser::symbol_type
  p_parser::make_RANGE (const location_type& l)
  {
    return symbol_type (token::RANGE, l);
  }

  p_parser::symbol_type
  p_parser::make_IFF (const location_type& l)
  {
    return symbol_type (token::IFF, l);
  }

  p_parser::symbol_type
  p_parser::make_IMPLIES (const location_type& l)
  {
    return symbol_type (token::IMPLIES, l);
  }

  p_parser::symbol_type
  p_parser::make_GE (const location_type& l)
  {
    return symbol_type (token::GE, l);
  }

  p_parser::symbol_type
  p_parser::make_LE (const location_type& l)
  {
    return symbol_type (token::LE, l);
  }

  p_parser::symbol_type
  p_parser::make_A (const location_type& l)
  {
    return symbol_type (token::A, l);
  }

  p_parser::symbol_type
  p_parser::make_BOOL (const location_type& l)
  {
    return symbol_type (token::BOOL, l);
  }

  p_parser::symbol_type
  p_parser::make_CLOCK (const location_type& l)
  {
    return symbol_type (token::CLOCK, l);
  }

  p_parser::symbol_type
  p_parser::make_CONST (const location_type& l)
  {
    return symbol_type (token::CONST, l);
  }

  p_parser::symbol_type
  p_parser::make_CTMC (const location_type& l)
  {
    return symbol_type (token::CTMC, l);
  }

  p_parser::symbol_type
  p_parser::make_C (const location_type& l)
  {
    return symbol_type (token::C, l);
  }

  p_parser::symbol_type
  p_parser::make_CEIL (const location_type& l)
  {
    return symbol_type (token::CEIL, l);
  }

  p_parser::symbol_type
  p_parser::make_DOUBLE (const location_type& l)
  {
    return symbol_type (token::DOUBLE, l);
  }

  p_parser::symbol_type
  p_parser::make_DTMC (const location_type& l)
  {
    return symbol_type (token::DTMC, l);
  }

  p_parser::symbol_type
  p_parser::make_E (const location_type& l)
  {
    return symbol_type (token::E, l);
  }

  p_parser::symbol_type
  p_parser::make_ENDINIT (const location_type& l)
  {
    return symbol_type (token::ENDINIT, l);
  }

  p_parser::symbol_type
  p_parser::make_ENDINVARIANT (const location_type& l)
  {
    return symbol_type (token::ENDINVARIANT, l);
  }

  p_parser::symbol_type
  p_parser::make_ENDMODULE (const location_type& l)
  {
    return symbol_type (token::ENDMODULE, l);
  }

  p_parser::symbol_type
  p_parser::make_ENDPLAYER (const location_type& l)
  {
    return symbol_type (token::ENDPLAYER, l);
  }

  p_parser::symbol_type
  p_parser::make_ENDREWARDS (const location_type& l)
  {
    return symbol_type (token::ENDREWARDS, l);
  }

  p_parser::symbol_type
  p_parser::make_ENDSYSTEM (const location_type& l)
  {
    return symbol_type (token::ENDSYSTEM, l);
  }

  p_parser::symbol_type
  p_parser::make_FALSE (const location_type& l)
  {
    return symbol_type (token::FALSE, l);
  }

  p_parser::symbol_type
  p_parser::make_FLOOR (const location_type& l)
  {
    return symbol_type (token::FLOOR, l);
  }

  p_parser::symbol_type
  p_parser::make_FORMULA (const location_type& l)
  {
    return symbol_type (token::FORMULA, l);
  }

  p_parser::symbol_type
  p_parser::make_FILTER (const location_type& l)
  {
    return symbol_type (token::FILTER, l);
  }

  p_parser::symbol_type
  p_parser::make_FUNC (const location_type& l)
  {
    return symbol_type (token::FUNC, l);
  }

  p_parser::symbol_type
  p_parser::make_F (const location_type& l)
  {
    return symbol_type (token::F, l);
  }

  p_parser::symbol_type
  p_parser::make_GLOBAL (const location_type& l)
  {
    return symbol_type (token::GLOBAL, l);
  }

  p_parser::symbol_type
  p_parser::make_G (const location_type& l)
  {
    return symbol_type (token::G, l);
  }

  p_parser::symbol_type
  p_parser::make_INIT (const location_type& l)
  {
    return symbol_type (token::INIT, l);
  }

  p_parser::symbol_type
  p_parser::make_INVARIANT (const location_type& l)
  {
    return symbol_type (token::INVARIANT, l);
  }

  p_parser::symbol_type
  p_parser::make_I (const location_type& l)
  {
    return symbol_type (token::I, l);
  }

  p_parser::symbol_type
  p_parser::make_INT (const location_type& l)
  {
    return symbol_type (token::INT, l);
  }

  p_parser::symbol_type
  p_parser::make_LABEL (const location_type& l)
  {
    return symbol_type (token::LABEL, l);
  }

  p_parser::symbol_type
  p_parser::make_LOG (const location_type& l)
  {
    return symbol_type (token::LOG, l);
  }

  p_parser::symbol_type
  p_parser::make_MAX (const location_type& l)
  {
    return symbol_type (token::MAX, l);
  }

  p_parser::symbol_type
  p_parser::make_MDP (const location_type& l)
  {
    return symbol_type (token::MDP, l);
  }

  p_parser::symbol_type
  p_parser::make_MIN (const location_type& l)
  {
    return symbol_type (token::MIN, l);
  }

  p_parser::symbol_type
  p_parser::make_MOD (const location_type& l)
  {
    return symbol_type (token::MOD, l);
  }

  p_parser::symbol_type
  p_parser::make_MODULE (const location_type& l)
  {
    return symbol_type (token::MODULE, l);
  }

  p_parser::symbol_type
  p_parser::make_X (const location_type& l)
  {
    return symbol_type (token::X, l);
  }

  p_parser::symbol_type
  p_parser::make_NONDETERMINISTIC (const location_type& l)
  {
    return symbol_type (token::NONDETERMINISTIC, l);
  }

  p_parser::symbol_type
  p_parser::make_PLAYER (const location_type& l)
  {
    return symbol_type (token::PLAYER, l);
  }

  p_parser::symbol_type
  p_parser::make_PMAX (const location_type& l)
  {
    return symbol_type (token::PMAX, l);
  }

  p_parser::symbol_type
  p_parser::make_PMIN (const location_type& l)
  {
    return symbol_type (token::PMIN, l);
  }

  p_parser::symbol_type
  p_parser::make_P (const location_type& l)
  {
    return symbol_type (token::P, l);
  }

  p_parser::symbol_type
  p_parser::make_POW (const location_type& l)
  {
    return symbol_type (token::POW, l);
  }

  p_parser::symbol_type
  p_parser::make_PROBABILISTIC (const location_type& l)
  {
    return symbol_type (token::PROBABILISTIC, l);
  }

  p_parser::symbol_type
  p_parser::make_PROB (const location_type& l)
  {
    return symbol_type (token::PROB, l);
  }

  p_parser::symbol_type
  p_parser::make_PTA (const location_type& l)
  {
    return symbol_type (token::PTA, l);
  }

  p_parser::symbol_type
  p_parser::make_RATE (const location_type& l)
  {
    return symbol_type (token::RATE, l);
  }

  p_parser::symbol_type
  p_parser::make_REWARDS (const location_type& l)
  {
    return symbol_type (token::REWARDS, l);
  }

  p_parser::symbol_type
  p_parser::make_RMAX (const location_type& l)
  {
    return symbol_type (token::RMAX, l);
  }

  p_parser::symbol_type
  p_parser::make_RMIN (const location_type& l)
  {
    return symbol_type (token::RMIN, l);
  }

  p_parser::symbol_type
  p_parser::make_R (const location_type& l)
  {
    return symbol_type (token::R, l);
  }

  p_parser::symbol_type
  p_parser::make_S (const location_type& l)
  {
    return symbol_type (token::S, l);
  }

  p_parser::symbol_type
  p_parser::make_SMG (const location_type& l)
  {
    return symbol_type (token::SMG, l);
  }

  p_parser::symbol_type
  p_parser::make_BDP (const location_type& l)
  {
    return symbol_type (token::BDP, l);
  }

  p_parser::symbol_type
  p_parser::make_STOCHASTIC (const location_type& l)
  {
    return symbol_type (token::STOCHASTIC, l);
  }

  p_parser::symbol_type
  p_parser::make_SYSTEM (const location_type& l)
  {
    return symbol_type (token::SYSTEM, l);
  }

  p_parser::symbol_type
  p_parser::make_TRUE (const location_type& l)
  {
    return symbol_type (token::TRUE, l);
  }

  p_parser::symbol_type
  p_parser::make_U (const location_type& l)
  {
    return symbol_type (token::U, l);
  }

  p_parser::symbol_type
  p_parser::make_W (const location_type& l)
  {
    return symbol_type (token::W, l);
  }

  p_parser::symbol_type
  p_parser::make_INVALID_CHAR (const location_type& l)
  {
    return symbol_type (token::INVALID_CHAR, l);
  }

  p_parser::symbol_type
  p_parser::make_IDENTIFIER (const std::string& v, const location_type& l)
  {
    return symbol_type (token::IDENTIFIER, v, l);
  }

  p_parser::symbol_type
  p_parser::make_INUMBER (const int& v, const location_type& l)
  {
    return symbol_type (token::INUMBER, v, l);
  }

  p_parser::symbol_type
  p_parser::make_FPNUMBER (const double& v, const location_type& l)
  {
    return symbol_type (token::FPNUMBER, v, l);
  }

  p_parser::symbol_type
  p_parser::make_UMINUS (const location_type& l)
  {
    return symbol_type (token::UMINUS, l);
  }



  // by_state.
  inline
  p_parser::by_state::by_state ()
    : state (empty_state)
  {}

  inline
  p_parser::by_state::by_state (const by_state& other)
    : state (other.state)
  {}

  inline
  void
  p_parser::by_state::clear ()
  {
    state = empty_state;
  }

  inline
  void
  p_parser::by_state::move (by_state& that)
  {
    state = that.state;
    that.clear ();
  }

  inline
  p_parser::by_state::by_state (state_type s)
    : state (s)
  {}

  inline
  p_parser::symbol_number_type
  p_parser::by_state::type_get () const
  {
    if (state == empty_state)
      return empty_symbol;
    else
      return yystos_[state];
  }

  inline
  p_parser::stack_symbol_type::stack_symbol_type ()
  {}


  inline
  p_parser::stack_symbol_type::stack_symbol_type (state_type s, symbol_type& that)
    : super_type (s, that.location)
  {
      switch (that.type_get ())
    {
      case 107: // module_command
        value.move< ast::Command > (that.value);
        break;

      case 133: // control_specification
        value.move< ast::ControlSpec > (that.value);
        break;

      case 103: // low_value
      case 104: // high_value
      case 105: // init_value
      case 109: // command_guard
      case 113: // next_state_expression
      case 114: // primed_state_expression
      case 118: // expression
      case 128: // reward_guard
      case 129: // reward_value
        value.move< ast::Expression > (that.value);
        break;

      case 116: // formula_definition
        value.move< ast::Formula > (that.value);
        break;

      case 122: // replacement
        value.move< ast::NameReplacement > (that.value);
        break;

      case 124: // rewards_definition
        value.move< ast::RewardSection > (that.value);
        break;

      case 127: // reward_specification
        value.move< ast::RewardSpec > (that.value);
        break;

      case 112: // prob_transition
        value.move< ast::Transition > (that.value);
        break;

      case 101: // variable_definition
      case 102: // global_variable_definition
        value.move< ast::Variable > (that.value);
        break;

      case 71: // "floating-point number"
        value.move< double > (that.value);
        break;

      case 70: // "integer number"
        value.move< int > (that.value);
        break;

      case 106: // module_variables_and_commands
        value.move< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > (that.value);
        break;

      case 69: // "identifier"
      case 100: // module_name
      case 108: // action_name
      case 117: // formula_name
      case 125: // optional_reward_name
      case 131: // player_name
      case 134: // variable_name
        value.move< std::string > (that.value);
        break;

      case 132: // player_control_list
        value.move< std::vector<ast::ControlSpec> > (that.value);
        break;

      case 119: // expression_list
        value.move< std::vector<ast::Expression> > (that.value);
        break;

      case 121: // replacement_list
        value.move< std::vector<ast::NameReplacement> > (that.value);
        break;

      case 130: // player_definition
        value.move< std::vector<ast::Player> > (that.value);
        break;

      case 126: // reward_list
        value.move< std::vector<ast::RewardSpec>  > (that.value);
        break;

      case 110: // transitions
      case 111: // transition_list
        value.move< std::vector<ast::Transition>  > (that.value);
        break;

      default:
        break;
    }

    // that is emptied.
    that.type = empty_symbol;
  }

  inline
  p_parser::stack_symbol_type&
  p_parser::stack_symbol_type::operator= (const stack_symbol_type& that)
  {
    state = that.state;
      switch (that.type_get ())
    {
      case 107: // module_command
        value.copy< ast::Command > (that.value);
        break;

      case 133: // control_specification
        value.copy< ast::ControlSpec > (that.value);
        break;

      case 103: // low_value
      case 104: // high_value
      case 105: // init_value
      case 109: // command_guard
      case 113: // next_state_expression
      case 114: // primed_state_expression
      case 118: // expression
      case 128: // reward_guard
      case 129: // reward_value
        value.copy< ast::Expression > (that.value);
        break;

      case 116: // formula_definition
        value.copy< ast::Formula > (that.value);
        break;

      case 122: // replacement
        value.copy< ast::NameReplacement > (that.value);
        break;

      case 124: // rewards_definition
        value.copy< ast::RewardSection > (that.value);
        break;

      case 127: // reward_specification
        value.copy< ast::RewardSpec > (that.value);
        break;

      case 112: // prob_transition
        value.copy< ast::Transition > (that.value);
        break;

      case 101: // variable_definition
      case 102: // global_variable_definition
        value.copy< ast::Variable > (that.value);
        break;

      case 71: // "floating-point number"
        value.copy< double > (that.value);
        break;

      case 70: // "integer number"
        value.copy< int > (that.value);
        break;

      case 106: // module_variables_and_commands
        value.copy< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > (that.value);
        break;

      case 69: // "identifier"
      case 100: // module_name
      case 108: // action_name
      case 117: // formula_name
      case 125: // optional_reward_name
      case 131: // player_name
      case 134: // variable_name
        value.copy< std::string > (that.value);
        break;

      case 132: // player_control_list
        value.copy< std::vector<ast::ControlSpec> > (that.value);
        break;

      case 119: // expression_list
        value.copy< std::vector<ast::Expression> > (that.value);
        break;

      case 121: // replacement_list
        value.copy< std::vector<ast::NameReplacement> > (that.value);
        break;

      case 130: // player_definition
        value.copy< std::vector<ast::Player> > (that.value);
        break;

      case 126: // reward_list
        value.copy< std::vector<ast::RewardSpec>  > (that.value);
        break;

      case 110: // transitions
      case 111: // transition_list
        value.copy< std::vector<ast::Transition>  > (that.value);
        break;

      default:
        break;
    }

    location = that.location;
    return *this;
  }


  template <typename Base>
  inline
  void
  p_parser::yy_destroy_ (const char* yymsg, basic_symbol<Base>& yysym) const
  {
    if (yymsg)
      YY_SYMBOL_PRINT (yymsg, yysym);
  }

#if YYDEBUG
  template <typename Base>
  void
  p_parser::yy_print_ (std::ostream& yyo,
                                     const basic_symbol<Base>& yysym) const
  {
    std::ostream& yyoutput = yyo;
    YYUSE (yyoutput);
    symbol_number_type yytype = yysym.type_get ();
    // Avoid a (spurious) G++ 4.8 warning about "array subscript is
    // below array bounds".
    if (yysym.empty ())
      std::abort ();
    yyo << (yytype < yyntokens_ ? "token" : "nterm")
        << ' ' << yytname_[yytype] << " ("
        << yysym.location << ": ";
    switch (yytype)
    {
            case 69: // "identifier"

#line 185 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< std::string > (); }
#line 1572 "Pparser/Pparser.cc" // lalr1.cc:636
        break;

      case 70: // "integer number"

#line 186 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< int > (); }
#line 1579 "Pparser/Pparser.cc" // lalr1.cc:636
        break;

      case 71: // "floating-point number"

#line 187 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< double > (); }
#line 1586 "Pparser/Pparser.cc" // lalr1.cc:636
        break;

      case 100: // module_name

#line 185 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< std::string > (); }
#line 1593 "Pparser/Pparser.cc" // lalr1.cc:636
        break;

      case 108: // action_name

#line 185 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< std::string > (); }
#line 1600 "Pparser/Pparser.cc" // lalr1.cc:636
        break;

      case 117: // formula_name

#line 185 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< std::string > (); }
#line 1607 "Pparser/Pparser.cc" // lalr1.cc:636
        break;

      case 125: // optional_reward_name

#line 185 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< std::string > (); }
#line 1614 "Pparser/Pparser.cc" // lalr1.cc:636
        break;

      case 131: // player_name

#line 185 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< std::string > (); }
#line 1621 "Pparser/Pparser.cc" // lalr1.cc:636
        break;

      case 134: // variable_name

#line 185 "Pparser/Pparser.yy" // lalr1.cc:636
        { debug_stream() << yysym.value.template as< std::string > (); }
#line 1628 "Pparser/Pparser.cc" // lalr1.cc:636
        break;


      default:
        break;
    }
    yyo << ')';
  }
#endif

  inline
  void
  p_parser::yypush_ (const char* m, state_type s, symbol_type& sym)
  {
    stack_symbol_type t (s, sym);
    yypush_ (m, t);
  }

  inline
  void
  p_parser::yypush_ (const char* m, stack_symbol_type& s)
  {
    if (m)
      YY_SYMBOL_PRINT (m, s);
    yystack_.push (s);
  }

  inline
  void
  p_parser::yypop_ (unsigned int n)
  {
    yystack_.pop (n);
  }

#if YYDEBUG
  std::ostream&
  p_parser::debug_stream () const
  {
    return *yycdebug_;
  }

  void
  p_parser::set_debug_stream (std::ostream& o)
  {
    yycdebug_ = &o;
  }


  p_parser::debug_level_type
  p_parser::debug_level () const
  {
    return yydebug_;
  }

  void
  p_parser::set_debug_level (debug_level_type l)
  {
    yydebug_ = l;
  }
#endif // YYDEBUG

  inline p_parser::state_type
  p_parser::yy_lr_goto_state_ (state_type yystate, int yysym)
  {
    int yyr = yypgoto_[yysym - yyntokens_] + yystate;
    if (0 <= yyr && yyr <= yylast_ && yycheck_[yyr] == yystate)
      return yytable_[yyr];
    else
      return yydefgoto_[yysym - yyntokens_];
  }

  inline bool
  p_parser::yy_pact_value_is_default_ (int yyvalue)
  {
    return yyvalue == yypact_ninf_;
  }

  inline bool
  p_parser::yy_table_value_is_error_ (int yyvalue)
  {
    return yyvalue == yytable_ninf_;
  }

  int
  p_parser::parse ()
  {
    // State.
    int yyn;
    /// Length of the RHS of the rule being reduced.
    int yylen = 0;

    // Error handling.
    int yynerrs_ = 0;
    int yyerrstatus_ = 0;

    /// The lookahead symbol.
    symbol_type yyla;

    /// The locations where the error started and ended.
    stack_symbol_type yyerror_range[3];

    /// The return value of parse ().
    int yyresult;

    // FIXME: This shoud be completely indented.  It is not yet to
    // avoid gratuitous conflicts when merging into the master branch.
    try
      {
    YYCDEBUG << "Starting parse" << std::endl;


    // User initialization code.
    #line 68 "Pparser/Pparser.yy" // lalr1.cc:741
{
  // Initialize the initial location.
  yyla.location.begin.filename = yyla.location.end.filename = &driver.file;
}

#line 1747 "Pparser/Pparser.cc" // lalr1.cc:741

    /* Initialize the stack.  The initial state will be set in
       yynewstate, since the latter expects the semantical and the
       location values to have been already stored, initialize these
       stacks with a primary value.  */
    yystack_.clear ();
    yypush_ (YY_NULLPTR, 0, yyla);

    // A new symbol was pushed on the stack.
  yynewstate:
    YYCDEBUG << "Entering state " << yystack_[0].state << std::endl;

    // Accept?
    if (yystack_[0].state == yyfinal_)
      goto yyacceptlab;

    goto yybackup;

    // Backup.
  yybackup:

    // Try to take a decision without lookahead.
    yyn = yypact_[yystack_[0].state];
    if (yy_pact_value_is_default_ (yyn))
      goto yydefault;

    // Read a lookahead token.
    if (yyla.empty ())
      {
        YYCDEBUG << "Reading a token: ";
        try
          {
            yyla.type = yytranslate_ (yylex (&yyla.value, &yyla.location, driver));
          }
        catch (const syntax_error& yyexc)
          {
            error (yyexc);
            goto yyerrlab1;
          }
      }
    YY_SYMBOL_PRINT ("Next token is", yyla);

    /* If the proper action on seeing token YYLA.TYPE is to reduce or
       to detect an error, take that action.  */
    yyn += yyla.type_get ();
    if (yyn < 0 || yylast_ < yyn || yycheck_[yyn] != yyla.type_get ())
      goto yydefault;

    // Reduce or error.
    yyn = yytable_[yyn];
    if (yyn <= 0)
      {
        if (yy_table_value_is_error_ (yyn))
          goto yyerrlab;
        yyn = -yyn;
        goto yyreduce;
      }

    // Count tokens shifted since error; after three, turn off error status.
    if (yyerrstatus_)
      --yyerrstatus_;

    // Shift the lookahead token.
    yypush_ ("Shifting", yyn, yyla);
    goto yynewstate;

  /*-----------------------------------------------------------.
  | yydefault -- do the default action for the current state.  |
  `-----------------------------------------------------------*/
  yydefault:
    yyn = yydefact_[yystack_[0].state];
    if (yyn == 0)
      goto yyerrlab;
    goto yyreduce;

  /*-----------------------------.
  | yyreduce -- Do a reduction.  |
  `-----------------------------*/
  yyreduce:
    yylen = yyr2_[yyn];
    {
      stack_symbol_type yylhs;
      yylhs.state = yy_lr_goto_state_(yystack_[yylen].state, yyr1_[yyn]);
      /* Variants are always initialized to an empty instance of the
         correct type. The default '$$ = $1' action is NOT applied
         when using variants.  */
        switch (yyr1_[yyn])
    {
      case 107: // module_command
        yylhs.value.build< ast::Command > ();
        break;

      case 133: // control_specification
        yylhs.value.build< ast::ControlSpec > ();
        break;

      case 103: // low_value
      case 104: // high_value
      case 105: // init_value
      case 109: // command_guard
      case 113: // next_state_expression
      case 114: // primed_state_expression
      case 118: // expression
      case 128: // reward_guard
      case 129: // reward_value
        yylhs.value.build< ast::Expression > ();
        break;

      case 116: // formula_definition
        yylhs.value.build< ast::Formula > ();
        break;

      case 122: // replacement
        yylhs.value.build< ast::NameReplacement > ();
        break;

      case 124: // rewards_definition
        yylhs.value.build< ast::RewardSection > ();
        break;

      case 127: // reward_specification
        yylhs.value.build< ast::RewardSpec > ();
        break;

      case 112: // prob_transition
        yylhs.value.build< ast::Transition > ();
        break;

      case 101: // variable_definition
      case 102: // global_variable_definition
        yylhs.value.build< ast::Variable > ();
        break;

      case 71: // "floating-point number"
        yylhs.value.build< double > ();
        break;

      case 70: // "integer number"
        yylhs.value.build< int > ();
        break;

      case 106: // module_variables_and_commands
        yylhs.value.build< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > ();
        break;

      case 69: // "identifier"
      case 100: // module_name
      case 108: // action_name
      case 117: // formula_name
      case 125: // optional_reward_name
      case 131: // player_name
      case 134: // variable_name
        yylhs.value.build< std::string > ();
        break;

      case 132: // player_control_list
        yylhs.value.build< std::vector<ast::ControlSpec> > ();
        break;

      case 119: // expression_list
        yylhs.value.build< std::vector<ast::Expression> > ();
        break;

      case 121: // replacement_list
        yylhs.value.build< std::vector<ast::NameReplacement> > ();
        break;

      case 130: // player_definition
        yylhs.value.build< std::vector<ast::Player> > ();
        break;

      case 126: // reward_list
        yylhs.value.build< std::vector<ast::RewardSpec>  > ();
        break;

      case 110: // transitions
      case 111: // transition_list
        yylhs.value.build< std::vector<ast::Transition>  > ();
        break;

      default:
        break;
    }


      // Compute the default @$.
      {
        slice<stack_symbol_type, stack_type> slice (yystack_, yylen);
        YYLLOC_DEFAULT (yylhs.location, slice, yylen);
      }

      // Perform the reduction.
      YY_REDUCE_PRINT (yyn);
      try
        {
          switch (yyn)
            {
  case 3:
#line 207 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.modelType = ast::MDP; }
#line 1948 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 4:
#line 209 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.modelType = ast::SMG; }
#line 1954 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 5:
#line 210 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.modelType = ast::BDP; }
#line 1960 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 16:
#line 231 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.modules.emplace_back(ast::Module{yystack_[2].value.as< std::string > (), yystack_[1].value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > ().first, yystack_[1].value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > ().second}); }
#line 1966 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 17:
#line 235 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< std::string > (), yystack_[0].value.as< std::string > ()); }
#line 1972 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 18:
#line 240 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Variable > () = {ast::NUMBER, yystack_[9].value.as< std::string > (), yystack_[6].value.as< ast::Expression > (), yystack_[4].value.as< ast::Expression > (), yystack_[1].value.as< ast::Expression > ()}; }
#line 1978 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 19:
#line 242 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Variable > () = {ast::NUMBER, yystack_[7].value.as< std::string > (), yystack_[4].value.as< ast::Expression > (), yystack_[2].value.as< ast::Expression > (), yystack_[4].value.as< ast::Expression > ()}; }
#line 1984 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 20:
#line 244 "Pparser/Pparser.yy" // lalr1.cc:859
    { yystack_[1].value.as< ast::Expression > ().type = ast::BOOL;
    yylhs.value.as< ast::Variable > () = {ast::BOOL, yystack_[5].value.as< std::string > (),
          ast::Expression{ast::BOOL, ast::FALSE, {}, "", 0, 0.0, astloc(yylhs.location)},
          ast::Expression{ast::BOOL, ast::TRUE, {}, "", 1, 1.0, astloc(yylhs.location)},
          yystack_[1].value.as< ast::Expression > ()}; }
#line 1994 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 21:
#line 250 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Variable > () = {ast::BOOL, yystack_[3].value.as< std::string > (),
          ast::Expression{ast::BOOL, ast::FALSE, {}, "", 0, 0.0, astloc(yylhs.location)},
          ast::Expression{ast::BOOL, ast::TRUE, {}, "", 1, 1.0, astloc(yylhs.location)},
          ast::Expression{ast::BOOL, ast::FALSE, {}, "", 0, 0.0, astloc(yylhs.location)}}; }
#line 2003 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 22:
#line 258 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.globals.emplace_back(yystack_[0].value.as< ast::Variable > ()); }
#line 2009 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 23:
#line 262 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()); }
#line 2015 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 24:
#line 266 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()); }
#line 2021 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 25:
#line 270 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()); }
#line 2027 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 26:
#line 274 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > () = {{}, {yystack_[0].value.as< ast::Command > ()}}; }
#line 2033 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 27:
#line 275 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > () = {{yystack_[0].value.as< ast::Variable > ()}, {}}; }
#line 2039 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 28:
#line 277 "Pparser/Pparser.yy" // lalr1.cc:859
    { (yystack_[1].value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > ()).second.emplace_back(yystack_[0].value.as< ast::Command > ()); std::swap(yylhs.value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > (), yystack_[1].value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > ()); }
#line 2045 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 29:
#line 279 "Pparser/Pparser.yy" // lalr1.cc:859
    { (yystack_[1].value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > ()).first.emplace_back(yystack_[0].value.as< ast::Variable > ()); std::swap(yylhs.value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > (), yystack_[1].value.as< std::pair<std::vector<ast::Variable>,std::vector<ast::Command>> > ()); }
#line 2051 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 30:
#line 284 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Command > () = {yystack_[5].value.as< std::string > (), yystack_[3].value.as< ast::Expression > (), yystack_[1].value.as< std::vector<ast::Transition>  > ()}; }
#line 2057 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 31:
#line 288 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::string > () = std::string(); }
#line 2063 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 32:
#line 289 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< std::string > (), yystack_[0].value.as< std::string > ()); }
#line 2069 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 33:
#line 293 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()); }
#line 2075 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 34:
#line 298 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::vector<ast::Transition>  > () = {{{ast::NUMBER, ast::NUMERAL, {}, "", 1, 1.0, astloc(yylhs.location)}, yystack_[0].value.as< ast::Expression > ()}}; }
#line 2081 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 35:
#line 299 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< std::vector<ast::Transition>  > (), yystack_[0].value.as< std::vector<ast::Transition>  > ()); }
#line 2087 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 36:
#line 303 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::vector<ast::Transition>  > () = {yystack_[0].value.as< ast::Transition > ()}; }
#line 2093 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 37:
#line 304 "Pparser/Pparser.yy" // lalr1.cc:859
    { (yystack_[2].value.as< std::vector<ast::Transition>  > ()).emplace_back(yystack_[0].value.as< ast::Transition > ()); std::swap(yylhs.value.as< std::vector<ast::Transition>  > (), yystack_[2].value.as< std::vector<ast::Transition>  > ()); }
#line 2099 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 38:
#line 309 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Transition > () = {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}; }
#line 2105 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 39:
#line 314 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::TRUE, {}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2111 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 40:
#line 316 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()); }
#line 2117 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 41:
#line 321 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::EQUAL, {yystack_[1].value.as< ast::Expression > ()}, yystack_[4].value.as< std::string > (), 0, 0.0, astloc(yylhs.location)}; }
#line 2123 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 42:
#line 323 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::AND, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2129 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 43:
#line 328 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.constants.emplace_back(ast::Constant{ast::NUMBER, yystack_[3].value.as< std::string > (), yystack_[1].value.as< ast::Expression > ()}); }
#line 2135 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 44:
#line 330 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.constants.emplace_back(ast::Constant{ast::DOUBLE, yystack_[3].value.as< std::string > (), yystack_[1].value.as< ast::Expression > ()}); }
#line 2141 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 45:
#line 332 "Pparser/Pparser.yy" // lalr1.cc:859
    { yystack_[1].value.as< ast::Expression > ().type = ast::BOOL;
  driver.constants.emplace_back(ast::Constant{ast::BOOL, yystack_[3].value.as< std::string > (), yystack_[1].value.as< ast::Expression > ()}); }
#line 2148 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 46:
#line 338 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.formulae.emplace_back(ast::Formula{yystack_[3].value.as< std::string > (), yystack_[1].value.as< ast::Expression > ()}); }
#line 2154 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 47:
#line 342 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< std::string > (), yystack_[0].value.as< std::string > ()); }
#line 2160 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 48:
#line 347 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = yystack_[1].value.as< ast::Expression > (); }
#line 2166 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 49:
#line 349 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::NUMERAL, {}, "", yystack_[0].value.as< int > (), (double) yystack_[0].value.as< int > (), astloc(yylhs.location)}; }
#line 2172 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 50:
#line 351 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::DOUBLE, ast::NUMERAL, {}, "", 0, yystack_[0].value.as< double > (), astloc(yylhs.location)}; }
#line 2178 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 51:
#line 353 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::TRUE, {}, "", 1, 1.0, astloc(yylhs.location)}; }
#line 2184 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 52:
#line 355 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::FALSE, {}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2190 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 53:
#line 357 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::IDENTIFIER, {}, yystack_[0].value.as< std::string > (), -1, -1.0, astloc(yylhs.location)}; }
#line 2196 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 54:
#line 359 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::MOD, {yystack_[3].value.as< ast::Expression > (), yystack_[1].value.as< ast::Expression > ()}, "mod", 0, 0.0, astloc(yylhs.location)}; }
#line 2202 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 55:
#line 361 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::POW, {yystack_[3].value.as< ast::Expression > (), yystack_[1].value.as< ast::Expression > ()}, "pow", 0, 0.0, astloc(yylhs.location)}; }
#line 2208 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 56:
#line 363 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::MIN, yystack_[1].value.as< std::vector<ast::Expression> > (), "min", 0, 0.0, astloc(yylhs.location)}; }
#line 2214 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 57:
#line 365 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::MAX, yystack_[1].value.as< std::vector<ast::Expression> > (), "max", 0, 0.0, astloc(yylhs.location)}; }
#line 2220 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 58:
#line 367 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::LOG, {yystack_[3].value.as< ast::Expression > (), yystack_[1].value.as< ast::Expression > ()}, "log", 0, 0.0, astloc(yylhs.location)}; }
#line 2226 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 59:
#line 369 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::CEIL, {yystack_[1].value.as< ast::Expression > ()}, "ceil", 0, 0.0, astloc(yylhs.location)}; }
#line 2232 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 60:
#line 371 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::FLOOR, {yystack_[1].value.as< ast::Expression > ()}, "floor", 0, 0.0, astloc(yylhs.location)}; }
#line 2238 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 61:
#line 373 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::PLUS, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2244 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 62:
#line 375 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::MINUS, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2250 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 63:
#line 377 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::TIMES, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2256 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 64:
#line 379 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::DIVIDE, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2262 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 65:
#line 381 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::UNARYMINUS, {yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2268 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 66:
#line 383 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::NUMBER, ast::ITE, {yystack_[4].value.as< ast::Expression > (), yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2274 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 67:
#line 385 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::OR, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2280 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 68:
#line 387 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::AND, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2286 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 69:
#line 389 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::XOR, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2292 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 70:
#line 391 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::IFF, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2298 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 71:
#line 393 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::IMPLIES, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2304 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 72:
#line 395 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::NOT, {yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2310 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 73:
#line 397 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::EQUAL, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2316 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 74:
#line 399 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::NOTEQ, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2322 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 75:
#line 401 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::LT, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2328 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 76:
#line 403 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::LE, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2334 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 77:
#line 405 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::GT, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2340 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 78:
#line 407 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::Expression > () = {ast::BOOL, ast::GE, {yystack_[2].value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()}, "", 0, 0.0, astloc(yylhs.location)}; }
#line 2346 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 79:
#line 411 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::vector<ast::Expression> > () = {yystack_[0].value.as< ast::Expression > ()}; }
#line 2352 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 80:
#line 412 "Pparser/Pparser.yy" // lalr1.cc:859
    { (yystack_[2].value.as< std::vector<ast::Expression> > ()).emplace_back(yystack_[0].value.as< ast::Expression > ()); std::swap(yylhs.value.as< std::vector<ast::Expression> > (), yystack_[2].value.as< std::vector<ast::Expression> > ()); }
#line 2358 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 81:
#line 417 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.renamings.emplace_back(ast::ModuleRenaming{yystack_[6].value.as< std::string > (), yystack_[4].value.as< std::string > (), yystack_[2].value.as< std::vector<ast::NameReplacement> > (), astloc(yylhs.location)}); }
#line 2364 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 82:
#line 421 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::vector<ast::NameReplacement> > () = {yystack_[0].value.as< ast::NameReplacement > ()}; }
#line 2370 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 83:
#line 422 "Pparser/Pparser.yy" // lalr1.cc:859
    { (yystack_[2].value.as< std::vector<ast::NameReplacement> > ()).emplace_back(yystack_[0].value.as< ast::NameReplacement > ()); std::swap(yylhs.value.as< std::vector<ast::NameReplacement> > (), yystack_[2].value.as< std::vector<ast::NameReplacement> > ()); }
#line 2376 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 84:
#line 426 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::NameReplacement > () = {yystack_[2].value.as< std::string > (),yystack_[0].value.as< std::string > ()}; }
#line 2382 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 85:
#line 431 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.labels.emplace_back(ast::Label{yystack_[4].value.as< std::string > (), yystack_[1].value.as< ast::Expression > ()}); }
#line 2388 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 86:
#line 436 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.rewards.emplace_back(ast::RewardSection{yystack_[2].value.as< std::string > (), yystack_[1].value.as< std::vector<ast::RewardSpec>  > ()}); }
#line 2394 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 87:
#line 440 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::string > () = std::string(""); }
#line 2400 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 88:
#line 441 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< std::string > (), yystack_[1].value.as< std::string > ()); }
#line 2406 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 89:
#line 445 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::vector<ast::RewardSpec>  > () = {}; }
#line 2412 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 90:
#line 447 "Pparser/Pparser.yy" // lalr1.cc:859
    { (yystack_[1].value.as< std::vector<ast::RewardSpec>  > ()).emplace_back(yystack_[0].value.as< ast::RewardSpec > ()); std::swap(yylhs.value.as< std::vector<ast::RewardSpec>  > (), yystack_[1].value.as< std::vector<ast::RewardSpec>  > ()); }
#line 2418 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 91:
#line 452 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::RewardSpec > () = {ast::STATE, "", yystack_[3].value.as< ast::Expression > (), yystack_[1].value.as< ast::Expression > ()}; }
#line 2424 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 92:
#line 454 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::RewardSpec > () = {ast::ACTION, yystack_[5].value.as< std::string > (), yystack_[3].value.as< ast::Expression > (), yystack_[1].value.as< ast::Expression > ()}; }
#line 2430 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 93:
#line 458 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()); }
#line 2436 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 94:
#line 462 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< ast::Expression > (), yystack_[0].value.as< ast::Expression > ()); }
#line 2442 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 95:
#line 467 "Pparser/Pparser.yy" // lalr1.cc:859
    { driver.players.emplace_back(ast::Player{yystack_[2].value.as< std::string > (), yystack_[1].value.as< std::vector<ast::ControlSpec> > (), astloc(yylhs.location)}); }
#line 2448 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 96:
#line 471 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< std::string > (), yystack_[0].value.as< std::string > ()); }
#line 2454 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 97:
#line 475 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< std::vector<ast::ControlSpec> > () = {yystack_[0].value.as< ast::ControlSpec > ()}; }
#line 2460 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 98:
#line 477 "Pparser/Pparser.yy" // lalr1.cc:859
    { (yystack_[2].value.as< std::vector<ast::ControlSpec> > ()).emplace_back(yystack_[0].value.as< ast::ControlSpec > ()); std::swap(yylhs.value.as< std::vector<ast::ControlSpec> > (), yystack_[2].value.as< std::vector<ast::ControlSpec> > ()); }
#line 2466 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 99:
#line 481 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::ControlSpec > () = {ast::PMODULE, yystack_[0].value.as< std::string > ()}; }
#line 2472 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 100:
#line 482 "Pparser/Pparser.yy" // lalr1.cc:859
    { yylhs.value.as< ast::ControlSpec > () = {ast::PACTION, yystack_[1].value.as< std::string > ()}; }
#line 2478 "Pparser/Pparser.cc" // lalr1.cc:859
    break;

  case 101:
#line 486 "Pparser/Pparser.yy" // lalr1.cc:859
    { std::swap(yylhs.value.as< std::string > (), yystack_[0].value.as< std::string > ()); }
#line 2484 "Pparser/Pparser.cc" // lalr1.cc:859
    break;


#line 2488 "Pparser/Pparser.cc" // lalr1.cc:859
            default:
              break;
            }
        }
      catch (const syntax_error& yyexc)
        {
          error (yyexc);
          YYERROR;
        }
      YY_SYMBOL_PRINT ("-> $$ =", yylhs);
      yypop_ (yylen);
      yylen = 0;
      YY_STACK_PRINT ();

      // Shift the result of the reduction.
      yypush_ (YY_NULLPTR, yylhs);
    }
    goto yynewstate;

  /*--------------------------------------.
  | yyerrlab -- here on detecting error.  |
  `--------------------------------------*/
  yyerrlab:
    // If not already recovering from an error, report this error.
    if (!yyerrstatus_)
      {
        ++yynerrs_;
        error (yyla.location, yysyntax_error_ (yystack_[0].state, yyla));
      }


    yyerror_range[1].location = yyla.location;
    if (yyerrstatus_ == 3)
      {
        /* If just tried and failed to reuse lookahead token after an
           error, discard it.  */

        // Return failure if at end of input.
        if (yyla.type_get () == yyeof_)
          YYABORT;
        else if (!yyla.empty ())
          {
            yy_destroy_ ("Error: discarding", yyla);
            yyla.clear ();
          }
      }

    // Else will try to reuse lookahead token after shifting the error token.
    goto yyerrlab1;


  /*---------------------------------------------------.
  | yyerrorlab -- error raised explicitly by YYERROR.  |
  `---------------------------------------------------*/
  yyerrorlab:

    /* Pacify compilers like GCC when the user code never invokes
       YYERROR and the label yyerrorlab therefore never appears in user
       code.  */
    if (false)
      goto yyerrorlab;
    yyerror_range[1].location = yystack_[yylen - 1].location;
    /* Do not reclaim the symbols of the rule whose action triggered
       this YYERROR.  */
    yypop_ (yylen);
    yylen = 0;
    goto yyerrlab1;

  /*-------------------------------------------------------------.
  | yyerrlab1 -- common code for both syntax error and YYERROR.  |
  `-------------------------------------------------------------*/
  yyerrlab1:
    yyerrstatus_ = 3;   // Each real token shifted decrements this.
    {
      stack_symbol_type error_token;
      for (;;)
        {
          yyn = yypact_[yystack_[0].state];
          if (!yy_pact_value_is_default_ (yyn))
            {
              yyn += yyterror_;
              if (0 <= yyn && yyn <= yylast_ && yycheck_[yyn] == yyterror_)
                {
                  yyn = yytable_[yyn];
                  if (0 < yyn)
                    break;
                }
            }

          // Pop the current state because it cannot handle the error token.
          if (yystack_.size () == 1)
            YYABORT;

          yyerror_range[1].location = yystack_[0].location;
          yy_destroy_ ("Error: popping", yystack_[0]);
          yypop_ ();
          YY_STACK_PRINT ();
        }

      yyerror_range[2].location = yyla.location;
      YYLLOC_DEFAULT (error_token.location, yyerror_range, 2);

      // Shift the error token.
      error_token.state = yyn;
      yypush_ ("Shifting", error_token);
    }
    goto yynewstate;

    // Accept.
  yyacceptlab:
    yyresult = 0;
    goto yyreturn;

    // Abort.
  yyabortlab:
    yyresult = 1;
    goto yyreturn;

  yyreturn:
    if (!yyla.empty ())
      yy_destroy_ ("Cleanup: discarding lookahead", yyla);

    /* Do not reclaim the symbols of the rule whose action triggered
       this YYABORT or YYACCEPT.  */
    yypop_ (yylen);
    while (1 < yystack_.size ())
      {
        yy_destroy_ ("Cleanup: popping", yystack_[0]);
        yypop_ ();
      }

    return yyresult;
  }
    catch (...)
      {
        YYCDEBUG << "Exception caught: cleaning lookahead and stack"
                 << std::endl;
        // Do not try to display the values of the reclaimed symbols,
        // as their printer might throw an exception.
        if (!yyla.empty ())
          yy_destroy_ (YY_NULLPTR, yyla);

        while (1 < yystack_.size ())
          {
            yy_destroy_ (YY_NULLPTR, yystack_[0]);
            yypop_ ();
          }
        throw;
      }
  }

  void
  p_parser::error (const syntax_error& yyexc)
  {
    error (yyexc.location, yyexc.what());
  }

  // Generate an error message.
  std::string
  p_parser::yysyntax_error_ (state_type yystate, const symbol_type& yyla) const
  {
    // Number of reported tokens (one for the "unexpected", one per
    // "expected").
    size_t yycount = 0;
    // Its maximum.
    enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
    // Arguments of yyformat.
    char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];

    /* There are many possibilities here to consider:
       - If this state is a consistent state with a default action, then
         the only way this function was invoked is if the default action
         is an error action.  In that case, don't check for expected
         tokens because there are none.
       - The only way there can be no lookahead present (in yyla) is
         if this state is a consistent state with a default action.
         Thus, detecting the absence of a lookahead is sufficient to
         determine that there is no unexpected or expected token to
         report.  In that case, just report a simple "syntax error".
       - Don't assume there isn't a lookahead just because this state is
         a consistent state with a default action.  There might have
         been a previous inconsistent state, consistent state with a
         non-default action, or user semantic action that manipulated
         yyla.  (However, yyla is currently not documented for users.)
       - Of course, the expected token list depends on states to have
         correct lookahead information, and it depends on the parser not
         to perform extra reductions after fetching a lookahead from the
         scanner and before detecting a syntax error.  Thus, state
         merging (from LALR or IELR) and default reductions corrupt the
         expected token list.  However, the list is correct for
         canonical LR with one exception: it will still contain any
         token that will not be accepted due to an error action in a
         later state.
    */
    if (!yyla.empty ())
      {
        int yytoken = yyla.type_get ();
        yyarg[yycount++] = yytname_[yytoken];
        int yyn = yypact_[yystate];
        if (!yy_pact_value_is_default_ (yyn))
          {
            /* Start YYX at -YYN if negative to avoid negative indexes in
               YYCHECK.  In other words, skip the first -YYN actions for
               this state because they are default actions.  */
            int yyxbegin = yyn < 0 ? -yyn : 0;
            // Stay within bounds of both yycheck and yytname.
            int yychecklim = yylast_ - yyn + 1;
            int yyxend = yychecklim < yyntokens_ ? yychecklim : yyntokens_;
            for (int yyx = yyxbegin; yyx < yyxend; ++yyx)
              if (yycheck_[yyx + yyn] == yyx && yyx != yyterror_
                  && !yy_table_value_is_error_ (yytable_[yyx + yyn]))
                {
                  if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                    {
                      yycount = 1;
                      break;
                    }
                  else
                    yyarg[yycount++] = yytname_[yyx];
                }
          }
      }

    char const* yyformat = YY_NULLPTR;
    switch (yycount)
      {
#define YYCASE_(N, S)                         \
        case N:                               \
          yyformat = S;                       \
        break
        YYCASE_(0, YY_("syntax error"));
        YYCASE_(1, YY_("syntax error, unexpected %s"));
        YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
        YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
        YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
        YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
#undef YYCASE_
      }

    std::string yyres;
    // Argument number.
    size_t yyi = 0;
    for (char const* yyp = yyformat; *yyp; ++yyp)
      if (yyp[0] == '%' && yyp[1] == 's' && yyi < yycount)
        {
          yyres += yytnamerr_ (yyarg[yyi++]);
          ++yyp;
        }
      else
        yyres += *yyp;
    return yyres;
  }


  const short int p_parser::yypact_ninf_ = -195;

  const signed char p_parser::yytable_ninf_ = -102;

  const short int
  p_parser::yypact_[] =
  {
     -14,  -195,  -195,  -195,    17,   721,  -195,    -5,   -20,    14,
     -30,    31,    34,    -7,   721,  -195,  -195,  -195,  -195,  -195,
    -195,  -195,  -195,  -195,    35,    48,    49,  -195,    25,  -195,
    -195,    36,    50,  -195,   -68,  -195,   -58,    51,  -195,  -195,
      47,    53,    54,   646,    -6,    39,    31,    64,  -195,    15,
    -195,    64,  -195,   -10,  -195,    42,   609,   646,   646,   646,
      52,  -195,    55,    56,    58,    65,    66,    67,  -195,  -195,
    -195,  -195,   646,   646,   646,   329,   -27,   646,    60,    57,
    -195,    70,  -195,  -195,  -195,    71,  -195,   -58,  -195,  -195,
      64,   483,  -195,    68,   342,   363,   386,   646,   646,   646,
     646,   646,   646,   646,   564,  -195,   143,   646,   646,   646,
     646,   646,   646,   646,   646,   646,   646,   646,   646,   646,
     646,   646,   646,  -195,   646,  -195,   137,   483,   646,    90,
     646,  -195,  -195,    73,   646,  -195,  -195,  -195,   167,   201,
      16,   483,   -71,   -39,    33,   120,  -195,   587,   506,   506,
     587,   587,   506,   527,   564,   446,   587,   587,   587,   -38,
     -38,  -195,  -195,    74,   483,   646,   425,    86,   -41,  -195,
     160,   483,   646,   483,    77,  -195,  -195,   646,  -195,   646,
    -195,   646,   646,   646,  -195,    79,   483,  -195,    98,   147,
      90,   667,    93,  -195,   224,   483,   248,   282,   564,   -26,
    -195,  -195,  -195,    83,   704,    91,   100,  -195,  -195,   107,
     469,   646,  -195,  -195,  -195,   646,  -195,    92,    94,  -195,
     646,    96,   -63,    95,    99,   109,  -195,    14,  -195,  -195,
    -195,  -195,  -195,   646,   305,  -195
  };

  const unsigned char
  p_parser::yydefact_[] =
  {
       0,     3,     4,     5,     0,     0,     1,     0,     0,     0,
       0,     0,     0,    87,     2,     6,     8,    12,    11,    10,
       9,    13,    14,    15,     0,     0,     0,    47,     0,   101,
      22,     0,     0,    17,     0,    96,     0,     0,    89,     7,
       0,     0,     0,     0,     0,     0,     0,    31,    27,     0,
      26,    31,    99,     0,    97,     0,     0,     0,     0,     0,
       0,    52,     0,     0,     0,     0,     0,     0,    51,    53,
      49,    50,     0,     0,     0,     0,     0,     0,     0,     0,
      32,     0,    16,    29,    28,     0,    95,     0,    88,    86,
      31,    93,    90,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,    72,    65,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,    46,     0,    21,     0,    23,     0,     0,
       0,   100,    98,     0,     0,    45,    44,    43,     0,     0,
       0,    79,     0,     0,     0,     0,    48,    74,    70,    71,
      78,    76,    69,    67,    68,     0,    73,    75,    77,    61,
      62,    63,    64,     0,    25,     0,     0,     0,     0,    82,
       0,    33,     0,    94,     0,    59,    60,     0,    57,     0,
      56,     0,     0,     0,    20,     0,    24,    85,     0,     0,
       0,     0,     0,    91,     0,    80,     0,     0,    66,     0,
      84,    81,    83,    51,     0,     0,    35,    36,    34,    40,
       0,     0,    58,    54,    55,     0,    19,    53,     0,    30,
       0,     0,     0,     0,     0,     0,    37,     0,    42,    39,
      38,    92,    18,     0,     0,    41
  };

  const short int
  p_parser::yypgoto_[] =
  {
    -195,  -195,  -195,  -195,   174,  -195,    -8,     1,  -195,  -195,
    -195,   -24,  -195,   140,   -47,  -195,  -195,  -195,   -25,   -17,
     -15,  -195,  -195,  -195,   -43,   110,  -195,  -195,    23,  -195,
    -195,  -195,  -195,  -195,    46,     3,  -195,  -195,  -195,   141,
    -194
  };

  const short int
  p_parser::yydefgoto_[] =
  {
      -1,     4,     5,    14,    15,    16,    52,    30,    17,   126,
     185,   163,    49,    50,    81,   170,   205,   206,   207,   208,
     209,    18,    19,    28,    91,   142,    20,   168,   169,    21,
      22,    38,    56,    92,    93,   174,    23,    36,    53,    54,
      31
  };

  const short int
  p_parser::yytable_[] =
  {
      75,    29,   229,    34,    85,    76,    24,   124,   215,    46,
     218,    33,    25,    86,    94,    95,    96,     6,    47,   107,
     178,   179,   108,   109,   110,   111,   227,     1,    51,   104,
     105,   106,    26,   218,   127,    48,   107,    82,    79,   108,
     109,   110,   111,   133,   121,   122,   189,     2,     3,    27,
      83,   190,   180,   179,   138,   139,   140,   141,   141,   144,
     145,   125,   216,    32,   147,   148,   149,   150,   151,   152,
     153,   154,   155,   156,   157,   158,   159,   160,   161,   162,
      77,   164,    87,    29,    29,   166,    37,   171,   112,   113,
     114,   173,   115,   116,   117,   118,   119,   120,   121,   122,
      33,    47,    43,    35,    40,   112,   113,   114,   177,   115,
     116,   117,   118,   119,   120,   121,   122,    41,    42,    45,
      55,    44,   186,   107,    57,   181,   108,   109,   110,   111,
      58,    59,    78,    80,   194,    88,   195,   128,   196,   197,
     198,    97,   165,   129,    98,    99,   107,   100,   210,   108,
     109,   110,   111,   134,   101,   102,   103,   130,   131,   167,
     172,   106,   184,   188,   191,   193,   199,   200,   173,   201,
     107,   -39,   164,   108,   109,   110,   111,   210,   211,   219,
     220,   221,  -101,   231,   225,   227,   233,   232,    39,    84,
     234,   224,   112,   113,   114,   226,   115,   116,   117,   118,
     119,   120,   121,   122,   107,   230,   228,   108,   109,   110,
     111,   143,   182,   202,   223,   112,   113,   114,   192,   115,
     116,   117,   118,   119,   120,   121,   122,   107,   132,     0,
     108,   109,   110,   111,   146,     0,     0,     0,     0,   112,
     113,   114,     0,   115,   116,   117,   118,   119,   120,   121,
     122,   107,     0,     0,   108,   109,   110,   111,   175,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,   112,   113,   114,     0,   115,   116,   117,
     118,   119,   120,   121,   122,   107,     0,     0,   108,   109,
     110,   111,   176,     0,     0,     0,   112,   113,   114,     0,
     115,   116,   117,   118,   119,   120,   121,   122,   107,     0,
       0,   108,   109,   110,   111,   212,     0,     0,     0,     0,
     112,   113,   114,     0,   115,   116,   117,   118,   119,   120,
     121,   122,   107,     0,     0,   108,   109,   110,   111,   213,
       0,     0,     0,     0,     0,   107,     0,     0,   108,   109,
     110,   111,     0,     0,   112,   113,   114,     0,   115,   116,
     117,   118,   119,   120,   121,   122,   107,     0,     0,   108,
     109,   110,   111,   214,     0,     0,     0,   112,   113,   114,
       0,   115,   116,   117,   118,   119,   120,   121,   122,   107,
       0,     0,   108,   109,   110,   111,   235,     0,     0,     0,
       0,   112,   113,   114,     0,   115,   116,   117,   118,   119,
     120,   121,   122,     0,   112,   113,   114,   123,   115,   116,
     117,   118,   119,   120,   121,   122,     0,     0,   107,     0,
     135,   108,   109,   110,   111,   112,   113,   114,     0,   115,
     116,   117,   118,   119,   120,   121,   122,     0,     0,   107,
       0,   136,   108,   109,   110,   111,     0,     0,   112,   113,
     114,     0,   115,   116,   117,   118,   119,   120,   121,   122,
       0,     0,   107,     0,   137,   108,   109,   110,   111,     0,
       0,     0,     0,     0,     0,     0,   107,     0,     0,   108,
     109,   110,   111,     0,     0,     0,     0,   112,   113,   114,
       0,   115,   116,   117,   118,   119,   120,   121,   122,   107,
       0,     0,     0,   187,   110,   111,     0,     0,   112,   113,
     114,     0,   115,   116,   117,   118,   119,   120,   121,   122,
     107,   183,     0,     0,     0,   110,   111,     0,     0,     0,
       0,   112,   113,   114,     0,   115,   116,   117,   118,   119,
     120,   121,   122,     0,   222,   112,   113,   114,     0,   115,
     116,   117,   118,   119,   120,   121,   122,   107,     0,     0,
       0,     0,   110,   111,     0,     0,     0,     0,     0,   113,
     114,     0,   115,   116,   117,   118,   119,   120,   121,   122,
    -102,     0,     0,     0,     0,  -102,  -102,     0,     0,     0,
       0,   114,     0,   115,   116,   117,   118,   119,   120,   121,
     122,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,    60,     0,     0,     0,     0,
       0,     0,     0,    89,     0,    61,    62,     0,     0,     0,
     115,   116,   117,   118,   119,   120,   121,   122,    63,    64,
       0,    65,    66,     0,     0,     0,     0,     0,     0,     0,
      67,     0,    60,     0,  -102,  -102,  -102,   119,   120,   121,
     122,     0,    61,    62,    68,     0,     0,     0,    69,    70,
      71,     0,     0,    60,    72,    63,    64,     0,    65,    66,
      73,     0,     0,    61,    62,    90,     0,    67,    74,     0,
       0,     0,     0,     0,     0,     0,    63,    64,     0,    65,
      66,    68,     0,     0,     0,    69,    70,    71,    67,     0,
      60,    72,     0,     0,     0,     0,     0,    73,     0,     0,
      61,    62,   203,     0,     7,    74,    69,    70,    71,     0,
       0,     0,    72,    63,    64,     0,    65,    66,    73,     8,
       0,     0,     0,     9,     0,    67,   204,     0,     0,    10,
       0,     0,     0,     0,     0,    11,     0,     0,    12,    68,
       0,     0,     0,   217,    70,    71,     0,    13,     0,    72,
       0,     0,     0,     0,     0,    73,     0,     0,     0,     0,
       0,     0,     0,    74
  };

  const short int
  p_parser::yycheck_[] =
  {
      43,    69,    65,    11,    51,    11,    11,    34,    34,    77,
     204,    69,    17,    23,    57,    58,    59,     0,    86,     3,
      91,    92,     6,     7,     8,     9,    89,    41,    86,    72,
      73,    74,    37,   227,    77,    34,     3,    22,    46,     6,
       7,     8,     9,    90,    82,    83,    87,    61,    62,    69,
      49,    92,    91,    92,    97,    98,    99,   100,   101,   102,
     103,    88,    88,    93,   107,   108,   109,   110,   111,   112,
     113,   114,   115,   116,   117,   118,   119,   120,   121,   122,
      86,   124,    92,    69,    69,   128,    93,   130,    72,    73,
      74,   134,    76,    77,    78,    79,    80,    81,    82,    83,
      69,    86,    77,    69,    69,    72,    73,    74,    92,    76,
      77,    78,    79,    80,    81,    82,    83,    69,    69,    69,
      69,    85,   165,     3,    77,    92,     6,     7,     8,     9,
      77,    77,    93,    69,   177,    93,   179,    77,   181,   182,
     183,    89,     5,    86,    89,    89,     3,    89,   191,     6,
       7,     8,     9,    85,    89,    89,    89,    87,    87,    69,
      87,   204,    88,    77,     4,    88,    87,    69,   211,    22,
       3,    88,   215,     6,     7,     8,     9,   220,    85,    88,
      80,    74,    90,    88,    90,    89,    77,    88,    14,    49,
     233,   215,    72,    73,    74,   220,    76,    77,    78,    79,
      80,    81,    82,    83,     3,   222,   221,     6,     7,     8,
       9,   101,    92,   190,   211,    72,    73,    74,   172,    76,
      77,    78,    79,    80,    81,    82,    83,     3,    87,    -1,
       6,     7,     8,     9,    91,    -1,    -1,    -1,    -1,    72,
      73,    74,    -1,    76,    77,    78,    79,    80,    81,    82,
      83,     3,    -1,    -1,     6,     7,     8,     9,    91,    -1,
      -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
      -1,    -1,    -1,    72,    73,    74,    -1,    76,    77,    78,
      79,    80,    81,    82,    83,     3,    -1,    -1,     6,     7,
       8,     9,    91,    -1,    -1,    -1,    72,    73,    74,    -1,
      76,    77,    78,    79,    80,    81,    82,    83,     3,    -1,
      -1,     6,     7,     8,     9,    91,    -1,    -1,    -1,    -1,
      72,    73,    74,    -1,    76,    77,    78,    79,    80,    81,
      82,    83,     3,    -1,    -1,     6,     7,     8,     9,    91,
      -1,    -1,    -1,    -1,    -1,     3,    -1,    -1,     6,     7,
       8,     9,    -1,    -1,    72,    73,    74,    -1,    76,    77,
      78,    79,    80,    81,    82,    83,     3,    -1,    -1,     6,
       7,     8,     9,    91,    -1,    -1,    -1,    72,    73,    74,
      -1,    76,    77,    78,    79,    80,    81,    82,    83,     3,
      -1,    -1,     6,     7,     8,     9,    91,    -1,    -1,    -1,
      -1,    72,    73,    74,    -1,    76,    77,    78,    79,    80,
      81,    82,    83,    -1,    72,    73,    74,    88,    76,    77,
      78,    79,    80,    81,    82,    83,    -1,    -1,     3,    -1,
      88,     6,     7,     8,     9,    72,    73,    74,    -1,    76,
      77,    78,    79,    80,    81,    82,    83,    -1,    -1,     3,
      -1,    88,     6,     7,     8,     9,    -1,    -1,    72,    73,
      74,    -1,    76,    77,    78,    79,    80,    81,    82,    83,
      -1,    -1,     3,    -1,    88,     6,     7,     8,     9,    -1,
      -1,    -1,    -1,    -1,    -1,    -1,     3,    -1,    -1,     6,
       7,     8,     9,    -1,    -1,    -1,    -1,    72,    73,    74,
      -1,    76,    77,    78,    79,    80,    81,    82,    83,     3,
      -1,    -1,    -1,    88,     8,     9,    -1,    -1,    72,    73,
      74,    -1,    76,    77,    78,    79,    80,    81,    82,    83,
       3,    85,    -1,    -1,    -1,     8,     9,    -1,    -1,    -1,
      -1,    72,    73,    74,    -1,    76,    77,    78,    79,    80,
      81,    82,    83,    -1,    85,    72,    73,    74,    -1,    76,
      77,    78,    79,    80,    81,    82,    83,     3,    -1,    -1,
      -1,    -1,     8,     9,    -1,    -1,    -1,    -1,    -1,    73,
      74,    -1,    76,    77,    78,    79,    80,    81,    82,    83,
       3,    -1,    -1,    -1,    -1,     8,     9,    -1,    -1,    -1,
      -1,    74,    -1,    76,    77,    78,    79,    80,    81,    82,
      83,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
      -1,    -1,    -1,    -1,    -1,    16,    -1,    -1,    -1,    -1,
      -1,    -1,    -1,    24,    -1,    26,    27,    -1,    -1,    -1,
      76,    77,    78,    79,    80,    81,    82,    83,    39,    40,
      -1,    42,    43,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
      51,    -1,    16,    -1,    77,    78,    79,    80,    81,    82,
      83,    -1,    26,    27,    65,    -1,    -1,    -1,    69,    70,
      71,    -1,    -1,    16,    75,    39,    40,    -1,    42,    43,
      81,    -1,    -1,    26,    27,    86,    -1,    51,    89,    -1,
      -1,    -1,    -1,    -1,    -1,    -1,    39,    40,    -1,    42,
      43,    65,    -1,    -1,    -1,    69,    70,    71,    51,    -1,
      16,    75,    -1,    -1,    -1,    -1,    -1,    81,    -1,    -1,
      26,    27,    65,    -1,    13,    89,    69,    70,    71,    -1,
      -1,    -1,    75,    39,    40,    -1,    42,    43,    81,    28,
      -1,    -1,    -1,    32,    -1,    51,    89,    -1,    -1,    38,
      -1,    -1,    -1,    -1,    -1,    44,    -1,    -1,    47,    65,
      -1,    -1,    -1,    69,    70,    71,    -1,    56,    -1,    75,
      -1,    -1,    -1,    -1,    -1,    81,    -1,    -1,    -1,    -1,
      -1,    -1,    -1,    89
  };

  const unsigned char
  p_parser::yystos_[] =
  {
       0,    41,    61,    62,    95,    96,     0,    13,    28,    32,
      38,    44,    47,    56,    97,    98,    99,   102,   115,   116,
     120,   123,   124,   130,    11,    17,    37,    69,   117,    69,
     101,   134,    93,    69,   100,    69,   131,    93,   125,    98,
      69,    69,    69,    77,    85,    69,    77,    86,   101,   106,
     107,    86,   100,   132,   133,    69,   126,    77,    77,    77,
      16,    26,    27,    39,    40,    42,    43,    51,    65,    69,
      70,    71,    75,    81,    89,   118,    11,    86,    93,   100,
      69,   108,    22,   101,   107,   108,    23,    92,    93,    24,
      86,   118,   127,   128,   118,   118,   118,    89,    89,    89,
      89,    89,    89,    89,   118,   118,   118,     3,     6,     7,
       8,     9,    72,    73,    74,    76,    77,    78,    79,    80,
      81,    82,    83,    88,    34,    88,   103,   118,    77,    86,
      87,    87,   133,   108,    85,    88,    88,    88,   118,   118,
     118,   118,   119,   119,   118,   118,    91,   118,   118,   118,
     118,   118,   118,   118,   118,   118,   118,   118,   118,   118,
     118,   118,   118,   105,   118,     5,   118,    69,   121,   122,
     109,   118,    87,   118,   129,    91,    91,    92,    91,    92,
      91,    92,    92,    85,    88,   104,   118,    88,    77,    87,
      92,     4,   128,    88,   118,   118,   118,   118,   118,    87,
      69,    22,   122,    65,    89,   110,   111,   112,   113,   114,
     118,    85,    91,    91,    91,    34,    88,    69,   134,    88,
      80,    74,    85,   129,   105,    90,   112,    89,   114,    65,
     113,    88,    88,    77,   118,    91
  };

  const unsigned char
  p_parser::yyr1_[] =
  {
       0,    94,    95,    96,    96,    96,    97,    97,    98,    98,
      98,    98,    98,    98,    98,    98,    99,   100,   101,   101,
     101,   101,   102,   103,   104,   105,   106,   106,   106,   106,
     107,   108,   108,   109,   110,   110,   111,   111,   112,   113,
     113,   114,   114,   115,   115,   115,   116,   117,   118,   118,
     118,   118,   118,   118,   118,   118,   118,   118,   118,   118,
     118,   118,   118,   118,   118,   118,   118,   118,   118,   118,
     118,   118,   118,   118,   118,   118,   118,   118,   118,   119,
     119,   120,   121,   121,   122,   123,   124,   125,   125,   126,
     126,   127,   127,   128,   129,   130,   131,   132,   132,   133,
     133,   134
  };

  const unsigned char
  p_parser::yyr2_[] =
  {
       0,     2,     2,     1,     1,     1,     1,     2,     1,     1,
       1,     1,     1,     1,     1,     1,     4,     1,    10,     8,
       6,     4,     2,     1,     1,     1,     1,     1,     2,     2,
       7,     0,     1,     1,     1,     1,     1,     3,     3,     1,
       1,     6,     3,     6,     6,     6,     5,     1,     3,     1,
       1,     1,     1,     1,     6,     6,     4,     4,     6,     4,
       4,     3,     3,     3,     3,     2,     5,     3,     3,     3,
       3,     3,     2,     3,     3,     3,     3,     3,     3,     1,
       3,     8,     1,     3,     3,     7,     4,     0,     3,     0,
       2,     4,     7,     1,     1,     4,     1,     1,     3,     1,
       3,     1
  };



  // YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
  // First, the terminals, then, starting at \a yyntokens_, nonterminals.
  const char*
  const p_parser::yytname_[] =
  {
  "\"end of file\"", "error", "$undefined", "\"!=\"", "\"->\"", "\"..\"",
  "\"<=>\"", "\"=>\"", "\">=\"", "\"<=\"", "\"A\"", "\"bool\"",
  "\"clock\"", "\"const\"", "\"ctmc\"", "\"C\"", "\"ceil\"", "\"double\"",
  "\"dtmc\"", "\"E\"", "\"endinit\"", "\"endinvariant\"", "\"endmodule\"",
  "\"endplayer\"", "\"endrewards\"", "\"endsystem\"", "\"false\"",
  "\"floor\"", "\"formula\"", "\"filter\"", "\"func\"", "\"F\"",
  "\"global\"", "\"G\"", "\"init\"", "\"invariant\"", "\"I\"", "\"int\"",
  "\"label\"", "\"log\"", "\"max\"", "\"mdp\"", "\"min\"", "\"mod\"",
  "\"module\"", "\"X\"", "\"nondeterministic\"", "\"player\"", "\"Pmax\"",
  "\"Pmin\"", "\"P\"", "\"pow\"", "\"probabilistic\"", "\"prob\"",
  "\"pta\"", "\"rate\"", "\"rewards\"", "\"Rmax\"", "\"Rmin\"", "\"R\"",
  "\"S\"", "\"smg\"", "\"bdp\"", "\"stochastic\"", "\"system\"",
  "\"true\"", "\"U\"", "\"W\"", "\"invalid character\"", "\"identifier\"",
  "\"integer number\"", "\"floating-point number\"", "'^'", "'|'", "'&'",
  "'!'", "'?'", "'='", "'<'", "'>'", "'+'", "'-'", "'*'", "'/'", "UMINUS",
  "':'", "'['", "']'", "';'", "'('", "'\\''", "')'", "','", "'\"'",
  "$accept", "model", "model_type", "sections", "section",
  "module_definition", "module_name", "variable_definition",
  "global_variable_definition", "low_value", "high_value", "init_value",
  "module_variables_and_commands", "module_command", "action_name",
  "command_guard", "transitions", "transition_list", "prob_transition",
  "next_state_expression", "primed_state_expression", "const_definition",
  "formula_definition", "formula_name", "expression", "expression_list",
  "module_renaming", "replacement_list", "replacement", "label_definition",
  "rewards_definition", "optional_reward_name", "reward_list",
  "reward_specification", "reward_guard", "reward_value",
  "player_definition", "player_name", "player_control_list",
  "control_specification", "variable_name", YY_NULLPTR
  };

#if YYDEBUG
  const unsigned short int
  p_parser::yyrline_[] =
  {
       0,   203,   203,   207,   209,   210,   214,   215,   219,   220,
     221,   222,   223,   224,   225,   226,   230,   235,   239,   241,
     243,   249,   257,   262,   266,   270,   274,   275,   276,   278,
     283,   288,   289,   293,   297,   299,   303,   304,   308,   313,
     315,   320,   322,   327,   329,   331,   337,   342,   346,   348,
     350,   352,   354,   356,   358,   360,   362,   364,   366,   368,
     370,   372,   374,   376,   378,   380,   382,   384,   386,   388,
     390,   392,   394,   396,   398,   400,   402,   404,   406,   411,
     412,   416,   421,   422,   426,   430,   435,   440,   441,   445,
     446,   451,   453,   458,   462,   466,   471,   475,   476,   481,
     482,   486
  };

  // Print the state stack on the debug stream.
  void
  p_parser::yystack_print_ ()
  {
    *yycdebug_ << "Stack now";
    for (stack_type::const_iterator
           i = yystack_.begin (),
           i_end = yystack_.end ();
         i != i_end; ++i)
      *yycdebug_ << ' ' << i->state;
    *yycdebug_ << std::endl;
  }

  // Report on the debug stream that the rule \a yyrule is going to be reduced.
  void
  p_parser::yy_reduce_print_ (int yyrule)
  {
    unsigned int yylno = yyrline_[yyrule];
    int yynrhs = yyr2_[yyrule];
    // Print the symbols being reduced, and their result.
    *yycdebug_ << "Reducing stack by rule " << yyrule - 1
               << " (line " << yylno << "):" << std::endl;
    // The symbols being reduced.
    for (int yyi = 0; yyi < yynrhs; yyi++)
      YY_SYMBOL_PRINT ("   $" << yyi + 1 << " =",
                       yystack_[(yynrhs) - (yyi + 1)]);
  }
#endif // YYDEBUG

  // Symbol number corresponding to token number t.
  inline
  p_parser::token_number_type
  p_parser::yytranslate_ (int t)
  {
    static
    const token_number_type
    translate_table[] =
    {
     0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    75,    93,     2,     2,     2,    74,    90,
      89,    91,    82,    80,    92,    81,     2,    83,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,    85,    88,
      78,    77,    79,    76,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,    86,     2,    87,    72,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,    73,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65,    66,    67,    68,    69,    70,    71,    84
    };
    const unsigned int user_token_number_max_ = 327;
    const token_number_type undef_token_ = 2;

    if (static_cast<int>(t) <= yyeof_)
      return yyeof_;
    else if (static_cast<unsigned int> (t) <= user_token_number_max_)
      return translate_table[t];
    else
      return undef_token_;
  }


} // pparser
#line 3195 "Pparser/Pparser.cc" // lalr1.cc:1167
#line 489 "Pparser/Pparser.yy" // lalr1.cc:1168


void
pparser::p_parser::error(location_type const & l, std::string const & m)
{
  driver.error(l, m);
}

ast::LocationType astloc(pparser::p_parser::location_type const & l)
{
  return {(int) l.begin.line, (int) l.begin.column, (int) l.end.line, (int) l.end.column};
}
