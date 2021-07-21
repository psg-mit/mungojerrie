#ifndef P_AST_HH_
#define P_AST_HH_

/** @file Past.hh

  @brief Abstract Syntax Tree for PRISM parser.

  @author Mateo Perez, Fabio Somenzi, Ashutosh Trivedi

  @copyright@parblock
  Copyright (c) 2021, Regents of the University of Colorado

  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

  Neither the name of the University of Colorado nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
  @endparblock

*/

/** @brief Namespace of abstract syntax tree components. */
namespace ast {

  enum ModelType { MDP, DTMC, SMG, BDP };

  struct LocationType {
    int first_line;
    int first_column;
    int last_line;
    int last_column;
  };

  enum Operator {
    PLUS, UNARYMINUS, MINUS, TIMES, DIVIDE, ITE, NUMERAL, IDENTIFIER, MIN, MAX,
    MOD, POW, LOG, CEIL, FLOOR, TRUE, FALSE, OR, AND, XOR, IFF, IMPLIES, NOT,
    EQUAL, NOTEQ, LE, LT, GE, GT
  };

  std::ostream & operator<<(std::ostream & os, Operator const & op);
  std::string to_string(Operator const & op);

  enum DataType {
    DOUBLE, NUMBER, BOOL
  };

  struct Expression {
    DataType type;
    Operator op;
    std::vector<Expression> operands;
    std::string identifier;
    int inumeral;
    double dnumeral;
    LocationType location;
  };

  std::ostream & operator<<(std::ostream & os, Expression const & ie);

  struct Variable {
    DataType type;
    std::string name;
    Expression low;
    Expression high;
    Expression initial;
  };

  std::ostream & operator<<(std::ostream & os, Variable const & v);

  struct Formula {
    std::string name;
    Expression expression;
  };

  std::ostream & operator<<(std::ostream & os, Formula const & f);
  
  struct Label {
    std::string name;
    Expression expression;
  };

  std::ostream & operator<<(std::ostream & os, Label const & l);

  struct Constant {
    DataType type;
    std::string name;
    Expression expression;
  };

  std::ostream & operator<<(std::ostream & os, Constant const & c);

  struct Transition {
    Expression probability;
    Expression destination;
  };

  std::ostream & operator<<(std::ostream & os, Transition const & t);

  struct Command {
    std::string action;
    Expression guard;
    std::vector<Transition> transitions;
  };

  std::ostream & operator<<(std::ostream & os, Command const & c);

  struct Module {
    std::string name;
    std::vector<Variable> variables;
    std::vector<Command> commands;
  };

  std::ostream & operator<<(std::ostream & os, Module const & m);

  struct NameReplacement {
    std::string oldname;
    std::string newname;
  };

  std::ostream & operator<<(std::ostream & os, NameReplacement const & m);

  struct ModuleRenaming {
    std::string newname;
    std::string oldname;
    std::vector<NameReplacement> replacements;
    LocationType location;
  };

  std::ostream & operator<<(std::ostream & os, ModuleRenaming const & m);

  enum RewardType { STATE, ACTION };

  struct RewardSpec {
    RewardType type;
    std::string name;
    Expression guard;
    Expression reward;
  };

  std::ostream & operator<<(std::ostream & os, RewardSpec const & rs);

  struct RewardSection {
    std::string name;
    std::vector<RewardSpec> specs;
  };

  std::ostream & operator<<(std::ostream & os, RewardSection const & rv);

  enum ControlType { PMODULE, PACTION };

  struct ControlSpec {
    ControlType type;
    std::string name;
  };

  std::ostream & operator<<(std::ostream & os, ControlSpec const & cs);

  struct Player {
    std::string name;
    std::vector<ControlSpec> controls;
    LocationType location;
  };

  std::ostream & operator<<(std::ostream & os, Player const & p);
}

#endif
