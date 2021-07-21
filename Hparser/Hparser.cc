/* @file Hparser.cc

  @brief HOA compiler based on the cpphoafparser library.

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

#include <iostream>
#include <fstream>
#include "Hparser.hh"

// Uncomment to get a chatty parser.
//#define HPARSER_PRINT

// This is a convoluted way to silence clang without upsetting GCC.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wunused-private-field"
#include "cpphoafparser/consumer/hoa_intermediate.hh"
#ifdef HPARSER_PRINT
#include "cpphoafparser/consumer/hoa_consumer_print.hh"
#else
#include "cpphoafparser/consumer/hoa_consumer_null.hh"
#endif
#include "cpphoafparser/parser/hoa_parser.hh"
#pragma GCC diagnostic pop

using namespace cpphoafparser;

/** @brief Class for HOA parsing. */
class BuildParity : public HOAIntermediate {
public:
  using ptr = std::shared_ptr<BuildParity>;

  BuildParity(Parity & automaton, HOAConsumer::ptr next,
              Verbosity::Level verbosity = Verbosity::Silent,
              bool noMdp = false) :
    HOAIntermediate(next), automaton(automaton) , verbosity(verbosity), noMdp(noMdp), maxWins(true), oddWins(true)
  {
    mgr = automaton.getManager();
  }

  void addState(unsigned int id,
                std::shared_ptr<std::string> info,
                label_expr::ptr labelExpr,
                std::shared_ptr<int_list> accSignature) override
  {
    automaton.addState(id);
    if (accSignature) {
      if (accSignature->size() != 0) {
        throw HOAConsumerException("Automaton state \"" + *info +
                                   "\" should have no priority");
      }
    }
    next->addState(id, info, labelExpr, accSignature);
  }

  void addStartStates(int_list const & stateConjunction) override
  {
    if (stateConjunction.size() != 1) {
      throw HOAConsumerException("The automaton should have exactly one"
                                 " initial state instead of " +
                                 std::to_string(stateConjunction.size()));
    }
    for (unsigned int state : stateConjunction) {
      automaton.makeInitial(state);
    }
    next->addStartStates(stateConjunction);
  }

  void setAPs(std::vector<std::string> const & aps) override
  {
    // Build an index of the named variables.  These should be among
    // the labels in the MDP.  Unless noMdp is true, the MDP has already
    // been created and the BDD variables for its atomic propositions
    // already have names registered with the BDD manager.
    using NameVarMap = std::map<std::string, BDD>;
    NameVarMap nameVarMap;
    for (int index = 0; index < mgr.ReadSize(); ++index) {
      if (mgr.hasVariableName(index)) {
        std::string varName = mgr.getVariableName(index);
        nameVarMap.emplace(varName, mgr.bddVar(index));
      }
    }
    // Add atomic propositions to automaton.  To support one form of
    // limit determinism, we reserve the name "epsilon" for the atomic
    // proposition that labels jumps from initial to final part of the
    // automaton.  This name is not supposed to be present among the
    // atomic propositions of the model.  The automaton keeps it
    // "hidden" by not adding "epsilon" the the set of atomic
    // propositions of the automaton.
    unsigned int apIndex = 0;
    for (std::string const & ap : aps) {
      BDD var;
      if (ap == std::string("epsilon")) {
        var = mgr.bddVar();
        if (!var.IsNamedVar()) {
          mgr.pushVariableName(ap);
        }
        automaton.setEpsilonBDD(var);
      } else {
        if (!noMdp) {
          auto cit = nameVarMap.find(ap);
          if (cit == nameVarMap.end()) {
            throw HOAConsumerException("Atomic proposition \"" + ap +
                                       "\" found in automaton, but not in model");
          }
          var = cit->second;
        } else {
          var = mgr.bddVar();
          if (!var.IsNamedVar()) {
            mgr.pushVariableName(ap);
          }
          nameVarMap.emplace(ap, var);
        }
        automaton.addAtomicProposition(var, ap);
      }
      apMap.emplace(apIndex, var);
      apIndex++;
    }
    next->setAPs(aps);
  }

  void addEdgeWithLabel(unsigned int stateId, label_expr::ptr labelExpr,
                        int_list const & conjSuccessors,
                        std::shared_ptr<int_list> accSignature) override
  {
    BDD label = convertLabel(*labelExpr);
    if (!checkWellFormedEpsilonLabel(label)) {
      throw HOAConsumerException("Label depends on epsilon, "
                                 "but is not simply epsilon: " +
                                 labelExpr->toString());
    }
    // Multiple successors are only possible with alternation.
    if (conjSuccessors.size() != 1) {
      throw HOAConsumerException("Wrong number (" +
                                 std::to_string(conjSuccessors.size()) +
                                 ") of successors in automaton edge");
    }
    // Add a transition for each successor.
    for (unsigned int succ : conjSuccessors) {
      if (acceptanceName == "parity" ||
          acceptanceName == "Streett") {
        if (accSignature) {
          if (accSignature->size() != 1) {
            throw HOAConsumerException("Each automaton edge should have"
                                       " exactly one priority and not " +
                                       std::to_string(accSignature->size()));
          }
          automaton.addTransition(stateId, succ, label,
                                  convertPriority(accSignature->at(0)));
        } else {
          throw HOAConsumerException("Edge without priority from " +
                                     std::to_string(stateId));
        }
      } else if (acceptanceName == "Buchi") {
        if (accSignature) {
          if (accSignature->size() != 1) {
            throw HOAConsumerException("Each automaton edge should have"
                                       " either 0 or 1 accepting sets and not" +
                                       std::to_string(accSignature->size()));
          }
          automaton.addTransition(stateId, succ, label, 1);
        } else {
          automaton.addTransition(stateId, succ, label, 0);
        }
      } else if (acceptanceName == "co-Buchi") {
        if (accSignature) {
          if (accSignature->size() != 1) {
            throw HOAConsumerException("Each automaton edge should have"
                                       " either 0 or 1 accepting sets and not" +
                                       std::to_string(accSignature->size()));
          }
          automaton.addTransition(stateId, succ, label, 2);
        } else {
          automaton.addTransition(stateId, succ, label, 1);
        }
      } else if (acceptanceName == "Rabin") {
        if (accSignature) {
          if (accSignature->size() != 1) {
            throw HOAConsumerException("Each automaton edge should have"
                                       " exactly one priority and not " +
                                       std::to_string(accSignature->size()));
          }
          automaton.addTransition(stateId, succ, label, 2 - accSignature->at(0));
        } else {
          throw HOAConsumerException("Edge without priority from " +
                                     std::to_string(stateId));
        }
      } else {
        throw HOAConsumerException("Unexpected acceptance type");
      }
    }
    next->addEdgeWithLabel(stateId, labelExpr, conjSuccessors, accSignature);
  }

  void provideAcceptanceName(const std::string& name,
                             const std::vector<IntOrString>& extraInfo) override
  {
    acceptanceName = name;
    // Check the "acc-name" line (if provided).
    if (name == "parity") {
      if (extraInfo.size() != 3) {
        throw HOAConsumerException("Wrong number of tokens (" +
                                   std::to_string(extraInfo.size()) +
                                   ") in parity acceptance specification");
      }
      if (!extraInfo.at(0).isString()) {
        throw HOAConsumerException("Non-string in place of either \"max\" or \"min\"");
      } else {
        if (extraInfo.at(0).getString() == "max") {
          maxWins = true;
        } else if (extraInfo.at(0).getString() == "min") {
          maxWins = false;
        } else {
          throw HOAConsumerException("Only max or min parity conditions are allowed");
        }
      }
      if (!extraInfo.at(1).isString()) {
        throw HOAConsumerException("Non-string found in place of either \"even\" or \"odd\"");
      } else {
        if (extraInfo.at(1).getString() == "odd") {
          oddWins = true;
        } else if (extraInfo.at(1).getString() == "even") {
          oddWins = false;
        } else {
          throw HOAConsumerException("Only even or odd parity conditions are allowed");
        }
      }
      if (!extraInfo.at(2).isInteger()) {
        throw HOAConsumerException("Non-integer found in place of number of priorities");
      } else {
        numPriorities = extraInfo.at(2).getInteger();
      }
    } else if (name == "Streett") {
      if (extraInfo.size() != 1) {
        throw HOAConsumerException("Wrong number of tokens (" +
                                   std::to_string(extraInfo.size()) +
                                   ") in Streett acceptance specification");
      }
      if (!extraInfo.at(0).isInteger() || extraInfo.at(0).getInteger() != 1) {
        throw HOAConsumerException("Only Streett 1 conditions are allowed");
      }
    } else if (name == "Rabin") {
      if (extraInfo.size() != 1) {
        throw HOAConsumerException("Wrong number of tokens (" +
                                   std::to_string(extraInfo.size()) +
                                   ") in Rabin acceptance specification");
      }
      if (!extraInfo.at(0).isInteger() || extraInfo.at(0).getInteger() != 1) {
        throw HOAConsumerException("Only Rabin 1 conditions are allowed");
      }
    } else if (name == "Buchi") {
      if (extraInfo.size() != 0) {
        throw HOAConsumerException("Wrong number of tokens (" +
                                   std::to_string(extraInfo.size()) +
                                   ") in Buechi acceptance specification");
      }
    } else if (name == "co-Buchi") {
      if (extraInfo.size() != 0) {
        throw HOAConsumerException("Wrong number of tokens (" +
                                   std::to_string(extraInfo.size()) +
                                   ") in co-Buechi acceptance specification");
      }
    } else {
      throw HOAConsumerException("Unsupported \"" + name +
                                 "\" acceptance condition. "
                                 "(Only parity, Buchi, co-Buchi, Streett, and Rabin are supported)");
    }
    next->provideAcceptanceName(name, extraInfo);
  }

  void  setAcceptanceCondition(unsigned int numberOfSets,
                               acceptance_expr::ptr accExpr) override
  {
    checkAcceptanceCondition(numberOfSets, accExpr);
    next->setAcceptanceCondition(numberOfSets, accExpr);
  }

  void addProperties(const std::vector<std::string>& properties) override
  {
    // Echo automaton properties if so requested.
    if (verbosity > Verbosity::Informative) {
      std::cout << "properties:";
      for (const std::string& property : properties) {
        std::cout << " " << property;
      }
      std::cout << std::endl;
    }
    next->addProperties(properties);
  }

  void setName(const std::string& name) override
  {
    if (verbosity > Verbosity::Terse) {
      std::cout << "name: ";
      HOAParserHelper::print_quoted(std::cout, name);
      std::cout << std::endl;
    }
  }

private:

  // Convert a transition label expression into a BDD.
  BDD convertLabel(label_expr const & expr) {
    switch (expr.getType()) {
    case label_expr::EXP_AND: {
      return convertLabel(*expr.getLeft()) & convertLabel(*expr.getRight());
    }
    case label_expr::EXP_OR: {
      return convertLabel(*expr.getLeft()) | convertLabel(*expr.getRight());
    }
    case label_expr::EXP_NOT: {
      return ~convertLabel(*expr.getLeft());
    }
    case label_expr::EXP_TRUE: {
      return mgr.bddOne();
    }
    case label_expr::EXP_FALSE: {
      return mgr.bddZero();
    }
    case label_expr::EXP_ATOM: {
      auto it = apMap.find(expr.getAtom().getAPIndex());
      if (it == apMap.end()) {
        throw HOAConsumerException("Unknown atom in automaton label");
      }
      return it->second;
    }
    default: throw HOAConsumerException("Invalid operator in automaton label");
    }
  }

  // Partial semantic check of acceptance expression.
  void checkAcceptanceCondition(unsigned int numberOfSets,
                                acceptance_expr::ptr accExpr)
  {
    if (acceptanceName == "Buchi") {
      if (numberOfSets != 1) {
        throw HOAConsumerException("Wrong number of sets in Buchi acceptance expression: " +
                                   std::to_string(numberOfSets));
      } else if (!accExpr->isAtom()) {
        throw HOAConsumerException("Buchi acceptance expression should consists of one atom");
      }
      AtomAcceptance atom  = accExpr->getAtom();
      AtomAcceptance::AtomType atomType = atom.getType();
      if (atomType != AtomAcceptance::TEMPORAL_INF) {
        throw HOAConsumerException("Wrong atom type for Buechi condition (should be Inf)");
      }
    } else if (acceptanceName == "co-Buchi") {
      if (numberOfSets != 1) {
        throw HOAConsumerException("Wrong number of sets in co-Buchi acceptance expression: " +
                                   std::to_string(numberOfSets));
      } else if (!accExpr->isAtom()) {
        throw HOAConsumerException("co-Buchi acceptance expression should consists of one atom");
      }
      AtomAcceptance atom  = accExpr->getAtom();
      AtomAcceptance::AtomType atomType = atom.getType();
      if (atomType != AtomAcceptance::TEMPORAL_FIN) {
        throw HOAConsumerException("Wrong atom type for co-Buechi condition (should be Fin)");
      }
    } else if (acceptanceName == "Streett") {
      if (numberOfSets != 2) {
        throw HOAConsumerException("Wrong number of sets in Streett 1 acceptance expression: " +
                                   std::to_string(numberOfSets));
      } else if (!accExpr->isOR()) {
        throw HOAConsumerException("Unexpected node type in acceptance expression");
      }
    } else if (acceptanceName == "Rabin") {
      if (numberOfSets != 2) {
        throw HOAConsumerException("Wrong number of sets in Rabin 1 acceptance expression: " +
                                   std::to_string(numberOfSets));
      } else if (!accExpr->isAND()) {
        throw HOAConsumerException("Unexpected node type in acceptance expression");
      }
    } else if (acceptanceName != "parity") {
      throw HOAConsumerException("Unexpected acceptance name: " + acceptanceName);
    }
  }

  // Convert priority to equivalent max-odd value.
  int convertPriority(int priority)
  {
    if (maxWins) {
      if (oddWins) {          // max odd
        return priority;
      } else {                // max even
        return priority + 1;
      }
    } else {
      if (oddWins) {          // min odd
        if (numPriorities & 1U) {
          return numPriorities - 1 - priority;
        } else {
          return numPriorities - priority;
        }
      } else {                // min even
        if (numPriorities & 1U) {
          return numPriorities - priority;
        } else {
          return numPriorities - 1 - priority;
        }
        return -1;
      }
    }
  }

  // A label that depends on epsilon should only be "epsilon."
  bool checkWellFormedEpsilonLabel(BDD label)
  {
    BDD epsilon = automaton.getEpsilonBDD();
    if (label == epsilon) return true;
    unsigned int index  = epsilon.NodeReadIndex();
    return label.BooleanDiff(index).IsZero();
  }

  Parity & automaton;
  Cudd mgr;
  std::map<unsigned int, BDD> apMap;
  Verbosity::Level verbosity;
  bool noMdp;
  std::string acceptanceName;
  bool maxWins;
  bool oddWins;
  int numPriorities;
};

/** Read an automaton from input file and build Parity object. */
void Hparser(Parity & automaton, std::string const & filename,
             Verbosity::Level verbosity, bool noMdp)
{
#ifdef HPARSER_PRINT
  HOAConsumer::ptr printer(new HOAConsumerPrint(std::cout));
#else
  HOAConsumer::ptr printer(new HOAConsumerNull());
#endif
  BuildParity::ptr builder(new BuildParity(automaton, printer, verbosity, noMdp));

  std::ifstream ifs(filename, std::ifstream::in);
  if (ifs.is_open()) {
    HOAParser::parse(ifs, builder);
  } else {
    throw std::runtime_error(std::string("Could not open ") + filename);
  }
}

/** Read an automaton from input stream and build Parity object. */
void Hparser(Parity & automaton, std::istream & is,
             Verbosity::Level verbosity, bool noMdp)
{
#ifdef HPARSER_PRINT
  HOAConsumer::ptr printer(new HOAConsumerPrint(std::cout));
#else
  HOAConsumer::ptr printer(new HOAConsumerNull());
#endif
  BuildParity::ptr builder(new BuildParity(automaton, printer, verbosity, noMdp));

  HOAParser::parse(is, builder);
}
