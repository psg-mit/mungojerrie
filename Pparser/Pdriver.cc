/** @file Pdriver.cc

  @brief Driver for PRISM parser.

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

#include <sstream>
#include <cmath>
#include <stdexcept>
#include <iterator>
#include "Pwrapper.hh"
#include "Verbosity.hh"

using namespace std;

p_driver::p_driver(Model & model, vector<pair<string,string>> const & defines,
                   Verbosity::Level verbosity)
  : verbosity(verbosity), model(model),
    trace_scanning(false), trace_parsing(false), defines(defines)
{
  // Actions are named in PRISM, but numbered in the Model class.
  // Hence we dynamically translate.
  actnum=0;
  // This will be updated once the AST is available.
  nameLengthLowerBound = 1;

  if (verbosity > Verbosity::Terse) {
    char const sep[] = ", ";
    char const * psep = sep + 2;
    for (auto const & def : defines) {
      cout << psep << def.first << " = " << def.second;
      psep = sep;
    }
    if (defines.size() > 0) {
      cout << endl;
    }
  }

  if (verbosity > Verbosity::Logorrheic) {
    model.getManager().makeVerbose();
    trace_scanning = true;
    trace_parsing = true;
  }
}

int p_driver::parse(string const & f)
{
  file = f;
  scan_begin();
  pparser::p_parser parser(*this);
  parser.set_debug_level(trace_parsing);
  int res = parser.parse();
  scan_end();
  if (res == 0) { // Syntax is OK.
    // Apply command-line definitions of constants.
    applyDefines();
    // Build the maps needed for formula substitution and module renaming.
    for (size_t i = 0; i != formulae.size(); ++i) {
      formulaMap.emplace(formulae.at(i).name, i);
    }
    for (size_t i = 0; i != modules.size(); ++i) {
      moduleMap.emplace(modules.at(i).name, i);
    }
    substituteFormulae();
    renameModules();
    // Build maps that give values to constants and bounds to variables.
    buildMaps();
    // Infer type of numerical expressions and do some checking.
    checkAst();
    // Fold constants.
    foldConstants();
    if (verbosity > Verbosity::Informative) {
      dumpAst();
    }
    // Build the object of class Model.
    buildModel();
    model.replaceUnitProbabilityTransitions();
    model.sortEdgesByPriority();
    model.sanityCheck();

    if (verbosity > Verbosity::Terse) {
      cout << "total number of states: " << model.numNodes() << endl;
    }
  }
  return res;
}

void p_driver::error(pparser::location const & l, string const & m)
{
  cerr << l << ": " << m << endl;
}

void p_driver::error(string const & m)
{
  cerr << m << endl;
}

[[noreturn]] void p_driver::semantic_error(ast::LocationType const & l,
                                           string const & m) const
{
  string copy = file;
  pparser::position pbegin(&copy, l.first_line, l.first_column);
  pparser::position pend(&copy, l.last_line, l.last_column);
  pparser::location loc(pbegin, pend);
  ostringstream os;
  os << loc << ": " << m;
  throw runtime_error(os.str());
}

void p_driver::buildMaps()
{
  // Compute the value of each constant.  For bool variables, false is
  // translated to 0 and true is translated to 1.
  for (auto const & c : constants) {
    if (c.type == ast::NUMBER) {
      intConstantMap.emplace(c.name, evaluateIntegerExpression(c.expression));
    } else if (c.type == ast::DOUBLE) {
      doubleConstantMap.emplace(c.name, evaluateDoubleExpression(c.expression));
    } else if (c.type == ast::BOOL) {
      boolConstantMap.emplace(c.name, isEnabled(c.expression));
    } else {
      semantic_error(c.expression.location, "unknown constant type");
    }
  }
  // Compute low, high, and initial value for each variable.
  for (auto const & variable : globals) {
    getVarLimits(variable, -1);
  }
  for (size_t i = 0; i != modules.size(); ++i) {
    for (auto const & variable : modules.at(i).variables) {
      getVarLimits(variable, (int) i);
    }
  }
  // Count the modules where each named action is used.
  // Give a unique name to each anonymous action.
  Action anonAct = 0;
  for (size_t i = 0; i != modules.size(); ++i) {
    for (auto & command : modules.at(i).commands) {
      string & actName = command.action;
      Action action = getActionNumber(actName);
      bool anonymous = actName == "";
      if (anonymous) {
        actName = string("_a") + Util::to_decimal(anonAct);
        anonAct++;
      } else {
        auto ait = actionModules.find(action);
        if (ait == actionModules.end()) {
          actionModules.emplace(action, set<int>{(int) i});
        } else {
          auto & modset = ait->second;
          modset.insert(i);
        }
      }
    }
  }
  if (verbosity > Verbosity::Informative) {
    cout << "Named action modules: " << actionModules << endl;
  }
  for (auto const & am : actionModules) {
    actionCounts.emplace(am.first, (int) am.second.size());
  }
  // This is to speed up the creation of node names by reducing
  // the number of memory allocations.
  setNameLengthBound();
  if (modelType == ast::SMG || modelType == ast::BDP) {
    // Check that each module and action is controlled by at most one player.
    for (size_t i = 0; i != players.size(); ++i) {
      playerMap.emplace(players.at(i).name,(Player) i);
      for (auto const & control : players.at(i).controls) {
        if (control.type == ast::PMODULE) {
          auto const mit = modulePlayer.find(control.name);
          if (mit == modulePlayer.end()) {
            modulePlayer.emplace(control.name, players.at(i).name);
          } else {
            semantic_error(players.at(i).location, "module " + control.name +
                           " previously assigned to player " + mit->second +
                           " now re-assigned to player " + players.at(i).name);
          }
        } else { // control is a named action
          auto const ait = actionPlayer.find(control.name);
          if (ait == actionPlayer.end()) {
            actionPlayer.emplace(control.name, players.at(i).name);
          } else {
            semantic_error(players.at(i).location, "action " + control.name +
                           " previously assigned to player " + ait->second +
                           " now re-assigned to player " + players.at(i).name);
          }
        }
      }
    }
    // Check that each module and action is controlled by some player.
    for (auto const & module : moduleMap) {
      auto const mit = modulePlayer.find(module.first);
      if (mit == modulePlayer.end()) {
        bool found = false;
        for (auto const & command : modules.at(module.second).commands) {
          if (command.action == "") {
            found = true;
            break;
          }
        }
        if (found) {
          throw runtime_error("module " + module.first +
                              " not controlled by any player");
        }
      }
    }
    for (auto const & action : actionMap) {
      bool anonymous = action.first == "";
      if (!anonymous) {
        auto const ait = actionPlayer.find(action.first);
        if (ait == actionPlayer.end()) {
          throw runtime_error("named action " + action.first +
                              " not controlled by any player");
        }
        auto const pit = playerMap.find(ait->second);
        if (pit == playerMap.end()) {
          throw runtime_error("player " + ait->second + " unknown");
        }
        actionOwner.emplace(action.second, pit->second);
      }
    }
    for (size_t i = 0; i != modules.size(); ++i) {
      auto const mit = modulePlayer.find(modules.at(i).name);
      if (mit == modulePlayer.end()) {
        bool found = false;
        for (auto const & command : modules.at(i).commands) {
          if (command.action == "") {
            found = true;
            break;
          }
        }
        if (found) {
          throw runtime_error("module " + modules.at(i).name + " unassigned");
        } else {
          continue;
        }
      }
      auto const pit = playerMap.find(mit->second);
      if (pit == playerMap.end()) {
        throw runtime_error("player " + mit->second + " unknown");
      }
      for (auto & command : modules.at(i).commands) {
        string & actName = command.action;
        bool anonymous = Util::is_prefix(actName, "_a");
        if (anonymous) {
          Action action = getActionNumber(actName);
          actionOwner.emplace(action, pit->second);
        }
      }
    }
    if (verbosity > Verbosity::Informative) {
      cout << "Action Owners" << actionOwner << endl;
    }
  }
}

void p_driver::getVarLimits(ast::Variable const & variable, int modIndex)
{
  int low, high, initial;
  if (variable.type == ast::NUMBER) {
    low = evaluateIntegerExpression(variable.low);
    high = evaluateIntegerExpression(variable.high);
    initial = evaluateIntegerExpression(variable.initial);
  } else if (variable.type == ast::BOOL) {
    low = 0;
    high = 1;
    initial = isEnabled(variable.initial) ? 1 : 0;
  } else {
    semantic_error(variable.initial.location, "Unknown variable type: " +
                   to_string(variable.type));
  }
  varMap.emplace(variable.name, varInfo.size());
  varInfo.push_back({variable.type, variable.name, low, high, initial, modIndex});
}

void p_driver::setNameLengthBound()
{
  nameLengthLowerBound = varInfo.size();
  for (auto const & vi : varInfo) {
    nameLengthLowerBound += vi.name.length();
  }
}

void p_driver::applyDefines()
{
  for (auto const & def: defines) {
    string const & constName = def.first;
    string const & constVal = def.second;
    for (auto & c : constants) {
      if (constName == c.name) {
        ast::DataType type = c.type;
        ast::LocationType location = c.expression.location;
        if (type == ast::DOUBLE) {
          double val = stod(constVal);
          c.expression = {type, ast::NUMERAL, {}, "", 0, val, location};
        } else if (type == ast::NUMBER) {
          int val = stoi(constVal);
          c.expression = {type, ast::NUMERAL, {}, "", val, (double) val, location};
        } else if (type == ast::BOOL) {
          if (constVal == "true") {
            c.expression = {type, ast::TRUE, {}, "", 1, (double) 1, location};
          } else if (constVal == "false") {
            c.expression = {type, ast::FALSE, {}, "", 0, (double) 0, location};
          } else {
            throw invalid_argument("Illegal value (" + constVal +
                                   ") for Boolean constant \"" +
                                   constName + "\" in command line");
          }
        }
      }
    }
  }
}

void p_driver::substituteFormulae()
{
  for (auto & c : constants) {
    subst(c.expression);
  }
  for (auto & l : labels) {
    subst(l.expression);
  }
  for (auto & m : modules) {
    for (auto & v : m.variables) {
      subst(v.low);
      subst(v.high);
      subst(v.initial);
    }
    for (auto & c : m.commands) {
      subst(c.guard);
      for (auto & t : c.transitions) {
        subst(t.probability);
        subst(t.destination);
      }
    }
  }
  for (auto & r : rewards) {
    for (auto & rs : r.specs) {
      subst(rs.guard);
      subst(rs.reward);
    }
  }
}

void p_driver::subst(ast::Expression & ex)
{
  if (ex.op == ast::IDENTIFIER) {
    auto fit = formulaMap.find(ex.identifier);
    if (fit != formulaMap.end()) {
      auto const & replacement = formulae.at(fit->second).expression;
      ex = replacement;
    }
  } else if (ex.op == ast::EQUAL && ex.operands.size() == 1) {
    // next-state assignment
    auto fit = formulaMap.find(ex.identifier);
    if (fit != formulaMap.end()) {
      auto const & replacement = formulae.at(fit->second).expression;
      if (replacement.op != ast::IDENTIFIER) {
        semantic_error(ex.location, "primed variable replacement \"" +
                       ex.identifier + "\" does not expand to an identifier");
      }
      ex.identifier = replacement.identifier;
    }
    subst(ex.operands.at(0));
  } else if (ex.op == ast::TRUE || ex.op == ast::FALSE ||
             ex.op == ast::NUMERAL || ex.op == ast::PLUS ||
             ex.op == ast::MINUS || ex.op == ast::TIMES ||
             ex.op == ast::DIVIDE || ex.op == ast::OR || ex.op == ast::AND ||
             ex.op == ast::XOR || ex.op == ast::IFF || ex.op == ast::IMPLIES ||
             ex.op == ast::EQUAL || ex.op == ast::NOTEQ || ex.op == ast::LT ||
             ex.op == ast::LE || ex.op == ast::GT || ex.op == ast::GE ||
             ex.op == ast::MOD || ex.op == ast::POW || ex.op == ast::MIN || ex.op == ast::MAX ||
             ex.op == ast::LOG || ex.op == ast::CEIL || ex.op == ast::FLOOR ||
             ex.op == ast::ITE || ex.op == ast::NOT || ex.op == ast::UNARYMINUS) {
    for (auto & op : ex.operands) {
      subst(op);
    }
  } else {
    semantic_error(ex.location, "Unknown operator (" + to_string(ex.op) +
                   ") in expression");
  }
}

void p_driver::renameModules()
{
  for (auto & rn : renamings) {
    // Get base module.
    auto const mit = moduleMap.find(rn.oldname);
    if (mit == moduleMap.end()) {
      semantic_error(rn.location, "Trying to rename undefined module \"" +
                     rn.oldname + "\"");
    }
    // Create replacement map.
    ReplacementMap rmap;
    for (auto const & rp : rn.replacements) {
      rmap.emplace(rp.oldname, rp.newname);
    }
    ast::Module const & oldmodule = modules.at(mit->second);
    ast::Module newmodule{rn.newname, {}, {}};
    for (auto const & v : oldmodule.variables) {
      auto vit = rmap.find(v.name);
      if (vit == rmap.end()) {
        semantic_error(rn.location, "no replacement for \"" + v.name +
                       "\" in renaming of \"" + rn.oldname + "\"");
      }
      std::string vname = vit->second;
      ast::Expression newlo = rename(v.low, rmap);
      ast::Expression newhi = rename(v.high, rmap);
      ast::Expression newinit = rename(v.initial, rmap);
      ast::Variable newv{v.type,vname,newlo,newhi,newinit};
      newmodule.variables.push_back(newv);
    }
    for (auto const & c : oldmodule.commands) {
      auto const ait = rmap.find(c.action);
      string newaction = ait != rmap.end() ? ait->second : c.action;
      ast::Expression newguard = rename(c.guard, rmap);
      ast::Command newc{newaction, newguard, {}};
      for (auto const & t : c.transitions) {
        ast::Transition newt{rename(t.probability, rmap),
            rename(t.destination, rmap)};
        newc.transitions.push_back(newt);
      }
      newmodule.commands.push_back(newc);
    }
    size_t modindex = modules.size();
    moduleMap.emplace(rn.newname, modindex);
    modules.push_back(newmodule);
  }
}

ast::Expression p_driver::rename(ast::Expression const & ex,
                                 ReplacementMap const & rmap) const
{
  ast::Expression newex{ex.type, ex.op, {}, ex.identifier,
                        ex.inumeral, ex.dnumeral, ex.location};
  if (ex.op == ast::IDENTIFIER) {
    auto rit = rmap.find(ex.identifier);
    if (rit != rmap.end()) {
      newex.identifier = rit->second;
    }
  } else if (ex.op == ast::EQUAL && ex.operands.size() == 1) {
    // next-state assignment
    auto rit = rmap.find(ex.identifier);
    if (rit != rmap.end()) {
      newex.identifier = rit->second;
    } else {
      bool global = false;
      for (auto const & var : globals) {
        if (var.name == ex.identifier) {
          global = true;
          break;
        }
      }
      if (!global) {
        semantic_error(ex.location,
                       "missing replacement for next-state variable \"" +
                       ex.identifier + "\"");
      }
    }
    newex.operands.push_back(rename(ex.operands.at(0), rmap));
  } else if (ex.op == ast::TRUE || ex.op == ast::FALSE ||
             ex.op == ast::NUMERAL || ex.op == ast::PLUS ||
             ex.op == ast::MINUS || ex.op == ast::TIMES ||
             ex.op == ast::DIVIDE || ex.op == ast::OR || ex.op == ast::AND ||
             ex.op == ast::XOR || ex.op == ast::IFF || ex.op == ast::IMPLIES ||
             ex.op == ast::EQUAL || ex.op == ast::NOTEQ || ex.op == ast::LT ||
             ex.op == ast::LE || ex.op == ast::GT || ex.op == ast::GE ||
             ex.op == ast::MOD || ex.op == ast::POW || ex.op == ast::MIN || ex.op == ast::MAX ||
             ex.op == ast::LOG || ex.op == ast::CEIL || ex.op == ast::FLOOR ||
             ex.op == ast::ITE || ex.op == ast::NOT || ex.op == ast::UNARYMINUS) {
    for (auto & op : ex.operands) {
      newex.operands.push_back(rename(op, rmap));
    }
  } else {
    semantic_error(ex.location, "Unknown operator (" +  to_string(ex.op) +
                   ") in expression");
  }
  return newex;
}

// We assume here that formula substitutions and module renamings have
// already taken place.
void p_driver::foldConstants()
{
  for (auto & c : constants) {
    foldInExpression(c.expression);
  }
  for (auto & l : labels) {
    foldInExpression(l.expression);
  }
  for (auto & m : modules) {
    for (auto & v : m.variables) {
      foldInExpression(v.low);
      foldInExpression(v.high);
      foldInExpression(v.initial);
    }
    for (auto & c : m.commands) {
      foldInExpression(c.guard);
      for (auto & t : c.transitions) {
        foldInExpression(t.probability);
        foldInPrimedExpression(t.destination);
      }
    }
  }
  for (auto & r : rewards) {
    for (auto & rs : r.specs) {
      foldInExpression(rs.guard);
      foldInExpression(rs.reward);
    }
  }
}

bool p_driver::foldInExpression(ast::Expression & ex)
{
  if (ex.op == ast::IDENTIFIER) {
    if (ex.type == ast::BOOL) {
      auto cit = boolConstantMap.find(ex.identifier);
      if (cit != boolConstantMap.end()) {
        bool value = cit->second;
        if (value) {
          ex.op = ast::TRUE;
          ex.inumeral = 1;
          ex.dnumeral = 1.0;
        } else {
          ex.op = ast::FALSE;
          ex.inumeral = 0;
          ex.dnumeral = 0.0;
        }
        ex.identifier = "";
        return true;
      } else {
        auto vit = varMap.find(ex.identifier);
        if (vit ==varMap.end()) {
          semantic_error(ex.location, "Unknown Boolean variable " +
                         ex.identifier);
        }
        ex.inumeral = vit->second;
        return false;
      }
    } else if (ex.type == ast::DOUBLE) {
      auto cit = doubleConstantMap.find(ex.identifier);
      if (cit != doubleConstantMap.end()) {
        ex.op = ast::NUMERAL;
        ex.identifier = "";
        ex.inumeral = 0;
        ex.dnumeral = cit->second;
        return true;
      } else {
        auto iit = intConstantMap.find(ex.identifier);
        if (iit != intConstantMap.end()) {
          ex.op = ast::NUMERAL;
          ex.identifier = "";
          ex.inumeral = 0;
          ex.dnumeral = (double) iit->second;
          return true;
        } else {
          auto vit = varMap.find(ex.identifier);
          if (vit ==varMap.end()) {
            semantic_error(ex.location, "Unknown double variable " +
                           ex.identifier);
          }
          ex.inumeral = vit->second;
          return false;
        }
      }
    } else if (ex.type == ast::NUMBER) {
      auto cit = intConstantMap.find(ex.identifier);
      if (cit != intConstantMap.end()) {
        ex.op = ast::NUMERAL;
        ex.identifier = "";
        ex.inumeral = cit->second;
        ex.dnumeral = (double) ex.inumeral;
        return true;
      } else {
        auto vit = varMap.find(ex.identifier);
        if (vit ==varMap.end()) {
          semantic_error(ex.location, "Unknown integer variable " +
                         ex.identifier);
        }
        ex.inumeral = vit->second;
        return false;
      }
    } else {
      semantic_error(ex.location, "Unknown data type");
    }
  } else if (ex.op == ast::NUMERAL || ex.op == ast::TRUE || ex.op == ast::FALSE) {
    return true;
  } else if (ex.operands.size() > 0) {
    bool allConstants = true;
    for (ast::Expression & op : ex.operands) {
      bool thisOpConst = foldInExpression(op);
      allConstants = allConstants && thisOpConst;
    }
    if (allConstants) {
      if (ex.type == ast::BOOL) {
        bool value = isEnabled(ex);
        if (value) {
          ex.op = ast::TRUE;
          ex.inumeral = 1;
          ex.dnumeral = 1.0;
        } else {
          ex.op = ast::FALSE;
          ex.inumeral = 0;
          ex.dnumeral = 0.0;
        }
      } else if (ex.type == ast::DOUBLE) {
        ex.inumeral = 0;
        ex.dnumeral = evaluateDoubleExpression(ex);
        ex.op = ast::NUMERAL;
      } else if (ex.type == ast::NUMBER) {
        ex.inumeral = evaluateIntegerExpression(ex);
        ex.dnumeral = (double) ex.inumeral;
        ex.op = ast::NUMERAL;
      } else {
        semantic_error(ex.location,
                       "Unknown data type found during constant folding");
      }
      ex.identifier = "";
      ex.operands = {};
    }
    return allConstants;
  } else {
    semantic_error(ex.location, "Unexpected operator " + to_string(ex.op) +
                   " found during constant folding");
  }
}

void p_driver::foldInPrimedExpression(ast::Expression & ex)
{
  if (ex.op == ast::TRUE) {
    return;
  } else if (ex.op == ast::EQUAL) {
    foldInExpression(ex.operands.at(0));
    auto vit = varMap.find(ex.identifier);
    if (vit ==varMap.end()) {
      semantic_error(ex.location, "Unknown primed variable " +
                     ex.identifier);
    }
    ex.inumeral = vit->second;
  } else if (ex.op == ast::AND) {
    foldInPrimedExpression(ex.operands.at(0));
    foldInPrimedExpression(ex.operands.at(1));
  }
}

void p_driver::dumpAst() const
{
  if (modelType == ast::MDP) {
    cout << "mdp\n";
  } else if (modelType == ast::DTMC) {
    cout << "dtmc\n";
  } else if (modelType == ast::BDP) {
    cout << "bdp\n";
  } else {
    cout << "smg\n";
  }

  for (auto const & f : formulae) {
    cout << f << endl;
  }

  for (auto const & c : constants) {
    cout << c << endl;
  }

  for (auto const & l : labels) {
    cout << l << endl;
  }

  for (auto const & g : globals) {
    cout << "global " << g << endl;
  }

  for (auto const & m : modules) {
    cout << m << endl;
  }

  for (auto const & r : rewards) {
    cout << r << endl;
  }

  for (auto const & p : players) {
    cout << p << endl;
  }
}

void p_driver::checkAst()
{
  // Check number of players.
  if (modelType == ast::MDP) {
    if (players.size() != 0) {
      semantic_error(players.at(0).location, "Player specification in mdp");
    }
  } else if (modelType == ast::DTMC) {
    if (players.size() != 0) {
      semantic_error(players.at(0).location, "Player specification in dtmc");
    }
  } else if (modelType == ast::BDP) {
    if (players.size() < 2) {
      throw runtime_error("Fewer than two players in a bdp");
    } else if (players.size() > 2) {
      semantic_error(players.at(2).location, "Currently there may not be more than two players");
    }
  } else {
    if (players.size() < 2) {
      throw runtime_error("Fewer than two players in an smg");
    } else if (players.size() > 2) {
      semantic_error(players.at(2).location, "Currently there may not be more than two players");
    }
  }
  for (auto & c : constants) {
    // The type of a constant is given in its definition.
    if (c.type == ast::NUMBER) {
      checkIntegerExpression(c.expression);
    } else if (c.type == ast::DOUBLE) {
      checkDoubleExpression(c.expression);
    } else if (c.type == ast::BOOL) {
      checkBooleanExpression(c.expression);
    } else {
      semantic_error(c.expression.location, "unknown constant type");
    }
  }
  for (auto & l : labels) {
    // Labels denote predicates over states.  Hence the numerical
    // expressions in them are taken to be of type int.
    checkBooleanExpression(l.expression);
  }
  for (size_t i = 0; i != modules.size(); ++i) {
    for (auto & c : modules.at(i).commands) {
      // Guards are predicated over states.  Hence the numerical
      // expressions in them are taken to be of type int.
      checkBooleanExpression(c.guard);
      bool anonymous = Util::is_prefix(c.action, "_a");
      for (auto & t : c.transitions) {
        // Only commands with anonymous actions can write global variables.
        checkPrimedBooleanExpression(t.destination, (int) i, anonymous);
        checkDoubleExpression(t.probability);
      }
    }
  }
  // Additional checks for state variables.
  for (auto const & var : varInfo) {
    auto icit = intConstantMap.find(var.name);
    if (icit != intConstantMap.end()) {
      throw runtime_error("Variable \"" + var.name +
                         "\" already declared as integer constant");
    }
    auto dcit = doubleConstantMap.find(var.name);
    if (dcit != doubleConstantMap.end()) {
      throw runtime_error("Variable \"" + var.name +
                         "\" already declared as double constant");
    }
    auto bcit = boolConstantMap.find(var.name);
    if (bcit != boolConstantMap.end()) {
      throw runtime_error("Variable \"" + var.name +
                         "\" already declared as Boolean constant");
    }
    if (var.low > var.high) {
      throw runtime_error("Empty range for state variable " + var.name);
    }
    if (var.initial < var.low || var.initial > var.high) {
      throw runtime_error("Initial value out of range for variable " + var.name);
    }
  }
  for (auto & reward : rewards) {
    for (auto & rs : reward.specs) {
      checkBooleanExpression(rs.guard);
      checkDoubleExpression(rs.reward);
    }
  }
  // Checks for stochastic game players.
  for (auto const & player : players) {
    for (auto const & control : player.controls) {
      if (control.type == ast::PMODULE) {
        auto const mit  = moduleMap.find(control.name);
        if (mit == moduleMap.end()) {
          semantic_error(player.location,
                        "unknown module name in player " + player.name + "'s controls");
        }
      } else { // control is a named action
        auto const ait = actionMap.find(control.name);
        if (ait == actionMap.end()) {
          semantic_error(player.location,
                         "unknown action name in player " + player.name + "'s controls");
        }
      }
    }
  }
}

void p_driver::checkBooleanExpression(ast::Expression & ex)
{
  if (ex.op == ast::OR || ex.op == ast::AND || ex.op == ast::XOR ||
      ex.op == ast::IFF || ex.op == ast::IMPLIES) {
    if (ex.operands.size() != 2) {
      semantic_error(ex.location, "Binary operator (" + to_string(ex.op) +
                     ") does not have two operands.");
    }
    checkBooleanExpression(ex.operands.at(0));
    checkBooleanExpression(ex.operands.at(1));
  } else if (ex.op == ast::EQUAL || ex.op == ast::NOTEQ ||
      ex.op == ast::LT || ex.op == ast::LE || ex.op == ast::GT ||
      ex.op == ast::GE) {
    if (ex.operands.size() != 2) {
      semantic_error(ex.location, "Relational operator (" + to_string(ex.op) +
                     ") does not have two operands.");
    }
    checkIntegerExpression(ex.operands.at(0));
    checkIntegerExpression(ex.operands.at(1));
  } else if (ex.op == ast::ITE) {
    if (ex.operands.size() != 3) {
      semantic_error(ex.location, "ITE does not have three operands.");
    }
    checkBooleanExpression(ex.operands.at(0));
    checkBooleanExpression(ex.operands.at(1));
    checkBooleanExpression(ex.operands.at(2));
  } else if (ex.op == ast::TRUE || ex.op == ast::FALSE) {
    if (ex.operands.size() != 0) {
      semantic_error(ex.location, "Boolean constant (" + to_string(ex.op) +
                     ") has operands.");
    }
  } else if (ex.op == ast::IDENTIFIER) {
    ex.type = ast::BOOL;
    if (ex.identifier.size() == 0) {
      semantic_error(ex.location, "unexpected empty identifier");
    }
    if (ex.operands.size() != 0) {
      semantic_error(ex.location, "Boolean identifier (" + ex.identifier +
                     ") has operands.");
    }
  } else if (ex.op == ast::NOT) {
    if (ex.operands.size() != 1) {
      semantic_error(ex.location, "NOT node does not have one operand.");
    }
    checkBooleanExpression(ex.operands.at(0));
  } else {
    semantic_error(ex.location, "Illegal operator (" + to_string(ex.op) +
                   ") in Boolean expression.");
  }
}

void p_driver::checkPrimedBooleanExpression(
  ast::Expression & ex,
  int moduleIndex,
  bool anonymous)
{
  // A primed expression has a restricted form.  It can be the
  // Boolean constant "true," an assignment to a primed state
  // variable, or a conjunction of such assignments.
  if (ex.op == ast::EQUAL) {
    if (ex.operands.size() == 0) {
      semantic_error(ex.location, "EQUAL node with no operands.");
    }
    auto const it = varMap.find(ex.identifier);
    if (it == varMap.end()) {
      semantic_error(ex.location, "Unknown variable \"" + ex.identifier +
                     "\" in primed EQUAL node.");
    }
    size_t varIndex = it->second;
    int mindex = varInfo.at(varIndex).module;
    if (mindex != moduleIndex && mindex != -1) {
      semantic_error(ex.location, "Variable \"" + ex.identifier +
                     "\" in primed EQUAL node is from different module.");
    }
    if (mindex == -1 && !anonymous) {
      semantic_error(ex.location, "Global variable \"" + ex.identifier +
                     "\" modified by named action");
    }
    if (varInfo.at(varIndex).type == ast::NUMBER) {
      checkIntegerExpression(ex.operands.at(0));
    } else {
      checkBooleanExpression(ex.operands.at(0));
    }
  } else if (ex.op == ast::AND) {
    if (ex.operands.size() != 2) {
      semantic_error(ex.location, "AND node does not have two operands.");
    }
    checkPrimedBooleanExpression(ex.operands.at(0), moduleIndex, anonymous);
    checkPrimedBooleanExpression(ex.operands.at(1), moduleIndex, anonymous);
  } else if (ex.op == ast::TRUE) {
    if (ex.operands.size() != 0) {
      semantic_error(ex.location, "TRUE node has operands.");
    }
  } else {
    semantic_error(ex.location, "Illegal operator (" + to_string(ex.op) +
                   ") in primed Boolean expression.");
  }
}

void p_driver::checkIntegerExpression(ast::Expression & ex)
{
  if (ex.op == ast::PLUS || ex.op == ast::MINUS || ex.op == ast::TIMES ||
      ex.op == ast::DIVIDE) {
    if (ex.operands.size() != 2) {
      semantic_error(ex.location, "Binary operator (" + to_string(ex.op) +
                     ") does not have two operands.");
    }
    checkIntegerExpression(ex.operands.at(0));
    checkIntegerExpression(ex.operands.at(1));
  } else if (ex.op == ast::UNARYMINUS) {
    if (ex.operands.size() != 1) {
      semantic_error(ex.location, "Unary operator (" + to_string(ex.op) +
                     ") does not have one operand.");
    }
    checkIntegerExpression(ex.operands.at(0));
  } else if (ex.op == ast::ITE) {
    if (ex.operands.size() != 3) {
      semantic_error(ex.location, "ITE node does not have three operands.");
    }
    checkBooleanExpression(ex.operands.at(0));
    checkIntegerExpression(ex.operands.at(1));
    checkIntegerExpression(ex.operands.at(2));
  } else if (ex.op == ast::NUMERAL) {
    if (ex.operands.size() != 0) {
      semantic_error(ex.location, "Numeral has operands.");
    }
  } else if (ex.op == ast::TRUE || ex.op == ast::FALSE) {
    if (ex.operands.size() != 0) {
      semantic_error(ex.location, "Boolean constant has operands.");
    }
  } else if (ex.op == ast::IDENTIFIER) {
    if (ex.operands.size() != 0) {
      semantic_error(ex.location, "Identifier has operands.");
    }
  } else if (ex.op == ast::MOD || ex.op == ast::POW) {
    if (ex.operands.size() != 2) {
      semantic_error(ex.location, "Binary function \"" + ex.identifier +
                     "\" does not have two operands.");
    }
    checkIntegerExpression(ex.operands.at(0));
    checkIntegerExpression(ex.operands.at(1));
  } else if (ex.op == ast::MIN || ex.op == ast::MAX) {
    if (ex.operands.size() == 0) {
      semantic_error(ex.location, ex.identifier +
                     " function called without operands.");
    }
    for (auto & op : ex.operands) {
      checkIntegerExpression(op);
    }
  } else {
    semantic_error(ex.location, "Illegal operator (" + to_string(ex.op) +
                   ") in integer expression.");
  }
}

void p_driver::checkDoubleExpression(ast::Expression & ex)
{
  ex.type = ast::DOUBLE;
  if (ex.op == ast::PLUS || ex.op == ast::MINUS || ex.op == ast::TIMES ||
      ex.op == ast::DIVIDE) {
    if (ex.operands.size() != 2) {
      semantic_error(ex.location, "Binary operator (" + to_string(ex.op) +
                     ") does not have two operands.");
    }
    checkDoubleExpression(ex.operands.at(0));
    checkDoubleExpression(ex.operands.at(1));
  } else if (ex.op == ast::UNARYMINUS) {
    if (ex.operands.size() != 1) {
      semantic_error(ex.location, "Unary operator (" + to_string(ex.op) +
                     ") does not have one operand.");
    }
    checkDoubleExpression(ex.operands.at(0));
  } else if (ex.op == ast::ITE) {
    if (ex.operands.size() != 3) {
      semantic_error(ex.location, "ITE node does not have three operands.");
    }
    checkBooleanExpression(ex.operands.at(0));
    checkDoubleExpression(ex.operands.at(1));
    checkDoubleExpression(ex.operands.at(2));
  } else if (ex.op == ast::NUMERAL) {
    if (ex.operands.size() != 0) {
      semantic_error(ex.location, "Numeral has operands.");
    }
  } else if (ex.op == ast::IDENTIFIER) {
    if (ex.operands.size() != 0) {
      semantic_error(ex.location, "Identifier has operands.");
    }
  } else if (ex.op == ast::MOD || ex.op == ast::POW) {
    if (ex.operands.size() != 2) {
      semantic_error(ex.location, "Binary function \"" + ex.identifier +
                     "\" does not have two operands.");
    }
    checkDoubleExpression(ex.operands.at(0));
    checkDoubleExpression(ex.operands.at(1));
  } else if (ex.op == ast::MIN || ex.op == ast::MAX) {
    if (ex.operands.size() == 0) {
      semantic_error(ex.location, ex.identifier +
                     " function called without operands.");
    }
    for (auto & op : ex.operands) {
      checkDoubleExpression(op);
    }
  } else if (ex.op == ast::CEIL || ex.op == ast::FLOOR) {
    if (ex.operands.size() != 1) {
      semantic_error(ex.location, "Unary function \"" + ex.identifier +
                     "\" does not have one operand.");
    }
    checkDoubleExpression(ex.operands.at(0));
  } else {
    semantic_error(ex.location, "Illegal operator (" + to_string(ex.op) +
                   ") in double expression.");
  }
}

void p_driver::buildModel()
{
  // Add the initial state to the model and return the number of decision
  // states (both reachable and unreachable).
  // Probabilistic nodes are numbered after all decision nodes.
  Node probnum = initialState();
  // Build transitions.
  size_t nIterations = 0;
  queue<Node> q{{model.getInitial()}};
  while (!q.empty()) {
    Node state = q.front();
    q.pop();
    Environment env = decode(state);
    if (verbosity > Verbosity::Silent) {
      if (++nIterations % 1000000 == 0) {
        size_t nnodes = model.numNodes();
        cout << "constructed " << nnodes << " states so far" << endl;
      }
      if (verbosity > Verbosity::Verbose) {
        cout << "state " << env << endl;
      }
    }
    // Collect commands enabled in this state.
    EnabledCommands enabled;
    collectEnabledCommands(env, enabled);
    // Add enabled transitions to this state.
    auto it = enabled.cbegin();
    while (it != enabled.cend()) {
      auto act = it->first;
      if (verbosity > Verbosity::Verbose) {
        cout << "  [" << model.getActionName(act) << "]";
      }
      auto const & lb = it->second.cbegin();
      auto const & ub = it->second.cend();
      buildTransitions(state, env, lb, ub, act, q, probnum);
      if (verbosity > Verbosity::Verbose) {
        cout << endl;
      }
      ++it;
    }
    if (modelType == ast::SMG) {
      // Figure out which player controls this state.
      Player player = -2; // impossible: will be replaced
      for (auto const & ec : enabled) {
        auto const ait = actionOwner.find(ec.first);
        if (ait == actionOwner.end()) {
          throw runtime_error("unknown action: " +
                              model.getActionName(ec.first));
        }
        if (player == -2) {
          player = ait->second;
        } else if (player != ait->second) {
          throw runtime_error("shared control in state " +
                              model.getNodeName(encode(env)));
        }
      }
      model.setNodePlayer(state, player);
    }
    
    if (modelType == ast::BDP) {
      model.setBDP();
      // Figure out which player controls this state.
      Player player = -2; // impossible: will be replaced
      for (auto const & ec : enabled) {
        auto const ait = actionOwner.find(ec.first);
        if (ait == actionOwner.end()) {
          throw runtime_error("unknown action: " +
                              model.getActionName(ec.first));
        }
        if (player == -2) {
          player = ait->second;
        } else if (player != ait->second) {
          throw runtime_error("shared control in state " +
                              model.getNodeName(encode(env)));
        }
      }
      model.setNodePlayer(state, player);
    }

    // Add rewards.
    for (auto const & rsection : rewards) {
      for (auto const & rs : rsection.specs) {
        addReward(state, rs, env);
      }
    }
  }

  // Add labels to nodes.
  for (auto const & l : labels) {
    string vname = l.name;
    BDD prop = model.getManager().bddVar();
    model.addAtomicProposition(prop, vname);
    for (auto nit = model.cbegin(); nit != model.cend(); ++nit) {
      Node state = nit->first;
      if (model.isDecisionNode(state)) {
        Environment env = decode(state);
        if (isEnabled(l.expression, env)) {
          model.addLabel(state, prop);
        }
      }
    }
  }
}

Node p_driver::initialState()
{
  Node nDecisionStates = 1;
  Environment initPart;
  for (auto const & v : varInfo) {
    nDecisionStates *= v.high - v.low + 1;
    initPart.push_back(v.initial);
  }
  if (verbosity > Verbosity::Terse) {
    cout << "number of decision states: " << nDecisionStates
         << " (both reachable and unreachable)" << endl;
  }

  Node initialState = encode(initPart);
  model.addDecisionNode(initialState, makeName(initPart));
  model.makeInitial(initialState);

  if (verbosity > Verbosity::Informative) {
    cout << "initial state: " << initialState << endl;
  }

  return nDecisionStates;
}

void p_driver::collectEnabledCommands(Environment const & env,
                                      EnabledCommands & commands)
{
  EnabledMap enabled;
  ActionModules enabledModules;
  for (size_t i = 0; i != modules.size(); ++i) {
    auto const & module = modules.at(i);
    for (size_t j = 0; j != module.commands.size(); ++j) {
      auto const & command = module.commands.at(j);
      if (isEnabled(command.guard, env)) {
        Action action = getActionNumber(command.action);
        if (Util::is_prefix(command.action, "_a")) {
          commands.push_back({action,{{i,j}}});
        } else {
          enabled.emplace(action, pair<size_t,size_t>{i,j});
          auto ait = enabledModules.find(action);
          if (ait == enabledModules.end()) {
            enabledModules.emplace(action, set<int>{(int) i});
          } else {
            auto & modSet = ait->second;
            modSet.insert((int) i);
          }
        }
      }
    }
  }
  // At this point enabled commands with anonymous actions are in "commands,"
  // while enabled commands with named actions are in "enabled."  For each
  // named action of an enabled command, we have collected the set of modules
  // in which there are enabled commands for that action.  If this set is
  // different from the one in actionModules, there is no synchronization
  // on that action.
  auto it = enabled.cbegin();
  while (it != enabled.cend()) {
    Action action = it->first;
    auto const lb = enabled.lower_bound(action);
    auto const ub = enabled.upper_bound(action);
    auto const & mset = enabledModules.at(action);
    int nmodules = mset.size();
    if (nmodules == actionCounts.at(action)) {
      // Named action is enabled.
      ptrdiff_t dist = distance(lb,ub);
      if (dist != nmodules) {
        throw logic_error("Nondeterministic named action \"" +
                          model.getActionName(action) + "\" in state " +
                          model.getNodeName(encode(env)));
      } else {
        CommandList ec;
        ec.reserve(dist);
        for (auto cit = lb; cit != ub; ++cit) {
          ec.push_back(cit->second);
        }
        commands.push_back({action, ec});
      }
    }
    it = ub;
  }
}

void p_driver::buildTransitions(
  Node state,
  Environment const & env,
  CommandList::const_iterator const & lb,
  CommandList::const_iterator const & ub,
  Action action,
  queue<Node> & q,
  Node & probnum) const
{
  // Synchronize all transitions from "state" that are enabled by "action."
  // Check whether we need a probabilistic node.
  bool probabilistic = false;
  for (auto rit = lb; rit != ub; ++rit) {
    size_t i = rit->first;  // module index
    size_t j = rit->second; // command index
    auto const & command = modules.at(i).commands.at(j);
    if (command.transitions.size() > 1) {
      for (auto const & t : command.transitions) {
        Probability prob = evaluateDoubleExpression(t.probability, env);
        if (prob != 0.0 && prob != 1.0) {
          probabilistic = true;
          break;
        }
      }
      if (probabilistic) break;
    }
  }
  // In case of probabilistic transitions, we need to keep track of the
  // decision state from which the transitions originate to retrieve the reward.
  Node source = state;
  if (probabilistic) {
    state = probnum;
    probnum++;
    model.addProbabilisticNode(state, string("p") + Util::to_decimal(state));
    model.addDecisionTransition(source, state, action);
  }
  VarValueStack vstack;
  buildTransitionsRecur(state, source, env, lb, ub, action, 1.0, q, vstack);
}

void p_driver::buildTransitionsRecur(
  Node state,
  Node source,
  Environment const & env,
  CommandList::const_iterator const & lb,
  CommandList::const_iterator const & ub,
  Action action,
  Probability probability,
  queue<Node> & q,
  VarValueStack & vstack) const
{
  if (lb == ub) {
    // We reached the bottom of the recursion.  In "env" we have the
    // values of the module variables in the origin state, in "vstack"
    // we have the changes from origin state to destination state, and
    // in "probability" we have the probability of transition, which is
    // 1.0 if the transition is deterministic (i.e., state == source).
    Environment newenv{env};
    for (auto const & p : vstack) {
      newenv[p.first] = p.second;
    }
    Node destination = encode(newenv);
    if (!model.isNode(destination)) {
      model.addDecisionNode(destination, makeName(newenv));
      q.push(destination);
    }
    if (state == source) {
      model.addDecisionTransition(state,destination,action);
    } else {
      model.addTransitionProbability(state,destination,probability);
    }
  } else {
    // Look at the transitions of another module.
    size_t i = lb->first;
    size_t j = lb->second;
    if (verbosity > Verbosity::Verbose) {
      cout << " " << lb->second;
    }
    auto const & command = modules.at(i).commands.at(j);
    for (auto const & t : command.transitions) {
      Probability newp = probability *
        evaluateDoubleExpression(t.probability, env);
      if (newp > 0) {
        size_t stacksize = vstack.size();
        updateDestination(t.destination, vstack, env);
        buildTransitionsRecur(state, source, env, next(lb), ub, action,
                              newp, q, vstack);
        vstack.resize(stacksize);
      }
    }
  }
}

void p_driver::updateDestination(
  ast::Expression const & ex,
  VarValueStack & vstack,
  Environment const & env) const
{
  if (ex.op == ast::EQUAL) {
    // Get variable index.
    size_t index = ex.inumeral;
    int dest;
    if (varInfo[index].type == ast::NUMBER) {
      dest = evaluateIntegerExpression(ex.operands[0],env);
      if (dest <  varInfo[index].low || dest > varInfo[index].high) {
        semantic_error(ex.location, string("Value ") + Util::to_decimal(dest) +
                       " is out of range for " + varInfo[index].name);
      }
    } else {
      dest = isEnabled(ex.operands[0],env) ? 1 : 0;
    }
    vstack.push_back({index,dest});
  } else if (ex.op == ast::AND) {
    updateDestination(ex.operands[0], vstack, env);
    updateDestination(ex.operands[1], vstack, env);
  } else if (ex.op != ast::TRUE) {
    throw logic_error("Unexpected operand in primed Boolean expression");
  }
}

Action p_driver::getActionNumber(string const & name)
{
  Action action;
  auto const it = actionMap.find(name);
  if (it == actionMap.end()) {
    action = actnum;
    actionMap.emplace(name, action);
    model.setActionName(action, name);
    actnum++;
  } else {
    action = it->second;
  }
  return action;
}

void p_driver::addReward(Node state, ast::RewardSpec const & rewardSpec,
                         Environment const & env)
{
  if (!isEnabled(rewardSpec.guard, env)) {
    return;
  }
  double reward = evaluateDoubleExpression(rewardSpec.reward, env);

  if (rewardSpec.type == ast::ACTION) {
    Action action = getActionNumber(rewardSpec.name);
    model.addActionStateReward(state, action, reward);
  } else if (rewardSpec.type == ast::STATE) {
    model.addStateReward(state, reward);
  } else {
    throw out_of_range("Illegal reward type");
  }
}

string p_driver::makeName(vector<int> const & parts) const
{
  string stateName;
  stateName.reserve(nameLengthLowerBound);
  for (size_t i = 0; i != varInfo.size(); ++i) {
    stateName += varInfo[i].name;
    stateName += Util::to_decimal(parts[i]);
  }
  return stateName;
}

Node p_driver::encode(Environment const & parts) const
{
  if (parts.size() != varInfo.size()) {
    throw logic_error("Size mismatch in encode");
  }
  Node x = 0;
  for (size_t i = 0; i != parts.size(); ++i) {
    VarInfo const & vi = varInfo[i];
    Node multiplier = vi.high - vi.low + 1;
    x = x * multiplier + (Node) (parts[i] - vi.low);
  }
  return x;
}

p_driver::Environment p_driver::decode(Node x) const
{
  Environment dec(varInfo.size());
  for (size_t i = varInfo.size(); i != 0; --i) {
    VarInfo const & vi = varInfo[i-1];
    Node multiplier = vi.high - vi.low + 1;
    dec[i-1] = (int) (x % multiplier) + vi.low;
    x /= multiplier;
  }
  return dec;
}

void p_driver::increment(Environment & p) const
{
  for (size_t i = varInfo.size(); i != 0; i--) {
    VarInfo const & vi = varInfo[i-1];
    p[i-1]++;
    if (p[i-1] > vi.high) {
      p[i-1] = vi.low;
    } else {
      return;
    }
  }
}

bool p_driver::isEnabled(ast::Expression const & ex,
                         Environment const & env) const
{
  switch(ex.op) {
  case ast::EQUAL: {
    int left = evaluateIntegerExpression(ex.operands.at(0),env);
    int right = evaluateIntegerExpression(ex.operands.at(1),env);
    return left == right;
  }
  case ast::NOTEQ: {
    int left = evaluateIntegerExpression(ex.operands.at(0),env);
    int right = evaluateIntegerExpression(ex.operands.at(1),env);
    return left != right;
  }
  case ast::LE: {
    int left = evaluateIntegerExpression(ex.operands.at(0),env);
    int right = evaluateIntegerExpression(ex.operands.at(1),env);
    return left <= right;
  }
  case ast::LT: {
    int left = evaluateIntegerExpression(ex.operands.at(0),env);
    int right = evaluateIntegerExpression(ex.operands.at(1),env);
    return left < right;
  }
  case ast::GE: {
    int left = evaluateIntegerExpression(ex.operands.at(0),env);
    int right = evaluateIntegerExpression(ex.operands.at(1),env);
    return left >= right;
  }
  case ast::GT: {
    int left = evaluateIntegerExpression(ex.operands.at(0),env);
    int right = evaluateIntegerExpression(ex.operands.at(1),env);
    return left > right;
  }
  case ast::IDENTIFIER: {
    if (ex.inumeral < 0) {
      auto cit = boolConstantMap.find(ex.identifier);
      if (cit == boolConstantMap.end()) {
        semantic_error(ex.location, "Unknown bool identifier: " + ex.identifier);
      } else {
        return cit->second != 0;
      }
    } else {
      return env.at(ex.inumeral);
    }
  }
  case ast::AND:
    return isEnabled(ex.operands.at(0),env) && isEnabled(ex.operands.at(1),env);
  case ast::OR:
    return isEnabled(ex.operands.at(0),env) || isEnabled(ex.operands.at(1),env);
  case ast::NOT:
    return !isEnabled(ex.operands.at(0),env);
  case ast::ITE:
    return isEnabled(ex.operands.at(0),env)
      ? isEnabled(ex.operands.at(1),env)
      : isEnabled(ex.operands.at(2),env);
  case ast::XOR:
    return isEnabled(ex.operands.at(0),env) ^ isEnabled(ex.operands.at(1),env);
  case ast::IFF:
    return isEnabled(ex.operands.at(0),env) ^ !isEnabled(ex.operands.at(1),env);
  case ast::IMPLIES:
    return !isEnabled(ex.operands.at(0),env) || isEnabled(ex.operands.at(1),env);
  case ast::TRUE:
    return true;
  case ast::FALSE:
    return false;
  default:
    semantic_error(ex.location, "Unsupported operator in command guard");
  }
}

int p_driver::evaluateIntegerExpression(
  ast::Expression const & ex,
  Environment const & env) const
{
  if (ex.op == ast::NUMERAL) {
    if (ex.type != ast::NUMBER) {
      semantic_error(ex.location, "Non-integer value in integer expression");
    }
    return ex.inumeral;
  } else if (ex.op == ast::TRUE || ex.op == ast::FALSE) {
    return ex.inumeral;
  } else if (ex.op == ast::IDENTIFIER) {
    if (ex.inumeral < 0) {
      auto cit = intConstantMap.find(ex.identifier);
      if (cit == intConstantMap.end()) {
        semantic_error(ex.location, "Unknown integer identifier: " +
                       ex.identifier);
      } else {
        return cit->second;
      }
    } else {
      return env.at(ex.inumeral);
    }
  } else if (ex.op == ast::ITE) {
    bool guard = isEnabled(ex.operands.at(0), env);
    int leftValue = evaluateIntegerExpression(ex.operands.at(1), env);
    int rightValue = evaluateIntegerExpression(ex.operands.at(2), env);
    return guard ? leftValue : rightValue;
  } else {
    int leftValue = evaluateIntegerExpression(ex.operands.at(0), env);
    if (ex.op == ast::UNARYMINUS) {
      return -leftValue;
    } else if (ex.op == ast::MIN) {
      for (size_t i = 1; i != ex.operands.size(); ++i) {
        int value = evaluateIntegerExpression(ex.operands.at(i), env);
        if (value < leftValue) {
          leftValue = value;
        }
      }
      return leftValue;
    } else if (ex.op == ast::MAX) {
      for (size_t i = 1; i != ex.operands.size(); ++i) {
        int value = evaluateIntegerExpression(ex.operands.at(i), env);
        if (value > leftValue) {
          leftValue = value;
        }
      }
      return leftValue;
    }
    int rightValue = evaluateIntegerExpression(ex.operands.at(1), env);
    if (ex.op == ast::PLUS) {
      return leftValue + rightValue;
    } else if (ex.op == ast::MINUS) {
      return leftValue - rightValue;
    } else if (ex.op == ast::TIMES) {
      return leftValue * rightValue;
    } else if (ex.op == ast::DIVIDE) {
      return leftValue / rightValue;
    } else if (ex.op == ast::MOD) {
      return leftValue % rightValue;
    } else if (ex.op == ast::POW) {
      return pow(leftValue, rightValue);
    } else {
      semantic_error(ex.location, "Unsupprted or unknown integer operator");
    }
  }
}

double p_driver::evaluateDoubleExpression(
  ast::Expression const & ex,
  Environment const & env) const
{
  if (ex.op == ast::NUMERAL) {
    return ex.dnumeral;
  } else if (ex.op == ast::IDENTIFIER) {
    if (ex.inumeral < 0) {
      auto it = doubleConstantMap.find(ex.identifier);
      if (it == doubleConstantMap.end()) {
        // Allow typecast from int to double.
        auto iit = intConstantMap.find(ex.identifier);
        if (iit == intConstantMap.end()) {
          semantic_error(ex.location, "Unknown double identifier: " + ex.identifier);
        } else {
          return (double) iit->second;
        }
      } else {
        return (double) it->second;
      }
    }
    return (double) env.at(ex.inumeral);
  } else if (ex.op == ast::ITE) {
    bool guard = isEnabled(ex.operands.at(0), env);
    double leftValue = evaluateDoubleExpression(ex.operands.at(1), env);
    double rightValue = evaluateDoubleExpression(ex.operands.at(2), env);
    return guard ? leftValue : rightValue;
  } else {
    double leftValue = evaluateDoubleExpression(ex.operands.at(0), env);
    if (ex.op == ast::UNARYMINUS) {
      return -leftValue;
    } else if (ex.op == ast::CEIL) {
      return ceil(leftValue);
    } else if (ex.op == ast::FLOOR) {
      return floor(leftValue);
    } else if (ex.op == ast::MIN) {
      for (size_t i = 1; i != ex.operands.size(); ++i) {
        double value = evaluateDoubleExpression(ex.operands.at(i), env);
        if (value < leftValue) {
          leftValue = value;
        }
      }
      return leftValue;
    } else if (ex.op == ast::MAX) {
      for (size_t i = 1; i != ex.operands.size(); ++i) {
        double value = evaluateDoubleExpression(ex.operands.at(i), env);
        if (value > leftValue) {
          leftValue = value;
        }
      }
      return leftValue;
    }
    double rightValue = evaluateDoubleExpression(ex.operands.at(1), env);
    if (ex.op == ast::PLUS) {
      return leftValue + rightValue;
    } else if (ex.op == ast::MINUS) {
      return leftValue - rightValue;
    } else if (ex.op == ast::TIMES) {
      return leftValue * rightValue;
    } else if (ex.op == ast::DIVIDE) {
      return leftValue / rightValue;
    } else if (ex.op == ast::POW) {
      return pow(leftValue,rightValue);
    } else if (ex.op == ast::LOG) {
      return log(leftValue) / log(rightValue);
    } else {
      semantic_error(ex.location, "Unsupprted or unknown double operator");
    }
  }
}


namespace ast {

  ostream & operator<<(ostream & os, Operator const & op)
  {
    switch (op) {
    case PLUS:       os << "+";     break;
    case MINUS:      os << "-";     break;
    case TIMES:      os << "*";     break;
    case DIVIDE:     os << "/";     break;
    case UNARYMINUS: os << "-";     break;
    case TRUE:       os << "true";  break;
    case FALSE:      os << "false"; break;
    case OR:         os << "|";     break;
    case AND:        os << "&";     break;
    case XOR:        os << "^";     break;
    case IFF:        os << "<=>";   break;
    case IMPLIES:    os << "=>";    break;
    case NOT:        os << "!";     break;
    case EQUAL:      os << "=";     break;
    case NOTEQ:      os << "!=";    break;
    case LT:         os << "<";     break;
    case LE:         os << "<=";    break;
    case GT:         os << ">";     break;
    case GE:         os << ">=";    break;
    default:         os << "unknown operator";
    }
    return os;
  }

  string to_string(Operator const & op) {
    ostringstream oss;
    oss << op;
    return oss.str();
  }

  ostream & operator<<(ostream & os, Expression const & e)
  {
    if (e.op == NUMERAL) {
      if (e.type == NUMBER) {
        os << e.inumeral;
      } else {
        os << e.dnumeral;
      }
    } else if (e.op == TRUE || e.op == FALSE) {
      os << e.op;
    } else if (e.op == IDENTIFIER) {
      os << e.identifier;
    } else if (e.op == MOD || e.op == POW || e.op == MIN || e.op == MAX ||
               e.op == LOG || e.op == CEIL || e.op == FLOOR) {
      os << e.identifier << '(';
      char sep[] = ", ";
      char const * psep = sep + 2;
      for (auto const & operand : e.operands) {
        os << psep << operand;
        psep = sep;
      }
      os << ')';
    } else if (e.op == ITE) {
      os << '(' << e.operands.at(0) << " ? " << e.operands.at(1) << " : "
         << e.operands.at(2) << ')';
    } else {
      os << '(';
      if (e.op == UNARYMINUS || e.op == NOT) {
        os << e.op << e.operands.at(0);
      } else if (e.op == EQUAL && e.operands.size() == 1) {
        // next state expression
        os << e.identifier << "\' " << e.op << ' ' << e.operands.at(0);
      } else {
        os << e.operands.at(0) << ' ' << e.op << ' ' << e.operands.at(1);
      }
      os << ')';
    }
    return os;
  }

  ostream & operator<<(ostream & os, Variable const & v)
  {
    os << v.name;
    if (v.type == NUMBER) {
      os << " : [" << v.low << ".." << v.high << "] init ";
    } else {
      os << " : bool init ";
    }
    os << v.initial << ";";
    return os;
  }

  ostream & operator<<(ostream & os, Formula const & f)
  {
    os << "formula \"" << f.name << "\" = " << f.expression << ";";
    return os;
  }

  ostream & operator<<(ostream & os, Label const & l)
  {
    os << "label \"" << l.name << "\" = " << l.expression << ";";
    return os;
  }

  ostream & operator<<(ostream & os, Constant const & c)
  {
    if (c.type == DOUBLE) {
      os << "const double ";
    } else if (c.type == NUMBER) {
      os << "const int ";
    } else {
      os << "const bool ";
    }
    os << c.name << " = " << c.expression << ";";
    return os;
  }

  ostream & operator<<(ostream & os, Transition const & t)
  {
    os << t.probability << " : " << t.destination;
    return os;
  }

  ostream & operator<<(ostream & os, Command const & c)
  {
    os << "[" << c.action << "] " << c.guard << " -> ";
    if (c.transitions.size() > 1) {
      char sep[] = " + ";
      char const * psep = sep + 3;
      for (auto const & t : c.transitions) {
        os << psep << t;
        psep = sep;
      }
    } else {
      os << c.transitions.at(0).destination;
    }
    os << ";";
    return os;
  }

  ostream & operator<<(ostream & os, Module const & m)
  {
    os << "module " << m.name << "\n";
    for (auto const & v : m.variables) {
      os << "  " << v << "\n";
    }
    for (auto const & c : m.commands) {
      os << "  " << c << "\n";
    }
    os << "endmodule";
    return os;
  }

  ostream & operator<<(ostream & os, NameReplacement const & nr)
  {
    os << nr.oldname << '=' << nr.newname;
    return os;
  }

  ostream & operator<<(ostream & os, ModuleRenaming const & mr)
  {
    os << "module " << mr.newname << '='
       << mr.oldname << '[';
    for (auto const & nr : mr.replacements) {
      os << ' ' << nr;
    }
    os << " ] endmodule";
    return os;
  }

  ostream & operator<<(ostream & os, RewardSpec const & rs)
  {
    if (rs.type == ACTION) {
      os << "[" << rs.name << "] ";
    }
    os << rs.guard << " : " << rs.reward << ";";
    return os;
  }
  
  ostream & operator<<(ostream & os, RewardSection const & rv)
  {
    os << "rewards";
    if (rv.name != string("")) {
      os << " \"" << rv.name << "\"";
    }
    os << "\n";
    for (auto const & rs : rv.specs) {
      os << "  " << rs << "\n";
    }
    os << "endrewards";
    return os;
  }

  ostream & operator<<(ostream & os, ControlSpec const & cs)
  {
    if (cs.type == PMODULE) {
      os << cs.name;
    } else {
      os << "[" << cs.name << "]";
    }
    return os;
  }

  ostream & operator<<(ostream & os, Player const & p)
  {
    os << "player " << p.name << "\n  ";
    char const sep[] = ", ";
    char const * psep = sep + 2;
    for (auto const & cs : p.controls) {
      os << psep << cs;
      psep = sep;
    }
    os << "\nendplayer";
    return os;
  }

}
