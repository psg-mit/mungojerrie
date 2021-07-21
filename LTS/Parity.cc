/** @file Parity.cc

  @brief Parity automata.

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
#include <stdexcept>
#include <queue>
#include <fstream>
#include "Util.hh"
#include "Parity.hh"
#include "Stree.hh"
#include "Hparser.hh"

using namespace std;

Parity::Parity(Cudd mgr) : LTS(mgr) {}


Parity::Parity(LTS const & lts) : LTS(lts) {}


Parity::Parity(Cudd mgr, std::string const & filename,
               Verbosity::Level verbosity, bool noMdp) : LTS(mgr)
{
  Hparser(*this, filename, verbosity, noMdp);
  makeComplete();
  if (getMaxPriority() > 1 && isDeterministic()) {
    minimumIndex();
  }
  pruneVarSet();
}

Parity::Parity(Cudd mgr, std::istream & is,
               Verbosity::Level verbosity, bool noMdp) : LTS(mgr)
{
  Hparser(*this, is, verbosity, noMdp);
  makeComplete();
  if (getMaxPriority() > 1 && isDeterministic()) {
    minimumIndex();
  }
  pruneVarSet();
}


bool ParityTransition::operator==(ParityTransition const & other) const
{
  return (destination == other.destination) &&
    (label.getNode() == other.label.getNode()) && (priority == other.priority);
}


bool ParityTransition::operator<(ParityTransition const & other) const
{
  if (destination < other.destination) return true;
  if (destination > other.destination) return false;
  if (label.getNode() < other.label.getNode()) return true;
  if (label.getNode() > other.label.getNode()) return false;
  return priority < other.priority;
}


ostream & operator<<(ostream & os, ParityTransition const & p)
{
  os << '[' << p.label << "] " << p.destination
     << " {" << p.getPriority() << '}';
  return os;
}


void Parity::addTransition(State source, State destination,
                           BDD label, Priority priority)
{
  Transition tran{destination, label, priority};
  auto it = transitions.find(source);
  if (it == transitions.end()) {
    Delta delta(1, tran);
    transitions.insert({source, delta});
  } else {
    it->second.push_back(tran);
  }
}


Priority Parity::getPriority(State source, BDD letter,
                             State destination, bool silent) const
{
  auto sit = transitions.find(source);
  if (sit == transitions.end()) {
    throw logic_error("missing transitions");
  }
  for (auto const & t : sit->second) {
    if (destination == t.destination && letter <= t.label) {
      return t.priority;
    }
  }
  if (silent) {
    return -1;
  } else {
    ostringstream os;
    os << letter;
    throw domain_error("no matching transition for (" + to_string(source) +
                       "," + os.str() + "," + to_string(destination) + ")");
  }
}


Priority Parity::getMaxPriority(void) const
{
  Priority maxpriority = 0;
  for (auto const & sd : transitions) {
    for (Transition const & t : sd.second) {
      if (t.priority > maxpriority) {
        maxpriority = t.priority;
      }
    }
  }
  return maxpriority;
}


Priority Parity::getMinPriority(void) const
{
  Priority minpriority = 0;
  bool first = true;
  for (auto const & sd : transitions) {
    for (Transition const & t : sd.second) {
      if (t.priority < minpriority || first) {
        minpriority = t.priority;
        first = false;
      }
    }
  }
  return minpriority;
}


vector<Parity::EdgeSet> Parity::edgePartition(void) const
{
  vector<EdgeSet> partition(getMaxPriority()+1);
  for (State s : states) {
    auto it = transitions.find(s);
    if (it != transitions.end()) {
      for (auto const & t : it->second) {
        auto & pmap = partition[t.priority];
        pmap.insert({s,t});
      }
    }
  }
  return partition;
}

ostream & operator<<(ostream & os, Parity const & p)
{
  p.print(os);
  return os;
}


void Parity::printDot(string graphname, string filename, bool showtrees) const
{
  printDot(graphname, DotAttributes{}, filename, showtrees);
}

namespace {
  string nodeAttributes(State state, Parity::DotAttributes const & attributes) {
    ostringstream os;
    auto sit = attributes.nodeShapes.find(state);
    if (sit != attributes.nodeShapes.end()) {
      os << ",shape=\"" << sit->second << "\"";
    }
    auto cit = attributes.nodeColors.find(state);
    if (cit != attributes.nodeColors.end()) {
      os << ",color=\"" << cit->second << "\"";
    }
    return os.str();
  }

  string nodeLabel(State state, Parity::DotAttributes const & attributes) {
    ostringstream os;
    os << state;
    auto lit = attributes.nodeLabels.find(state);
    if (lit != attributes.nodeLabels.end()) {
      os << ':' << lit->second;
    }
    return os.str();
  }

  string edgeAttributes(State source, State destination,
                        Parity::DotAttributes const & attributes) {
    ostringstream os;
    auto cit = attributes.edgeColors.find({source,destination});
    if (cit != attributes.edgeColors.end()) {
      os << ",color=\"" << cit->second << "\"";
    }
    return os.str();
  }
}

void Parity::printDot(string graphname, DotAttributes const & attributes, string filename, bool showtrees) const
{
  streambuf * buf;
  ofstream of;

  if (filename == "-") {
    buf = cout.rdbuf();
  } else {
    of.open(filename);
    if (!of.is_open())
      throw logic_error("Cannot open file " + filename + ".");
    buf = of.rdbuf();
  }

  ostream ofs(buf);

  if (showtrees)
    throw domain_error("showtrees not yet implemented");

  Util::replaceAll(graphname, "\n", "\\n");
  ofs << "digraph \"" << graphname << "\" {\nnode [shape=ellipse];\n"
       << "\"title\" [label=\"" << graphname << "\",shape=plaintext];\n";

  for (State i : states) {
    if (i == initialState) {
      ofs << "\"" << i << "_init\" [style=invis]\n\"" << i
           << "_init\" -> " << i << "\n";
    }
    ofs << i << " [label=\"" << nodeLabel(i, attributes)
         << "\""  << nodeAttributes(i, attributes) << "];\n";
  }
  for (State i : states) {
    auto it = transitions.find(i);
    if (it == transitions.end()) continue;
    Delta const & delta = it->second;
    for (Transition const & t : delta) {
      ofs << i << " -> " << t.destination << " [label=\"" << t.label
           << "\\n(" << t.priority << ")\""
           << edgeAttributes(i, t.destination, attributes) << "];\n";
    }
  }
  ofs << "}" << endl;

  if (filename != "-") {
    of.close();
  }
} // Parity::printDot


namespace {
  string changeNamesInLabel(BDD const & label, map<string,string> const & rep)
  {
    ostringstream os;
    os << label;
    string str{os.str()};
    for (auto it = rep.cbegin(); it != rep.cend(); ++it) {
      Util::replaceAll(str, it->first, it->second);
    }
    return str;
  }

  string acceptanceString(unsigned n)
  {
    if (n <= 2) { // special case to avoid needless parentheses
      return "Inf(1) | Fin(0)";
    }
    if (n % 2 == 0) {
      return "Inf(" + to_string(n-1) + ") | (" + acceptanceString(n-1) + ')';
    } else {
      return "Fin(" + to_string(n-1) + ") & (" + acceptanceString(n-1) + ')';
    }
  }
}


void Parity::printHOA(string const & name, string filename) const
{
  streambuf * buf;
  ofstream of;

  if (filename == "-") {
    buf = cout.rdbuf();
  } else {
    of.open(filename);
    if (!of.is_open())
      throw logic_error("Cannot open file " + filename + ".");
    buf = of.rdbuf();
  }

  ostream ofs(buf);

  // Map state "names" to indices.
  map<State,size_t> stIndices;
  for (size_t i = 0; i != states.size(); ++i) {
    stIndices.insert({states.at(i), i});
  }
  unsigned numPriorities = getMaxPriority()+1;
  // Write header.
  ofs << "HOA: v1\n";
  ofs << "name: \"" << name << "\"\n";
  ofs << "States: " << numStates() << '\n';
  if (states.size() > 0) {
    ofs << "Start: " << stIndices.at(getInitial()) << '\n';
  }
  ofs << "acc-name: parity max odd " << numPriorities << '\n';
  ofs << "Acceptance: " << max(numPriorities, 2u) << ' '
       << acceptanceString(numPriorities) << '\n';
  ofs << "properties: " << (isDeterministic() ? "deterministic " : "")
       << (isComplete() ? "complete " : "") << "colored\n";
  ofs << "properties: trans-acc trans-labels explicit-labels\n";
  map<string,string> replacements{{"true","t"}, {"false","f"}};
  size_t vindex = 0;
  ofs << "AP: " << varSet.size() + hasEpsilonTransitions();
  for (auto it = varSet.cbegin(); it != varSet.cend(); ++it, ++vindex) {
    ostringstream os;
    os << *it;
    string name{os.str()};
    ofs << " \"" << name << "\"";
    replacements.insert({name,to_string(vindex)});
  }
  if (hasEpsilonTransitions()) {
    ofs << " \"epsilon\"";
    replacements.insert({string("epsilon"),to_string(vindex)});
  }
  ofs << "\n--BODY--\n";
  for(State s : states) {
    ofs << "State: " << stIndices.at(s) << " \"" << s << "\"\n";
    auto dit = transitions.find(s);
    if (dit != transitions.end()) {
      auto const & delta = dit->second;
      for (auto const & t : delta) {
        ofs << "  [" << changeNamesInLabel(t.label, replacements) << "] "
             << stIndices.at(t.destination) << " {" << t.priority << "}\n";
      }
    }
  }
  ofs << "--END--\n";
  
  if (filename != "-") {
    of.close();
  }
} // Parity::printHOA


/**
   @brief Helper class to perform DPW determinization.
*/
class Parity::Determ {
public:
  Determ(Parity const & NPW, vector<Stree> & trees) : NPW(NPW), trees(trees) {}
  void computeNextTree(State source, BDD const & letter,
                       Stree & newt, int & color);
  bool findOrAdd(Stree const & newtree, int & reftree);
  void mergeOrAdd(Delta & delta, BDD const & label,
                  State successor, Priority color);
private:
  Parity const & NPW;
  vector<Stree> & trees;

}; // Parity::Determ


void Parity::Determ::computeNextTree(State source, BDD const & letter,
                                     Stree & newt, int & color)
{
  Stree const & tree = trees.at(source);
  newt = tree;
  // Compute order in which the tree nodes will be visited.
  vector<int> order;
  newt.preorder(order);

  // 1. Replace the label of each node with the result of applying
  //    the transition function of the nondeterministic automaton.
  for (int node : order) {
    StateSet const & currentSubset = newt.getLabel(node);
    StateSet nextSubset;
    StateSet acceptSubset;
    for (State state : currentSubset) {
      auto it = NPW.transitions.find(state);
      if (it != NPW.transitions.end()) {
        Delta const & delta = it->second;
        for (auto const & trans : delta) {
          if (letter <= trans.label) {
            nextSubset.insert(trans.destination);
            if (trans.priority == 1) {
              acceptSubset.insert(trans.destination);
            }
          }
        }
      }
    }
    newt.setLabel(node, nextSubset);
    // 2. Spawn fair subsets of each node.
    if (!acceptSubset.empty()) {
      newt.addLastChild(node, acceptSubset);
    }
  }

  // 3. Shift states to siblings to the left.
  newt.restrictLabels();

  // 4. Remove the descendants of all nodes that flash green.  A node flashes
  //    green if its label is the union of the labels of its children.
  int f = NPW.states.size();  // Piterman says n+1, but he starts from 1.
  // We use the old order because the newly added nodes have no children;
  // hence they cannot flash green.
  for (int node : order) {
    if (newt.isGreen(node)) {
      if (node < f) f = node;
      newt.removeDescendants(node);
    }
  }

  // 5. Remove all nodes with empty labels, except possibly the root
  //    of the tree.
  int e = NPW.states.size();
  newt.preorder(order); // order has changed since it was last computed.
  for (int node : order) {
    if (newt.getLabel(node).empty()) {
      if (node < e) e = node;
      newt.removeSubtree(node);
    }
  }

  // 6. Renumber nodes.
  newt.makeContiguous();

  // Find color of new tree/state.
  if (f < e)
    color = 2 * (NPW.states.size() - f) - 1;
  else if (e == 0)
    color = 0;
  else
    color = 2 * (NPW.states.size() - e);

  if (NPW.verbosity > 1)
    cout << "f=" << f << ", e=" << e << " ";

} // Parity::Determ::computeNextTree


bool Parity::Determ::findOrAdd(
  Stree const & newtree, int & reftree)
{
  for (int tindex = 0; tindex != (int) trees.size(); ++tindex) {
    Stree const & t = trees.at(tindex);
    if (newtree.isIsomorphic(t)) {
      reftree = tindex;
      return true;
    }
  }
  reftree = trees.size();
  trees.push_back(newtree);
  return false;

} // Parity::Determ::findOrAdd


void Parity::Determ::mergeOrAdd(Delta & delta, BDD const & label,
                                State successor, Priority color)
{
  // Inductively, there's at most another transition with the same destination
  // and the same priority.
  for (auto & tran : delta) {
    if (tran.destination == successor && tran.priority == color) {
      tran.label |= label;
      return;
    }
  }
  delta.push_back(Transition{successor, label, color});

} // Parity::Determ::mergeOrAdd


/**
 * Build a deterministic parity automaton from a nondeterministic parity
 * automaton with prioritiess 0 and 1 (that is, a nondeterminstic Buechi
 * automaton).  Uses Piterman's algorithm.
 */
Parity Parity::determinize(void) const {

  int index = states.size() * 2;
  // If there are no states, return an automaton that accepts nothing.
  if (index == 0) {
    Parity DPW(mgr);
    DPW.addState(0);
    DPW.makeInitial(0);
    DPW.addTransition(0,0,mgr.bddOne(),0);
    return DPW;
  }
  Parity DPW(mgr);
  DPW.setVerbosity(verbosity);
  vector<Stree> trees;
  Determ innards(*this, trees);

  // Create initial state: the tree has one nodee (the singleton with the
  // initial state of the NBW) and "undef" parent.  It is State 0.
  Stree itree;
  StateSet itreelabel;
  itreelabel.insert(initialState);
  itree.addLastChild(Stree::undef, itreelabel);
  trees.push_back(itree);

  DPW.makeInitial(0);          // the initial tree we just built
  DPW.varSet = varSet;         // same alphabet as the NBW

  // Initialize queue.
  queue<int> q;
  q.push(0);

  while (!q.empty()) {
    int tindex = q.front();
    q.pop();
    Stree t = trees[tindex];
    if (verbosity > 0) {
      cout << "\nTree " << tindex << ": " << t << endl;
    }
    Delta delta;
    vector<BDD> letters;
    if (t.size() > 0) {
      StateSet const & allstates = t.getLabel(0);
      this->letters(allstates, letters);
    } else {
      letters.push_back(mgr.bddOne());
    }
    for (BDD const & ltr : letters) {
      Stree newt;
      int color, reftree;
      innards.computeNextTree(tindex, ltr, newt, color);
      bool found = innards.findOrAdd(newt, reftree);
      if (!found) {
        q.push(reftree);
      }
      if (verbosity > 0) {
        cout << "Letter: " << ltr << " reftree(" << (found ? "old" : "new")
             << ") " << trees.at(reftree) << " color: " << color << endl;
      }
      innards.mergeOrAdd(delta, ltr, reftree, color);
    }
    DPW.addState(tindex, delta);
  }

  return DPW;

} // Parity::determinize


// When we call this function, the maximum priority has its minimum value
// and there are no gaps.  However, priority 0 may still be in use even
// if it's not needed.  Here we heuristically attempt to fix the problem.
// The function returns true if and only if it changes one or more priorities.
bool Parity::zeroToTwo(unsigned maxpriority)
{
  bool raised = false;
  if (maxpriority < 2) {
    return raised;
  }
  // Find minimum priority of transitions into each state excluding self loops.
  map<State,unsigned> minInPriority;
  for (State state : states) {
    auto dit = transitions.find(state);
    if (dit != transitions.end()) {
      auto const & delta = dit->second;
      for (auto const & t : delta) {
        State destination = t.destination;
        if (destination == state) continue; // ignore self-loops
        unsigned priority = t.priority;
        auto mit = minInPriority.find(destination);
        if (mit == minInPriority.end()) {
          minInPriority.emplace(destination, priority);
        } else if (priority < mit->second) {
          mit->second = priority;
        }
      }
    }
  }
  // Hoist priority from 0 to 2 if minInPriority >= 2.
  for (State state : states) {
    auto dit = transitions.find(state);
    if (dit != transitions.end()) {
      auto & delta = dit->second;
      auto mit = minInPriority.find(state);
      if (mit != minInPriority.end()) {
        if (mit->second < 2) {
          // If there is only one self loop out of the state, we can hoist
          // from 0 to 2 even though the incoming priority is lower than 2.
          if (delta.size() != 1 || delta.at(0).destination != state) {
            continue;
          }
        }
      }
      for (auto & t : delta) {
        if (t.priority == 0) {
          // Heuristically try to homogenize priorities.
          Priority homogenized = 2;
          State dest = t.destination;
          if (dest != state) {
            auto sit = transitions.find(dest);
            if (sit != transitions.end()) {
              auto const & ddelta = sit->second;
              for (auto & dt : ddelta) {
                if (dt.label == t.label && dt.priority <= 2) {
                  homogenized = dt.priority;
                  break;
                }
              }
            }
          }
          t.priority = homogenized;
          raised = true;
        }
      }
    }
  }
  return raised;

} // Parity::zeroToTwo


Priority Parity::CartonMaceirasRecur(
  StateSet const & Q, EdgeSet forbiddenEdges)
{
  // Decompose Q into SCCs.
  SCCs SCCs = getSCCs(Q, Q, forbiddenEdges);
  if (verbosity)
    cout << "SCCs: " << SCCs << endl;
  // m is the length of the longest positive chain in Q.
  // It starts at 0 because we haven't found any positive chains yet.
  Priority m = 0;
  vector<State> transient;
  for (StateSet const & scc : SCCs) {
    if (isTrivial(scc, forbiddenEdges)) {
      // This is a heuristic to exploit the freedom
      // in priority for transient states.
      transient.push_back(*scc.begin());
      continue;
    }
    Priority mu = 0;
    // Find maximum priority in this SCC.
    Priority maxpriority = 0;
    for (State s : scc) {
      // Nontrivial SCC: all states have outgoing transitions.
      Delta const & delta = transitions.find(s)->second;
      for (auto const & t : delta) {
        if (forbiddenEdges.find({s,t}) != forbiddenEdges.end()) continue;
        // The priority of an edge that leads out of the SCC does not
        // contribute to pi(scc).
        if (scc.find(t.destination) == scc.end()) continue;
        Priority thispriority = t.priority;
        if (thispriority > maxpriority) {
          maxpriority = thispriority;
        }
      }
    }
    if (maxpriority > 0) {
      // Collect edges of maximum priority.
      EdgeSet newForbidden;
      for (State s : scc) {
        Delta const & delta = transitions.find(s)->second;
        for (auto const & t : delta) {
          if (forbiddenEdges.find({s,t}) != forbiddenEdges.end()) continue;
          if (scc.find(t.destination) == scc.end()) continue;
          if (t.priority == maxpriority) {
            newForbidden.insert({s,t});
          }
        }
      }
      forbiddenEdges.insert(newForbidden.begin(), newForbidden.end());
      mu = CartonMaceirasRecur(scc, forbiddenEdges);
      mu += (maxpriority - mu) & 1;
      for (Edge e : newForbidden) {
        State state = e.first;
        Transition & transition = e.second;
        Delta & delta = transitions.find(state)->second;
        for (auto & t : delta) {
          if (t == transition) {
            t.priority = mu;
          }
        }
      }
    } else {
      for (State s : scc) {
        Delta & delta = transitions.find(s)->second;
        for (auto & t : delta) {
          if (forbiddenEdges.find({s,t}) == forbiddenEdges.end()) {
            t.priority = 0;
          }
        }
      }
    }
    m = max(m, mu);
  }
  for (State s : transient) {
    auto dit = transitions.find(s);
    if (dit != transitions.end()) {
      auto & delta = dit->second;
      for (auto & t : delta) {
        if (forbiddenEdges.find({s,t}) == forbiddenEdges.end()) {
          t.priority = m;
        }
      }
    }
  }
  return m;

} // Parity::CartonMaceirasRecur


unsigned Parity::CartonMaceiras(void)
{
  StateSet stateset(states.begin(), states.end());
  unsigned maxposchain = CartonMaceirasRecur(stateset, EdgeSet{});
  //(void) zeroToTwo(maxposchain);
  return maxposchain;

} // Parity::CartonMaceiras


unsigned Parity::minimumIndex(void)
{
  if (!isDeterministic()) {
    throw logic_error("Index minimization attempted for NPW");
  }
  unsigned maxposchain = CartonMaceiras();
  Parity CDPW{*this};
  for (auto & sd : CDPW.transitions) {
    for (Transition & t : sd.second) {
      t.priority++;
    }
  }
  // Try to avoid priority 0.
  unsigned maxnegchain = CDPW.CartonMaceiras();
  if (maxnegchain < maxposchain) {
    for (auto & sd : CDPW.transitions) {
      for (Transition & t : sd.second) {
        t.priority++;
      }
    }
    *this = CDPW;
  }
  return maxposchain;

} // Parity::minimumIndex


Parity Parity::toLDPW(bool epsilonJumps) const
{
  if (!isDeterministic()) {
    throw logic_error("Conversion from NPW to LDPW not supported");
  }
  Priority maxPriority = getMaxPriority();
  if (maxPriority <= 1) {
    // Already in the desired (Buechi) form: return a copy of this DPW.
    return *this;
  }
  // All odd priorities are used if the automaton is of minimum index.
  Priority maxOddPriority = maxPriority - ((maxPriority & 1) == 0);
  // Set nstates to one more than the max state number.  Since there
  // may be holes in the state numbering, plain numStates is not OK.
  size_t nstates = numStates();
  for (State s : states) {
    if (s >= nstates) nstates = s + 1;
  }

  // State-numbering lambda.  Argument priority should be 0 or odd.
  auto numberState = [nstates] (State state, Priority priority) {
    return nstates * ((priority + 1) / 2) + state;
  };

  // Initialize limit-deterministic automaton.
  Parity NPW(mgr);
  NPW.verbosity = verbosity;
  NPW.varSet = varSet;
  // Build map from variable names to BDD variables.
  using NameVarMap = map<string,BDD>;
  NameVarMap nameVarMap;
  for (int index = 0; index != mgr.ReadSize(); ++index) {
    if (mgr.hasVariableName(index)) {
      nameVarMap.emplace(mgr.getVariableName(index), mgr.bddVar(index));
    }
  }

  BDD evar;
  if (epsilonJumps) {
    // Add epsilon atomic proposition for jumps if not yet present.
    auto eit = nameVarMap.find("epsilon");
    if (eit == nameVarMap.end()) {
      evar = mgr.bddVar();
      mgr.pushVariableName("epsilon");
    } else {
      evar = eit->second;
    }
    NPW.setEpsilonBDD(evar);
  }

  // Create pancake stack.  One pancake for waiting (priority 0) and one
  // pancake for every odd priority.
  for (Priority pri = 0; pri <= maxOddPriority; pri += 1 + (pri > 0)) {
    for (State s : states) {
      // Create new state.
      State newstate = numberState(s, pri);
      NPW.addState(newstate);
      if (epsilonJumps && pri > 0) {
        // Add jump: a non-accepting transition labeled "epsilon."
        NPW.addTransition(s, newstate, evar, 0);
      }
      // Add transitions within the pancake.
      auto it = transitions.find(s);
      for (auto const & t : it->second) {
        State newdest = numberState(t.destination, pri);
        if (pri == 0 || t.priority < pri) {
          NPW.addTransition(newstate, newdest, t.label, 0);
        } else if (t.priority == pri) {
          NPW.addTransition(newstate, newdest, t.label, 1);
        }
        if (!epsilonJumps && pri > 0 && t.priority <= pri) {
          NPW.addTransition(s, newdest, t.label, t.priority == pri ? 1 : 0);
        }
      }
    }
  }
  NPW.makeInitial(numberState(getInitial(), 0));

  // Remove states with no successors and merge traps.
  NPW.trim();
  NPW.directSimulationMinimization();
  if (!NPW.isComplete()) {
    NPW.addTrap(0);
  }
  return NPW;

} // Parity::toLDPW


/**
 * @brief Helper class for simulation equivalence computation.
 */
class Parity::Simul {
public:
  using InvTransition = pair<BDD,StateSet>;
  using InvDelta = vector<InvTransition>;

  Simul(Parity const & DPW, set<StatePair> & simul) : simul(simul)
  {
    // Compute inverse of transition function.  Use a set of letters that
    // works globally for the automaton.
    DPW.letters(StateSet(DPW.states.begin(), DPW.states.end()), ltrs);
    for (State s : DPW.states) {
      InvDelta v;
      // Initialize inverse transitions to <letter,empty set>.
      for (BDD const & letter : ltrs) {
        v.emplace_back(letter, StateSet{});
      }
      inverse.emplace(s,v);
    }
    // Enter all sources of transitions in the state sets of the
    // respective destinations.
    for (State s : DPW.states) {
      Delta const & delta = DPW.transitions.find(s)->second;
      for (BDD const & letter : ltrs) {
        for (Transition const & t : delta) {
          if (letter <= t.label) {
            InvDelta & inv = inverse.find(t.destination)->second;
            bool found = false;
            for (auto & pr : inv) {
              if (pr.first == letter) {
                pr.second.insert(s);
                found = true;
              }
            }
            if (!found) {
              inv.emplace_back(letter, StateSet{s});
            }
          }
        }
      }
    }
  }

  void enqueue(StatePair const & sp)
  {
    q.push(sp);
    enq.insert(sp);
  }

  StatePair dequeue(void)
  {
    StatePair pq = q.front();
    q.pop();
    enq.erase(pq);
    return pq;
  }

  bool qIsEmpty(void)
  {
    return q.empty();
  }

  // Enqueue all pairs of the relation that may be affected by
  // the removal of (p,q).
  void perturb(State p, State q)
  {
    InvDelta const & invp = inverse.find(p)->second;
    InvDelta const & invq = inverse.find(q)->second;

    // We assume that sets are in the same order.
    for (size_t i = 0; i != ltrs.size(); ++i) {
      StateSet const & predp = invp[i].second;
      StateSet const & predq = invq[i].second;
      for (State pp : predp) {
        for (State pq : predq) {
          if (pp == pq) continue;
          StatePair spair(max(pp, pq), min(pp, pq));
          if (simul.find(spair) != simul.end()) {
            if (enq.find(spair) == enq.end()) {
              enqueue(spair);
            }
          }
        }
      }
    }
  }

private:
  set<StatePair> & simul;
  map<State,InvDelta> inverse;
  vector<BDD> ltrs;
  queue<StatePair> q;
  set<StatePair> enq;

}; // Parity::Simul


// Check whether states p and q satisfy the conditions for q to
// directly simulate p, given the current relation simul.
bool Parity::directCompareStates(State p, State q, set<StatePair> const & simul) const
{
  // Minimal set of letters needed to compare transitions out of p and q.
  vector<BDD> ltrs;
  letters(StateSet{p,q}, ltrs);

  // Find, for each letter, the combinations of destination and
  // priority available from q.
  multimap<BDD,pair<State,Priority>,Util::BddCompare> qchoices;
  Delta const & deltaq = transitions.find(q)->second;
  for (BDD const & letter : ltrs) {
    for (auto const & t : deltaq) {
      if (letter <= t.label) {
        qchoices.emplace(letter,pair<State,Priority>{t.destination,t.priority});
      }
    }
  }
#if 0
  // Diagnostic output.
  cout << "Choices for " << q << " : " << endl;
  for (BDD const & letter : ltrs) {
    cout << letter << " : ";
    auto const lb = qchoices.lower_bound(letter);
    auto const ub = qchoices.upper_bound(letter);
    for (auto it = lb; it != ub; ++it) {
      cout << " (" << it->second.first << "," << it->second.second << ")";
    }
    cout << endl;
  }
#endif

  // Check whether all transitions out of p can be matched
  // by transitions out of q for the same letter.
  Delta const & deltap = transitions.find(p)->second;
  for (BDD const & letter : ltrs) {
    for (auto const & t: deltap) {
      if (letter <= t.label) {
        State pdest = t.destination;
        Priority ppri = t.priority;
        bool oddp = ppri & 1;
        bool matched = false;
        auto const lb = qchoices.lower_bound(letter);
        if (lb == qchoices.end()) {
          return false;
        } else {
          // Check all transitions from q on the same letter until
          // a match is found.
          auto const ub = qchoices.upper_bound(letter);
          for (auto it = lb; it != ub; ++it) {
            State qdest = it->second.first;
            Priority qpri = it->second.second;
            bool oddq = qpri & 1;
            if (oddp && !oddq) continue;                 // wrong parity
            if (oddp && oddq && ppri > qpri) continue;   // acceptance too weak
            if (!oddp && !oddq && ppri < qpri) continue; // rejection too strong
            if (pdest == qdest) {
              matched = true;
              break;
            }
            auto sit = simul.find(StatePair{pdest,qdest});
            if (sit != simul.end()) {
              matched = true;
              break;
            }
          }
        }
        if (!matched) return false;
      }
    }
  }
  return true;

} // Parity::directCompareStates


Parity::TransitionMap Parity::inverseTransitions() const
{
  TransitionMap inverse;
  for (State s : states) {
    inverse.insert({s, Delta{}});
  }
  for (auto const & sd: transitions) {
    State source = sd.first;
    for (Transition const & t : sd.second) {
      State destination = t.destination;
      Transition tran{source, t.label, t.priority};
      auto it = inverse.find(destination);
      if (it == inverse.end()) {
        throw logic_error("missing inverse for state " + to_string(destination));
      } else {
        it->second.push_back(tran);
      }
    }
  }
  return inverse;

} // Parity::inverseTransitions


/**
 * Compute the set of pairs of states (p,q) such that they are in
 * simulation relation and p != q.
 */
void Parity::directSimulationRelation(set<StatePair> & simul) const
{
  // Very naive implementation.
  simul.clear();
  Simul shelper(*this, simul);

  StateSet stateset{states.begin(), states.end()};
  if (verbosity) {
    vector<BDD> ltrs;
    letters(stateset, ltrs);
    cout << "Letters: " << ltrs << endl;
  }

  for (State q : states) {
    for (State p : states) {
      if (p != q) {
        StatePair pq{p,q};
        simul.insert(pq);
        shelper.enqueue(pq);
      }
    }
  }

  if (verbosity) {
    cout << "Initially: ";
    for (StatePair const & elem : simul) {
      cout << " (" << elem.first << "," << elem.second << ")";
    }
    cout << endl;
  }

  while (!shelper.qIsEmpty()) {
    StatePair pq = shelper.dequeue();
    State p = pq.first;
    State q = pq.second;
    bool match = directCompareStates(p,q,simul);
    if (!match) {
      simul.erase(pq);
      shelper.perturb(p,q);
    }
  }

  if (verbosity) {
    cout << "Finally: ";
    for (StatePair const & elem : simul) {
      cout << " (" << elem.first << "," << elem.second << ")";
    }
    cout << endl;
  }

} // Parity::directSimulationRelation


void Parity::directSimulationMinimization(void)
{
  // Find states to be removed.
  set<StatePair> simul;
  directSimulationRelation(simul);

  // If two states are simulation equivalent, keep the one of smaller index
  // unless the other is the initial state.
  State initial = getInitial();
  map<State,State> replacements; // maps a state to be replaced to its replacement
  for (StatePair pq : simul) {
    State p = pq.first;
    State q = pq.second;
    if (p > q) {
      auto sit = simul.find({q,p});
      if (sit != simul.end()) {
        // p and q are simulation equivalent
        if (p == initial) {
          replacements.insert({q,p});
        } else {
          replacements.insert({p,q});
        }
      }
    }
  }

  // Replace transitions.
  // Add transitions from replaced state to the transitions
  // from the replacing state.
  for (auto const & pq : replacements) {
    State p = pq.first;
    State q = pq.second;
    auto const & delta = transitions.find(p)->second;
    for (Transition const & t : delta) {
      if (t.destination != p) {
        addTransition(q, t.destination, t.label, t.priority);
      }
    }
  }
  // Change transitions to replaced state to go to replacing state.
  for (State state: states) {
    Delta & delta = transitions.find(state)->second;
    for (Transition & tran : delta) {
      auto it = replacements.find(tran.destination);
      if (it != replacements.end()) {
        tran.destination = it->second;
      }
    }
  }

  // Clean up the automaton.

  // Remove transitions from replaced states.
  for (auto const & sp : replacements) {
    transitions.erase(sp.first);
  }
  // Remove replaced states from state vector.
  size_t n = states.size();
  size_t i = 0, j = 0;
  while (i != n) {
    if (replacements.find(states[i]) == replacements.end()) {
      if (i != j) {
        states[j] = states[i];
      }
      ++j;
    }
    ++i;
  }
  states.resize(j);

  mergeTransitions();

  // Remove arcs to direct-simulated states.
  StateSet stateset{states.begin(), states.end()};
  TransitionMap inverse = inverseTransitions();
  if (verbosity) {
    printDot("From");
    cout << "Inverse\n" << inverse << endl;
  }
  set<pair<State,Transition>> delenda;
  for (StatePair pq : simul) {
    State p = pq.first;
    State q = pq.second;
    // Make sure neither state has been removed because of simulation equivalence.
    if (stateset.find(p) == stateset.end()) continue;
    if (stateset.find(q) == stateset.end()) continue;

    // Theorem applies.  Drop arcs to p from parents of both p and q.
    if (verbosity) {
      cout << p << " is simulated by " << q << endl; // diagnostic print
    }
    auto iit = inverse.find(p);
    if (iit != inverse.end()) {
      Delta const & idelta = iit->second;
      // Check each predecessor of p.
      for (Transition it : idelta) {
        State ppred = it.destination;
        BDD label = it.label;
        auto dit = transitions.find(ppred);
        if (dit == transitions.end()) {
          throw logic_error("Inconsistent transitions");
        }
        Delta & delta = dit->second;
        for (Transition const & t : delta) {
          if (t.destination == q && label <= t.label) {
            if (it.priority % 2 == 0) {
              if (t.priority %2 == 1 || t.priority <= it.priority) {
                delenda.insert({ppred,Transition{p,label,it.priority}});
                if (verbosity) {
                  cout << "Dropping transition from " << ppred << " to " << p
                       << " with label " << label << endl;
                }
                break;
              }
            } else {
              if (t.priority %2 == 1 && t.priority >= it.priority) {
                delenda.insert({ppred,Transition{p,label,it.priority}});
                if (verbosity) {
                  cout << "Dropping transition from " << ppred << " to " << p
                       << " with label " << label << endl;
                }
                break;
              }
            }
          } 
        }
      }
    }
  }
  if (verbosity) {
    cout << "Delenda: " << delenda << endl;
  }

  for (State state : states) {
    auto dit = transitions.find(state);
    if (dit != transitions.end()) {
      Delta & delta = dit->second;
      Delta newdelta;
      for (Transition const & t : delta) {
        if (delenda.find({state,t}) == delenda.end()) {
          newdelta.push_back(t);
        }
      }
      delta = newdelta;
    }
  }

  // Remove unreachable states.
  StateSet reachable{initialState};
  queue<State> q;
  q.push(initialState);

  while (!q.empty()) {
    State state = q.front();
    q.pop();
    auto dit = transitions.find(state);
    if (dit != transitions.end()) {
      Delta const & delta = dit->second;
      for (Transition const & t : delta) {
        State destination = t.destination;
        if (reachable.find(destination) == reachable.end()) {
          q.push(destination);
          reachable.insert(destination);
        }
      }
    }
  }
  if (verbosity) {
    cout << "Reachable: " << reachable << endl;
  }

  // Remove transitions from unreachable states.
  for (State state : states) {
    if (reachable.find(state) == reachable.end()) {
      transitions.erase(state);
    }
  }
  // Remove unreachable states from state vector.
  n = states.size();
  i = 0;
  j = 0;
  while (i != n) {
    if (reachable.find(states[i]) != reachable.end()) {
      if (i != j) {
        states[j] = states[i];
      }
      ++j;
    }
    ++i;
  }
  states.resize(j);

  mergeTransitions();

} // Parity::directSimulationMinimization


// Classify states according to the priorities for each letter of the automaton.
// Only correct for deterministic (though, possibly, incomplete) automata.
map<State,unsigned> Parity::classifyStates(void) const
{
  // Collect info about the automaton.
  unsigned maxpriority = getMaxPriority();
  StateSet stateset{states.begin(), states.end()};
  vector<BDD> ltrs;
  letters(stateset, ltrs);
  size_t nletters = ltrs.size();

  // Compute a signature for each state based on the priority for each
  // input letter, which is well defined for deterministic automata.
  map<State,unsigned> outputClass;
  unsigned cnumber = 0;
  map<unsigned long,unsigned> classMap;
  for (State s : states) {
    auto dit = transitions.find(s);
    // Encode priorities as a number in base maxpriority+2 with nletters digits.
    // Use digit maxpriority+1 if there is no transition for that letter.
    unsigned long oclass = 0;
    for (size_t i = 0; i != nletters; ++i) {
      BDD const & l = ltrs.at(i);
      Priority pri = 0;
      bool found = false;
      if (dit != transitions.end()) {
        auto delta = dit->second;
        for (auto const & t : delta) {
          if (l <= t.label) {
            found = true;
            if (t.priority > pri) {
              pri = t.priority;
            }
          }
        }
      }
      if (!found) pri = maxpriority+1;
      oclass = oclass * (maxpriority+2) + pri;
    }
    // Get ordinal number of this output class.
    auto cit = classMap.find(oclass);
    unsigned classid;
    if (cit == classMap.end()) {
      classid = cnumber;
      classMap.emplace(oclass, classid);
      cnumber++;
    } else {
      classid = cit->second;
    }
    outputClass.emplace(s,classid);
  }
  return outputClass;

} // Parity::classifyStates


// Only works for deterministic automata.
void Parity::stateMinimization(void)
{
  map<State,unsigned> outputClass = classifyStates();

  map<State,State> replacements = stateEquivalence(outputClass);

  // Replace.
  for (auto const & sp : replacements) {
    auto const delta = transitions.find(sp.first)->second;
    for (Transition const & t : delta) {
      addTransition(sp.second, t.destination, t.label, t.priority);
    }
  }
  for (State state: states) {
    Delta & delta = transitions.find(state)->second;
    for (Transition & tran : delta) {
      auto it = replacements.find(tran.destination);
      if (it != replacements.end()) {
        tran.destination = it->second;
      }
    }
  }
  // Clean up.
  for (auto const & sp : replacements) {
    transitions.erase(sp.first);
  }
  size_t n = states.size();
  size_t i = 0, j = 0;
  while (i != n) {
    if (replacements.find(states[i]) == replacements.end()) {
      if (i != j) {
        states[j] = states[i];
      }
      ++j;
    }
    ++i;
  }
  states.resize(j);

  mergeTransitions();

} // Parity::stateMinimization


void Parity::trim(void)
{
  // Create map to be able to check whether the endpoints of a transition
  // are in the same SCC.
  SCCs sccs = getSCCs();
  map<State,size_t> sccMap;
  for (size_t i = 0; i != sccs.size(); ++i) {
    StateSet const & scc = sccs.at(i);
    for (State s : scc) {
      sccMap.emplace(s,i);
    }
  }
  StateSet targets;
  // Target states have at least one outgoing transition of odd priority
  // to a state in the same SCC.
  for (State s : states) {
    auto sit = sccMap.find(s);
    if (sit == sccMap.end()) {
      // State is unreachable.
      continue;
    }
    size_t sindex = sit->second;
    auto dit = transitions.find(s);
    if (dit != transitions.end()) {
      auto const & delta = dit->second;
      for (auto const & t : delta) {
        if (t.priority & 1) {
          if (sindex == sccMap.at(t.destination)) {
            targets.insert(s);
          }
        }
      }
    }
  }
  LTS::trim(targets, sccs);
  if (verbosity) {
    cout << "Trimmed automaton:\n" << *this;
  }

} // Parity::trim


void Parity::makeComplete(void)
{
  if (!isComplete()) {
    addTrap(0);
  }

} // Parity::makeComplete


// Add a trap state of the given priority to which all missing transitions
// are directed.
// The automaton may already contain a suitable state, but, to keep things
// simple, we let downstream simplification steps merge it with the one we
// introduce here.
void Parity::addTrap(unsigned priority)
{
  StateSet stateset{states.begin(), states.end()};
  // Choose a number for the trap state.  If simplifications have removed
  // some states, re-use the lowest of their numbers; otherwise pick a new
  // number.
  State trapstate;
  for (trapstate = 0; trapstate != numStates() + 1; ++trapstate) {
    if (stateset.find(trapstate) == stateset.end()) {
      break;
    }
  }

  // Add trap state with self-loop.
  addState(trapstate);
  addTransition(trapstate, trapstate, mgr.bddOne(), priority);
  
  // Add transition to trap state for each missing transition.
  // Ignore epsilon transitions.
  for (State s : states) {
    BDD lbljoin = mgr.bddZero();
    auto dit = transitions.find(s);
    if (dit != transitions.end()) {
      auto & delta = dit->second;
      for (auto const & t : delta) {
        if (t.label != epsilon) {
          lbljoin |= t.label;
        }
      }
    }
    if (lbljoin != mgr.bddOne()) {
      addTransition(s, trapstate, ~lbljoin, priority);
    }
  }

} // Parity::addTrap


void Parity::makeSafety(bool wascomplete)
{
  // Set the priority of each transition to 1.  Correctness of this step
  // follows from the automaton being trim.
  for (auto & statedelta : transitions) {
    for (Transition & transition : statedelta.second) {
      transition.priority = 1;
    }
  }
  // Complete the automaton if the original automaton was complete.
  if (wascomplete && !isComplete()) {
    addTrap(0);
  }
  // Try to simplify the result.
  if (isDeterministic(false)) { // false: don't check for completeness
    stateMinimization();
  } else {
#if 0
    *this = determinize();
    CartonMaceiras();
    printDot("test");
#endif
    directSimulationMinimization();
  }

} // Parity::makeSafety


// In Alpern and Schneider's construction, the environment of a trim
// automaton accepts all the words that have no run in the automaton.
Parity Parity::buildEnvironment(void) const
{
  // Apply Rabin-Scott subset construction.
  map<State,StateSet> subsetMap;
  Parity env{subsetsLTS(subsetMap)};
  if (verbosity) {
    cout << "Environment subset map: " << subsetMap << endl;
  }

  // Fill in acceptance condition.  Only the state of the environment
  // mapping to the empty set should be accepting.
  for (State s: env.states) {
    auto sit = subsetMap.find(s);
    if (sit == subsetMap.end()) {
      throw logic_error("state " + to_string(s) +
                        " of environment lacks subset");
    }
    Priority priority = sit->second.empty() ? 1 : 0;
    auto dit = env.transitions.find(s);
    if (dit == env.transitions.end()) {
      throw logic_error("state " + to_string(s) +
                        " of environment lacks transitions");
    }
    Delta & delta = dit->second;
    for (auto & t : delta) {
      t.priority = priority;
    }
  }
  // The environment is always deterministic and complete.
  env.stateMinimization();
  return env;

} // Parity::buildEnvironment


// Computes liveness component from trim automaton.
void Parity::makeLiveness(void)
{
  // If a trim automaton is complete, it accepts a liveness property.
  // Hence it is its own liveness component.
  if (isComplete()) {
    return;
  }
  if (isDeterministic(false)) { // false: don't check for completeness
    // Simplified procedure for deterministic automata.
    addTrap(1);
    stateMinimization();
  } else {
    Parity env{buildEnvironment()};
    env.trim();
    // Take union of completed automaton and environment.
    if (!env.isTriviallyEmpty()) {
      unite(env);
    }
  }

} // Parity::makeLiveness


// Implements the Alpern and Schneider construction.
void Parity::safetyLiveness(Parity & safety, Parity & liveness) const
{
  // Initialize both components to the trimmed version of the automaton.
  safety = *this;
  safety.trim();
  safety.setVerbosity(verbosity);

  liveness = safety;
  liveness.setVerbosity(verbosity);

  // If the given automaton is complete, we insist on the safety component
  // being complete; otherwise we don't.  (The liveness component is complete
  // by definition.)
  bool complete = isComplete();
  safety.makeSafety(complete);
  liveness.makeLiveness();

} // Parity::safetyLiveness


// An automaton is trivially empty if all its transitions have even priorities.
// In general, this is only a sufficient condition for emptiness, but, if
// the automaton is of minimum index, the condition is also necessary.
bool Parity::isTriviallyEmpty(void) const
{
  for (auto const & et : transitions) {
    for (auto const & t : et.second) {
      if (t.priority & 1) return false;
    }
  }
  return true;

} // Parity::isTriviallyEmpty


// An automaton is deemed "terminal" if every accepting component
// is a sink whose transitions are all accepting.  Usually there is
// one such component, which has a single state with a universal self-loop.
bool Parity::isTerminal(bool ignoreTransient) const
{
  if (numStates() == 0) return false;
  SCCs sccs = getSCCs();
  map<State,size_t> sccMap;

  for (auto const & scc : sccs) {
    bool someAccept = false;
    bool someReject = false;
    bool bottom = true;
    for (State s : scc) {
      auto dit = transitions.find(s);
      if (dit != transitions.end()) {
        BDD join = mgr.bddZero();
        auto const & delta = dit->second;
        for (Transition const & t : delta) {
          if (scc.find(t.destination) == scc.end()) {
            if (!ignoreTransient && (t.priority & 1)) {
              return false;
            }
            bottom = false;
          } else if (t.priority & 1) {
            someAccept = true;
          } else {
            someReject = true;
          }
          join |= t.label;
        }
        bottom &= join.IsOne();
      } else {
        throw logic_error("State " + to_string(s) + " has no transitions");
      }
      if (someAccept && (someReject || !bottom)) return false;
    }
  }
  return true;

} // Parity::isTerminal


StateSet Parity::getTrapStates() const
{
  SCCs sccs = getSCCs();
  // sccMap gives the SCC index for each state.
  map<State,size_t> sccMap;
  for (size_t i = 0; i != sccs.size(); ++i) {
    StateSet const & scc = sccs.at(i);
    for (State s : scc) {
      sccMap.emplace(s,i);
    }
  }
  // The set accepting initially contains the indices of the SCCs that contain
  // at least one accepting transition connecting two states of the SCC.
  // If the automaton is a DPW of minimum index or an NBW, an SCC in
  // accepting accepts at least one word.
  set<size_t> accepting;
  for (State s : states) {
    auto sit = sccMap.find(s);
    if (sit != sccMap.end()) {
      size_t sindex = sit->second;
      auto dit = transitions.find(s);
      if (dit != transitions.end()) {
        auto const & delta = dit->second;
        for (Transition const & t : delta) {
          if (t.priority & 1) { // odd priority
            if (sindex == sccMap.at(t.destination)) {
              accepting.insert(sindex);
            }
          }
        }
      }
    }
    else cout << "State not in map: " << s << endl;
  }
  // If a run of the automaton visits a state in an SCC in traps, it is not
  // an accepting run.
  StateSet traps;
  // Here we rely on the fact that the SCCs are returned in a reverse
  // topological order to add to accepting the SCCs that have transitions
  // to SCCs already in accepting.
  for (size_t i = 0; i != sccs.size(); ++i) {
    StateSet const & scc = sccs.at(i);
    if (accepting.find(i) == accepting.end()) {
      // Possibly a trap: let's check.
      bool pathToFairCycle = false;
      for (State s : scc) {
        size_t sindex = sccMap.at(s);
        auto dit = transitions.find(s);
        if (dit != transitions.end()) {
          Delta const & delta = dit->second;
          for (Transition const & t : delta) {
            size_t dindex = sccMap.at(t.destination);
            if (!t.label.IsZero()
                && accepting.find(dindex) != accepting.end()) {
              pathToFairCycle = true;
              accepting.insert(sindex);
              break;
            }
          }
        }
        if (pathToFairCycle) break;
      }
      if (!pathToFairCycle) {
        traps |= scc;
      }
    }
  }
  return traps;

} // Parity::getTrapStates


void Parity::unite(Parity const & second)
{
  // Find number of first new state.
  State ustate = numStates();
  for (State s : states) {
    if (s >= ustate) ustate = s + 1;
  }
  // Save old initial state.
  State oldInitial = initialState;
  // Add new initial state.
  addState(ustate);
  makeInitial(ustate);
  ustate++;
  // Maps the index of a state in the second automaton to its
  // index in the union automaton.
  map<State,State> secondToUnion;
  // Add states from second automaton to union.
  for (State s : second.states) {
    addState(ustate);
    secondToUnion[s] = ustate;
    ustate++;
  }
  // Add transitions from second automaton.
  for (State s : second.states) {
    State source = secondToUnion.at(s);
    auto dit = second.transitions.find(s);
    if (dit != second.transitions.end()) {
      Delta const & delta = dit->second;
      for (Transition const & t : delta) {
        addTransition(source,secondToUnion.at(t.destination),t.label,t.priority);
      }
    }
  }
  // Add transitions from new initial state to successors of old initial state.
  auto dit = transitions.find(oldInitial);
  if (dit != transitions.end()) {
    Delta & delta = dit->second;
    for (Transition const & t : delta) {
      addTransition(initialState,t.destination,t.label,t.priority);
    }
  }
  // Add transitions fron new initial state to successors of second initial state.
  dit = transitions.find(secondToUnion.at(second.initialState));
  if (dit != transitions.end()) {
    Delta & delta = dit->second;
    for (Transition const & t : delta) {
      addTransition(initialState,t.destination,t.label,t.priority);
    }
  }

} // Parity::unite


// Complements a parity automaton by shifting priorities either up or down by one.
// If the automaton is nondeterministic, it is determinized first.
Parity Parity::complement(void) const
{
  Parity CDPW = isDeterministic() ? *this : determinize();

  for (auto & sd : CDPW.transitions) {
    for (Transition & t : sd.second) {
      t.priority++;
    }
  }
  CDPW.CartonMaceiras();

  return CDPW;

} // Parity::complement


/**
   @brief Helper class to solve 3-priority parity games.

   @details The game is between the maximizing Spolier and the minimizing
   Duplicator.
*/
class Parity::OSPGame {
public:
  using Measure = long;
  /** @brief Constructor. */
  OSPGame(Parity const & graph, StateSet const & spoilerStates);
  /** @brief Computes progress measures. */
  void lift(void);
  /** @brief Retrieve duplicator's winning states and strategy. */
  void statesAndStrategies(StateSet & dwinning, StateMap & dstrategy,
                           StateMap & sstrategy) const;
private:
  Measure getMeasure(State s) const;
  Measure clipIncr(Measure x, Priority p) const;
  Measure val(State v);
  void enqueue(State s, queue<State> & q, StateSet & enq) const {
    if (enq.find(s) == enq.end()) {
      q.push(s);
      enq.insert(s);
    }
  }
  State dequeue(queue<State> & q, StateSet & enq) const {
    State s = q.front();
    q.pop();
    enq.erase(s);
    return s;
  }
  Parity const & graph;
  StateSet const & spoilerStates;
  TransitionMap inverse;
  Measure infinity;
  unordered_map<State,Measure> pmeasure;
  StateMap ssuccessor;

}; // Parity::OSPGame


Parity::OSPGame::OSPGame(Parity const & graph, StateSet const & spoilerStates) :
  graph(graph), spoilerStates(spoilerStates)
{
  inverse = graph.inverseTransitions();
  infinity = 1;
  for (auto const & sd : graph.transitions) {
    for (Transition const & t : sd.second) {
      infinity += t.priority == 2;
    }
  }

} // Parity::OSPGame::OSPGame


Parity::OSPGame::Measure Parity::OSPGame::getMeasure(State s) const
{
  auto mit = pmeasure.find(s);
  if (mit == pmeasure.end()) {
    return 0;
  } else {
    return mit->second;
  }

} // Parity::OSPGame::getMeasure


Parity::OSPGame::Measure Parity::OSPGame::clipIncr(Measure x, Priority p) const
{
  if (x == infinity) {
    return infinity;
  } else if (p == 3) {
    return 0;
  } else if (p == 2) {
    return x + 1;
  } else {
    return x;
  }

} // Parity::OSPGame::clipIncr


Parity::OSPGame::Measure Parity::OSPGame::val(State v)
{
  Measure mu;
  auto sd = graph.transitions.find(v);
  if (sd == graph.transitions.end())
    throw logic_error("Node without transitions");
  if (spoilerStates.find(v) != spoilerStates.end()) {
    // Compute max
    mu = 0;
    for (Transition const & t : sd->second) {
      Measure m  = getMeasure(t.destination);
      if (m == infinity) {
        ssuccessor.emplace(v,t.destination);
        return infinity;
      }
      Priority p = t.priority;
      m = clipIncr(m,p);
      if (m > mu) {
        mu = m;
      }
      if (mu == infinity) {
        ssuccessor.emplace(v,t.destination);
        break;
      }
    }
  } else {
    // Compute min
    mu = infinity;
    for (Transition const & t : sd->second) {
      Measure m = getMeasure(t.destination);
      if (m == infinity) {
        continue;
      }
      Priority p = t.priority;
      m = clipIncr(m,p);
      if (m == 0) {
        return 0;
      }
      if (m < mu) {
        mu = m;
      }
    }
  }
  return mu;

} // Parity::OSPGame::val


void Parity::OSPGame::statesAndStrategies(
  StateSet & dwinning,
  StateMap & dstrategy,
  StateMap & sstrategy) const
{
  for (auto const & state : graph.states) {
    Measure mu = getMeasure(state);
    if (mu < infinity) {
      // State is winning for Duplicator.
      dwinning.insert(state);
      if (spoilerStates.find(state) == spoilerStates.end()) {
        auto sd = graph.transitions.find(state);
        if (sd == graph.transitions.end())
          throw logic_error("Node without transitions");
        for (Transition const & t : sd->second) {
          Measure m = getMeasure(t.destination);
          if (mu == clipIncr(m, t.priority)) {
            dstrategy.emplace(state,t.destination);
            break;
          }
        }
      }
    } else {
      // State is winning for Spoiler.
      if (spoilerStates.find(state) != spoilerStates.end()) {
        auto sit = ssuccessor.find(state);
        if (sit == ssuccessor.end()) {
          throw logic_error("Spoiler winning state without strategy: " + to_string(state));
        }
        sstrategy.emplace(state,sit->second);
      }
    }
  }

} // Parity::OSPGame::statesAndStrategies


void Parity::OSPGame::lift(void)
{
  // Naive implementation of the progress measure algorithm.

  // Measure is implicitly initialized to 0 for all states.
  // Initialize queue to state with outgoing transitions of priority 2.
  queue<State> q;
  StateSet enq;
  for (auto const & sd : graph.transitions) {
    for (Transition const & t : sd.second) {
      if (t.priority == 2) {
        enqueue(sd.first, q, enq);
      }
    }
  }
  // Process queue until empty.
  while (!q.empty()) {
    State state = dequeue(q,enq);
    Measure oldmu = getMeasure(state);
    if (oldmu < infinity) {
      Measure mu = val(state);
      if (mu > oldmu) {
        pmeasure[state] = mu;
        auto iit = inverse.find(state);
        if (iit == inverse.end()) {
          throw logic_error("missing inverse for state " + to_string(state));
        }
        Delta const & idelta = iit->second;
        for (Transition it : idelta) {
          enqueue(it.destination, q, enq);
        }
      }
    }
  }
  // Diagnostic prints.
  if (graph.verbosity > 0) {
    cout << "Measures: " << pmeasure << endl;
  }

} // Parity::OSPGame::lift


void Parity::solveOneStreettPairGame(StateSet const & spoilerStates,
                                     StateSet & dwinning,
                                     StateMap & dstrategy,
                                     StateMap & sstrategy) const
{
  OSPGame game{*this, spoilerStates};
  game.lift();
  game.statesAndStrategies(dwinning, dstrategy, sstrategy);

} // Parity::solveOneStreettPairGame


/** @brief Helper class for simulation game graphs. */
struct GameState {
  using Player = int;

  GameState(State s, State d, Player p, BDD l) :
    sstate(s), dstate(d), player(p), letter(l) {}
  /* @brief Comparison function for ordered containers. */
  bool operator<(GameState const & other) const {
    if (sstate < other.sstate) return true;
    else if (sstate > other.sstate) return false;
    else if (dstate < other.dstate) return true;
    else if (dstate > other.dstate) return false;
    else if (player < other.player) return true;
    else if (player > other.player) return false;
    else return letter.getNode() < other.letter.getNode();
  }
  bool operator==(GameState const & other) const {
    return sstate == other.sstate &&
      dstate == other.dstate &&
      player == other.player &&
      letter == other.letter;
  }
  void print(ostream & os) const {
    os << '(' << sstate << ',' << dstate << ',' << player;
    if (!letter.IsZero()) {
      os << ',' << letter;
    }
    os << ')';
  }
  State sstate;
  State dstate;
  Player player;
  BDD letter;
};


namespace std {
  /** @brief Hash specialization to use in unordered containers. */
  template <> struct hash<GameState> {
    size_t operator()(GameState const & gs) const
    {
      return (hash<State>{}(gs.sstate) << 3) ^
        (hash<State>{}(gs.dstate) << 7) ^
        (hash<GameState::Player>{}(gs.player) << 11) ^
        hash<DdNode *>{}(gs.letter.getNode());
    }
  };
}


std::string to_string(GameState const & gs)
{
  ostringstream os;
  gs.print(os);
  return os.str();
}


/** @brief stream insertion operator for GameState class. */ 
ostream & operator<<(ostream & os, GameState const & gs) {
  gs.print(os);
  return os;
}


Parity Parity::buildLeadSimulationGame(
  StateSet & spoilerStates,
  DotAttributes & attributes) const
{
  /* There are three types of states in the game graph.  The states in which
   * Spoiler chooses a letter; those in which Duplicator chooses an enabled
   * transition; and those in which Spoiler chooses an enabled transition.
   * The last category uses the priority of the transition chosen by
   * Duplicator as marker.
   */
  GameState::Player const spoiler = -1;
  GameState::Player const duplicator = -2;

  if (!isComplete() || getMaxPriority() > 1) {
    throw domain_error("Automaton should be complete and have Buechi condition");
  }

  Parity game{getManager()};
  game.setVerbosity(readVerbosity());
  unordered_map<GameState,State> stateMap;
  State sstate = getInitial();
  State dstate = getInitial();
  GameState igstate{sstate, dstate, spoiler, mgr.bddZero()};
  State pstate = 0;
  stateMap.insert({igstate,pstate});
  queue<GameState> q;
  q.push(igstate);
  spoilerStates.insert(pstate);
  game.addState(pstate);
  game.makeInitial(pstate);
  pstate++;

  while (!q.empty()) {
    GameState gstate{q.front()};
    q.pop();
    sstate = gstate.sstate;
    dstate = gstate.dstate;
    GameState::Player player = gstate.player;
    auto smit = stateMap.find(gstate);
    if (smit == stateMap.end()) {
      throw logic_error("unexpected game state");
    }
    State current = smit->second;
    State nextstate;
    // Find successor states.
    if (player == spoiler) {
      // Letter choice state.
      vector<BDD> labels;
      letters(StateSet{sstate,dstate}, labels);
      for (BDD const & label : labels) {
        GameState ngamestate{sstate,dstate,duplicator,label};
        auto pit = stateMap.find(ngamestate);
        if (pit == stateMap.end()) {
          stateMap.insert({ngamestate,pstate});
          nextstate = pstate;
          pstate++;
          game.addState(nextstate);
          q.push(ngamestate);
        } else {
          nextstate = pit->second;
        }
        game.addTransition(current, nextstate, label, 1);
      }
    } else if (player == duplicator) {
      auto const & sd = transitions.find(dstate);
      if (sd == transitions.end()) {
        throw logic_error("State without transitions: " + to_string(dstate));
      }
      BDD letter = gstate.letter;
      for (Transition const & t : sd->second) {
        if (letter <= t.label) {
          // Transition is enabled.  The successor is a transition
          // selection state for Spoiler.
          GameState ngamestate{sstate,t.destination,t.priority,letter};
          auto pit = stateMap.find(ngamestate);
          if (pit == stateMap.end()) {
            stateMap.insert({ngamestate,pstate});
            nextstate = pstate;
            pstate++;
            game.addState(nextstate);
            spoilerStates.insert(nextstate);
            q.push(ngamestate);
          } else {
            nextstate = pit->second;
          }
          game.addTransition(current, nextstate, letter, 1);
        }
      }
    } else {
      // Spoiler's successor selection state.  The player field gives the
      // priority of the transition that was taken by duplicator.
      auto const & sd = transitions.find(sstate);
      if (sd == transitions.end()) {
        throw logic_error("State without transitions: " + to_string(sstate));
      }
      BDD letter = gstate.letter;
      Priority pdup = player;
      for (Transition const & t : sd->second) {
        if (letter <= t.label) {
          // Transition is enabled.  The successor is a letter selection state.
          GameState ngamestate{t.destination,dstate,spoiler,mgr.bddZero()};
          auto pit = stateMap.find(ngamestate);
          if (pit == stateMap.end()) {
            stateMap.insert({ngamestate,pstate});
            nextstate = pstate;
            pstate++;
            game.addState(nextstate);
            spoilerStates.insert(nextstate);
            q.push(ngamestate);
          } else {
            nextstate = pit->second;
          }
          Priority p = pdup == 1 ? 3 : t.priority == 1 ? 2 : 1;
          game.addTransition(current, nextstate, letter, p);
        }
      }
    }
  }
  game.mergeTransitions();

  // Set attributes for dot printing.
  for (auto const & e : stateMap) {
    GameState const & gs = e.first;
    attributes.nodeLabels.emplace(e.second, to_string(gs.sstate) +
                                  "," + to_string(gs.dstate));
    if (gs.player == spoiler) {
      attributes.nodeShapes.emplace(e.second, "box");
    } else if (gs.player != duplicator) {
      attributes.nodeShapes.emplace(e.second, "house");
    }      
  }

  return game;
}


bool Parity::leadSimulates(void) const
{
  StateSet spoilerStates;
  DotAttributes attributes;
  Parity game{buildLeadSimulationGame(spoilerStates, attributes)};
  StateSet dwinning;
  StateMap dstrategy, sstrategy;
  game.solveOneStreettPairGame(spoilerStates, dwinning, dstrategy, sstrategy);
  if (game.readVerbosity() > 0) {
    // Color states won by spoiler and the transitions of both strategies.
    for (State s : game.states) {
      if (dwinning.find(s) == dwinning.end()) {
        attributes.nodeColors.emplace(s,"red");
      }
    }
    for (auto const & edge : sstrategy) {
      attributes.edgeColors.emplace(edge,"red");
    }
    for (auto const & edge : dstrategy) {
      attributes.edgeColors.emplace(edge,"blue");
    }
    game.printDot("lead-simulation game", attributes);
  }
  // Does Duplicator win from the initial state of the game?
  return dwinning.find(game.getInitial()) != dwinning.end();

} // Parity::leadSimulates


Parity Parity::buildFairSimulationGame(
  Parity const & other,
  StateSet & spoilerStates,
  DotAttributes & attributes) const
{
  /* There are two types of states in the game graph.  The states in which
   * Spoiler chooses a letter and an enabled transition; and those in which
   * Duplicator chooses an enabled transition.
   * The second category uses the priority of the transition chosen by
   * Duplicator as marker.
   */
  GameState::Player const spoiler = -1;

  if (!isComplete() || getMaxPriority() > 1) {
    throw domain_error("Automaton should be complete and have Buechi condition");
  }
  if (!other.isComplete() || other.getMaxPriority() > 1) {
    throw domain_error("Other automaton should be complete and have Buechi condition");
  }
  if (getManager().getManager() != other.getManager().getManager()) {
    throw domain_error("The two automata should have the same BDD manager");
  }

  Parity game{getManager()};
  game.setVerbosity(readVerbosity());
  unordered_map<GameState,State> stateMap;
  State sstate = other.getInitial();
  State dstate = getInitial();
  GameState igstate{sstate, dstate, spoiler, mgr.bddZero()};
  State pstate = 0;
  stateMap.insert({igstate,pstate});
  queue<GameState> q;
  q.push(igstate);
  spoilerStates.insert(pstate);
  game.addState(pstate);
  game.makeInitial(pstate);
  pstate++;

  while (!q.empty()) {
    GameState gstate{q.front()};
    q.pop();
    sstate = gstate.sstate;
    dstate = gstate.dstate;
    GameState::Player player = gstate.player;
    auto smit = stateMap.find(gstate);
    if (smit == stateMap.end()) {
      throw logic_error("unexpected game state");
    }
    State current = smit->second;
    State nextstate;
    // Find successor states.
    if (player == spoiler) {
      auto const & sd = other.transitions.find(sstate);
      if (sd == other.transitions.end()) {
        throw logic_error("State without transitions: " + to_string(sstate));
      }
      vector<BDD> labels;
      other.letters({sstate}, labels);
      letters({dstate}, labels);
      for (BDD const & label : labels) {
        for (Transition const & t : sd->second) {
          if (label <= t.label) {
            // Transition is enabled.
            GameState ngamestate{t.destination,dstate,t.priority,label};
            auto pit = stateMap.find(ngamestate);
            if (pit == stateMap.end()) {
              stateMap.insert({ngamestate,pstate});
              nextstate = pstate;
              pstate++;
              game.addState(nextstate);
              q.push(ngamestate);
            } else {
              nextstate = pit->second;
            }
            game.addTransition(current, nextstate, label, 1);
          }
        }
      }
    } else {
      // Duplicator moves.
      auto const & sd = transitions.find(dstate);
      if (sd == transitions.end()) {
        throw logic_error("State without transitions: " + to_string(dstate));
      }
      BDD letter = gstate.letter;
      for (Transition const & t : sd->second) {
        if (letter <= t.label) {
          // Transition is enabled.  The successor is a transition
          // selection state for Spoiler.
          GameState ngamestate{sstate, t.destination, spoiler, mgr.bddZero()};
          auto pit = stateMap.find(ngamestate);
          if (pit == stateMap.end()) {
            stateMap.insert({ngamestate,pstate});
            nextstate = pstate;
            pstate++;
            game.addState(nextstate);
            spoilerStates.insert(nextstate);
            q.push(ngamestate);
          } else {
            nextstate = pit->second;
          }
          Priority p = t.priority == 1 ? 3 : gstate.player == 1 ? 2 : 1;
          game.addTransition(current, nextstate, letter, p);
        }
      }
    }
  }
  game.mergeTransitions();

  // Set attributes for dot printing.
  for (auto const & e : stateMap) {
    GameState const & gs = e.first;
    attributes.nodeLabels.emplace(e.second, to_string(gs.sstate) +
                                  "," + to_string(gs.dstate));
    if (gs.player == spoiler) {
      attributes.nodeShapes.emplace(e.second, "box");
    }      
  }

  return game;

} // Parity::buildFairSimulationGame


bool Parity::fairSimulates(Parity const & other) const
{
  StateSet spoilerStates;
  DotAttributes attributes;
  Parity game{buildFairSimulationGame(other, spoilerStates, attributes)};
  StateSet dwinning;
  StateMap dstrategy, sstrategy;
  game.solveOneStreettPairGame(spoilerStates, dwinning, dstrategy, sstrategy);
  if (game.readVerbosity() > 0) {
    // Color states won by spoiler and the transitions of both strategies.
    for (State s : game.states) {
      if (dwinning.find(s) == dwinning.end()) {
        attributes.nodeColors.emplace(s,"red");
      }
    }
    for (auto const & edge : sstrategy) {
      attributes.edgeColors.emplace(edge,"red");
    }
    for (auto const & edge : dstrategy) {
      attributes.edgeColors.emplace(edge,"blue");
    }
    game.printDot("fair-simulation game", attributes);
  }
  // Does Duplicator win from the initial state of the game?
  return dwinning.find(game.getInitial()) != dwinning.end();

} // Parity::fairSimulates


Parity Parity::buildDirectSimulationGame(
  Parity const & other,
  StateSet & spoilerStates,
  DotAttributes & attributes) const
{
  /* There are two types of states in the game graph.  The states in which
   * Spoiler chooses a letter and an enabled transition; and those in which
   * Duplicator chooses an enabled transition.
   * The second category uses the priority of the transition chosen by
   * Duplicator as marker.
   */
  GameState::Player const spoiler = -1;

  if (!isComplete()) {
    throw domain_error("Automaton should be complete");
  }
  if (!other.isComplete()) {
    throw domain_error("Other automaton should be complete");
  }
  if (getManager().getManager() != other.getManager().getManager()) {
    throw domain_error("The two automata should have the same BDD manager");
  }

  Parity game{getManager()};
  game.setVerbosity(readVerbosity());
  unordered_map<GameState,State> stateMap;
  State sstate = other.getInitial();
  State dstate = getInitial();
  GameState igstate{sstate, dstate, spoiler, mgr.bddZero()};
  State pstate = 0;
  stateMap.emplace(igstate,pstate);
  queue<GameState> q;
  q.push(igstate);
  spoilerStates.insert(pstate);
  game.addState(pstate);
  game.makeInitial(pstate);
  pstate++;
  // Add sink.
  State sink = pstate;
  spoilerStates.insert(sink);
  game.addState(sink);
  game.addTransition(sink, sink, mgr.bddOne(), 2);
  pstate++;

  while (!q.empty()) {
    GameState gstate{q.front()};
    q.pop();
    sstate = gstate.sstate;
    dstate = gstate.dstate;
    GameState::Player player = gstate.player;
    auto smit = stateMap.find(gstate);
    if (smit == stateMap.end()) {
      throw logic_error("unexpected game state");
    }
    State current = smit->second;
    State nextstate;
    // Find successor states.
    if (player == spoiler) {
      auto const & sd = other.transitions.find(sstate);
      if (sd == other.transitions.end()) {
        throw logic_error("Spoiler state without transitions: " + to_string(sstate));
      }
      vector<BDD> labels;
      other.letters({sstate}, labels);
      letters({dstate}, labels);
      for (BDD const & label : labels) {
        for (Transition const & t : sd->second) {
          if (label <= t.label) {
            // Transition is enabled.
            GameState ngamestate{t.destination,dstate,t.priority,label};
            auto pit = stateMap.find(ngamestate);
            if (pit == stateMap.end()) {
              stateMap.emplace(ngamestate,pstate);
              nextstate = pstate;
              pstate++;
              game.addState(nextstate);
              q.push(ngamestate);
            } else {
              nextstate = pit->second;
            }
            game.addTransition(current, nextstate, label, 1);
          }
        }
      }
    } else {
      // Duplicator moves.
      auto const & sd = transitions.find(dstate);
      if (sd == transitions.end()) {
        throw logic_error("Duplicator state without transitions: " + to_string(dstate));
      }
      BDD letter = gstate.letter;
      for (Transition const & t : sd->second) {
        if (letter <= t.label) {
          // Transition is enabled.  The successor is a transition
          // selection state for Spoiler (possibly the sink).
          Priority pspoil = gstate.player, pdup = t.priority;
          bool safe = (((pspoil & 1) == 0) &&
                       (((pdup & 1) == 1) || (pspoil >= pdup))) ||
            (((pspoil & 1) == 1) && (pdup >= pspoil));
          if (safe) {
            GameState ngamestate{sstate, t.destination, spoiler, mgr.bddZero()};
            auto pit = stateMap.find(ngamestate);
            if (pit == stateMap.end()) {
              stateMap.insert({ngamestate,pstate});
              nextstate = pstate;
              pstate++;
              game.addState(nextstate);
              spoilerStates.insert(nextstate);
              q.push(ngamestate);
            } else {
              nextstate = pit->second;
            }
            game.addTransition(current, nextstate, letter, 1);
          } else {
            game.addTransition(current, sink, letter, 2);
          }
        }
      }
    }
  }
  game.mergeTransitions();

  // Set attributes for dot printing.
  for (auto const & e : stateMap) {
    GameState const & gs = e.first;
    attributes.nodeLabels.emplace(e.second, to_string(gs.sstate) +
                                  "," + to_string(gs.dstate));
    if (gs.player == spoiler) {
      attributes.nodeShapes.emplace(e.second, "box");
    }      
  }
  attributes.nodeShapes.emplace(sink, "box");

  return game;

} // Parity::buildDirectSimulationGame


bool Parity::directSimulates(Parity const & other) const
{
  StateSet spoilerStates;
  DotAttributes attributes;
  Parity game{buildDirectSimulationGame(other, spoilerStates, attributes)};
  StateSet dwinning;
  StateMap dstrategy, sstrategy;
  game.solveOneStreettPairGame(spoilerStates, dwinning, dstrategy, sstrategy);
  if (game.readVerbosity() > 0) {
    // Color states won by spoiler and the transitions of both strategies.
    for (State s : game.states) {
      if (dwinning.find(s) == dwinning.end()) {
        attributes.nodeColors.emplace(s,"red");
      }
    }
    for (auto const & edge : sstrategy) {
      attributes.edgeColors.emplace(edge,"red");
    }
    for (auto const & edge : dstrategy) {
      attributes.edgeColors.emplace(edge,"blue");
    }
    game.printDot("direct-simulation game", attributes);
  }
  // Does Duplicator win from the initial state of the game?
  return dwinning.find(game.getInitial()) != dwinning.end();

} // Parity::directSimulates
