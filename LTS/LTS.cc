/** @file LTS.cc

  @brief Labeled transition systems (base class for automata classes).

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
#include <cstring>
#include <map>
#include <stack>
#include <queue>
#include <stdexcept>
#include <algorithm>
#include "Util.hh"
#include "LTS.hh"

using namespace std;

template<typename Tran>
LTS<Tran>::LTS(Cudd mgr) : mgr(mgr), verbosity(0), epsilon(mgr.bddZero()) {}


template<typename Tran>
void LTS<Tran>::addState(State s, Delta delta)
{
  auto it = transitions.find(s);
  if (it != transitions.end())
    throw logic_error("Trying to add existing state");
  states.push_back(s);
  transitions.insert({s, delta});
}


template<typename Tran>
void LTS<Tran>::addTransition(State source, State destination, BDD const & label)
{
  Transition tran{destination, label};
  auto it = transitions.find(source);
  if (it == transitions.end()) {
    throw logic_error("Adding transition to non-existing state");
  } else {
    it->second.push_back(tran);
  }
}


template<typename Tran>
void LTS<Tran>::makeInitial(State s)
{
  initialState = s;
}


template<typename Tran>
void LTS<Tran>::addAtomicProposition(BDD proposition, string name)
{
  if (!proposition.IsVar()) {
    throw logic_error("Proposition not a single variable");
  }
  varSet.insert(proposition);
  if (!proposition.IsNamedVar()) {
    mgr.pushVariableName(name);
  }
}


template<typename Tran>
string LTS<Tran>::getPropositionName(BDD proposition) const
{
  auto it = varSet.find(proposition);
  if (it == varSet.end()) {
    if (!proposition.IsVar()) {
      throw logic_error("Proposition not a single variable");
    } else {
      throw logic_error("Proposition not found");
    }
  }
  return mgr.getVariableName(proposition.NodeReadIndex());
}


template<typename Tran>
State LTS<Tran>::getSuccessor(State source, BDD letter) const
{
  auto it = transitions.find(source);
  if (it == transitions.end()) {
    throw logic_error("Non-existent node" + to_string(source));
  }
  for (auto const & t : it->second) {
    if (letter <= t.label) {
      return t.destination;
    }
  }
  ostringstream os;
  os << letter;
  throw logic_error("Successor not found for state " + to_string(source) +
                    " and letter " + os.str());
}


template<typename Tran>
StateSet LTS<Tran>::getSuccessors(State source, BDD letter) const
{
  StateSet successors;
  auto it = transitions.find(source);
  if (it == transitions.end()) {
    throw logic_error("Non-existent node " + to_string(source));
  }
  for (auto const & t : it->second) {
    if (letter <= t.label) {
      successors.insert(t.destination);
    }
  }
  return successors;
}


template<typename Tran>
StateSet LTS<Tran>::getEpsilonSuccessors(State source) const
{
  StateSet successors;
  auto it = transitions.find(source);
  if (it == transitions.end()) {
    throw logic_error("Non-existent node " + to_string(source));
  }
  for (auto const & t : it->second) {
    if (epsilon == t.label) {
      successors.insert(t.destination);
    }
  }
  return successors;
}


template<typename Tran>
bool LTS<Tran>::checkTransitions(void) const
{
  StateSet stateSet(states.begin(), states.end());
  for (State s : states) {
    typename map<State,Delta>::const_iterator it = transitions.find(s);
    if (it != transitions.end()) {
      vector<Transition> const & delta = it->second;
      for (Transition t : delta) {
        if (stateSet.find(t.destination) == stateSet.end())
          return false;
      }
    }
  }
  return true;
}


template<typename Tran>
void LTS<Tran>::mergeTransitions()
{
  // Merge transitions that differ at most in their labels.
  for (State state: states) {
    auto dit = transitions.find(state);
    if (dit != transitions.end()) {
      Delta & delta = dit->second;
      Delta newdelta;
      for (auto it = delta.begin(); it != delta.end(); ++it) {
        bool keep = true;
        for (auto cit = newdelta.begin(); cit != newdelta.end(); ++cit) {
          Transition temp = *cit;
          temp.label = it->label;
          if (*it == temp) {
            keep = false;
            cit->label |= it->label;
            break;
          }
        }
        if (keep) {
          newdelta.push_back(*it);
        }
      }
      delta = newdelta;
    }
  }
}


/**
 * @brief Extract the letters of the transitions out of a set of states.
 *
 * @details The objective is to find the letters from the smallest possible
 * alphabet.  If the automaton has inputs p, q, and r, it may still be
 * that no transition out of the given states depends on r.  Besides,
 * all transitions may have labels of the forms (p&q) or !(p&q).
 */
template<typename Tran>
void LTS<Tran>::letters(StateSet const & stateset, vector<BDD> & lv) const
{
  Cudd mgr = getManager();
  //set<BDD, Util::BddCompare> letterset{mgr.bddOne()};
  set<BDD, Util::BddCompare> letterset{lv.begin(), lv.end()};
  if (letterset.empty()) {
    letterset.insert(mgr.bddOne());
  }
  set<BDD, Util::BddCompare> newletters;
  for (State s : stateset) {
    typename map<State,Delta>::const_iterator dit = transitions.find(s);
    if (dit != transitions.end()) {
      Delta const & delta = dit->second;
      for (Transition const & t : delta) {
        if (t.label == epsilon) continue;
        for (BDD ltr : letterset) {
          BDD pltr = ltr & t.label;
          if (!(pltr.IsZero())) {
            if (newletters.find(pltr) == newletters.end()) {
              newletters.insert(pltr);
            }
          }
          BDD nltr = ltr & !(t.label);
          if (!(nltr.IsZero())) {
            if (newletters.find(nltr) == newletters.end()) {
              newletters.insert(nltr);
            }
          }
        }
        letterset = newletters;
        newletters.clear();
      }
    }
  }
  if (hasEpsilonTransitions()) {
    letterset.insert(epsilon);
  }
  lv.clear();
  lv.insert(lv.end(), letterset.begin(), letterset.end());
}


template<typename Tran>
vector<BDD> LTS<Tran>::getLetters(vector<State> const & statevec) const
{
  vector<BDD> v;
  letters(StateSet(statevec.begin(), statevec.end()), v);
  return v;
}


template<typename Tran>
bool LTS<Tran>::isDeterministic(bool complete) const
{
  // Check whether the labels of the transitions out of
  // each state are disjoint and add up to true.
  if (states.size() == 0) return !complete;
  for (State state : states) {
    BDD whole = mgr.bddZero();
    typename map<State,Delta>::const_iterator dit = transitions.find(state);
    if (dit != transitions.end()) {
      Delta const & delta = dit->second;
      for (Transition const & t : delta) {
        BDD const & label = t.label;
        if (label <= !whole) {
          whole |= label;
        } else {
          if (verbosity) {
            cout << "overlapping labels for state " << state
                 << ": " << label << "," << t.destination << endl;
          }
          return false;
        }
      }
    }
    if (complete && !whole.IsOne()) {
      if (verbosity) {
        cout << "incomplete transitions for state " << state
             << ": " << whole << endl;
      }
      return false;
    }
  }
  return true;
}


template<typename Tran>
bool LTS<Tran>::isComplete(void) const
{
  // Check whether the labels of the transitions out of
  // each state add up to true.
  if (states.size() == 0) return false;
  for (State state : states) {
    BDD whole = mgr.bddZero();
    typename map<State,Delta>::const_iterator dit = transitions.find(state);
    if (dit != transitions.end()) {
      Delta const & delta = dit->second;
      for (Transition const & t : delta) {
        BDD const & label = t.label;
        whole |= label;
      }
    }
    if (!whole.IsOne()) {
      if (verbosity) {
        cout << "incomplete transitions for state " << state
             << ": " << whole << endl;
      }
      return false;
    }
  }
  return true;
}

template<typename T>
ostream & operator<<(ostream & os, LTS<T> const & l)
{
  l.print(os);
  return os;
}

template<typename Tran>
void LTS<Tran>::print(ostream & os) const
{
  os << "Structure\n";
  for (State s : states) {
    if (s == initialState)
      os << "=> ";
    else
      os << "   ";
    os << s << " {";
    auto dit = transitions.find(s);
    if (dit != transitions.end()) {
      Delta const & delta = dit->second;
      char sep[] = ", ";
      char * psep = sep + 1;
      for (Transition const & t : delta) {
        os << psep << t;
        psep = sep;
      }
    }
    os << " } \n";
  }
}


/**
   @brief Class to compute the (maximal) SCCs of the LTS.

   @details Implements Tarjan's algorithm.  Only SCCs reachable from
   some state in "init" are returned.  Only states that are in
   "restriction" and edges that are not in "forbiddenEdges" are
   visited.
*/
template<typename Tran>
class LTS<Tran>::SccAnalyzer {
public:
  SccAnalyzer(LTS<Tran> const & lts, SCCs & sccs,
              StateSet const & init, StateSet const & restriction,
              EdgeSet const & forbiddenEdges) :
    lts(lts), count(0), sccs(sccs), init(init), restriction(restriction),
    forbiddenEdges(forbiddenEdges) {}

  void search(void)
  {
    for (State state : init) {
      if (!visited(state))
        searchScc(state);
    }
  }

private:
  struct DfsNode {
    DfsNode(bool onStack, unsigned dfnum, unsigned lnum) :
      stacked(onStack), dfn(dfnum), low(lnum) {}
    bool stacked;
    unsigned dfn;
    unsigned low;
  };

  using Info = unordered_map<State,DfsNode>;

#if 1
  void searchScc(State state) {
    // "Recursion" stack.
    using Witem = std::pair<State,size_t>;
    std::stack<Witem> work{{Witem{state,0}}};

    while (!work.empty()) {
      Witem const & edge = work.top();
      work.pop();

      if (edge.second == 0) {
        // This is the first time we look at this state.
        visit(edge.first);
      }
      DfsNode & vic = ndInfo.find(edge.first)->second;
      // Look for an unexplored edge.
      bool explore = false;
      auto lit = lts.transitions.find(edge.first);
      Delta const & delta = lit->second;
      for (size_t j = edge.second; j < delta.size(); ++j) {
        Transition const & t = delta[j];
        // Check whether restrictions on edges and states are satisfied.
        auto ait = forbiddenEdges.find({edge.first,t});
        if (ait != forbiddenEdges.end()) continue;
        State succ = t.destination;
        if (!inBounds(succ)) continue;
        // Check whether edge is unexplored.
        auto const sit = ndInfo.find(succ);
        if (sit == ndInfo.end()) {
          // Unexplored edge.  Push the next edge for the current state
          // and the first edge for the new state on the work stack.
          // The order is fixed by the need to search depth-first.
          work.push(Witem{edge.first,j+1});
          work.push(Witem{succ,0});
          explore = true;
          break;
        } else {
          // Explored edge.  Update the low index of the current state.
          DfsNode const & ic = sit->second;
          if (ic.dfn < vic.low && ic.stacked) {
            vic.low = ic.dfn;
          }
        }
      }
      if (explore) continue;
      // All edges out of the current state have been explored.
      if (sccEntryPoint(vic.dfn, vic.low)) {
        collectScc(edge.first);
      }
      // Update the low index of the parent state.
      if (!work.empty()) {
        Witem const & top = work.top();
        DfsNode & tic = ndInfo.find(top.first)->second;
        if (vic.low < tic.low) {
          tic.low = vic.low;
        }
      }
    }
  }
#else
  // This is the old recursive implementation.
  unsigned searchScc(State state) {
    unsigned low = count;
    unsigned dfn = visit(state);
    auto lit = lts.transitions.find(state);
    if (lit != lts.transitions.end()) {
      Delta const & delta = lit->second;
      for (Transition const & t : delta) {
        auto ait = forbiddenEdges.find({state,t});
        if (ait != forbiddenEdges.end()) continue;
        State succ = t.destination;
        if (inBounds(succ)) {
          auto const sit = ndInfo.find(succ);
          if (sit == ndInfo.end()) {
            unsigned sulow = searchScc(succ);
            // Update low link from low link.
            if (sulow < low) { low = sulow; }
          } else {
            // Update low link from DFS number.
            DfsNode const & ic = sit->second;
            if (ic.dfn < low && ic.stacked) {
              low = ic.dfn;
            }
          }
        }
      }
    }
    if (sccEntryPoint(dfn, low)) {
      collectScc(state);
    }
    return low;
  }
#endif

  bool visited(State state) const
  {
    return ndInfo.find(state) != ndInfo.end();
  }

  unsigned visit(State state)
  {
    stk.push(state);
    ndInfo.emplace(state,DfsNode{true,count,count});
    return count++;
  }

  bool sccEntryPoint(unsigned dfn, unsigned low) const
  {
    return dfn == low;
  }

  void collectScc(State state)
  {
    sccs.emplace_back(StateSet{});
    StateSet & component = sccs.back();
    State x;
    do {
      x = stk.top();
      stk.pop();
      DfsNode & ix = ndInfo.find(x)->second;
      ix.stacked = false;
      component.insert(x);
    } while (x != state);
  }

  bool inBounds(State state) const
  {
    return restriction.find(state) != restriction.end();
  }

  LTS<Tran> const & lts;
  stack<State,vector<State>> stk;
  Info ndInfo;
  unsigned count;
  SCCs & sccs;
  StateSet const & init;
  StateSet const & restriction;
  EdgeSet const & forbiddenEdges;
};

template<typename Tran>
typename LTS<Tran>::SCCs LTS<Tran>::getSCCs(
  StateSet const & init,
  StateSet const & restriction,
  EdgeSet const & forbiddenEdges) const
{
  SCCs sccs;
  SccAnalyzer sa(*this, sccs, init, restriction, forbiddenEdges);
  sa.search();
  return sccs;
}

template<typename Tran>
typename LTS<Tran>::SCCs LTS<Tran>::getSCCs(
  StateSet const & init,
  StateSet const & restriction) const
{
  return getSCCs(init, restriction, EdgeSet{});
}


template<typename Tran>
typename LTS<Tran>::SCCs LTS<Tran>::getSCCs(StateSet const & restriction) const
{
  return getSCCs(restriction, restriction);
}


template<typename Tran>
typename LTS<Tran>::SCCs LTS<Tran>::getSCCs(void) const
{
  if (numStates() == 0) {
    return SCCs{StateSet{}};
  }
  StateSet init({initialState});
  StateSet restriction(states.begin(),states.end());
  return getSCCs(init, restriction);
}


template<typename Tran>
void LTS<Tran>::findTransient(vector<State> & transient,
                        StateSet const & init,
                        StateSet const & restriction) const
{
  transient.clear();
  SCCs sccs = getSCCs(init, restriction);
  for (StateSet const & scc : sccs) {
    if (scc.size() == 0)
      throw logic_error("Empty SCC.");
    if (isTrivial(scc))
      transient.push_back(*scc.begin());
  }
  sort(transient.begin(), transient.end());
}

template<typename Tran>
void LTS<Tran>::findTransient(vector<State> & transient,
                        StateSet const & restriction) const
{
  findTransient(transient, restriction, restriction);
}

template<typename Tran>
void LTS<Tran>::findTransient(vector<State> & transient) const
{
  StateSet init({initialState});
  StateSet restriction(states.begin(), states.end());
  findTransient(transient, init, restriction);
}

/**
   @brief Class to compute partition refinement for the states of an LTS.
*/
template<typename Tran>
class LTS<Tran>::PartitionRefinement {
public:
  // Constructor.
  PartitionRefinement(LTS<Tran> const & automaton, map<State,unsigned> const & outputClass);
  // Refine partition.
  void hopcroft(void);
  // Print the current state partition.
  void printPartition(void) const;
  // Get the results of refinement in the form of replacement map.
  map<State,State> getReplacementMap(void) const;

private:
  struct BlockElement {
    BlockElement(int prev = -1, int next = -1, int block = -1) :
      prev(prev), next(next), block(block) {}
    int prev; // predecessor in block list (-1 if none)
    int next; // successor in block list (-1 if none)
    int block; // block that contains this state (-1 if none)
  };

  struct PartitionBlock {
    PartitionBlock(int nletters, int listIndex = -1, int elements = -1, int size = 0) :
      listIndex(listIndex), elements(elements), size(size),
      isWaiting(nletters, false) {}
    int listIndex; // where is it in BlockList? (-1 if not present)
    int elements;  // list header (-1 if list empty)
    size_t size;   // number of states in block
    vector<bool> isWaiting; // one flag per letter
  };

  struct BlockListElement {
    BlockListElement(int block = 0) : block(block) {}
    int block; // block index in partition vector
    vector<int> intersection; // with splitter
  };

  // For every destination state there is a vector that associates to
  // each letter a vectors of sources.
  using Inverse = vector< vector< vector<int> > >;
  // Each element is a pair (block, letter).
  using WaitingList = deque< pair<int, int> >;

  // Helper functions.
  void invertTable(void);
  void printInverse(void) const;
  void initWaiting(WaitingList & waiting);
  void getSplitter(WaitingList & waiting, int & block, int & letter);
  void buildBlockList(int splitter, int letter);
  void splitBlock(vector<int> const & intersection, int j, int q);
  void updateWaiting(WaitingList & waiting, int j, int q);
  void resetJList(void);

  LTS<Tran> const & automaton;
  vector<StateSet> const outputPartition;
  vector<BDD> letters;
  vector<PartitionBlock> partition;
  vector<BlockElement> elements;
  Inverse inverseTable;
  vector<BlockListElement> BlockList;
};


template<typename Tran>
LTS<Tran>::PartitionRefinement::PartitionRefinement(
  LTS<Tran> const & automaton,
  map<State,unsigned> const & outputClass) :
  automaton(automaton),
  outputPartition(Util::mapToPartition(outputClass)),
  letters(automaton.getLetters(automaton.states)),
  partition(outputPartition.size(), PartitionBlock(letters.size())),
  elements(automaton.numStates())
{
  // Walk backward because states are linked to front of lists.
  for (int i = automaton.numStates() - 1; i >= 0; --i) {
    State s = automaton.states.at(i);
    int index = outputClass.at(s);
    BlockElement & e = elements.at(i);
    PartitionBlock & p = partition.at(index);
    e.block = index;
    e.next = p.elements;
    if (e.next != -1)
      elements[e.next].prev = i;
    e.prev = -1; // redundant
    p.elements = i;
    ++(p.size);
  }
  invertTable();
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::invertTable(void) {
  vector< vector<int> > row(letters.size());
  inverseTable.resize(automaton.numStates(), row);
  map<State, size_t> invmap;
  for (size_t state = 0; state != automaton.numStates(); ++state) {
    State s = automaton.states.at(state);
    invmap.insert({s,state});
  }
  for (size_t state = 0; state != automaton.numStates(); ++state) {
    State s = automaton.states.at(state);
    Delta const & delta = automaton.transitions.at(s);
    for (size_t letter = 0; letter != letters.size(); ++letter) {
      BDD const & l = letters.at(letter);
      for (Transition const & t : delta) {
        if (l <= t.label) {
          inverseTable[invmap[t.destination]][letter].push_back(state);
        }
      }
    }
  }
  if (automaton.verbosity)
    printInverse(); // diagnostic
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::printInverse(void) const {
  for (size_t state = 0; state != automaton.numStates(); ++state) {
    cout << "state " << automaton.states.at(state) << ":";
    for (size_t letter = 0; letter != letters.size(); ++letter) {
      cout << " " << letters.at(letter) << " ->";
      for (int dest : inverseTable[state][letter])
        cout << " " << automaton.states.at(dest);
      if (letter < letters.size() - 1)
        cout << ",";
    }
    cout << endl;
  }
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::hopcroft(void)
{
  WaitingList waiting;
  initWaiting(waiting);
  // Refine.
  while (!waiting.empty()) {
    // Get splitter block and letter from waiting.
    int block, letter;
    getSplitter(waiting, block, letter);
    // Find blocks that intersect the preimage of the splitter.
    buildBlockList(block, letter);
    for (size_t i = 0; i != BlockList.size(); ++i) {
      int j = BlockList[i].block;
      vector<int> const & intersection = BlockList[i].intersection;
      // Skip if block contained in preimage of splitter.
      if (intersection.size() == partition[j].size) continue;
      if (automaton.verbosity)
        cout << "Splitting block " << j << " with letter " << letter << endl;
      int q = partition.size();
      splitBlock(intersection, j, q);
      updateWaiting(waiting, j, q);
      if (automaton.verbosity)
        printPartition(); // diagnostic
    }
    resetJList();
  }
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::initWaiting(WaitingList & waiting)
{
  size_t largest = 0;
  size_t maxsize = 0;
  for (size_t i = 0; i != partition.size(); ++i) {
    if (partition.at(i).size > maxsize) {
      maxsize = partition.at(i).size;
      largest = i;
    }
  }
  for (size_t i = 0; i != partition.size(); ++i) {
    if (i == largest) continue;
    if (partition.at(i).size == 0) continue;
    for (size_t l = 0; l != letters.size(); ++l) {
      waiting.push_back({i, l});
      partition[i].isWaiting[l] = true;
    }
  }
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::getSplitter(WaitingList & waiting, int & block, int & letter)
{
  pair<int, int> splitter = waiting.front();
  waiting.pop_front();
  block = splitter.first;
  letter = splitter.second;
  partition[block].isWaiting[letter] = false;
  if (automaton.verbosity) {
    cout << "splitter " << block << "/" << letter << ": ";
    for (int e = partition[block].elements; e != -1; e = elements[e].next)
      cout << " " << e;
    cout << endl;
  }
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::buildBlockList(int splitter, int letter)
{
  for (int i = partition[splitter].elements;
       i != -1; i = elements[i].next) {
    vector<int> const & preimg = inverseTable[i][letter];
    for (size_t j = 0; j != preimg.size(); ++j) {
      int state = preimg[j];
      int block = elements[state].block;
      int index = partition[block].listIndex;
      if (index == -1) {
        index = BlockList.size();
        BlockList.push_back(BlockListElement(block));
        partition[block].listIndex = index;
      }
      BlockList[index].intersection.push_back(state);
    }
  }
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::splitBlock(vector<int> const & intersection, int j, int q)
{
  partition.push_back(PartitionBlock(letters.size(), -1, -1,
                                     intersection.size()));
  partition[j].size -= intersection.size();
  if (partition[j].size == 0)
    throw logic_error("Trying to create empty block!");
  for (size_t i = intersection.size(); i != 0; --i) {
    int state = intersection[i-1];
    BlockElement & e = elements[state];
    // Unlink this element from its current block list.
    if (e.prev != -1)
      elements[e.prev].next = e.next;
    else
      partition[j].elements = e.next;
    if (e.next != -1)
      elements[e.next].prev = e.prev;
    // Add this element to the new block.
    e.next = partition[q].elements;
    e.prev = -1;
    if (e.next != -1)
      elements[e.next].prev = state;
    partition[q].elements = state;
    e.block = q;
  }
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::updateWaiting(WaitingList & waiting, int j, int q)
{
  for (size_t l = 0; l != letters.size(); ++l) {
    if (partition[j].isWaiting[l] ||
        partition[q].size < partition[j].size) {
      waiting.push_back({q, l});
      partition[q].isWaiting[l] = true;
    } else {
      waiting.push_back({j, l});
      partition[j].isWaiting[l] = true;
    }
  }
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::resetJList(void)
{
  for (size_t i = 0; i != BlockList.size(); ++i) {
    partition[BlockList[i].block].listIndex = -1;
  }
  BlockList.resize(0);
}


template<typename Tran>
void LTS<Tran>::PartitionRefinement::printPartition(void) const
{
  for (size_t i = 0; i != partition.size(); ++i) {
    for (int e = partition.at(i).elements; e != -1; e = elements.at(e).next) {
      cout << " " << automaton.states.at(e);
      if (elements.at(e).next != -1 && elements.at(elements.at(e).next).prev != e)
        throw logic_error("Unexpected!");
    }
    if (i + 1 != partition.size())
      cout << " |";
  }
  cout << endl;
}


template<typename Tran>
map<State,State> LTS<Tran>::PartitionRefinement::getReplacementMap(void) const
{
  map<State,State> rm;
  for (size_t i = 0; i != partition.size(); ++i) {
    if (partition.at(i).size == 1) continue;
    int rep = partition.at(i).elements;
    State repstate = automaton.states.at(rep);
    for (int e = elements[rep].next; e != -1; e = elements.at(e).next) {
      if (e != rep)
        rm.insert({automaton.states.at(e), repstate});
    }
  }
  return rm;
}


template<typename Tran>
map<State,State> LTS<Tran>::stateEquivalence(map<State,unsigned> const & outputClass) const
{
  PartitionRefinement pr(*this, outputClass);
  pr.hopcroft();
  if (verbosity)
    pr.printPartition();
  return pr.getReplacementMap();
}


template<typename Tran>
void LTS<Tran>::trim(StateSet const & targets, SCCs const & sccs)
{
  if (states.size() == 0) return;
  // Retain only states that can reach nontrivial SCCs that
  // contain target states.  Here we rely on the fact that the SCCs
  // are returned in a reverse topological order.
  StateSet trimstates;
  for (StateSet const & scc : sccs) {
    if (!isTrivial(scc) && !disjoint(scc, targets)) {
      trimstates |= scc;
    } else {
      bool pathToFairCycle = false;
      for (State s : scc) {
        auto dit = transitions.find(s);
        if (dit != transitions.end()) {
          Delta const & delta = dit->second;
          for (Transition const & t : delta) {
            if (!t.label.IsZero()
                && trimstates.find(t.destination) != trimstates.end()) {
              pathToFairCycle = true;
              trimstates |= scc;
              break;
            }
          }
        }
        if (pathToFairCycle) break;
      }
    }
  }
  for (State s : states) {
    if (trimstates.find(s) == trimstates.end()) {
      transitions.erase(s);
    } else {
      auto dit = transitions.find(s);
      if (dit != transitions.end()) {
        Delta & delta = dit->second;
        typename vector<Transition>::iterator it = delta.begin();
        while (it != delta.end()) {
          if (trimstates.find(it->destination) == trimstates.end())
            it = delta.erase(it);
          else
            ++it;
        }
      }
    }
  }
  states = vector<State>(trimstates.begin(), trimstates.end());
}


template<typename Tran>
void LTS<Tran>::trim(StateSet const & targets)
{
  SCCs sccs = getSCCs();
  trim(targets, sccs);
}


template<typename Tran>
void LTS<Tran>::printDot(std::string graphname) const
{
  Util::replaceAll(graphname, "\n", "\\n");
  cout << "digraph \"" << graphname << "\" {\nnode [shape=ellipse];\n"
       << "\"title\" [label=\"" << graphname << "\",shape=plaintext];\n";
  for (State i : states) {
    if (i == initialState) {
      cout << "\"" << i << "_init\" [style=invis]\n\"" << i << "_init\" -> " << i << "\n";
    }
    cout << i << " [label=\"" << i << "\"];\n";
  }
  for (State i : states) {
    auto it = transitions.find(i);
    if (it != transitions.end()) {
      Delta const & delta = it->second;
      for (Transition const & t : delta) {
        cout << i << " -> " << t.destination << " [label=\"" << t.label
             << "\"];\n";
      }
    }
  }
  cout << "}" << endl;
}


// Checks whether an SCC is trivial under the assumption
// that the states in scc actually form an SCC.
// Ignores forbidden edges.
template<typename Tran>
bool LTS<Tran>::isTrivial(StateSet const & scc,
                    EdgeSet const & forbidden) const
{
  if (scc.size() != 1) return false;
  State state = *scc.begin(); // grab only state
  auto dit = transitions.find(state);
  if (dit == transitions.end()) {
    return true;
  }
  Delta const & delta = dit->second;
  for (Transition const & t : delta) {
    if (forbidden.find({state, t}) != forbidden.end()) {
      continue;
    }
    if (t.destination == state) {
      return false;
    }
  }
  return true;
}

template<typename Tran>
void LTS<Tran>::pruneVarSet(void)
{
  set<BDD, Util::BddCompare> usedVars;
  bool hasEpsilon = false;
  for (State s : states) {
    typename map<State,Delta>::const_iterator it = transitions.find(s);
    if (it != transitions.end()) {
      vector<Transition> const & delta = it->second;
      for (Transition t : delta) {
        vector<unsigned> support = t.label.SupportIndices();
        for (unsigned index : support) {
          BDD const var = mgr.bddVar(index);
          if (var == epsilon) {
            hasEpsilon = true;
          } else {
            usedVars.insert(mgr.bddVar(index));
          }
        }
      }
    }
  }
  varSet = usedVars;
  if (!hasEpsilon) {
    epsilon = mgr.bddZero();
  }
}


/** @brief Helper class to perform Rabin-Scott subset construction. */
template<typename Tran>
class LTS<Tran>::SubsetConstruction {
public:
  using SubsetMap = map<State,StateSet>;
  SubsetConstruction(LTS<Tran> const & lts, SubsetMap & subsets) :
    lts(lts), subsets(subsets) {}
  StateSet computeNextSubset(State source, BDD const & letter);
  bool findOrAdd(StateSet const & newsub, State & refsub);
  void mergeOrAdd(Delta & delta, BDD const & label, State successor);
  StateSet getSubset(State s) const;
private:
  LTS<Tran> const & lts;
  SubsetMap & subsets;
};


template<typename Tran>
StateSet
LTS<Tran>::SubsetConstruction::getSubset(State s) const
{
  auto it = subsets.find(s);
  if (it == subsets.end()) {
    throw logic_error("invalid subset index: " + to_string(s));
  }
  return it->second;
}


template<typename Tran>
StateSet LTS<Tran>::SubsetConstruction::computeNextSubset(
  State source, BDD const & letter)
{
  StateSet subset = subsets.at(source);
  StateSet newsub; 

  for (State s : subset) {
    auto it = lts.transitions.find(s);
    if (it != lts.transitions.end()) {
      Delta const & delta = it->second;
      for (auto const & trans : delta) {
        if (letter <= trans.label) {
          newsub.insert(trans.destination);
        }
      }
    }
  }
  return newsub;
}


template<typename Tran>
bool LTS<Tran>::SubsetConstruction::findOrAdd(
  StateSet const & newsub, State & refsub)
{
  for (auto it = subsets.cbegin(); it != subsets.cend(); ++it) {
    StateSet const & sub = it->second;
    if (sub == newsub) {
      refsub = it->first;
      return true;
    }
  }
  refsub = subsets.size();
  subsets.insert({refsub, newsub});
  return false;
}


template<typename Tran>
void LTS<Tran>::SubsetConstruction::mergeOrAdd(
  Delta & delta, BDD const & label, State successor)
{
  // Inductively, there's at most another transition with the same destination.
  for (auto & tran : delta) {
    if (tran.destination == successor) {
      tran.label |= label;
      return;
    }
  }
  Transition newtran;
  newtran.destination = successor;
  newtran.label = label;
  delta.push_back(newtran);
}


template<typename Tran>
LTS<Tran> LTS<Tran>::subsetsLTS(map<State,StateSet> & subsets) const
{
  LTS<Tran> sublts{mgr};
  sublts.setVerbosity(verbosity);

  SubsetConstruction sc(*this, subsets);

  StateSet isub;
  if (numStates() > 0) {
    isub.insert(getInitial());
  }
  subsets.insert({0, isub});
  sublts.varSet = varSet; // same alphabet as the LTS

  // Initialize queue.
  queue<State> q;
  q.push(0);

  while (!q.empty()) {
    State sindex = q.front();
    q.pop();
    StateSet s = subsets.at(sindex);
    if (verbosity) {
      cout << "Subset " << sindex << ": " << s << endl;
    }
    Delta delta;
    vector<BDD> ltrs;
    if (s.size() > 0) {
      letters(s, ltrs);
    } else {
      ltrs.push_back(mgr.bddOne());
    }
    for (BDD const & ltr : ltrs) {
      StateSet newsub{sc.computeNextSubset(sindex, ltr)};
      State refsub;
      bool found = sc.findOrAdd(newsub, refsub);
      if (!found) {
        q.push(refsub);
      }
      if (verbosity) {
        cout << "Letter: " << ltr << " refsub(" << (found ? "old" : "new")
             << ") " << subsets.at(refsub) << endl;
      }
      sc.mergeOrAdd(delta, ltr, refsub);
    }
    sublts.addState(sindex);
    sublts.transitions.insert({sindex, delta});
  }
  
  sublts.makeInitial(0);
  return sublts;
}


template<typename Tran>
LTS<Tran> LTS<Tran>::product(LTS<Tran> const & second)
{
  LTS<Tran> prod{getManager()};
  if (numStates() == 0 || second.numStates() == 0) {
    return prod;
  }
  // Map product states to pairs of states in first and second.
  map<pair<State,State>,State> cartesian;
  // Create initial state for product.
  State pstate = 0;
  State fstate = getInitial();
  State sstate = second.getInitial();
  pair<State,State> cstate{fstate,sstate};
  cartesian.insert({cstate,pstate});
  prod.addState(pstate);
  prod.makeInitial(pstate);
  pstate++;

  // Create reachable part of product.
  queue<pair<State,State>> q;
  q.push({fstate,sstate});
  while (!q.empty()) {
    cstate = q.front();
    q.pop();
    fstate = cstate.first;
    sstate = cstate.second;
    auto cit = cartesian.find(cstate);
    if (cit == cartesian.end()) {
      throw logic_error("unexpected product state");
    }
    State current = cit->second;
    // Get transition labels.
    vector<BDD> labels;
    letters(StateSet{fstate}, labels);
    second.letters(StateSet{sstate}, labels);
    // Find all successor states.
    for (BDD const & letter : labels) {
      StateSet fnext = getSuccessors(fstate, letter);
      StateSet snext = second.getSuccessors(sstate, letter);
      for (State fn : fnext) {
        for (State sn : snext) {
          State nextstate;
          auto pit = cartesian.find({fn,sn});
          if (pit == cartesian.end()) {
            cartesian.insert({{fn,sn},pstate});
            nextstate = pstate;
            prod.addState(nextstate);
            q.push({fn,sn});
            pstate++;
          } else {
            nextstate = pit->second;
          }
          prod.addTransition(current, nextstate, letter);
        }
      }
    }
  }
  prod.mergeTransitions();
  return prod;
}


/* Force instantition of class. */
#include "Parity.hh"

template class LTS<ParityTransition>;
