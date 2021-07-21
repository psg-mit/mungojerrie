#ifndef LTS_HH_
#define LTS_HH_

/** @file LTS.hh

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

#include <string>
#include "cuddObj.hh"
#include "State.hh"

/**
   @brief Base class for the labeled edges of the transition graph.
*/
class BaseTransition {
  template<typename T> friend class LTS;
public:
  BaseTransition() {}
  BaseTransition(State d, BDD l) : destination(d), label(l) {}
  /** @brief Checks whether two transitions have the same endpoints and label.
  */
  bool operator==(BaseTransition const &) const = delete;
  /** @brief Comparison function used to store transitions in ordered
      containers.
  */
  bool operator<(BaseTransition const &) const = delete;
  /* @brief Returns the destination of a transition. */
  State getDestination(void) const { return destination; }
  /* @brief Returns a copy of the label of a transition. */
  BDD getLabel(void) const { return label; }  
protected:
  State destination;
  BDD label;
};

/** @brief Overloads stream insertion operator. */
std::ostream & operator<<(std::ostream & os, BaseTransition const & p);

/**
   @brief Class of labeled transition systems.

   @details Automata classes extend this class by adding an acceptance
   condition.
*/
template<typename Tran>
class LTS {
public:
  using Transition = Tran;
  using Edge = std::pair<State,Transition>;
  using EdgeSet = std::set<Edge>;
  using Delta = std::vector<Transition>;
  /** @brief Type of sets of pairs of state and BDD. */
  using PairsStateBdd = std::set< std::pair<State,BDD>, Util::PairWithBddCompare<State>>;
  /** @brief Constant iterator type. */
  using const_iterator = std::vector<State>::const_iterator;
  /** @brief Returns a const_iterator pointing to the first state. */
  const_iterator cbegin() { return states.cbegin(); }
  /** @brief Returns a const_iterator pointing "just past the last" state. */
  const_iterator cend() { return states.cend(); }

  /** @brief Constructor. */
  LTS(Cudd mgr);
  /** @brief Returns the number of states in the LTS. */
  size_t numStates(void) const { return states.size(); }
  /** @brief Returns the BDD manager of the LTS. */
  Cudd getManager(void) const { return mgr; }
  /** @brief Adds a state to the LTS. */
  void addState(State s, Delta delta = Delta{});
  /** @brief Adds a transition to a state of the LTS. */
  void addTransition(State source, State destination, BDD const & label);
  /** @brief Makes a state initial. */
  void makeInitial(State s);
  /** @brief Returns the initial state of the LTS. */
  State getInitial(void) const { return initialState; }
  /** @brief Adds atomic proposition to model. */
  void addAtomicProposition(BDD proposition, std::string name);
  /** @brief Returns name of an atomic proposition. */
  std::string getPropositionName(BDD proposition) const;
  /** @brief Returns the set of atomic propositions. */
  std::set<BDD, Util::BddCompare> getAtomicPropositions(void) const {
    return varSet;
  }
  /** @brief Returns successor of source state for input letter in deterministic LTS. */
  State getSuccessor(State source, BDD letter) const;
  /** @brief Returns states reachable from source for input letter. */
  StateSet getSuccessors(State source, BDD letter) const;
  /** @brief Returns states reachable from source via epsilon transitions. */
  StateSet getEpsilonSuccessors(State source) const;
  /** @brief Checks sanity of transitions. */
  bool checkTransitions(void) const;
  /** @brief Stream insertion operator helper function. */
  void print(std::ostream & os = std::cout) const;
  /** @brief Returns the letters in the labels of a set of states. */
  void letters(StateSet const & stateset, std::vector<BDD> & lv) const;
  /** @brief Alternate interface to letters. */
  std::vector<BDD> getLetters(std::vector<State> const & statevec) const;
  /** @brief Returns the verbosity level of the LTS. */
  int readVerbosity(void) const { return verbosity; }
  /** @brief Sets the verbosity level of the LTS. */
  void setVerbosity(int v) { verbosity = v; }
  /** @brief Checks whether the LTS is deterministic. */
  bool isDeterministic(bool complete = true) const;
  /** @brief Checks whether the LTS is complete. */
  bool isComplete(void) const;
  /** @brief Type of the collection of SCCs. */
  using SCCs = std::vector<StateSet>;
  /** @brief Computes the SCCs reachable from init and contained in restriction. */
  SCCs getSCCs(StateSet const & init, StateSet const & restriction,
               EdgeSet const & forbiddenEdges) const;
  /** @brief Computes the SCCs reachable from init and contained in restriction. */
  SCCs getSCCs(StateSet const & init,
               StateSet const & restriction) const;
  /** @brief Computes the SCCs of the LTS contained in restriction. */
  SCCs getSCCs(StateSet const & restriction) const;
  /** @brief Computes the SCCs of the LTS. */
  SCCs getSCCs(void) const;
  /** @brief Finds transient states of LTS reachable from init and contained in restriction. */
  void findTransient(std::vector<State> & transient, StateSet const & init,
                     StateSet const & restriction) const;
  /** @brief Finds transient states of LTS contained in restriction. */
  void findTransient(std::vector<State> & transient, StateSet const & restriction) const;
  /** @brief Finds transient states of LTS. */
  void findTransient(std::vector<State> & transient) const;
  /** @brief Returns state equivalence relation. */
  std::map<State,State> stateEquivalence(std::map<State,unsigned> const & outputClass) const;
  /** @brief Removes states with no path to target states. */
  void trim(StateSet const & targets);
  /** @brief Removes states with no path to target states.
   *  @details Uses previously computed SCCs. */
  void trim(StateSet const & targets, SCCs const & sccs);
  /** @brief Writes LTS in dot format. */
  void printDot(std::string graphname = std::string("LTS")) const;
  /** @brief Returns the result of the Rabin-Scott subset construction. */
  LTS subsetsLTS(std::map<State,StateSet> & subsets) const;
  /** @brief Sets the epsilon proposition for LDBWs. */
  void setEpsilonBDD(BDD const & e) { epsilon = e; }
  /** @brief Gets the epsilon proposition for LDBWs. */
  BDD const & getEpsilonBDD(void) const { return epsilon; }
  /** @brief Checks whether there exist epsilon transitions. */
  bool hasEpsilonTransitions(void) const { return !epsilon.IsZero(); }
  /** @brief Returns the synchronous product of two LTSs. */
  LTS product(LTS const & second);
protected:
  /** @brief Merge transitions that differ at most in their labels. */
  void mergeTransitions(void);
  /** @brief Check whether an SCC is trivial. */
  bool isTrivial(StateSet const & scc,
                 EdgeSet const & forbidden = EdgeSet{}) const;
  /** @brief Prune the variable set of unused atomic propositions. */
  void pruneVarSet(void);
  using TransitionMap = std::map<State,Delta>;

  Cudd mgr;
  int verbosity;
  std::vector<State> states;
  State initialState;
  TransitionMap transitions;
  std::set<BDD, Util::BddCompare> varSet;
  BDD epsilon;
private:
  class SccAnalyzer;
  class PartitionRefinement;
  class SubsetConstruction;
};

/** @brief Overloads the stream insertion operator. */
template<typename T>
std::ostream & operator<<(std::ostream & os, LTS<T> const & l);

/** @brief Return product of two LTS's. */
template<typename Tran>
LTS<Tran> product(LTS<Tran> const & first, LTS<Tran> const & second);

#endif
