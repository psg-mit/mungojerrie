#ifndef PARITY_HH_
#define PARITY_HH_

/** @file Parity.hh

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

#include "Verbosity.hh"
#include "LTS.hh"

using Priority = short int;

/**
   @brief Class of parity-annotated transitions.
*/
class ParityTransition : public BaseTransition {
  friend class Parity;
  friend std::ostream & operator<<(std::ostream & os, ParityTransition const & p);
public:
  ParityTransition() {}
  ParityTransition(State d, BDD l, Priority p) :
    BaseTransition(d,l), priority(p) {}
  ParityTransition(State d, BDD l) : ParityTransition(d,l,0) {}
  /** @brief Checks whether two transitions have the same endpoints, label,
      and priority.
  */
  bool operator==(ParityTransition const & right) const;
  /** @brief Comparison function used to store transitions in ordered
      containers.
  */
  bool operator<(ParityTransition const & right) const;
  /* @brief Returns the priority of a transition. */
  Priority getPriority(void) const { return priority; }
private:
  Priority priority;
};

/** @brief Overloads stream insertion operator. */
std::ostream & operator<<(std::ostream & os, ParityTransition const & p);

/**
   @brief Class of parity automata.

   @details Accepting runs have odd maximum recurring priority.
   Priorities are attached to the transitions.
*/
class Parity: public LTS<ParityTransition> {
public:

  struct DotAttributes {
    std::map<State, std::string> nodeShapes;
    std::map<State, std::string> nodeColors;
    std::map<State, std::string> nodeLabels;
    std::map<std::pair<State, State>, std::string> edgeColors;
  };
  /** @brief Constructor. */
  Parity(Cudd mgr);
  /** @brief Constructor from HOA file. */
  Parity(Cudd mgr, std::string const & filename,
         Verbosity::Level verbosity = Verbosity::Silent, bool noMdp = false);
  /** @brief Constructor from HOA stream. */
  Parity(Cudd mgr, std::istream & is,
         Verbosity::Level verbosity = Verbosity::Silent, bool noMdp = false);
  /** @brief Adds a transition to a state of the automaton. */
  void addTransition(State source, State destination,
                     BDD label, Priority priority);
  /** @brief Converts automaton to dot format. */ 
  void printDot(std::string graphname = std::string("parity"), 
                std::string filename = std::string("-"),
                bool showtrees = false) const;
  /** @brief Converts automaton to dot format. */
  void printDot(std::string graphname,
                DotAttributes const & attributes, 
                std::string filename = std::string("-"), 
                bool showtrees = false) const;
  /** @brief Writes automaton in HOA format. */
  void printHOA(std::string const & name = std::string("parity"),
                std::string filename = std::string("-")) const;
  /** @brief Gets the priority of a transition.

      @details In silent mode the function returns -1 if no
      transition with the given endpoints and label exists.
   */
  Priority getPriority(State source, BDD label,
                       State destination, bool silent = false) const;
  /** @brief Gets maximum priority of automaton. */
  Priority getMaxPriority(void) const;
  /** @brief Gets minimum priority of automaton. */
  Priority getMinPriority(void) const;
  /** @brief Returns the partition of the edges according to their priorities. */
  std::vector<EdgeSet> edgePartition(void) const;
  /** @brief Classify states according to the priorities of their transitions. */
  std::map<State,unsigned> classifyStates(void) const;
  /** @brief Returns determinized automaton. */
  Parity determinize(void) const;
  /** @brief Reduces index of acceptance condition.
   *  @return the length of the longest positive chain. */
  unsigned CartonMaceiras(void);
  /** @brief Reduces index of acceptance condition.
   *  @return the length of the longest positive chain.
      
   *  @details Avoids use of priority 0 if possible. */
  unsigned minimumIndex(void);
  /** @brief Converts DPW to LDPW with priorities 0 and 1. */
  Parity toLDPW(bool epsilonJumps = false) const;
  /** @brief Computes direct simulation relation. */
  void directSimulationRelation(std::set<StatePair> & simul) const;
  /** @brief Minimizes automaton based on direct simulation. */
  void directSimulationMinimization(void);
  /** @brief Minimizes deterministic automaton based on state equivalence. */
  void stateMinimization(void);
  /** @brief Trims automaton of states on no accepting path. */
  void trim(void);
  /** @brief Adds rejecting trap if necessary to make automaton complete. */
  void makeComplete(void);
  /** @brief Decomposes automaton into safety and liveness components. */
  void safetyLiveness(Parity & safety, Parity & liveness) const;
  /** @brief Checks whether the automaton is trivially empty. */
  bool isTriviallyEmpty(void) const;
  /** @brief Checks whether the automaton is terminal.
   *  @details If ignoreTransient is false, it returns
   *  false if there are transient accepting transitions. */
  bool isTerminal(bool ignoreTransient = true) const;
  /** @brief Returns set of trap states.
   *  @details No word is accepted from a trap state. */
  StateSet getTrapStates(void) const;
  /** @brief Takes the nondeterministic union with another automaton. */
  void unite(Parity const & second);
  /** @brief Return complement of automaton. */
  Parity complement(void) const;
  /** @brief Solve one-Streett-pair game.
   *  @return Strategy for duplicator. */
  void solveOneStreettPairGame(StateSet const & spoilerStates,
                               StateSet & dwinning,
                               StateMap & dstrategy,
                               StateMap & sstrategy) const;
  /** @brief Checks whether automaton lead-simulates itself. */
  bool leadSimulates(void) const;
  /** @brief Checks whether automaton fair-simulates other. */
  bool fairSimulates(Parity const & other) const;
  /** @brief Checks whether automaton direct-simulates other. */
  bool directSimulates(Parity const & other) const;
private:
  /** @brief Constructor from base-class object. */
  Parity(LTS const & lts);
  Priority CartonMaceirasRecur(StateSet const & Q, EdgeSet forbiddenEdges);
  bool zeroToTwo(unsigned maxpriority);
  bool directCompareStates(State p, State q,
                           std::set<StatePair> const & simul) const;
  void addTrap(unsigned priority);
  void makeSafety(bool wascomplete);
  Parity buildEnvironment(void) const;
  void makeLiveness(void);
  TransitionMap inverseTransitions(void) const;
  Parity buildLeadSimulationGame(StateSet & spoilerStates,
                                 DotAttributes & attributes) const;
  Parity buildFairSimulationGame(Parity const & other,
                                 StateSet & spoilerStates,
                                 DotAttributes & attributes) const;
  Parity buildDirectSimulationGame(Parity const & other,
                                   StateSet & spoilerStates,
                                   DotAttributes & attributes) const;
  class Determ;
  friend class Determ;
  class Simul;
  friend class Simul;
  class OSPGame;
  friend class OSPGame;
};

/** @brief Overloads stream insertion operator. */
std::ostream & operator<<(std::ostream & os, Parity const & p);

#endif
