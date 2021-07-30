#ifndef GYM_HH_
#define GYM_HH_

/** @file Gym.hh

  @brief Interface between learner and model/objective.

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

#include "Model.hh"
#include <boost/functional/hash.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

using GymObservation = struct GymObservation {
  friend class boost::serialization::access;
  
  Node first; /// Model state 
  State second; /// Automaton state 
  short int trackerState; /// Priority-tracker state (0 if unused)

  GymObservation(Node first, State second, short int trackerState=0) : first(first), 
                                                                       second(second),
                                                                       trackerState(trackerState) {}
  GymObservation(void) {
    first = 0;
    second = 0;
    trackerState = 0;
  }
  bool operator==(GymObservation const & other) const
  {
    return (first == other.first &&
            second == other.second &&
            trackerState == other.trackerState);
  }
  bool operator<(GymObservation const & other) const
  {
    return (first < other.first) ||
           (first == other.first && second < other.second) ||
           (first == other.first && second == other.second && trackerState < other.trackerState);
  }
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    (void)version;
    ar & first;
    ar & second;
    ar & trackerState;
  }
};

struct GymObservationHasher {
  std::size_t operator()(GymObservation const & observation) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, boost::hash_value(observation.first));
    boost::hash_combine(seed, boost::hash_value(observation.second));
    boost::hash_combine(seed, boost::hash_value(observation.trackerState));
    return seed;
  }
};

struct GymObservationCompare {
  bool operator()(GymObservation const & first, GymObservation const & second) const
  {
    return std::tuple<Node, State, short int>{first.first, first.second, first.trackerState} <
      std::tuple<Node, State, short int>{second.first, second.second, second.trackerState};
  }
};

using GymAction = std::pair<std::pair<Action, State>, State>; /// ((model action, automaton next state), automaton epsilon)
static State constexpr invalidState = ~0;
static std::pair<Action, State> constexpr invalidAction = {-1, -1};
static GymObservation const terminalState = {(Node)-1, (State)-1, (short int)-1};

using Qtype = std::unordered_map<GymObservation, std::map<GymAction, double>, GymObservationHasher>;
using OrderedQtype = std::map<GymObservation, std::map<GymAction, double>, GymObservationCompare>;

struct GymOptions {
  GymOptions() {
    episodeLength = 30;
    zeta = 0.99;
    tolerance = std::vector<double>{0.01};
    rewardType = GymRewardTypes::default_type;
    priEpsilon = 0.001;
    concatActionsInCSV = false;
  }
  enum class GymRewardTypes {
    default_type = 0, /*!< Default type is zeta-reach for 1-1/2 and parity for 2-1/2 players */
    prism = 1, /*!< Reward from PRISM file. Continuing (non-episodic) setting. 
                  Automaton epsilon edges receive zero reward. */
    zeta_reach = 2, /*!< Zeta-based reachability | See [“Omega-Regular Objectives in Model-Free Reinforcement Learning”.](https://link.springer.com/chapter/10.1007/978-3-030-17462-0_27) */
    zeta_acc = 3, /*!< Zeta-based reachability with reward on accepting transitions | See [“Faithful and Effective Reward Schemes for Model-Free Reinforcement Learning of Omega-Regular
Objectives”.](https://link.springer.com/chapter/10.1007%2F978-3-030-59152-6_6) */
    zeta_discount = 4, /*!< Zeta-based discounted reward on accepting transitions | See [“Faithful and Effective Reward Schemes for Model-Free Reinforcement Learning of Omega-Regular
Objectives”.](https://link.springer.com/chapter/10.1007%2F978-3-030-59152-6_6) */
    reward_on_acc = 5, /*!< Reward on each accepting transition. MAY LEAD TO INCORRECT STRATEGIES | See ["A learning based approach to control synthesis of Markov decision processes for linear temporal logic specifications".](https://ieeexplore.ieee.org/document/7039527) */
    multi_discount = 6, /*!< Multi-discount reward | See ["Control Synthesis from Linear Temporal Logic Specifications using Model-Free Reinforcement Learning".](https://ieeexplore.ieee.org/document/9196796) */
    parity = 7, /*!< Reward from parity objectives | See [“Model-Free Reinforcement Learning for Stochastic Parity Games”.](https://drops.dagstuhl.de/opus/volltexte/2020/12833/) */
    pri_tracker = 8 /*!< Reward from priority tracker gadget | See [“Model-Free Reinforcement Learning for Stochastic Parity Games”.](https://drops.dagstuhl.de/opus/volltexte/2020/12833/) */
  };
  unsigned int episodeLength;
  double zeta;
  double gammaB;
  std::vector<double> tolerance;
  double priEpsilon;
  GymRewardTypes rewardType;
  bool noResetOnAcc; /*!< Turns off resetting episode step when an accepting edge is passed 
                          for zeta-reach and zeta-acc (not recommended) */
  bool terminalUpdateOnTimeLimit; /*!< Treat end of episodes that end due to episode limit 
                                       as transitioning to zero value sink (not recommended) */
  bool p1NotStrategic;  /*!< Does not allow player 1 to change their strategy to the optimal 
                             counter-strategy to player 0 during the verification of the 
                             learned strategies. Instead, player 1 uses learned strategy. */
  bool concatActionsInCSV;
};

/** @brief Class for a learning interface. */
class Gym {
public:
  /** @brief Class for encapsulation of return values of Gym. */
  struct GymInfo{
    GymObservation observation;
    Reward reward;
    BDD letter;
    bool done;
    std::vector<GymAction> actions;
    Player player;
    bool discountOverride = false;
    double discount = 0.0;
    bool terminationOverride = false; 
  };

  /** @brief Constructor from model. */
  Gym(Model model, Parity dpw, GymOptions options);
  /** @brief Resets statistics. */
  void resetStats(void);
  /** @brief Prints statistics about a run. */
  void printStats(void) const;
  /** @brief Prints the dot representation of learned pruned product graph. 
   *  @details The first tolerance specified via GymOptions at construction is used.
   *  @param Q Q-table the strategy is extracted from. */
  void printDotLearn(Qtype const & Q, std::string filename = std::string("-")) const;
  /** @brief Prints the PRISM representation of learned pruned product graph. 
   *  @details The first tolerance specified via GymOptions at construction is used.
   *  @param Q Q-table the strategy is extracted from. */
  void printPrismLearn(Qtype const & Q, std::string filename = std::string("-")) const;
  /** @brief Returns the probability of satisfaction in the learned pruned product 
   *  graph for each tolerance specified via GymOptions at construction.
   *  @param Q Q-table the strategy is extracted from.
   *  @param statsOn Indicates verbose printing. 
   *  @returns Map from each tolerance to associated probability of satisfaction. */
  std::map<double, double> getProbabilityOfSat(Qtype const & Q, bool statsOn) const;
  /** @brief Constructs Markov chain from Q-table. 
   *  @param Q Q-table the strategy is extracted from.
   *  @param tolerance Relative fraction from the maximum where values in Q-table 
   *  are considered indistinguishable. A uniform random strategy is produced if multiple
   *  actions are indistinguishable in value.
   *  @param p1strategic If true, then player 1's strategy is not fixed
   *  based on the Q-table. This results in an MDP where only player 1 has choices.
   *  @param statsOn Indicates verbose printing. 
   *  @param priEpsilon Probability used to advance priority tracker. */
  Model QtoModel(Qtype const & Q, double tolerance, bool p1strategic, bool statsOn, double priEpsilon=0.001) const;
  /** @brief Resets environment. */ 
  GymInfo reset(void);
  /** @brief Environment transition. */
  std::vector<std::pair<double, Gym::GymInfo>> transitions(GymAction action) const;
  /** @brief Takes step in environment according to action. */
  GymInfo step(GymAction);
  /** @brief Returns a reference to model. */
  Model const & getModel(void) const {
    return model;
  }
  /** @brief Returns the player controlling state GymObservation. */
  Player getPlayer(GymObservation const & observation) const;
  /** @brief Returns a string representing GymObservation. */
  std::string toString(GymObservation const & observation) const;
  /** @brief Returns a string representing GymAction. */
  std::string toString(GymAction const & action) const;
  /** @brief Returns a string representing Qtype. */
  std::string toString(Qtype const & Q) const;
  /** @brief Saves Q-table. */
  void saveQ(Qtype const & Q, std::string saveType, std::string filename) const;
  /** @brief Saves Q-tables. */
  void saveQ(Qtype const & Q1, Qtype const & Q2, std::string saveType, std::string filename) const;
  /** @brief Loads Q-table. */
  void loadQ(Qtype & Q, std::string saveType, std::string filename) const;
  /** @brief Loads Q-tables. */
  void loadQ(Qtype & Q1, Qtype & Q2, std::string saveType, std::string filename) const;
  /** @brief Saves mixed strategy from Q-table to CSV file for all visited states. */
  void saveStrat(Qtype & Q, std::string filename) const;
  /** @brief Saves mixed strategy from Q-table to file for all visited states. */
  void saveStratBDP(Qtype & Q, std::string filename) const;
  /** @brief Saves the gym as an MDP.*/
  void saveMDP(std::string filename, double discount) const;

private:
  /** @brief Computes an ID from model and automaton.
   *  @details Provides a soft check that the same
   *  model and automaton are used with a Q-table save file. */
  std::size_t getProductID(void) const;
  
  Model model;
  Parity automaton;
  GymOptions::GymRewardTypes rewardType;
  unsigned int episodeStep;
  unsigned int episodeLength;
  double zeta;
  double gammaB;
  std::vector<double> tolerance;
  double priEpsilon;
  Priority maxPriority;
  int numAccept;
  int acceptingEps;
  int trappedEps;
  long numSteps;
  double cumulativeRew;
  long numEpisodes;
  bool resetOnAcc;
  bool noTerminalUpdate;
  bool p1strategic;
  bool concatActionsInCSV;
  Node modelState;
  State autoState;
  short trackerState;
  StateSet traps;
};

#endif
