#ifndef MODEL_HH_
#define MODEL_HH_

/** @file Model.hh

  @brief Markov Decision Processes and Stochastic Games.

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

#include <vector>
#include <set>
#include <string>
#include "Set.hh"
#include "Util.hh"
#include "cuddObj.hh"
#include "config.h"
#include "Parity.hh"
#include "Verbosity.hh"
#include "ModelOptions.hh"

/** @brief Type of nodes. */
using Node = unsigned long;
/**
   @brief Type of priorities.
   @details Valid priorities are nonnegative.
*/
using Priority = short int;
/**
   @brief Type of players.
   @details Strategic players are nonnegative.
*/
using Player = int;
/**
   @brief Type of actions.
   @details Valid actions are nonnegative.
*/
using Action = short int;
/** @brief Type of rewards. */
using Reward = double;
/** @brief Type of probabilities. */
using Probability = double;

/**
   @brief Class for MDPs and related structures.
*/
class Model {
public:
  using Index = unsigned;
  using Propositions = std::set<BDD, Util::BddCompare>;
private:

  /** @brief Class for both decision and probabilistic transitions. */
  struct Transition {
    Transition(Node s, Node d, Action a, Probability p, Reward r, Priority pi = 0) :
      source(s), destination(d), action(a), priority(pi), probability(p), reward(r) {}
    Node source;
    Node destination;
    Action action;
    Priority priority;
    Probability probability;
    Reward reward;
  };

  using Edges = std::vector<Index>;

  /** @brief Class for both decision and probabilistic nodes. */
  struct NodeAttributes{
    NodeAttributes(std::string nm, Player pl) :
      name(nm), player(pl) {}
    std::string name;
    Propositions propositions;
    Player player;
    Edges outEdges;
    Edges inEdges;
  };

  using AttributeMap = std::unordered_map<Node,NodeAttributes>;
  using LabelMap = std::map<std::string, BDD>;
  using TransitionList = std::vector<Transition>;
  using NodeCounts = std::map<Player, size_t>;

public:
  /** @brief Constant iterator type. */
  using const_iterator = AttributeMap::const_iterator;
  /** @brief Returns a const_iterator pointing to the first node. */
  const_iterator cbegin() { return attributes.cbegin(); }
  /** @brief Returns a const_iterator pointing "just past the last" node. */
  const_iterator cend() { return attributes.cend(); }

  /** @brief Class for return value of getStrategy. */
  struct StrategyResults{
    std::unordered_map<Node, Action> strategy;
    std::unordered_map<Node, double> vals;
  };

  /** @brief Class for return value of getAttractorWithStrat. */
  struct AttractorResults{
    std::set<Node> attr;
    std::map<Node, Action> strategy;
  };

  /** @brief Class to store return value of probMcNaughton. */
  struct PlayerSets{
    std::set<Node> w0;
    std::set<Node> w1;
    std::map<Node, Action> strategy;
  };

  /** @brief Constructor. */
  Model(Cudd mgr, ModelOptions options = ModelOptions{});
  /**
   * @brief Constructor from PRISM file.
   * @details The pverbosity parameter is for the parser,
   * not for the model itself.
   */
  Model(Cudd mgr, std::string const & filename, 
        Verbosity::Level pverbosity = Verbosity::Silent,
        ModelOptions options = ModelOptions{});
  /** @brief Constructor for generating model-parity_automaton product graph. */
  Model(Model const & model, Parity const & automaton, ModelOptions options = ModelOptions{});
  /** @brief Parses ModelOptions. */
  void parseOptions(ModelOptions options);
  /** @brief Returns current options. */
  ModelOptions getOptions(void) const;
  /** @brief Returns the total number of nodes in the model. */
  size_t numNodes(void) const { return attributes.size(); }
  /** @brief Returns the number of decision nodes in the model. */
  size_t numDecisionNodes(void) const;
  /** @brief Returns the node counts map without probPlayer. */
  NodeCounts getNodeCounts(void) const;
  /** @brief Returns 1 if the model is a stochastic game. */
  bool isGame(void) const;
  /** @brief Returns 1 if the model is a brnaching decision process. */
  bool isBDP() const {return BDP;}
  /** @brief set model to be a branchign decision process. */
  void setBDP() {BDP = true;}
  /** @brief Returns number of strategic players **/ 
  int getNumOfStrategicPlayers(void) const;
  /** @brief Checks whether a node exists with a given index. */
  bool isNode(Node node) const;
  /** @brief Checks whether a decision node exists with a given index. */
  bool isDecisionNode(Node node) const;
  /** @brief Returns the number of probabilistic nodes in the model. */
  size_t numProbabilisticNodes(void) const;
  /** @brief Returns the BDD manager of the model. */
  Cudd getManager(void) const { return mgr; }
  /** @brief Adds a decision node to the model. */
  void addDecisionNode(Node node, std::string name, Player player = 0);
  /** @brief Adds a probabilistic node to the model. */
  void addProbabilisticNode(Node node, std::string name = std::string(""));
  /** @brief Returns node name. */
  std::string getNodeName(Node node) const;
  /** @brief Adds an atomic proposition to a node. */
  void addLabel(Node node, BDD proposition);
  /** @brief Looks up proposition from label. */
  LabelMap::const_iterator lookUpLabel(std::string const & label) const;
  /** @brief Adds a transition to a decision node. */
  void addDecisionTransition(Node source, Node destination, Action a,
                             Reward reward = 0.0, Priority p = 0);
  /** @brief Adds a transition to a probabilistic node. */
  void addProbabilisticTransition(Node source, Node destination,
                                  Probability p, Reward r = 0.0);
  /** @brief Returns the probability of a probabilistic transition.
   ** @details Returns 0.0 if there is no transition between
   ** source and destination nodes. */
  Probability getTransitionProbability(Node source, Node destination) const;
  /** @brief Adds probability to a probabilistic transition.
   ** @details Creates transition if it does not exist yet.
   ** An existing transition must have the same reward. */
  void addTransitionProbability(Node source, Node destination,
                                Probability probability, Reward reward = 0.0);
  /** @brief Adds reward to existing reward for all transitions from a decision node.
   ** @details The reward is added to the deterministic transitions our of
   ** the node and, if the decision node has probabilistic successors, 
   ** it is also added to the transitions out of those nodes. */
  void addStateReward(Node node, Reward reward);
  /** @brief Adds reward to existing reward for all transitions from a 
   *  node enabled by an action. */
  void addActionStateReward(Node node, Action action, Reward reward);
  /** @brief Makes a decision node the initial node of the model. */
  void makeInitial(Node node);
  /** @brief Returns the initial state of the model. */
  Node getInitial(void) const { return initialState; }
  /** @brief Adds atomic proposition to model. */
  void addAtomicProposition(BDD proposition, std::string name);
  /** @brief Returns name of an atomic proposition. */
  std::string getPropositionName(BDD proposition) const;
  /** @brief Returns the set of atomic propositions. */
  Propositions getAtomicPropositions(void) const {
    return varSet;
  }
  /** @brief Returns the letter labeling a decision node. */
  BDD getNodeLetter(Node decisionnode) const;
  /** @brief Returns the name of an action number. */
  std::string getActionName(Action action) const;
  /** @brief Sets the name of an action. */
  void setActionName(Action action, std::string const & name);
  /** @brief Gets the player of a node. */
  Player getNodePlayer(Node state) const;
  /** @brief Sets the player of a decision node. */
  void setNodePlayer(Node decisionnode, Player player);
  /** @brief Returns the set of decision nodes in the model for one player. */
  std::set<Node> getDecisionNodes(Player player = 0) const;
  /** @brief Returns a vector of actions possible from the given node. */
  std::vector<Action> getActions(Node state) const;
  /** @brief Returns the next decision node and priority given an action through sampling. */
  std::pair<Node, Priority> getSuccessor(Node state, Action action) const;
  /** @brief Returns a the decision node successors of state with probabilities. */
  std::map<Node, double> getSuccessors(Node state, Action action) const;
  /** @brief Returns the decision node successors of state with probabilities. */
  std::map<std::pair<Node, Priority>, double> getSuccessorsNoProb(Node state, Action action) const;

  /** @brief Returns reward from transition from a node enabled by an action. */
  Reward getActionStateReward(Node state, Action action) const;
  /** @brief Stream insertion operator overload. */
  friend std::ostream & operator<<(std::ostream & os, Model const & l);
  /** @brief Returns the verbosity level of the model. */
  Verbosity::Level readVerbosity(void) const { return verbosity; }
  /** @brief Sets the verbosity level of the model. */
  void setVerbosity(Verbosity::Level v) { verbosity = v; }
  /** @brief Replaces probabilistic transitions with deterministic ones. */
  void replaceUnitProbabilityTransitions();
  /** @brief Sorts edges into and out of nodes by their priorities. */
  void sortEdgesByPriority(void);
  /** @brief Checks sanity of model. */
  void sanityCheck(void) const;
  /** @brief Processes the name of the product state */
  std::string prettyPrintState(int n) const;
  /** @brief Write strategy to CSV in the given filename. 
   *  @details If the strategy has a value of "invalid" (a don't care) at a state
   *  then a random action is output for this state.
  */
  void printStrategyCSV(std::unordered_map<Node, Action>, std::string filename) const;
  /** @brief Write all available actions to CSV in the given filename. 
   *  @details All actions are given there own row.
  */
  void printActionsCSV(std::string filename) const;
  /** @brief Write model in dot format. */
  void printDot(std::string graphname = std::string("Model"), std::string filename = std::string("-")) const;
  /** @brief Write model in PRISM format. */
  void printPrism(std::string const & modulename = std::string("m"),
                  std::string filename = std::string("-"))  const;
  /** @brief Produces a model with all non-strategic actions removed. 
   *  @details All nodes not reachable from initial state removed if 
   *  removeUnreachable is true. Nodes with invalid action are not pruned 
   *  if makeMC is false. Otherwise, a random action is assigned to nodes 
   *  with an invalid action assigned.
  */
  Model pruneToStrategy(std::unordered_map<Node, Action> const & strategy, 
                        bool removeUnreachable,
                        bool makeMC) const;
  /** @brief Inverts property by incrementing all priorities by 1. */
  void invertProperty(void);

  // Model Checking
  /** @brief Computes maximum probabilities of reaching WECs for 1-1/2 player models
   *  and probabilities for parity game for 2-1/2 player models.
   *  Note that this offers a computation speed up for 1-1/2 players since the SSP is skipped,
   *  but it does not offer a computation speed up for 2-1/2 players.
   */
  std::unordered_map<Node, double> getProbabilityOfSat(void) const;
  /** @brief Computes maximum probabilities of reaching WECs for 1-1/2 player model.
   *  @details Forced treatment of model as 1-1/2 player model.
   */
  std::unordered_map<Node, double> getProbabilityOfSat1Player(void) const;
  /** @brief Computes strategy to reach target for 1-1/2 player model. 
   *  @details Does not compute strategy within target.
   *  Every node is guaranteed to be assigned a value.
  */
  Model::StrategyResults getStrategyReach1Player(std::set<Node> const & target,
                                                 std::set<Node> const & restriction,
                                                 std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes strategy for WECs and to reach them for 1-1/2 player models.
   *  @details Every node is guaranteed to be assigned a value.
  */
  Model::StrategyResults getStrategy1Player(void) const;
  /** @brief Computes getStrategy1Player or getStrategy2Player depending on
   *  whether the model is 1-1/2 players or 2-1/2 players. */
  Model::StrategyResults getStrategy(void) const;
  /** @brief Computes optimal strategy for a BDP */
  Model::StrategyResults getBDPStrategy(void) const;

  /** @brief Computes the winning-end-components of model within restrictions. */
  std::vector< std::set<Node> > getWECs(std::set<Node> const & restriction,
                                        std::set<std::pair<Node, Action>> const & forbiddenEdges) const;
  /** @brief Computes the winning-end-components of model within restriction. */
  std::vector< std::set<Node> > getWECs(std::set<Node> const & restriction) const;
  /** @brief Computes the winning-end-components of model. */
  std::vector< std::set<Node> > getWECs(void) const;
  /** @brief Computes the maximal-end-components of model within restrictions. */
  std::vector< std::set<Node> > getMECs(std::set<Node> const & restriction,
                                        std::set<std::pair<Node, Action>> const & forbiddenEdges) const;
  /** @brief Computes the maximal-end-components of model within restriction. */
  std::vector< std::set<Node> > getMECs(std::set<Node> const & restriction) const;
  /** @brief Computes the maximal-end-components of model. */
  std::vector< std::set<Node> > getMECs(void) const;
  /** @brief Computes the attractor for target set and player.
   *  @details Negative player indicates probabilistic player. */
  std::unordered_set<Node> getAttractor(std::set<Node> const & target,
                                 Player targetPlayer,
                                 std::set<Node> const & restriction,
                                 std::set<std::pair<Node, Action>> const & forbiddenEdges) const;
  /** @brief Computes the attractor for target set and player.
   *  @details Negative player indicates probabilistic player. */
  std::unordered_set<Node> getAttractor(std::set<Node> const & target, 
                                 Player targetPlayer, 
                                 std::set<Node> const & restriction) const;
  /** @brief Computes the attractor for target set and player.
   *  @details Negative player indicates probabilistic player. */
  std::unordered_set<Node> getAttractor(std::set<Node> const & target, 
                                 Player targetPlayer) const;
  std::vector<std::set<Node>> getSCCs(std::set<Node> const & init,
                                      std::set<Node> const & restriction,
                                      std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes the SCCs of the model reachable from init and contained in restriction. */
  std::vector<std::set<Node>> getSCCs(std::set<Node> const & init,
                                      std::set<Node> const & restriction) const;
  /** @brief Computes the SCCs of the model contained in restriction. */
  std::vector<std::set<Node>> getSCCs(std::set<Node> const & restriction) const;
  /** @brief Computes the SCCs of the model. */
  std::vector<std::set<Node>> getSCCs(void) const;
  /** @brief Computes the attractor with computed strategy in return value. */
  Model::AttractorResults getAttractorWithStrat(std::set<Player> targetPlayers, std::set<Node> const & target,
                                                std::set<Node> const & restriction,
                                                std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes the ultra-weak attractor (2 player attractor where probabilistic nodes are adversarial). */
  Model::AttractorResults getUltraWeakAttr(Player targetPlayer, std::set<Node> const & target,
                                           std::set<Node> const & restriction,
                                           std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes the strong attractor (2 player attractor where probabilistic nodes helps targetPlayer). */
  Model::AttractorResults getStrongAttr(Player targetPlayer, std::set<Node> const & target,
                                        std::set<Node> const & restriction,
                                        std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes the weak attractor (attractor that targetPlayer can attract almost surely). */
  Model::AttractorResults getWeakAttr(Player targetPlayer, std::set<Node> const & target,
                                      std::set<Node> const & restriction,
                                      std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes strategy and winning sets for qualitative parity games.
   *  @details Game is entire model. Player 0 plays for max odd and player 1 for 
   *  max even assumed. Priorities are converted to be on vertices. */
  Model::PlayerSets probMcNaughton(void) const;
  /** @brief Computes strategy and winning sets for qualitative parity games.
   *  @details Game is played in restriction. Player 0 plays for max odd and player 1 for 
   *  max even assumed. Priorities are converted to be on vertices. */
  Model::PlayerSets probMcNaughton(std::set<Node> const & restriction,
                                   std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes strategy and winning sets for qualitative parity games.
   *  @details Game is played in restriction. Player 0 plays for max odd and player 1 for 
   *  max even assumed. Priorities are on vertices from "pri". */
  Model::PlayerSets probMcNaughton(std::map<Node, Priority> pri,
                                   std::set<Node> const & restriction,
                                   std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes strategy and winning sets for quantitative parity games.
   *  @details Game is entire model. Player 0 plays for max odd and player 1 for 
   *  max even assumed. Priorities are converted to be on vertices. */
  Model::StrategyResults getStrategy2Player(void) const;
  /** @brief Computes strategy and winning sets for quantitative parity games.
   *  @details Game is played in restriction. Player 0 plays for max odd and player 1 for 
   *  max even assumed. Priorities are converted to be on vertices. */
  Model::StrategyResults getStrategy2Player(std::set<Node> const & restriction,
                                            std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes strategy and winning sets for quantitative parity games.
   *  @details Game is played in restriction. Player 0 plays for max odd and player 1 for 
   *  max even assumed. Priorities are on vertices from "pri". */
  Model::StrategyResults getStrategy2Player(std::map<Node, Priority> pri,
                                            std::set<Node> const & restriction,
                                            std::set<Index> const & forbiddenEdges) const;
private:
  /** @brief Computes the WECs without checking input. */
  std::vector< std::set<Node> > getWECsNoCheck(std::set<Node> const & restriction,
                                                std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes the MECs without checking input. */
  std::vector< std::set<Node> > getMECsNoCheck(std::set<Node> const & restriction,
                                                std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes the attractor without checking input. */
  std::unordered_set<Node> getAttractorNoCheck(std::set<Node> const & target,
                                      Player targetPlayer,
                                      std::set<Node> const & restriction,
                                      std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes strategy within WECs. Does not perform input check. */
  std::unordered_map<Node, Action> getStrategyWECs(std::vector< std::set<Node> > const & WECs) const;
  /** @brief Computes strategy within WECs. */
  std::unordered_map<Node, Action> getStrategyWECs(void) const;
  /** @brief Computes value of stochastic shortest path leading to target using options specified solver
   *  for 1 player models. Does not perform input check.
   *  @details Ignores existance of edges leading to states where target is unreachable.
   *  States where target is unreachable are assigned value of double INFINITY. */
  std::unordered_map<Node, double> getSSP(std::set<Node> const & target,
                                       std::set<Node> const & restriction,
                                       std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes value of stochastic shortest path leading to target using LP. 
   *  Does not perform input check.
   *  @details Ignores existance of edges leading to states where target is unreachable.
   *  States where target is unreachable are assigned value of double INFINITY. */
  std::unordered_map<Node, double> getSSPORTools(std::set<Node> const & target,
                                       std::set<Node> const & restriction,
                                       std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes value of stochastic shortest path leading to target using incremental method. 
   *  Does not perform input check.
   *  @details Ignores existance of edges leading to states where target is unreachable.
   *  States where target is unreachable are assigned value of double INFINITY. */
  std::unordered_map<Node, double> getSSPIncr(std::set<Node> const & target,
                                    std::set<Node> const & restriction,
                                    std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes value of stochastic shortest path leading to target using policy iteration. 
   *  Does not perform input check.
   *  @details Ignores existance of edges leading to states where target is unreachable.
   *  States where target is unreachable are assigned value of double INFINITY. */
  std::unordered_map<Node, double> getSSPPoly(std::set<Node> const & target,
                                    std::set<Node> const & restriction,
                                    std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes probabilities to reach target using LP for 1-1/2 player models. Does not check input. */
  std::unordered_map<Node, double> getProbReachabilityORTools(std::set<Node> const & target,
                                                 std::set<Node> const & restriction,
                                                 std::set<Index> const & forbiddenEdges) const;
  /** @brief Computes probabilities to reach target using value iteration for 1-1/2 player models. Does not check input. */
  std::unordered_map<Node, double> getProbReachabilityIncr(std::set<Node> const & target,
                                              std::set<Node> const & restriction,
                                              std::set<Index> const & forbiddenEdges) const;
                                              
  /** @brief Computes optimal reward to reach final state using OR Tools. Does not check input. */                                            
  std::unordered_map<Node, double> getBDPValueUsingORTools() const;                                            
 
  /** @brief Helps in building transitions. */
   void addTransition(Transition const & tran);
  /** @brief Checks an SCC for triviality. */
  bool isTrivial(std::set<Node> const & scc) const;
  /** @brief Increments count of nodes owned by "player." */
  void incrementCount(Player player);
  /** @brief Decrements count of nodes owned by "player." */
  void decrementCount(Player player);


  Verbosity::Level verbosity;
  Cudd mgr;
  double epsilon; // tolerance for probability computations
  double tranEpsilon; // tolerance for sum of transistion probabilities to be considered 1
  ModelOptions::ReachType reachSolver; // solver for maximal reachability probability 
  ModelOptions::SSPType sspSolver; // solver for stochastic shortest path
  NodeCounts counts; // node count for each player
  AttributeMap attributes; // from a node's index to its attributes
  TransitionList transitions; // list of all model transitions
  Node initialState;
  Propositions varSet; // set of propositional variable BDDs
  LabelMap labelMap; // from label string to BDD
  std::map<Action, std::string> actionNames;
  bool sortedLists; // are the edge lists sorted?
  static Action constexpr invalid = -1;
  static Player constexpr probPlayer = -1;
  bool BDP = false; // true if the PRISM model is to be interpreted as a BDP

  class SccAnalyzer;
};

#endif
