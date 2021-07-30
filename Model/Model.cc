/** @file Model.cc

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

#include <iostream>
#include <cstring>
#include <map>
#include <stack>
#include <queue>
#include <utility>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <fstream>
#include "Util.hh"
#include "Set.hh"
#include "Model.hh"
#include "Pdriver.hh"

using namespace std;

constexpr decltype(Model::invalid) Model::invalid;
constexpr decltype(Model::probPlayer) Model::probPlayer;


void Model::incrementCount(Player player)
{
  auto cit = counts.find(player);
  if (cit == counts.end()) {
    counts.emplace(player,1);
  } else {
    (cit->second)++;
  }
}


void Model::decrementCount(Player player)
{
  auto cit = counts.find(player);
  if (cit == counts.end()) {
    throw logic_error("Decrementing zero node counter for player " +
                      to_string(player));
  } else {
    auto & count = cit->second;
    if (count == 0) {
      throw logic_error("Decrementing zero node counter for player " +
                        to_string(player));
    }
    count--;
  }
}

Model::NodeCounts Model::getNodeCounts(void) const
{
  NodeCounts rv = counts;
  rv.erase(probPlayer);
  return rv;
}

Model::Model(Cudd mgr, ModelOptions options) : mgr(mgr) , sortedLists(false)
{
  BDP = false;
  parseOptions(options);
}


Model::Model(Model const & model, Parity const & automaton, ModelOptions options)
  : sortedLists(false)
{
  
  parseOptions(options);

  if(!subsetOf(automaton.getAtomicPropositions(), model.varSet))
    throw logic_error("Propositions in automaton are not from model.");
  if(model.isGame() && !automaton.isDeterministic())
    cerr << "Warning: Forming product between 2-player model "
            "and non-deterministic automaton." << endl;
  mgr = model.mgr;
  varSet = model.varSet;
  actionNames = model.actionNames;
  queue<pair<Node, State>> q;
  map<pair<Node, State>, Node> existingNode;
  map<pair<Action, State>, Action> ndetActions;
  map<State, Action> epsilonActions;
  Node nCounter = 0;
  initialState = nCounter;
  NodeAttributes initAttr = model.attributes.find(model.initialState)->second;
  initAttr.inEdges.clear();
  initAttr.outEdges.clear();
  attributes.insert({initialState, initAttr});
  existingNode.emplace(pair<Node,State>{model.initialState, automaton.getInitial()}, initialState);
  set<State> nextStates;
  q.push({model.initialState, automaton.getInitial()});
  while (!q.empty()){
    pair<Node, State> current = q.front();
    q.pop();
    Node currentN = existingNode.find(current)->second;
    NodeAttributes & currentAttr = attributes.find(currentN)->second;
    currentAttr.name += string(", ") + to_string(current.second);
    incrementCount(currentAttr.player);
    currentAttr.outEdges.clear();
    BDD letter = mgr.bddOne();
    if (currentAttr.player != probPlayer) {
      for (auto const & prop : varSet){
        auto labels = currentAttr.propositions;
        if (labels.find(prop) == labels.end()) letter &= ~prop;
        else letter &= prop;
      }
      nextStates = automaton.getSuccessors(current.second, letter);
    } else {
      nextStates = set<State>{current.second};
    }
    auto const & d = model.attributes.find(current.first)->second.outEdges;
    for (State nextState : nextStates) {
      for (Index im : d) {
        transitions.push_back(model.transitions[im]);
        Index i = (Index) transitions.size()-1;
        Transition & t = transitions[i];
        t.source = currentN;
        if (currentAttr.player != probPlayer) {
          t.priority = automaton.getPriority(current.second, letter, nextState);
          if (nextStates.size() > 1) {
            auto nsit = ndetActions.find({t.action, nextState});
            Action act;
            if (nsit == ndetActions.end()) {
              Action newAction = actionNames.size();
              auto ait = actionNames.find(newAction);
              while (ait != actionNames.end())
                ait = actionNames.find(++newAction);
              actionNames.emplace(newAction, string(" ") + actionNames.find(t.action)->second
                + string("+") + to_string(nextState));
              ndetActions.emplace(pair<Action, State>{t.action, nextState}, newAction);
              act = newAction;          
            } else {
              act = nsit->second;
            }
            t.action = act;
          }
        } else {
          t.priority = 0;
        }
        auto it = existingNode.find({t.destination, nextState});
        if (it == existingNode.end()) {
          NodeAttributes nextAttr = model.attributes.find(t.destination)->second;
          nextAttr.inEdges.clear();
          nextAttr.inEdges.push_back(i);
          attributes.insert({++nCounter, nextAttr});
          existingNode.insert({{t.destination, nextState}, nCounter});
          q.push({t.destination, nextState});
          t.destination = nCounter;
        } else {
          Node endNode = it->second;
          t.destination = endNode;
          attributes.find(endNode)->second.inEdges.push_back(i);
        }
        currentAttr.outEdges.push_back(i);
      }
    }
    if (currentAttr.player != probPlayer) {
      auto const & epsilonSuccessors = automaton.getEpsilonSuccessors(current.second);
      for (State epsilonState : epsilonSuccessors) {
        if (epsilonActions.find(epsilonState) == epsilonActions.end()) {
          Action newAction = 1;
          auto ait = actionNames.find(newAction);
          while (ait != actionNames.end())
            ait = actionNames.find(++newAction);
          actionNames.emplace(newAction, " epsilon" + to_string(epsilonState));
          epsilonActions.emplace(epsilonState, newAction);
        }
        Transition t = Transition(currentN, 0, epsilonActions[epsilonState],
                                  1.0, 0.0, 0);
        transitions.push_back(t);
        Index i = transitions.size()-1;
        auto it = existingNode.find({current.first, epsilonState});
        if (it == existingNode.end()) {
          NodeAttributes epsilonAttr = model.attributes.find(current.first)->second;
          transitions[i].destination = ++nCounter;
          epsilonAttr.inEdges.clear();
          epsilonAttr.inEdges.push_back(i);
          attributes.emplace(nCounter, epsilonAttr);
          existingNode.emplace(pair<Node,State>{current.first, epsilonState}, nCounter);
          q.push({current.first, epsilonState});
        } else {
          Node endNode = it->second;
          transitions[i].destination = endNode;
          attributes.find(endNode)->second.inEdges.push_back(i);
        }
        currentAttr.outEdges.push_back(i);
      }
    }
  }
  sortEdgesByPriority();
  if (model.isBDP()) BDP = true; else BDP = false;

}


Model::Model(Cudd mgr, std::string const & filename, 
             Verbosity::Level pverbosity,
             ModelOptions options) : Model(mgr, options)
{
  BDP = false;
  if (filename != "") {
    p_driver driv(*this, options.defines, pverbosity);
    int res = driv.parse(filename);
    if (res != 0) {
      throw invalid_argument("Error parsing file");
    }
  }
}


void Model::sortEdgesByPriority(void)
{
  // Comparison lambda.
  auto lessthan = [this] (Index const & i, Index const & j) {
    if (transitions.at(i).priority < transitions.at(j).priority) return true;
    if (transitions.at(i).priority > transitions.at(j).priority) return false;
    return i < j;
  };

  for (auto & nattr : attributes) {
    // Only sort edges out of decision nodes.
    if (nattr.second.player == probPlayer) continue;
    Edges & outEdges = nattr.second.outEdges;
    std::sort(outEdges.begin(), outEdges.end(), lessthan);
    Edges & inEdges = nattr.second.inEdges;
    std::sort(inEdges.begin(), inEdges.end(), lessthan);
  }    
  sortedLists = true;
}


void Model::parseOptions(ModelOptions options)
{
  verbosity = options.verbosity;
  reachSolver = options.reachSolver;
  sspSolver = options.sspSolver;
  if (options.epsilon <= 0)
    throw logic_error("Epsilon must be a positive value.");
  if (options.tranEpsilon <= 0)
    throw logic_error("Tran-epsilon must be a positive value.");
  epsilon = options.epsilon;
  tranEpsilon = options.tranEpsilon;
}


ModelOptions Model::getOptions(void) const 
{
  ModelOptions options;
  options.verbosity = verbosity;
  options.reachSolver = reachSolver;
  options.sspSolver = sspSolver;
  options.epsilon = epsilon;
  options.tranEpsilon = tranEpsilon;
  return options;
}


size_t Model::numProbabilisticNodes(void) const
{
  auto cit = counts.find(probPlayer);
  if (cit == counts.end()) {
    return 0;
  } else {
    return cit->second;
  }
}


size_t Model::numDecisionNodes(void) const
{
  return attributes.size() - numProbabilisticNodes();
}

bool Model::isGame(void) const
{
  return (getNumOfStrategicPlayers() > 1);
}

int Model::getNumOfStrategicPlayers(void) const
{
  int numStratPlayers = 0;
  for (auto const & c : counts) {
    if (c.first == probPlayer || c.second == 0)
      continue;
    numStratPlayers++;
  }
  return numStratPlayers;
}

bool Model::isNode(Node node) const
{
  return attributes.find(node) != attributes.end();
}


bool Model::isDecisionNode(Node node) const
{
  auto nit = attributes.find(node);
  if (nit == attributes.end()) {
    return false;
  }
  return nit->second.player != probPlayer;
}
 
void Model::addDecisionNode(Node node, std::string name, Player player)
{
  auto it = attributes.find(node);
  if (it != attributes.end())
    throw logic_error("Trying to add existing node " + it->second.name);
  if (player < 0) {
    throw logic_error("Invalid player number: " + to_string(player));
  }
  attributes.emplace(node, NodeAttributes{name, player});
  incrementCount(player);
}


void Model::addProbabilisticNode(Node node, std::string name)
{
  auto it = attributes.find(node);
  if (it != attributes.end())
    throw logic_error("Trying to add existing node " + it->second.name);
  attributes.emplace(node, NodeAttributes{name, probPlayer});
  incrementCount(probPlayer);
}


std::string Model::getNodeName(Node node) const
{
  auto it = attributes.find(node);
  if (it == attributes.end()) {
    throw logic_error("Non-existing node " + to_string(node));
  }
  return it->second.name;
}


void Model::addLabel(Node node, BDD proposition)
{
  if (!proposition.IsVar()) {
    ostringstream os;
    os << proposition;
    throw logic_error("Proposition " + os.str() + " is not a simple variable");
  }
  auto it = attributes.find(node);
  if (it == attributes.end()) {
    ostringstream os;
    os << proposition;
    throw logic_error("Adding proposition " + os.str() +
                      " to non-existing node " + to_string(node));
  }
  it->second.propositions.insert(proposition);
}


Model::LabelMap::const_iterator Model::lookUpLabel(std::string const & label) const
{
  return labelMap.find(label);
}


void Model::addTransition(Transition const & tran)
{
  // We do not let one create nodes by creating transitions.
  auto pit = attributes.find(tran.destination);
  if (pit == attributes.end()) {
    throw logic_error("Adding transition to non-existing node");
  }
  auto sit = attributes.find(tran.source);
  Edges & outEdges = sit->second.outEdges;
  // Check for uniqueness.  It's enough to check the source's outgoing
  // transitions: if tran already exists, it's present in both lists.
  for (Index i : outEdges) {
    Transition & t = transitions.at(i);
    if (t.source == tran.source && t.destination == tran.destination &&
        t.action == tran.action) {
      if (tran.action == invalid) {
        t.probability += tran.probability;
      } else {
        throw logic_error("Duplicate transition from " +
                          getNodeName(tran.source) + " to " +
                          getNodeName(tran.destination) + " for action " +
                          getActionName(tran.action));
      }
      break;
    }
  }
  Index index = transitions.size();
  transitions.push_back(tran);
  outEdges.push_back(index);
  Edges & inEdges = pit->second.inEdges;
  inEdges.push_back(index);
  sortedLists = false;
}


void Model::addProbabilisticTransition(
  Node source,
  Node destination,
  Probability p,
  Reward r)
{
  auto sit = attributes.find(source);
  if (sit == attributes.end()) {
    throw logic_error("Adding transition from non-existing node");
  }
  if (sit->second.player != probPlayer) {
    throw logic_error("Adding probabilistic transition to decision node " +
                      to_string(source));
  }
  auto dit = attributes.find(destination);
  if (dit == attributes.end()) {
    throw logic_error("Adding transition to non-existing node");
  }
  if (dit->second.player == probPlayer) {
    throw logic_error("Adding probabilistic transition from " + to_string(source)
                      + " to probabilitic node " + to_string(destination));
  }
  if (p < 0 || p >= 1) {
    throw logic_error("Out-of-range probability value: " + to_string(p) +
                      " on transitions from " + to_string(source) + " to " +
                      to_string(destination));
  }
  Transition tran{source, destination, invalid, p, r};
  addTransition(tran);
}


void Model::addDecisionTransition(
  Node source,
  Node destination,
  Action a,
  Reward reward,
  Priority p)
{
  auto const sit = attributes.find(source);
  if (sit == attributes.end()) {
    throw logic_error("Adding transition from non-existing node " +
                      to_string(source));
  }
  if (sit->second.player == probPlayer) {
    throw logic_error("Adding decision transition to probabilistic node " +
                      sit->second.name);
  }
  Transition tran{source, destination, a, 1.0, reward, p};
  addTransition(tran);
}


Probability Model::getTransitionProbability(Node source, Node destination) const
{
  // We assume that there is at most one probabilistic transition
  // between source and destination.
  auto const sit = attributes.find(source);
  if (sit == attributes.end()) {
    throw logic_error("Getting probability of transition from non-existing"
                      " node " + to_string(source));
  }
  if (sit->second.player != probPlayer) {
    throw logic_error("Getting probability of transition from"
                      " non-probabilistic source node " + sit->second.name);
  }

  auto const & outEdges = sit->second.outEdges;
  for (Index i : outEdges) {
    Transition const & t = transitions.at(i);
    if (t.destination == destination) {
      return t.probability;
    }
  }
  // If we get here there is no transition between source and destination.
  return 0.0;
}


void Model::addTransitionProbability(Node source, Node destination,
                                     Probability probability, Reward reward)
{
  // We assume that there is at most one transition between source and
  // destination.
  auto const sit = attributes.find(source);
  if (sit == attributes.end()) {
    throw logic_error("Adding transition from non-existing node " +
                      to_string(source));
  }
  if (sit->second.player != probPlayer) {
    throw logic_error("Non-probabilistic source node " + sit->second.name);
  }
  auto const dit = attributes.find(destination);
  if (dit == attributes.end()) {
    throw logic_error("Adding transition into non-existing node " +
                      to_string(destination));
  }
  if (dit->second.player == probPlayer) {
    throw logic_error("Probabilistic destination node " + dit->second.name);
  }
  auto & outEdges = sit->second.outEdges;
  for (Index i : outEdges) {
    Transition & t = transitions.at(i);
    if (t.destination == destination) {
      if (t.reward != reward) {
        throw logic_error("cannot add to transitions with different rewards");
      }
      t.probability += probability;
      return;
    }
  }
  // Need new transition.
  addProbabilisticTransition(source, destination, probability, reward);
}


void Model::addStateReward(Node node, Reward reward)
{
  auto nit = attributes.find(node);
  if (nit == attributes.end()) {
    throw logic_error("Adding reward to non-existing node");
  }
  auto & outEdges = nit->second.outEdges;
  for (Index i : outEdges) {
    Transition & t = transitions.at(i);
    Node dnode = t.destination;
    if (nit->second.player != probPlayer) { // deterministic transition
      t.reward += reward;
    } else {                                // probabilistic transition
      auto pit = attributes.find(dnode);
      if (pit == attributes.end()) {
        throw logic_error("Nonexistent destination probabilistic node");
      }
      auto & pdelta = pit->second.outEdges;
      for (Index i : pdelta) {
        Transition & pt = transitions.at(i);
        pt.reward += reward;
      }
    }
  }
}


void Model::addActionStateReward(Node node, Action action, Reward reward)
{
  auto nit = attributes.find(node);
  if (nit == attributes.end()) {
    throw logic_error("Adding reward to non-existing node");
  }
  auto & outEdges = nit->second.outEdges;
  for (Index i : outEdges) {
    Transition & t = transitions.at(i);
    if (t.action == action) {
      Node dnode = t.destination;
      if (nit->second.player != probPlayer) { // deterministic transition
        t.reward += reward;
      } else {                                // probabilistic transition
        auto pit = attributes.find(dnode);
        if (pit == attributes.end()) {
          throw logic_error("Nonexistent destination probabilistic node");
        }
        auto & pdelta = pit->second.outEdges;
        for (Index i : pdelta) {
          Transition & pt = transitions.at(i);
          pt.reward += reward;
        }
      }
    }
  }
}


void Model::makeInitial(Node node)
{
  auto nit = attributes.find(node);
  if (nit == attributes.end()) {
    throw logic_error("Making non-existing node initial");
  }
  if (nit->second.player == probPlayer) {
    throw logic_error("Making non-decision node initial");
  }
  initialState = node;
}


void Model::addAtomicProposition(BDD proposition, string name)
{
  if (!proposition.IsVar()) {
    throw logic_error("Proposition not a single variable");
  }
  labelMap.emplace(name, proposition);
  varSet.insert(proposition);
  if (!proposition.IsNamedVar()) {
    mgr.pushVariableName(name);
  }
}


string Model::getPropositionName(BDD proposition) const
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


BDD Model::getNodeLetter(Node decisionnode) const
{
  auto it = attributes.find(decisionnode);
  if (it == attributes.end()) {
    throw logic_error("Unknown node");
  }
  if (it->second.player == probPlayer) {
    throw logic_error("Not a decision node");
  }
  auto labels = it->second.propositions;
  BDD letter = mgr.bddOne();
  for (auto const & prop : varSet) {
    if (labels.find(prop) == labels.end()) {
      letter &= ~prop;
    } else {
      letter &= prop;
    }
  }
  return letter;
}


string Model::getActionName(Action action) const
{
  auto it = actionNames.find(action);
  if (it == actionNames.end()) {
    return to_string(action);
  }
  return it->second;
}


void Model::setActionName(Action action, string const & name)
{
  actionNames.emplace(action,name);
}

Player Model::getNodePlayer(Node state) const
{
  auto nit = attributes.find(state);
  if (nit == attributes.end()) {
    throw logic_error("Getting player of non-existing node");
  }
  Player player = nit->second.player;
  return player;
}

void Model::setNodePlayer(Node decisionnode, Player player)
{
  auto nit = attributes.find(decisionnode);
  if (nit == attributes.end()) {
    throw logic_error("Setting player of non-existing node");
  }
  Player oldplayer = nit->second.player;
  if (oldplayer == probPlayer) {
    throw logic_error("Setting player of non-decision node");
  }
  if (player != oldplayer) {
    nit->second.player = player;
    incrementCount(player);
    decrementCount(oldplayer);
  }
}


set<Node> Model::getDecisionNodes(Player player) const
{
  set<Node> pnodes;
  for (auto const & a : attributes) {
    if (a.second.player == player) {
      pnodes.insert(a.first);
    }
  }
  return pnodes;
}

vector<Action> Model::getActions(Node state) const
{
  auto const & it = attributes.find(state);
  if (it == attributes.end())
    throw logic_error("Node does not exist.");
  
  vector<Action> acts;
  auto const & d = it->second.outEdges;
  for (auto i : d)
    acts.push_back(transitions[i].action);
  
  return acts;
}

Reward Model::getActionStateReward(Node state, Action action) const
{
  auto const & it = attributes.find(state);
  if (it == attributes.end())
    throw logic_error("Node does not exist.");
  
  for (auto i : it->second.outEdges) {
    if (transitions[i].action == action) {
      return transitions[i].reward;
    }
  }
  throw logic_error("Action does not exist.");
}

pair<Node, Priority> Model::getSuccessor(Node state, Action action) const
{
  auto const & it = attributes.find(state);
  if (it == attributes.end())
    throw logic_error("Node does not exist.");
  
  auto const & d = it->second.outEdges;
  Node dest;
  Priority prior;
  bool found = false;
  for (auto i : d) {
    if (transitions[i].action == action) {
      dest = transitions[i].destination;
      prior = transitions[i].priority;
      found = true;
      break;
    }
  }
  if (!found)
    throw logic_error("Action does not exist out of node.");
  auto destAttr = attributes.find(dest)->second;
  while (destAttr.player == probPlayer) {
    vector<double> dist;
    for (auto i : destAttr.outEdges) {
      dist.push_back(transitions[i].probability);
    }
    int sample = Util::sample_discrete_distribution(dist.begin(), dist.end());
    dest = transitions[destAttr.outEdges[sample]].destination;
    prior = transitions[destAttr.outEdges[sample]].priority;
    destAttr = attributes.find(dest)->second;
  }

  return make_pair(dest, prior);
}

map<pair<Node, Priority>, double> Model::getSuccessorsNoProb(Node state, Action action) const
{
  auto const & it = attributes.find(state);
  if (it == attributes.end())
    throw logic_error("Node does not exist.");
  
  auto const & d = it->second.outEdges;
  Node dest;
  Priority prior;
  bool found = false;
  for (auto i : d) {
    if (transitions[i].action == action) {
      dest = transitions[i].destination;
      prior = transitions[i].priority;
      found = true;
      break;
    }
  }
  if (!found)
    throw logic_error("Action does not exist out of node.");

  vector<pair<pair<Node, Priority>, double>> rv{{{dest, prior}, 1.}};

  int size = rv.size();
  for(int i = 0; i < size;) {
    auto e = rv[i];
    auto destAttr = attributes.find(e.first.first)->second;
    if(destAttr.player == probPlayer) {
      rv.erase(rv.begin() + i);
      size--;
      for (auto j : destAttr.outEdges) {
        rv.push_back({{transitions[j].destination, transitions[j].priority}, e.second * transitions[j].probability});
        size++;
      }
    } else {
      i++;
    }
  }

  map<pair<Node, Priority>, double> rv1;
  for(auto e : rv) {
    rv1.insert({e.first, e.second});
  }
  return rv1;
}

map<Node, double> Model::getSuccessors(Node state, Action action) const
{
  map<Node, double> rv;
  auto const & it = attributes.find(state);
  if (it == attributes.end())
    throw logic_error("Node does not exist.");

  auto const & d = it->second.outEdges;
  Node dest;
  bool found = false;
  for (auto i : d) {
    if (transitions[i].action == action) {
      dest = transitions[i].destination;
      found = true;
    }
  }
  if (!found)
    throw logic_error("Action does not exist out of node.");
  auto destAttr = attributes.find(dest)->second;
  if (destAttr.player == probPlayer) {
    for (auto i : destAttr.outEdges) {
      if (attributes.find(transitions[i].destination)->second.player == probPlayer)
        throw logic_error("Model contains sequential probabilistic nodes.");
      rv[transitions[i].destination] = transitions[i].probability;
    }
  } else {
    rv[dest] = 1.0;
  }
  
  return rv;
}

void Model::replaceUnitProbabilityTransitions()
{
  set<Node> delenda;
  for (auto & nattr : attributes) {
    if (nattr.second.player != probPlayer) continue;
    Node pnode = nattr.first;
    Edges const & outEdges = nattr.second.outEdges;
    if (outEdges.size() == 1) {
      // Remove this probabilistic node.
      Transition const & outTrans = transitions.at(outEdges.at(0));
      if (fabs(outTrans.probability - 1.0) > 1e-16) {
        throw logic_error("Total probability should be 1");
      }
      Edges const & inEdges = nattr.second.inEdges;
      if (inEdges.size() != 1) {
        throw logic_error("Probabilistic node should have one predecessor");
      }
      Transition & inTrans = transitions.at(inEdges.at(0));
      Node source = inTrans.source;
      auto sit = attributes.find(source);
      if (sit == attributes.end()) {
        throw logic_error("Nonexisting predecessor node");
      }
      inTrans.destination = outTrans.destination;
      inTrans.reward = outTrans.reward;
      delenda.insert(pnode);
      auto dit = attributes.find(outTrans.destination);
      Edges & dInEdges = dit->second.inEdges;
      for (Index & i : dInEdges) {
        if (i == outEdges.at(0)) {
          i = inEdges.at(0);
          break;
        }
      }
    }
  }
  // Expunge probabilistic nodes.
  for (auto const & dn : delenda) {
    attributes.erase(dn);
    decrementCount(probPlayer);
  }
}


void Model::sanityCheck(void) const
{
  for (auto const & attr : attributes) {
    auto const & outEdges = attr.second.outEdges;
    if (outEdges.size() == 0) {
      throw logic_error("Node " + attr.second.name +
                        " without outgoing transitions");
    }
    double totalProbability = 0.0;
    // Collect actions to check whether they are used more than once per node.
    set<Action> actions;
    for (Index i : outEdges) {
      Transition const & t = transitions.at(i);
      auto dit = attributes.find(t.destination);
      if (dit == attributes.end()) {
        throw logic_error("Transition with non-existing destination");
      }
      if (attr.second.player == probPlayer) {
        if (t.action != invalid) {
          throw logic_error("Probabilistic transition labeled with action");
        }
        if (dit->second.player == probPlayer) {
          throw logic_error("Transition between two probabilistic nodes");
        }
        if (t.probability == 0.0) {
          throw logic_error("Transition with zero probability");
        } else if (t.probability == 1.0) {
          throw logic_error("Probabilistic transition with unit probability "
                            "between " + to_string(t.source) + " and " +
                            to_string(t.destination));
        }
        totalProbability += t.probability;
      } else {
        if (actions.find(t.action) != actions.end()) {
          throw logic_error("Action \"" + to_string(t.action) +
                            "\" labels more than one transition");
        }
        actions.insert(t.action);
        if (t.action == invalid) {
          throw logic_error("Decision transition with invalid action");
        }
        if (t.probability != 1.0) {
          throw logic_error("Decision transition with non-unit probability");
        }
      }
    }
    if (attr.second.player == probPlayer && fabs(totalProbability - 1.0) > tranEpsilon) {
      throw logic_error("Probabilities do not sum to 1 (" +
                        to_string(totalProbability) +
                        ") for node " + getNodeName(attr.first));
    }
  }
  if (!sortedLists) {
    throw logic_error("Edge lists have not been sorted");
  }
  auto it0 = counts.find(0);
  if (it0 == counts.end() || it0->second == 0)
    throw logic_error("Player 0 does not have any decision nodes.");
  for (auto const & x : counts) {
    if (x.first != probPlayer && x.first != 0 && x.first != 1 && x.second != 0)
      throw logic_error("There are more than 2 players. Player " + to_string(x.first) + " has " + 
                        to_string(x.second) + " node(s).");
  }
}


ostream & operator<<(ostream & os, Model const & m)
{
  os << "Model\n";
  for (auto const & attr : m.attributes) {
    Node s = attr.first;
    if (s == m.initialState)
      os << "=> ";
    else
      os << "   ";
    os << attr.second.name;
    if (attr.second.player != m.probPlayer) {
      os << " (" << attr.second.player << ")";
    }
    os << " [";
    char sep[] = ", ";
    char const * psep = sep + 2;
    for (auto const & prop : attr.second.propositions) {
      os << psep << prop;
      psep = sep;
    }
    os << "] {";
    Model::Edges const & outEdges = attr.second.outEdges;
    psep = sep + 1;
    for (Model::Index i : outEdges) {
      Model::Transition const & t = m.transitions.at(i);
      auto dit = m.attributes.find(t.destination);
      if (dit == m.attributes.end()) {
        throw logic_error("Destination node without attributes");
      }
      os << psep;
      if (t.action == m.invalid) {
        os << t.probability << ": ";
      } else {
        os << "[" << m.getActionName(t.action) << "] ";
      }
      os << "(" << dit->second.name << "," << t.reward << ",(" << 
            t.priority << "))";
      psep = sep;
    }
    os << " } \n";
  }
  return os;
}


Model Model::pruneToStrategy(unordered_map<Node, Action> const & strategy,
                             bool removeUnreachable,
                             bool makeMC) const
{
  Model m(*this);
  unordered_set<Node> reached;
  queue<Node> q;
  if (!removeUnreachable) {
    for (auto & p : m.attributes) {
      p.second.inEdges.clear();
      p.second.outEdges.clear();
      q.push(p.first);
    }
  } else {
    for (auto & p : m.attributes) {
      p.second.inEdges.clear();
      p.second.outEdges.clear();
    }
    q.push(m.initialState);
  }
  m.transitions.clear();
  m.counts.clear();

  while (!q.empty()) {
    Node n = q.front();
    q.pop();
    if (reached.find(n) != reached.end())
      continue;
    reached.insert(n);
    NodeAttributes & nAttr = m.attributes.find(n)->second;
    m.incrementCount(nAttr.player);
    if (nAttr.player == probPlayer) {
      for (auto i : attributes.find(n)->second.outEdges) {
        m.transitions.push_back(transitions[i]);
        nAttr.outEdges.push_back((Index) m.transitions.size()-1);
        m.attributes.find(transitions[i].destination)->second.inEdges.push_back((Index) m.transitions.size()-1);
        if (removeUnreachable)
          q.push(transitions[i].destination);
      }
      continue;
    }
    auto it = strategy.find(n);
    if (it == strategy.end()) {
      throw logic_error("Strategy does not contain all decision nodes in model.");
    }
    bool found = false;
    NodeAttributes const & thisAttr = attributes.find(n)->second;
    for (auto i : thisAttr.outEdges) {
      Transition t = transitions[i];
      if (it->second == invalid) {
        m.transitions.push_back(t);
        nAttr.outEdges.push_back((Index) m.transitions.size()-1);
        m.attributes.find(transitions[i].destination)->second.inEdges.push_back((Index) m.transitions.size()-1);
        if (removeUnreachable)
          q.push(t.destination);
        if (makeMC)
          break;
      } else {
        if (t.action == it->second) {
          m.transitions.push_back(t);
          nAttr.outEdges.push_back((Index) m.transitions.size()-1);
          m.attributes.find(transitions[i].destination)->second.inEdges.push_back((Index) m.transitions.size()-1);
          if (removeUnreachable)
            q.push(t.destination);
          found = true;
          break;
        }
      }
    }
    if (!found && it->second != invalid)
      throw logic_error("Strategy contains action that does not exist.");
  }

  if (removeUnreachable) {
    for (auto it = m.attributes.begin(); it != m.attributes.end();) {
      if (reached.find(it->first) == reached.end())
        it = m.attributes.erase(it);
      else 
        it++;
    }
  }

  return m;
}

void Model::invertProperty(void)
{
  for (Transition & t : transitions) {
    t.priority++;
  }
}

void Model::printStrategyCSV(unordered_map<Node, Action> strategy, string filename) const
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

  // Get sorted list of nodes.
  std::vector<Node> keys;
  keys.reserve(strategy.size());
  for (auto const &it : strategy) {
    keys.push_back(it.first);
  }
  std::sort(keys.begin(), keys.end());

  // Print header
  ofs << "State, Player, Action" << endl;

  // Print CSV  
  for (auto const & n : keys) {
    auto const & it = attributes.find(n);
    if (it == attributes.end()) 
      throw out_of_range("State " + to_string(n) + " is in the strategy, but not in the model.");
    auto const & nAttr = it->second;
    if (nAttr.player != probPlayer) {
      // State
      ofs << "\"" << nAttr.name << "\", ";
      // Player
      ofs << nAttr.player << ", ";
      // Action
      bool found = false;
      for (auto const t : nAttr.outEdges) {
        if (strategy[n] == invalid || transitions[t].action == strategy[n]) {
          ofs << "\"" << getActionName(transitions[t].action) << "\"";
          found = true;
          break;
        }
      }
      if (!found)
        throw out_of_range("The action " + to_string(strategy[n]) + " is available from state " + to_string(n));
      
      ofs << endl;
    }
  }

  if (filename != "-") {
    of.close();
  }
  
}

void Model::printActionsCSV(string filename) const
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

  // Get sorted list of nodes.
  std::vector<Node> keys;
  keys.reserve(attributes.size());
  for (auto const &it : attributes) {
    keys.push_back(it.first);
  }
  std::sort(keys.begin(), keys.end());

  // Print header
  ofs << "State, Player, Action" << endl;  

  // Print CSV  
  for (auto const & n : keys) {
    auto const & nAttr = attributes.find(n)->second;
    if (nAttr.player != probPlayer) {
      // State
      ofs << "\"" << nAttr.name << "\", ";
      // Player
      ofs << nAttr.player << ", ";
      size_t count = 0;
      for (auto const t : nAttr.outEdges) {
        // Action
        ofs << "\"" << getActionName(transitions[t].action) << "\"";
        if (++count < nAttr.outEdges.size()) {
          ofs << " & ";
        }
      }
      ofs << endl;
    }
  }

  if (filename != "-") {
    of.close();
  }
  
}

void Model::printDot(std::string graphname, std::string filename) const
{
  streambuf * buf;
  ofstream of;

  // Check for output to cout.
  if (filename == "-") {
    buf = cout.rdbuf();
  } else {
    of.open(filename);
    if (!of.is_open())
      throw logic_error("Cannot open file " + filename + ".");
    buf = of.rdbuf();
  }

  ostream ofs(buf);

  Util::replaceAll(graphname, "\n", "\\n");
  ofs << "digraph \"" << graphname << "\" {\nnode [shape=circle];\n"
       << "\"title\" [label=\"" << graphname << "\",shape=plaintext];\n";

  // Get sorted list of nodes to ensure output reproducibility.
  std::vector<Node> keys;
  keys.reserve(attributes.size());
  for (auto const & atr : attributes) {
    keys.push_back(atr.first);
  }
  std::sort(keys.begin(), keys.end());

  // Output the list of nodes.
  char sep[] = ", ";
  char const *psep;

  for (auto const & n : keys) {
    NodeAttributes const & second = attributes.at(n);
    if (n == initialState) {
      ofs << "\"" << n << "_init\" [style=invis]\n\"" << n << "_init\" -> " << n << "\n";
    }
    ofs << n << " [label=\"";
    if (second.player != probPlayer) {
      ofs << second.name;
      ofs << "\\n";
      psep = sep + 2;
      for (auto prop : second.propositions) {
        ofs << psep << prop;
        psep = sep;
      }
      if (second.player == 1) {
        ofs << "\", shape=house];\n";
      } else {
        ofs << "\", shape=box];\n";
        if (second.player != 0) {
          // This should not happen.
          ofs << "\\n(" << second.player << ")";
        }
      }
    } else {
      ofs << "\",width=0.25];\n";
    }
  }

  // Output the list of edges.
  for (auto const & n : keys) {
    NodeAttributes const & second = attributes.at(n);
    Edges const & outEdges = second.outEdges;
    for (Index i : outEdges) {
      Transition const & t = transitions.at(i);
      ofs << n << " -> " << t.destination;
      if (t.action != invalid) { // decision transition
        ofs << " [label=\" " << getActionName(t.action);
        if (t.reward != 0.0) {
          ofs << "," << t.reward << "\\n";
        }
        ofs << "(" << t.priority << ")" << " \"];";
      } else {                   // probabilistic transition
        ofs << " [label=\" " << t.probability;
        if (t.reward != 0.0) {
          ofs << "," << t.reward << "\\n";
        }
        ofs << "(" << t.priority << ")" <<" \"];";
      }
      ofs << "\n";
    }
  }
  ofs << "}" << endl;

  if (filename != "-") {
    of.close();
  }
}

string Model::prettyPrintState(int n) const {
  string n_name = attributes.at(n).name;
  // Replace ',' with "_x"
  size_t idx = 0;
  idx = n_name.find(",", idx);
  if (idx != string::npos) 
    n_name.replace(idx, 2, "_x_");
  // Replace all non-alphanumeric characters with '_'
  for (auto it = n_name.begin(); it != n_name.end(); it++) {
    if (!isalnum(*it))
      *it = '_';
  }
  return n_name;
}

void Model::printPrism(string const & modulename, string filename) const
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

  int n_players = getNumOfStrategicPlayers();
  if (n_players > 1) ofs << "smg\n"; else ofs << "mdp\n";


  char lsep[] = " | ";
  char const * psep;

  // Get sorted list of nodes.
  std::vector<Node> keys;
  keys.reserve(attributes.size());
  for (auto const & it : attributes) {
    keys.push_back(it.first);
  }
  std::sort(keys.begin(), keys.end());

  ofs << "\n";
  // Print state names in terms of original inpout file
  for (auto const & n : keys) {    
    if ( attributes.at(n).player != probPlayer) 
      ofs << "const int " << prettyPrintState(n) <<  " = " << n << "; \n";
  }

  ofs << "\n";

  for (auto const & proposition : varSet) {
    ofs << "label \"" << proposition << "\" =";
    psep = lsep + 2;
    for (auto const & n : keys) {
      NodeAttributes const & second = attributes.at(n);
      if (second.player == probPlayer) continue;
      Propositions props = second.propositions;
      if (props.find(proposition) != props.end()) {
        ofs << psep << "state=" << prettyPrintState(n);
        psep = lsep;
      }
    }
    if (psep == lsep + 2) {
      ofs << " false"; // no state labeled by this proposition
    }
    ofs << ";\n";
  }
  
  set<Node> sortedNodes;
  for (auto const & na : attributes) {
    if (na.second.player != probPlayer) {
      sortedNodes.insert(na.first);
    }
  }

  ofs << "module " << modulename << "\n";
  // Since the variable name is not saved, we just use the name "state."
  ofs << "  state : [" << *sortedNodes.begin() << ".." << *sortedNodes.rbegin()
       << "] init " << getInitial() << ";\n";
  char sep[] = " + ";
  auto save = ofs.precision(16);
  for (Node node : sortedNodes) {
    auto ait = attributes.find(node);
    Edges const & outEdges = ait->second.outEdges;
    for (Index i : outEdges) {
      Transition const & t = transitions.at(i);
      ofs << "  [" << getActionName(t.action) << "] state=" << prettyPrintState(node) << " -> ";
      Node destination = t.destination;
      if (sortedNodes.find(destination) != sortedNodes.end()) {
        ofs << "(state'=" << prettyPrintState(destination) << ");\n";
      } else {
        auto pit = attributes.find(destination);
        Edges const & pOutEdges = pit->second.outEdges;
        psep = sep + 3;
        for (Index j : pOutEdges) {
          Transition const & pt = transitions.at(j);
          ofs << psep;
          ofs << pt.probability << " : (state'=" << prettyPrintState(pt.destination) << ")";
          psep = sep;
        }
        ofs << ";\n";
      }
    }
  }
  ofs.precision(save);
  ofs << "endmodule\n";
  ofs << "rewards\n";
  for (Node node : sortedNodes) {
    auto ait = attributes.find(node);
    Edges const & outEdges = ait->second.outEdges;
    for (Index i : outEdges) {
      Transition const & t = transitions.at(i);
      ofs << "  [" << getActionName(t.action) << "] state=" << prettyPrintState(node) << " : ";
      Node destination = t.destination;
      if (sortedNodes.find(destination) != sortedNodes.end()) {
        // The destination is a decision node.  The entire reward is
        // available from the transition.
        ofs << t.reward << ";\n";
      } else {
        // The destination is a probabilistic node.  We need to collect also
        // the rewards from the transitions out of that node.  We are limited
        // to all probabilistic transitions having the same reward.
        auto pit = attributes.find(destination);
        Edges const & pOutEdges = pit->second.outEdges;
        bool first = true;
        Reward reward = 0.0;
        for (Index j : pOutEdges) {
          Transition const & pt = transitions.at(j);
          if (first) {
            reward = pt.reward;
            first = false;
          } else {
            if (reward != pt.reward) {
              throw out_of_range("non-uniform probabilistic transition rewards");
            }
          }
        }
        ofs << reward + t.reward << ";\n";
      }
    }
  }
 ofs << "endrewards\n";

 if (n_players > 1)
 {
   for (int pl = 0; pl < n_players; pl++)
   {
     ofs << "player p" << pl << "\n";
     vector<string> edList;
     for (Node node : sortedNodes)
     {
       auto ait = attributes.find(node);
       if (ait->second.player == pl)
       {
         Edges const &outEdges = ait->second.outEdges;
         for (Index i : outEdges)
         {
           Transition const &t = transitions.at(i);
           string actName = getActionName(t.action);
           if (find(edList.begin(), edList.end(), actName) == edList.end())
             edList.push_back(actName);
         }
       }
     }
     std::string separator = "  ";
     for (auto ed : edList)
     {
       ofs << separator << "[" << ed << "]";
       separator = ",";
     }
     ofs << endl;
     ofs << "endplayer\n";
   }
 }

 if (filename != "-") {
    of.close();
  }
}
