/** @file Gym.cc

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

#include <queue>
#include <map>
#include <utility>
#include <fstream>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include "Gym.hh"

using namespace std;

Gym::Gym(Model model, Parity automaton, GymOptions options) : model(model), 
                                                              automaton(automaton)
{
  rewardType = options.rewardType;
  if (rewardType == GymOptions::GymRewardTypes::default_type) {
    if (!model.isGame())
      rewardType = GymOptions::GymRewardTypes::zeta_reach;
    else
      rewardType = GymOptions::GymRewardTypes::parity;
  } else if (model.isGame() && rewardType != GymOptions::GymRewardTypes::prism &&
             rewardType != GymOptions::GymRewardTypes::parity) {
    cerr << "Warning: Only --reward-type prism and --reward-type parity are "
    "guaranteed to work with 2-1/2 players." << endl;
  }

  episodeLength = options.episodeLength;
  double zetaSet = options.zeta;
  vector<double> toleranceSet = options.tolerance;

  modelState = model.getInitial();
  autoState = automaton.getInitial();
  traps = automaton.getTrapStates();
  episodeStep = 0;
  resetStats();
  
  if (!(zetaSet >= 0 && zetaSet < 1))
    throw logic_error("Value of zeta must be within bounds 0 <= zeta < 1.");
  
  if (automaton.isTerminal(false) && rewardType != GymOptions::GymRewardTypes::parity) {
    zeta = 0.0;
  } else {
    zeta = zetaSet;
  }
  
  for (double t : toleranceSet) {
    if (t < 0)
      throw logic_error("Tolerance must be greater than or equal to zero.");
  }
  tolerance = toleranceSet;
  
  if (!(options.gammaB >= 0 && options.gammaB < 1))
    throw logic_error("Value of gammaB must be within bounds 0 <= gammaB < 1.");
  gammaB = options.gammaB;

  if (!(options.priEpsilon > 0 && options.priEpsilon < 1))
    throw logic_error("Value of priEpsilon must be within bounds 0 < priEpsilon < 1.");
  priEpsilon = options.priEpsilon;
  
  maxPriority = automaton.getMaxPriority();
  if (maxPriority > 1 && rewardType != GymOptions::GymRewardTypes::prism &&
      rewardType != GymOptions::GymRewardTypes::parity &&
      rewardType != GymOptions::GymRewardTypes::pri_tracker)
    throw(invalid_argument("Priorities greater than 1 are not supported for this reward type."));
  
  resetOnAcc = !options.noResetOnAcc;
  noTerminalUpdate = !options.terminalUpdateOnTimeLimit;
  p1strategic = !options.p1NotStrategic;
  concatActionsInCSV = options.concatActionsInCSV;
}

set<GymAction> getGreedyActions(GymObservation const & observation, 
                                Qtype const & Q,
                                double tolerance,
                                Player player)
{
  set<GymAction> actions;
  double valA = 0.0;
  auto const & actionMap = Q.find(observation)->second;

  if (player == 0) {
    double valMaxA = -INFINITY;
    for (auto const & x : actionMap) {
      if (x.second > valMaxA) {
        valMaxA = x.second;
      }
    }
    valA = valMaxA;
  } else if (player == 1) {
    double valMinA = +INFINITY;
    for (auto const & x : actionMap) {
      if (x.second < valMinA) {
        valMinA = x.second;
      }
    }
    valA = valMinA;
  }

  double epsilon = abs(valA)*tolerance;
  for (auto const & x : actionMap) {
    if (fabs(x.second - valA) <= epsilon) {
      actions.insert(x.first);
    }
  }

  return actions;
}

set<GymAction> getMinActions(GymObservation const & observation, 
                                Qtype const & Q,
                                double tolerance,
                                Player player)
{
  set<GymAction> actions;
  double valA = 0.0;
  auto const & actionMap = Q.find(observation)->second;

    double valMinA = +INFINITY;
    for (auto const & x : actionMap) {
      if (x.second < valMinA) {
        valMinA = x.second;
      }
    }
    valA = valMinA;

  double epsilon = abs(valA)*tolerance;
  for (auto const & x : actionMap) {
    if (fabs(x.second - valA) <= epsilon) {
      actions.insert(x.first);
    }
  }

  return actions;
}

void Gym::resetStats(void)
{
  numAccept = 0;
  acceptingEps = 0;
  trappedEps = 0;
  numSteps = 0;
  cumulativeRew = 0;
  numEpisodes = 0;
}

void Gym::printStats(void) const
{
  if (rewardType != GymOptions::GymRewardTypes::parity &&
      rewardType != GymOptions::GymRewardTypes::pri_tracker)
    cout << "Total number of accepting edges seen: " << numAccept << endl;

  cout << "Number of accepting episodes: " << acceptingEps << endl;

  if (traps.size() > 0)
    cout << "Number of trapped episodes  : " << trappedEps << endl;

  cout << "Number of steps: " << numSteps << endl;
  
  if (numEpisodes != 0)
    cout << "Average total reward per episode: " << cumulativeRew/(double)numEpisodes << endl;
  else
    cout << "Average total reward per episode: " << "undefined" << endl;
  
  if (automaton.isTerminal(false) && rewardType != GymOptions::GymRewardTypes::parity)
    cout << "Terminal automaton was detected. Zeta set to zero." << endl;
  
  cout << "Random seed used: " << Util::get_urng_seed() << endl;
}

void Gym::printDotLearn(Qtype const & Q, string filename) const
{
  if (tolerance.size() > 1) 
    cerr << "Warning: multiple tolerances specified. Using first specified tolerance"
    " for --dot-learn." << endl;
  
  Model qmodel = QtoModel(Q, tolerance[0], p1strategic, false, priEpsilon);
  unordered_map<Node, double> results;
  double qprob;

  if (p1strategic && qmodel.isGame()) {
    qmodel.invertProperty();
    results = qmodel.getProbabilityOfSat1Player();
    qprob = 1-results[qmodel.getInitial()];
  } else {
    results = qmodel.getProbabilityOfSat1Player();
    qprob = results[qmodel.getInitial()];
  }

  qmodel.printDot("Psat=" + to_string(qprob), filename);
}

void Gym::printPrismLearn(Qtype const & Q, string filename) const
{
  if (tolerance.size() > 1) {
    cerr << "Warning: multiple tolerances specified. Using first specified tolerance"
    " for --dot-learn." << endl;
  }
  Model qmodel = QtoModel(Q, tolerance[0], p1strategic, false, priEpsilon);
  unordered_map<Node, double> results;
  double qprob;

  if (p1strategic && qmodel.isGame()) {
    qmodel.invertProperty();
    results = qmodel.getProbabilityOfSat1Player();
    qprob = 1-results[qmodel.getInitial()];
  } else {
    results = qmodel.getProbabilityOfSat1Player();
    qprob = results[qmodel.getInitial()];
  }

  qmodel.printPrism("Psat=" + to_string(qprob), filename);
}

map<double, double> Gym::getProbabilityOfSat(Qtype const & Q, bool statsOn) const
{
  map<double, double> rv;
  for (double t : tolerance) {
    Model qmodel = QtoModel(Q, t, p1strategic, statsOn, priEpsilon);
    unordered_map<Node, double> results;
    double qprob;

    if (p1strategic && qmodel.isGame()) {
      qmodel.invertProperty();
      results = qmodel.getProbabilityOfSat1Player();
      qprob = 1-results[qmodel.getInitial()];
    } else {
      results = qmodel.getProbabilityOfSat1Player();
      qprob = results[qmodel.getInitial()];
    }

    rv.emplace(t, qprob);
  }
  return rv;
}

Model Gym::QtoModel(Qtype const & Q,
                    double tolerance,
                    bool p1strat,
                    bool statsOn,
                    double priEpsilon) const
{
  // Constructed model does not have propositions on nodes
  queue<pair<GymObservation,Node>> q;
  map<GymObservation, Node> obsToNode;
  Node n = 0;
  Model m(model.getManager(), model.getOptions());
  m.setActionName(0, "_");
  GymObservation obs = {model.getInitial(), automaton.getInitial(), 0};
  Node obsN = n++;
  m.addDecisionNode(obsN, toString(obs), model.getNodePlayer(obs.first));
  // m.addLabel(obsN, model.getNodeLetter(obs.first));
  obsToNode[obs] = obsN;
  m.makeInitial(obsN);
  q.push({obs, obsN});

  unsigned long count = 0;
  while(!q.empty()) {
    pair<GymObservation, Node> curr = q.front();
    GymObservation currObs = curr.first;
    Node currN = curr.second;
    q.pop();
    auto currLetter = model.getNodeLetter(currObs.first);
    auto currPlayer = model.getNodePlayer(currObs.first);
    set<GymAction> acts;
    bool inQtable = (Q.find(currObs) != Q.end());
    if (!inQtable)
      count++;
    if (inQtable && !(p1strat && currPlayer == 1))
      acts = getGreedyActions(currObs, Q, tolerance, currPlayer);
    else {
      auto autoSuccessors = automaton.getSuccessors(currObs.second, currLetter);
      for (auto mActions : model.getActions(currObs.first)) {
        for (State successor : autoSuccessors)
          acts.insert({pair<Action, State>{mActions, successor}, invalidState});
      }
      for (State aState : automaton.getEpsilonSuccessors(currObs.second))
        acts.insert({invalidAction, aState});
    }
    double numActs = (double) acts.size();

    Node currProbN = ~0L;
    if (numActs != 1 && !(p1strat && currPlayer == 1)) {
      currProbN = n++;
      m.addProbabilisticNode(currProbN, "p" + to_string(currProbN));
      m.addDecisionTransition(currN, currProbN, 0, 0, 0);
    }

    Action p1Act = 0;
    for (auto a : acts) {
      Node actN = n++;
      m.addDecisionNode(actN, "action=(" + toString(a) + ")", currPlayer);
      // m.addLabel(actN, currLetter);
      if (numActs != 1.0 && !(p1strat && currPlayer == 1))
        m.addProbabilisticTransition(currProbN, actN, 1/numActs);
      else if (p1strat && currPlayer == 1) {
        m.addDecisionTransition(currN, actN, p1Act++, 0, 0);
        m.setActionName(p1Act, "_");
      }
      else
        m.addDecisionTransition(currN, actN, 0, 0, 0);

      if (a.second == invalidState) {
        Priority pri = automaton.getPriority(currObs.second, currLetter, a.first.second);
        map<Node, double> modelSuccessors = model.getSuccessors(currObs.first, a.first.first);
        Node actProbN = ~0L;
        if (modelSuccessors.size() > 1 || 
            (rewardType == GymOptions::GymRewardTypes::pri_tracker && currObs.trackerState == 0)) {
          actProbN = n++;
          m.addProbabilisticNode(actProbN, "p" + to_string(actProbN));
          m.addDecisionTransition(actN, actProbN, 0, 0, pri);
        }
        for (auto ms : modelSuccessors) {
          short int trackerState = currObs.trackerState;
          if (rewardType == GymOptions::GymRewardTypes::pri_tracker) {
            if (trackerState != 0 && pri > 1+2*(trackerState-1))
              trackerState = (short)(1+floor(pri/2));
          }
          GymObservation obs = {ms.first, a.first.second, trackerState};
          Node obsN;
          auto nit = obsToNode.find(obs);
          if (nit == obsToNode.end()) {
            obsN = n++;
            m.addDecisionNode(obsN, toString(obs), model.getNodePlayer(obs.first));
            // m.addLabel(obsN, currLetter);
            obsToNode[obs] = obsN;
            q.push({obs,obsN});
          } else {
            obsN = nit->second;
          }
          if (rewardType != GymOptions::GymRewardTypes::pri_tracker) {
            if (modelSuccessors.size() > 1)
              m.addProbabilisticTransition(actProbN, obsN, ms.second, 0);
            else
              m.addDecisionTransition(actN, obsN, 0, 0, pri);
          } else {
            if (currObs.trackerState == 0) {
              m.addProbabilisticTransition(actProbN, obsN, ms.second*(1-priEpsilon), 0);
              GymObservation obsC = {ms.first, a.first.second, (short)(1+floor(pri/2))};
              auto cit = obsToNode.find(obsC);
              Node cN;
              if (cit == obsToNode.end()) {
                cN = n++;
                m.addDecisionNode(cN, toString(obsC), model.getNodePlayer(obsC.first));
                // m.addLabel(obsN, currLetter);
                obsToNode[obsC] = cN;
                q.push({obsC,cN});
              } else {
                cN = cit->second;
              }
              m.addProbabilisticTransition(actProbN, cN, ms.second*priEpsilon, 0);
            } else if (modelSuccessors.size() > 1){
              m.addProbabilisticTransition(actProbN, obsN, ms.second, 0);
            } else {
              m.addDecisionTransition(actN, obsN, 0, 0, pri);
            }
          }
        }
      } else {
        Priority pri = 0;//automaton.getPriority(currObs.second, currLetter, a.second);
        GymObservation obs = {currObs.first, a.second, currObs.trackerState};
        Node obsN;
        auto nit = obsToNode.find(obs);
        if (nit == obsToNode.end()) {
          obsN = n++;
          m.addDecisionNode(obsN, toString(obs), model.getNodePlayer(obs.first));
          // m.addLabel(obsN, currLetter);
          obsToNode[obs] = obsN;
          q.push({obs,obsN});
        } else {
          obsN = nit->second;
        }
        m.addDecisionTransition(actN, obsN, 0, 0, pri);
      }
    }
  }
  if (statsOn) {
    cout << "There were " << count << " unseen decision nodes "
    "reachable under learned strategy for tol " << tolerance << "." << endl;
  }

  m.setVerbosity(Verbosity::Silent);

  m.replaceUnitProbabilityTransitions();
  m.sortEdgesByPriority();
  m.sanityCheck();
  return m;
}

Gym::GymInfo Gym::reset(void)
{
  modelState = model.getInitial();
  autoState = automaton.getInitial();
  trackerState = 0;
  episodeStep = 0;
  Gym::GymInfo rv;
  rv.observation = {modelState, autoState};
  rv.reward = 0.0;
  rv.letter = model.getNodeLetter(modelState);
  rv.done = false;
  rv.player = model.getNodePlayer(modelState);
  rv.actions = {};
  auto autoSuccessors = automaton.getSuccessors(autoState, rv.letter);
  for (auto mAction : model.getActions(modelState)) {
    for (State successor : autoSuccessors)
      rv.actions.emplace_back(std::pair<Action, State>{mAction, successor}, invalidState);
  }
  for (auto aState : automaton.getEpsilonSuccessors(autoState))
    rv.actions.emplace_back(invalidAction, aState);
  return rv;
}

std::vector<std::pair<double, Gym::GymInfo>> Gym::transitions(GymAction action) const
{
  Node oldNode = modelState;
  State oldState = autoState;
  std::map<std::pair<Node, Priority>, double> _modelStates = {{{modelState, 0}, 1.0}};
  State _autoState = autoState;

  auto oldLetter = model.getNodeLetter(oldNode);
  Priority priority = 0;

  if (action.second == invalidState) {
    _autoState = action.first.second;
    priority = automaton.getPriority(oldState, oldLetter, _autoState);
    _modelStates = model.getSuccessorsNoProb(modelState, action.first.first);
  } else {
    _autoState = action.second;
  }

  std::vector<std::pair<double, Gym::GymInfo>> rvs;

  for(auto modelStateAndProb : _modelStates) {
    auto _modelState = modelStateAndProb.first.first;
    // std::cout << "xx" << model.getNodeName(_modelState) << std::endl;

    double modelStateProb = modelStateAndProb.second;

    Gym::GymInfo rv;
    rv.observation = {_modelState, _autoState};
    rv.letter = model.getNodeLetter(_modelState);
    rv.done = false;
    rv.player = model.getNodePlayer(_modelState);
    rv.actions = {};
    auto autoSuccessors = automaton.getSuccessors(autoState, rv.letter);
    for (auto mAction : model.getActions(_modelState)){
      for (State successor : autoSuccessors)
        rv.actions.emplace_back(std::pair<Action, State>{mAction, successor}, invalidState);
    }
    for (auto aState : automaton.getEpsilonSuccessors(autoState))
      rv.actions.emplace_back(invalidAction, aState);

    // Create transitions
    if (rewardType == GymOptions::GymRewardTypes::prism) {
      if (action.second == invalidState) {
        rv.reward = model.getActionStateReward(oldNode, action.first.first);
      } else {
        rv.reward = 0.0;
      }
      if (episodeStep + 1 >= episodeLength) {
          rv.done = true;
          if (noTerminalUpdate)
            rv.terminationOverride = true;
      }
      rvs.emplace_back(1.0 * modelStateProb, rv);
    } else if (traps.find(autoState) != traps.end()) {
      rv.done = true;
      rv.reward = 0.0;
      rv.observation = terminalState;
      rvs.emplace_back(1.0 * modelStateProb, rv);
    } else if (rewardType != GymOptions::GymRewardTypes::parity && 
               rewardType != GymOptions::GymRewardTypes::pri_tracker) {
      if (priority == 0) {
        if (episodeStep + 1 >= episodeLength) {
          rv.done = true;
          if (noTerminalUpdate)
            rv.terminationOverride = true;
        }
        rv.reward = 0.0;
        rvs.emplace_back(1.0 * modelStateProb, rv);
      } else if (priority == 1) {
        if (rewardType == GymOptions::GymRewardTypes::zeta_reach ||
            rewardType == GymOptions::GymRewardTypes::zeta_acc) {
          {
            Gym::GymInfo rv1 = rv;
            rv1.observation = terminalState;
            if (rewardType == GymOptions::GymRewardTypes::zeta_reach)
              rv1.reward = 1.0;
            else if (rewardType == GymOptions::GymRewardTypes::zeta_acc)
              rv1.reward = 1.0-zeta;
            rv1.done = true;
            rvs.emplace_back(zeta * modelStateProb, rv1);
          }
          {
            Gym::GymInfo rv2 = rv;
            if (rewardType == GymOptions::GymRewardTypes::zeta_reach)
              rv2.reward = 0.0;
            else if (rewardType == GymOptions::GymRewardTypes::zeta_acc)
              rv2.reward = 1.0-zeta;
            if (episodeStep + 1 >= episodeLength) {
              rv2.done = true;
              if (noTerminalUpdate)
                rv2.terminationOverride = true;
            }
            rvs.emplace_back((1-zeta) * modelStateProb, rv2);
          }
        } else if (rewardType == GymOptions::GymRewardTypes::zeta_discount ||
                   rewardType == GymOptions::GymRewardTypes::reward_on_acc ||
                   rewardType == GymOptions::GymRewardTypes::multi_discount) {
          if (episodeStep + 1 >= episodeLength) {
            rv.done = true;
            if (noTerminalUpdate)
              rv.terminationOverride = true;
          }
          if (rewardType == GymOptions::GymRewardTypes::zeta_discount) {
            rv.reward = 1.0;
            rv.discountOverride = true;
            rv.discount = zeta;
          } else if (rewardType == GymOptions::GymRewardTypes::reward_on_acc) {
            rv.reward = 1.0;
          } else if (rewardType == GymOptions::GymRewardTypes::multi_discount) {
            rv.reward = 1 - gammaB;
            rv.discountOverride = true;
            rv.discount = gammaB;        
          }
          rvs.emplace_back(1.0 * modelStateProb, rv);
        }
      } else {
        throw(invalid_argument("Priorities greater than 1 are not supported for this reward type."));
      }
    } else if (rewardType == GymOptions::GymRewardTypes::parity) {
      double epsilonPow = pow(1-zeta, 1.0+(maxPriority-priority));
      {
        Gym::GymInfo rv1 = rv;
        rv1.observation = terminalState;
        rv1.reward = priority % 2;
        rv1.done = true;
        rvs.emplace_back((1-epsilonPow) * modelStateProb, rv1);
      }
      {
        Gym::GymInfo rv2 = rv;
        rv2.reward = 0.0;
        rvs.emplace_back(epsilonPow * modelStateProb, rv2);
      }
    } else if (rewardType == GymOptions::GymRewardTypes::pri_tracker) {
      // rv.reward = 0.0;
      // if (trackerState == 0) {
      //   int sample = Util::sample_discrete_distribution({zeta, 1.0-zeta});
      //   if (sample) {
      //     trackerState = 1+floor(priority/2);
      //   }
      // } else {
      //   if (priority == 1+2*(trackerState-1)) {
      //     int sample = Util::sample_discrete_distribution({zeta, 1.0-zeta});
      //     if (sample) {
      //       rv.observation = terminalState;
      //       rv.reward = 1.0;
      //       rv.done = true;
      //       acceptingEps++;
      //     } else if (resetOnAcc)
      //       episodeStep = 0;
      //   } else if (priority > 1+2*(trackerState-1)) {
      //     trackerState = 1+floor(priority/2);
      //   } else {
      //     if (++episodeStep >= episodeLength) {
      //       rv.done = true;
      //       if (noTerminalUpdate)
      //         rv.terminationOverride = true;
      //     }
      //   }
      // }
      // if (!rv.done)
      //   rv.observation.trackerState = trackerState;
      throw(invalid_argument("pri_tracker not implemented"));
    }
  
  }

  return rvs;
}


Gym::GymInfo Gym::step(GymAction action)
{
  numSteps++;
  Gym::GymInfo rv;
  Node oldNode = modelState;
  State oldState = autoState;
  auto oldLetter = model.getNodeLetter(modelState);
  Priority priority = 0;
  if (action.second == invalidState) {
    autoState = action.first.second;
    priority = automaton.getPriority(oldState, oldLetter, autoState);
    auto val = model.getSuccessor(modelState, action.first.first);
    modelState = val.first;
  } else {
    autoState = action.second;
  }
  rv.observation = {modelState, autoState};
  rv.letter = model.getNodeLetter(modelState);
  rv.done = false;
  rv.player = model.getNodePlayer(modelState);
  rv.actions = {};
  auto autoSuccessors = automaton.getSuccessors(autoState, rv.letter);
  for (auto mAction : model.getActions(modelState)){
    for (State successor : autoSuccessors)
      rv.actions.emplace_back(std::pair<Action, State>{mAction, successor}, invalidState);
  }
  for (auto aState : automaton.getEpsilonSuccessors(autoState))
    rv.actions.emplace_back(invalidAction, aState);

  if (rewardType == GymOptions::GymRewardTypes::prism) {
    if (action.second == invalidState) {
      rv.reward = model.getActionStateReward(oldNode, action.first.first);
    } else {
      rv.reward = 0.0;
    }
    if (++episodeStep >= episodeLength) {
        rv.done = true;
        if (noTerminalUpdate)
          rv.terminationOverride = true;
    }
  } else if (traps.find(autoState) != traps.end()) {
    trappedEps++;
    rv.done = true;
    rv.reward = 0.0;
    rv.observation = terminalState;
  } else if (rewardType != GymOptions::GymRewardTypes::parity && 
             rewardType != GymOptions::GymRewardTypes::pri_tracker) {
    if (priority == 0) {
      if (++episodeStep >= episodeLength) {
        rv.done = true;
        if (noTerminalUpdate)
          rv.terminationOverride = true;
      }
      rv.reward = 0.0;
    } else if (priority == 1) {
      numAccept++;
      if (rewardType == GymOptions::GymRewardTypes::zeta_reach ||
          rewardType == GymOptions::GymRewardTypes::zeta_acc) {
        if (resetOnAcc)
          episodeStep = 0;
        int sample = Util::sample_discrete_distribution({zeta, 1.0-zeta});
        if (sample) {
          rv.observation = terminalState;
          if (rewardType == GymOptions::GymRewardTypes::zeta_reach)
            rv.reward = 1.0;
          else if (rewardType == GymOptions::GymRewardTypes::zeta_acc)
            rv.reward = 1.0-zeta;
          rv.done = true;
          acceptingEps++;
        } else {
          if (rewardType == GymOptions::GymRewardTypes::zeta_reach)
            rv.reward = 0.0;
          else if (rewardType == GymOptions::GymRewardTypes::zeta_acc)
            rv.reward = 1.0-zeta;
          if (++episodeStep >= episodeLength) {
            rv.done = true;
            if (noTerminalUpdate)
              rv.terminationOverride = true;
          }
        }
      } else if (rewardType == GymOptions::GymRewardTypes::zeta_discount ||
                 rewardType == GymOptions::GymRewardTypes::reward_on_acc ||
                 rewardType == GymOptions::GymRewardTypes::multi_discount) {
        if (++episodeStep >= episodeLength) {
          rv.done = true;
          if (noTerminalUpdate)
            rv.terminationOverride = true;
        }
        if (rewardType == GymOptions::GymRewardTypes::zeta_discount) {
          rv.reward = 1.0;
          rv.discountOverride = true;
          rv.discount = zeta;
        } else if (rewardType == GymOptions::GymRewardTypes::reward_on_acc) {
          rv.reward = 1.0;
        } else if (rewardType == GymOptions::GymRewardTypes::multi_discount) {
          rv.reward = 1 - gammaB;
          rv.discountOverride = true;
          rv.discount = gammaB;        
        }
      }
    } else {
      throw(invalid_argument("Priorities greater than 1 are not supported for this reward type."));
    }
  } else if (rewardType == GymOptions::GymRewardTypes::parity) {
    double epsilonPow = pow(1-zeta, 1.0+(maxPriority-priority));
    int sample = Util::sample_discrete_distribution({1-epsilonPow, epsilonPow});
    if (sample) {
      rv.observation = terminalState;
      rv.reward = priority % 2;
      rv.done = true;
      if (priority % 2) acceptingEps++;
    } else {
      rv.reward = 0.0;
    }
  } else if (rewardType == GymOptions::GymRewardTypes::pri_tracker) {
    rv.reward = 0.0;
    if (trackerState == 0) {
      int sample = Util::sample_discrete_distribution({zeta, 1.0-zeta});
      if (sample) {
        trackerState = 1+floor(priority/2);
      }
    } else {
      if (priority == 1+2*(trackerState-1)) {
        int sample = Util::sample_discrete_distribution({zeta, 1.0-zeta});
        if (sample) {
          rv.observation = terminalState;
          rv.reward = 1.0;
          rv.done = true;
          acceptingEps++;
        } else if (resetOnAcc)
          episodeStep = 0;
      } else if (priority > 1+2*(trackerState-1)) {
        trackerState = 1+floor(priority/2);
      } else {
        if (++episodeStep >= episodeLength) {
          rv.done = true;
          if (noTerminalUpdate)
            rv.terminationOverride = true;
        }
      }
    }
    if (!rv.done)
      rv.observation.trackerState = trackerState;
  }

  cumulativeRew += rv.reward;
  if (rv.done)
    numEpisodes++;
  
  return rv;
}

Player Gym::getPlayer(GymObservation const & obs) const
{
  if (obs == terminalState)
    return 0;
  return model.getNodePlayer(obs.first);
}

string Gym::toString(GymObservation const & obs) const
{
  if (obs == terminalState)
    return "Terminal State";
  return model.getNodeName(obs.first) + ", " + to_string(obs.second) + ", " + to_string(obs.trackerState);
}

string Gym::toString(GymAction const & act) const
{
  if (act.first == invalidAction)
    return "Epsilon: " + to_string(act.second);
  else 
    return "Action: " + model.getActionName(act.first.first) + 
      ", Auto: " + to_string(act.first.second);
}

string Gym::toString(Qtype const & Q) const
{
  string rv;
  for (auto it = Q.cbegin(); it != Q.cend(); ++it) {
    auto const & m = it->second;
    for (auto const & v : m) {
      rv += toString(it->first) + " --> {" + toString(v.first) +
             + ", " + to_string(v.second) + "}\n";
    }
  }
  return rv;
}

void Gym::saveQ(Qtype const & Q, string saveType, string filename) const
{
  ofstream ofs(filename);
  boost::archive::text_oarchive oa(ofs);
  oa << saveType;
  oa << getProductID();
  oa << Q;
}

void Gym::saveQ(Qtype const & Q1, Qtype const & Q2, string saveType, string filename) const
{
  ofstream ofs(filename);
  boost::archive::text_oarchive oa(ofs);
  oa << saveType;
  oa << getProductID();
  oa << Q1;
  oa << Q2;
}

void Gym::loadQ(Qtype & Q, string saveType, string filename) const
{
  try {
    ifstream ifs(filename);
    boost::archive::text_iarchive ia(ifs);
    string loadType;
    ia >> loadType;
    if (saveType != loadType)
      throw runtime_error("Load type (" + loadType + ") does not match save type (" + saveType + ")");
    size_t prodID;
    ia >> prodID;
    if (prodID != getProductID())
      cerr << "Warning: Load file specified used different MDP and/or automaton." << endl;
    ia >> Q;
  } catch (exception const & e) {
    cerr << "Error loading file " + filename + "." << endl;
    throw;
  }
}

void Gym::loadQ(Qtype & Q1, Qtype & Q2, string saveType, string filename) const
{
  try {
    ifstream ifs(filename);
    boost::archive::text_iarchive ia(ifs);
    string loadType;
    ia >> loadType;
    if (saveType != loadType)
      throw runtime_error("Load type (" + loadType + ") does not match save type (" + saveType + ")");
    size_t prodID;
    ia >> prodID;
    if (prodID != getProductID())
      cerr << "Warning: Load file specified used different MDP and/or automaton." << endl;
    ia >> Q1;
    ia >> Q2;
  } catch (exception const & e) {
    cerr << "Error loading file " + filename + "." << endl;
    throw;
  }
}

size_t Gym::getProductID(void) const
{
  size_t seed = 0;
  boost::hash_combine(seed, boost::hash_value(model.numNodes()));
  boost::hash_combine(seed, boost::hash_value(model.numDecisionNodes()));
  boost::hash_combine(seed, boost::hash_value(automaton.classifyStates()));
  return seed;
}

void Gym::saveStrat(Qtype & Q, std::string filename) const
{
  if (tolerance.size() > 1) {
    cerr << "Warning: multiple tolerances specified. Using first specified tolerance"
    " for --save-learner-strategy." << endl;
  }

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

  OrderedQtype ordered(Q.begin(), Q.end());

  // Print header
  ofs << "State, Player, Action(s)" << endl;

  // Print rows
  for (auto const & x : ordered) {
    if (x.first == terminalState)
      continue;
    auto actions = getGreedyActions(x.first, Q, tolerance[0], getPlayer(x.first));
    string modelStateName = model.getNodeName(x.first.first);

    // State name
    ofs << "\"" << modelStateName << ", " << x.first.second;
    ofs << ", " << x.first.trackerState << "\", ";
    // Player
    ofs << model.getNodePlayer(x.first.first) << ", ";
    // Action
    size_t count = 0;
    for (auto const & act : actions) {
      if (act.first == invalidAction) { // Epsilon action
        ofs << "\"" << " epsilon" << act.second << "\""; 
      } else {
        ofs << "\"" << model.getActionName(act.first.first);
        ofs << "+" << act.first.second << "\"";
      }
      if (++count < actions.size()) {
        ofs << " & ";
      }
    }
    ofs << endl;
  }

  if (filename != "-") {
    of.close();
  }
}

void Gym::saveStratBDP(Qtype & Q, std::string filename) const
{
  if (tolerance.size() > 1) {
    cerr << "Warning: multiple tolerances specified. Using first specified tolerance"
    " for --save-learner-strategy." << endl;
  }

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

  OrderedQtype ordered(Q.begin(), Q.end());

  for (auto const & x : ordered) {
    if (x.first == terminalState)
      continue;
    ofs << toString(x.first) << " --> ";
    auto actions = getMinActions(x.first, Q, tolerance[0], getPlayer(x.first));
    size_t count = 0;
    for (auto a : actions) {
      ofs << toString(a);
      if (++count < actions.size())
        ofs << " && ";
    }
    ofs << "\n";
  }

  if (filename != "-") {
    of.close();
  }
}

void Gym::saveMDP(std::string filename, double discount) const {
  std::map<GymObservation, int> states;
  std::map<GymObservation, std::vector<GymAction>> state_actions;
  std::map<GymObservation, bool> state_done;

  Gym gym = *this;
  Gym::GymInfo info = gym.reset();
  state_actions.insert({info.observation, info.actions});
  state_done.insert({info.observation, info.done});

  std::queue<GymObservation> to_process;
  to_process.push(info.observation);

  while(!to_process.empty()) {
    GymObservation state = to_process.front();
    to_process.pop();
    // std::cout << " ggg" << std::endl;
    if(!states.count(state)) {
      states.insert({state, states.size()});
      if (!state_done[state]) {
        // std::cout << gym.getModel().getNodeName(state.first) << " fff" << std::endl;
        gym.modelState = state.first;
        gym.autoState = state.second;
        for (auto act : state_actions[state]) {
          for(auto transition : gym.transitions(act)) {
            to_process.push(transition.second.observation);
            // std::cout << "add" << (transition.second.done) << std::endl;
            state_actions.insert({transition.second.observation, transition.second.actions});
            state_done.insert({transition.second.observation, transition.second.done});
          }
        }
      }
    }
  }
  
  // Output MDP to file
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

  ofs << "states: " << states.size() << std::endl;
  ofs << "initial: " << states[gym.reset().observation] << std::endl;
  ofs << "transitions:" << std::endl;
  for(auto state : states) {
    GymObservation obs = state.first;
    if(state_done[obs]) {
      ofs << state.second << " terminal" << std::endl;
    } else {
      gym.modelState = obs.first;
      gym.autoState = obs.second;
      for(auto act : state_actions[obs]) {
        for(auto transition : gym.transitions(act)) {
          auto nextInfo = transition.second;
          ofs << state.second << " ";
          ofs << " act: ";
          if (act.first == invalidAction) { // Epsilon action
            ofs << "epsilon" << act.second;
          } else {
            ofs << "\"" << model.getActionName(act.first.first);
            ofs << "+" << act.first.second << "\"";
          }
          ofs << " next: " << states[nextInfo.observation] << 
          " probability: " << transition.first <<
          " reward: " << transition.second.reward <<
          " discount: " << (transition.second.discountOverride ? transition.second.discount : discount) << std::endl;
        }
      }
    }
  }
}
