/** @file ModelGames.cc

  @brief Algorithms to solve stochatic games.

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
#include "Util.hh"
#include "Set.hh"
#include "Model.hh"

using namespace std;

// NOTE How does attractor deal with probability 0 edges
Model::AttractorResults Model::getAttractorWithStrat(set<Player> targetPlayers, set<Node> const & target,
                                                     set<Node> const & restriction,
                                                     set<Index> const & forbiddenEdges) const
{
  Model::AttractorResults rv;
  queue<Node> q;
  unordered_map<Node, size_t> m;

  for (Node n : target) {
    if (restriction.find(n) == restriction.end())
      continue;
    rv.attr.insert(n);
    rv.strategy[n] = invalid;
    q.push(n);
    m.emplace(n, 0);
  }

  while (!q.empty()) {
    Node n = q.front();
    q.pop();
    NodeAttributes const & currAttr = attributes.find(n)->second;
    for (auto i : currAttr.inEdges) {
      if (forbiddenEdges.find(i) != forbiddenEdges.end() ||
          restriction.find(transitions[i].source) == restriction.end())
        continue;
      Node p = transitions[i].source;
      auto it = m.find(p);
      if (it == m.end()) {
        NodeAttributes const & predAttr = attributes.find(transitions[i].source)->second;
        if(targetPlayers.find(predAttr.player) != targetPlayers.end())//
          it = m.emplace(p, 1).first;
        else {
          size_t tbe = 0;
          for (auto pI : predAttr.outEdges)
            if (forbiddenEdges.find(pI) == forbiddenEdges.end() &&
                restriction.find(transitions[pI].destination) != restriction.end())//
              tbe++;
          it = m.emplace(p, tbe).first;
        }
      }
      size_t & toBeExamined = it->second;
      if (toBeExamined > 0) {
        toBeExamined--;
        if (toBeExamined == 0) {
          rv.attr.insert(p);//
          if (targetPlayers.find(attributes.find(p)->second.player) != targetPlayers.end())
            rv.strategy[p] = transitions[i].action;//
          else
            rv.strategy[p] = invalid;
          q.push(p);
        } 
      }
    }
  }

  return rv;
}

Model::AttractorResults Model::getUltraWeakAttr(Player targetPlayer, set<Node> const & target,
                               set<Node> const & restriction,
                               set<Index> const & forbiddenEdges) const
{
  set<Player> targetPlayers;
  targetPlayers.insert(targetPlayer);
  Model::AttractorResults rv = getAttractorWithStrat(targetPlayers, target, restriction, forbiddenEdges);

  return rv;
}

Model::AttractorResults Model::getStrongAttr(Player targetPlayer, set<Node> const & target,
                               set<Node> const & restriction,
                               set<Index> const & forbiddenEdges) const
{
  set<Player> targetPlayers;
  targetPlayers.insert(targetPlayer);
  targetPlayers.insert(probPlayer);
  Model::AttractorResults rv = getAttractorWithStrat(targetPlayers, target, restriction, forbiddenEdges);

  return rv;
}                                       

// Makes assumption that other player is (1-targetPlayer). NOTE
Model::AttractorResults Model::getWeakAttr(Player targetPlayer, set<Node> const & target,
                                           set<Node> const & restriction,
                                           set<Index> const & forbiddenEdges) const
{
  Model::AttractorResults rv;

  auto UjResults = getUltraWeakAttr(targetPlayer, target, restriction, forbiddenEdges);
  set<Node> & Uj = UjResults.attr;
  auto SjResults = getStrongAttr(targetPlayer, Uj, restriction, forbiddenEdges);
  set<Node> & Sj = SjResults.attr;

  set<Node> opponentRestriction;
  set<Node> opponentTarget;
  for (Node n : restriction) {
    if (Uj.find(n) == Uj.end())
      opponentRestriction.insert(n);
    if (Sj.find(n) == Sj.end())
      opponentTarget.insert(n);
  }
  Player opponentPlayer = 1-targetPlayer;
  set<Node> playerRestriction(restriction);

  size_t prevSize;

  while(true) {
    prevSize = Sj.size();
    for (Node n : Sj) {
      if (opponentTarget.find(n) != opponentTarget.end())
        opponentTarget.erase(n); // Check that this is fast NOTE
    }
    for (Node n : Uj) {
      if (opponentRestriction.find(n) != opponentRestriction.end())
        opponentRestriction.erase(n);
    }
    auto CjResults = getStrongAttr(opponentPlayer, opponentTarget, opponentRestriction, forbiddenEdges);
    set<Node> & Cj = CjResults.attr;
    for (Node n : Cj) {
      if (playerRestriction.find(n) != playerRestriction.end())
        playerRestriction.erase(n);
    }
    auto UjResults = getUltraWeakAttr(targetPlayer, target, playerRestriction, forbiddenEdges);
    Uj = UjResults.attr;
    auto SjResults = getStrongAttr(targetPlayer, Uj, playerRestriction, forbiddenEdges);
    Sj = SjResults.attr;

    if (Sj.size() == prevSize) {
      rv.attr = Sj;
      for (auto const & na : SjResults.strategy) {
        if (attributes.find(na.first)->second.player == targetPlayer)
          rv.strategy[na.first] = na.second;
        else
          rv.strategy[na.first] = invalid;
      }
      for (auto const & na : UjResults.strategy) {
        if (attributes.find(na.first)->second.player == targetPlayer)
          rv.strategy[na.first] = na.second;
        else
          rv.strategy[na.first] = invalid;
      }
      break;
    }
  }

  return rv;
}

Model::PlayerSets Model::probMcNaughton(void) const 
{
  set<Node> restriction;
  for (auto const & it : attributes) {
    restriction.insert(it.first);
  }

  return probMcNaughton(restriction, set<Index>());
}

Model::PlayerSets Model::probMcNaughton(set<Node> const & restriction,
                                        set<Index> const & forbiddenEdges) const
{
  map<Node, Priority> pri;
  for (Node n : restriction) {
    NodeAttributes const & nAttr = attributes.find(n)->second;
    bool first = true;
    Priority pN = 0;
    for (auto const & t : nAttr.outEdges) {
      if (first) {
        first = false;
        pN = transitions[t].priority;
      } else if (pN != transitions[t].priority) {
        throw runtime_error("2 1/2 player model checking only supports priorities " \
                            "on vertices, i.e all outgoing edges have the same priority. " \
                            " Consider augmenting your MDP so that this is true.");
      }
    }
    pri[n] = pN;
  }

  return probMcNaughton(pri, restriction, forbiddenEdges);
}

// Add check if proper game NOTE
// Assumes players are 0 and 1 NOTE
// Strategy for non-winning player randomly set
Model::PlayerSets Model::probMcNaughton(map<Node, Priority> pri,
                                        set<Node> const & restriction,
                                        set<Index> const & forbiddenEdges) const
{
  Model::PlayerSets rv;
  set<Node> restric = restriction;

  // Base case
  if (restric.empty())
    return rv;
  Priority c = 0; // Maximum priority
  bool allSame = true;
  bool first = true;
  for (Node n : restric) {
    if (first) {
      c = pri[n];
      first = false;
    } else {
      if (c != pri[n])
        allSame = false;
      if (pri[n] > c) {
        c = pri[n];
      }
    }
  }
  if (allSame) {
    if (c % 2) // Odd
      rv.w0 = restric;
    else
      rv.w1 = restric;
    for (Node n : restric) {
      NodeAttributes const & nAttr = attributes.find(n)->second;
      for (auto const & t : nAttr.outEdges) {
        if (nAttr.player == probPlayer) {
          rv.strategy[n] = invalid;
          break;
        }
        if (restric.find(transitions[t].destination) != restric.end() &&
            forbiddenEdges.find(t) == forbiddenEdges.end()) {
          rv.strategy[n] = transitions[t].action;
          break;
        }
      }
      if (rv.strategy.find(n) == rv.strategy.end())
        throw runtime_error("Game is improper. Report bug.");
    }
    return rv;
  }

  set<Node> target;
  for (Node n : restric) {
    if (pri[n] == c)
      target.insert(n);
  }
  // Recurse
  if (c % 2) { // Odd
    while (true) {
      auto satr0Results = getStrongAttr(0, target, restric, forbiddenEdges);
      set<Node> restricPrime = restric;
      for (Node n : satr0Results.attr)
        restricPrime.erase(n);
      Model::PlayerSets winningSetsPrime = probMcNaughton(pri, restricPrime, forbiddenEdges);
      if (winningSetsPrime.w1.empty()) {
        for (Node n : restric) {
          if (rv.w1.find(n) == rv.w1.end())
            rv.w0.insert(n);
          else {
            if (rv.strategy.find(n) == rv.strategy.end()) {
              NodeAttributes const & nAttr = attributes.find(n)->second;
              if (nAttr.player == probPlayer)
                rv.strategy[n] = invalid;
              else {
                bool found = false;
                for (Index const & t : nAttr.outEdges) {
                  if (restric.find(transitions[t].destination) != restric.end() &&
                      forbiddenEdges.find(t) == forbiddenEdges.end()) {
                    rv.strategy[n] = transitions[t].action; // Arbitrary action
                    found = true;
                    break;
                  }
                }
                if (!found)
                  throw runtime_error("Game is improper.");
              }
            }
          }
        }
        for (Node n : rv.w0) {
          NodeAttributes const & nAttr = attributes.find(n)->second;
          if (nAttr.player == probPlayer)
            rv.strategy[n] = invalid;
          else if (nAttr.player == 1) {
            bool found = false;
            for (Index const & t : nAttr.outEdges) {
              if (restric.find(transitions[t].destination) != restric.end() &&
                  forbiddenEdges.find(t) == forbiddenEdges.end()) {
                rv.strategy[n] = transitions[t].action; // Arbitrary action
                found = true;
                break;
              }
            }
            if (!found)
              throw runtime_error("Game is improper.");
          } else {
            if (winningSetsPrime.w0.find(n) != winningSetsPrime.w0.end())
              rv.strategy[n] = winningSetsPrime.strategy[n];
            else if (target.find(n) == target.end())
              rv.strategy[n] = satr0Results.strategy[n];
            else {
              NodeAttributes const & nAttr = attributes.find(n)->second;
              for (Index const & t : nAttr.outEdges) {
                if (rv.w0.find(transitions[t].destination) != rv.w0.end() &&
                    forbiddenEdges.find(t) == forbiddenEdges.end()) {
                  rv.strategy[n] = transitions[t].action;
                  break;
                }
              }
            }
          }
        }
        return rv;
      }
      auto satr1Results = getStrongAttr(1, winningSetsPrime.w1, restric, forbiddenEdges);
      for (Node n : satr1Results.attr) {
        rv.w1.insert(n);
        NodeAttributes const & nAttr = attributes.find(n)->second;
        if (nAttr.player == probPlayer)
          rv.strategy[n] = invalid;
        else if (nAttr.player == 0) {
          bool found = false;
          for (Index const & t : nAttr.outEdges) {
            if (restric.find(transitions[t].destination) != restric.end() &&
                forbiddenEdges.find(t) == forbiddenEdges.end()) {
              rv.strategy[n] = transitions[t].action; // Arbitrary action
              found = true;
              break;
            }
          }
          if (!found)
            throw runtime_error("Game is improper.");
        } else {
          if (winningSetsPrime.w1.find(n) != winningSetsPrime.w1.end())
            rv.strategy[n] = winningSetsPrime.strategy[n];
          else
            rv.strategy[n] = satr1Results.strategy[n];
        }
      }
      for (Node n : satr1Results.attr) {
        restric.erase(n);
      }
    }
  } else { // Even
    while (true) {
      auto satr1Results = getStrongAttr(1, target, restric, forbiddenEdges);
      set<Node> restricPrime = restric;
      for (Node n : satr1Results.attr)
        restricPrime.erase(n);
      Model::PlayerSets winningSetsPrime = probMcNaughton(pri, restricPrime, forbiddenEdges);
      if (winningSetsPrime.w0.empty()) {
        for (Node n : restric) {
          if (rv.w0.find(n) == rv.w0.end())
            rv.w1.insert(n);
          else {
            if (rv.strategy.find(n) == rv.strategy.end()) {
              NodeAttributes const & nAttr = attributes.find(n)->second;
              if (nAttr.player == probPlayer)
                rv.strategy[n] = invalid;
              else {
                bool found = false;
                for (Index const & t : nAttr.outEdges) {
                  if (restric.find(transitions[t].destination) != restric.end() &&
                      forbiddenEdges.find(t) == forbiddenEdges.end()) {
                    rv.strategy[n] = transitions[t].action; // Arbitrary action
                    found = true;
                    break;
                  }
                }
                if (!found)
                  throw runtime_error("Game is improper.");
              }
            }
          }
        }
        for (Node n : rv.w1) {
          NodeAttributes const & nAttr = attributes.find(n)->second;
          if (nAttr.player == probPlayer)
            rv.strategy[n] = invalid;
          else if (nAttr.player == 0) {
            bool found = false;
            for (Index const & t : nAttr.outEdges) {
              if (restric.find(transitions[t].destination) != restric.end() &&
                  forbiddenEdges.find(t) == forbiddenEdges.end()) {
                rv.strategy[n] = transitions[t].action; // Arbitrary action
                found = true;
                break;
              }
            }
            if (!found)
              throw runtime_error("Game is improper.");
          } else {
            if (winningSetsPrime.w1.find(n) != winningSetsPrime.w1.end())
              rv.strategy[n] = winningSetsPrime.strategy[n];
            else if (target.find(n) == target.end())
              rv.strategy[n] = satr1Results.strategy[n];
            else {
              NodeAttributes const & nAttr = attributes.find(n)->second;
              for (Index const & t : nAttr.outEdges) {
                if (rv.w1.find(transitions[t].destination) != rv.w1.end() &&
                    forbiddenEdges.find(t) == forbiddenEdges.end()) {
                  rv.strategy[n] = transitions[t].action;
                  break;
                }
              }
            }
          }
        }
        return rv;
      }
      auto watr0Results = getWeakAttr(0, winningSetsPrime.w0, restric, forbiddenEdges);
      for (Node n : watr0Results.attr) {
        rv.w0.insert(n);
        NodeAttributes const & nAttr = attributes.find(n)->second;
        if (nAttr.player == probPlayer)
          rv.strategy[n] = invalid;
        else if (nAttr.player == 1) {
          bool found = false;
          for (Index const & t : nAttr.outEdges) {
            if (restric.find(transitions[t].destination) != restric.end() &&
                forbiddenEdges.find(t) == forbiddenEdges.end()) {
              rv.strategy[n] = transitions[t].action; // Arbitrary action
              found = true;
              break;
            }
          }
          if (!found)
            throw runtime_error("Game is improper.");
        } else {
          if (winningSetsPrime.w0.find(n) != winningSetsPrime.w0.end())
            rv.strategy[n] = winningSetsPrime.strategy[n];
          else
            rv.strategy[n] = watr0Results.strategy[n];
        }
      }
      for (Node n : watr0Results.attr) {
        restric.erase(n);
      }
      for (Node n : restric) {
        NodeAttributes const & nAttr = attributes.find(n)->second;
        for (Index const & t : nAttr.outEdges) {
          Node d = transitions[t].destination;
          if (watr0Results.attr.find(d) != watr0Results.attr.end() && 
              attributes.find(d)->second.player == probPlayer)
            pri[d] = c+1;
        }
      }
    }
  }
}

Model::StrategyResults Model::getStrategy2Player(void) const
{
  sanityCheck();

  set<Node> restriction;
  for (auto const & it : attributes) {
    restriction.insert(it.first);
  }
  
  return getStrategy2Player(restriction, set<Index>());
}

Model::StrategyResults Model::getStrategy2Player(set<Node> const & restriction,
                                                 set<Index> const & forbiddenEdges) const
{
  sanityCheck();

  map<Node, Priority> pri;
  for (Node n : restriction) {
    NodeAttributes const & nAttr = attributes.find(n)->second;
    bool first = true;
    Priority pN = 0;
    for (auto const & t : nAttr.outEdges) {
      if (first) {
        first = false;
        pN = transitions[t].priority;
      } else if (pN != transitions[t].priority) {
        throw runtime_error("2 1/2 player model checking only supports priorities " \
                            "on vertices, i.e all outgoing edges have the same priority. " \
                            " Consider augmenting your MDP so that this is true.");
      }
    }
    pri[n] = pN;
  }

  return getStrategy2Player(pri, restriction, forbiddenEdges);
}

Model::StrategyResults Model::getStrategy2Player(map<Node, Priority> pri,
                                                 set<Node> const & restriction,
                                                 set<Index> const & forbiddenEdges) const
{
  set<Node> playerNodes;
  for (Node n : restriction) {
    NodeAttributes const & nAttr = attributes.find(n)->second;
    if (nAttr.player == 1)
      playerNodes.insert(n);
  }

  Model::StrategyResults rv;
  unordered_map<Node, Action> f; // Strategy for p1

  // Initialize
  auto qualiResults = probMcNaughton(map<Node, Priority>(pri), restriction, forbiddenEdges);
  if (qualiResults.w0.empty()) {
    for (Node n : restriction) {
      rv.vals[n] = 0.0;
      rv.strategy[n] = qualiResults.strategy[n];
    }

    return rv;
  }
  // Skipping initialization optimizations and intializing off qualiResults
  for (auto const & t : qualiResults.strategy) {
    if (attributes.find(t.first)->second.player == 1)
      f[t.first] = t.second;
    else
      f[t.first] = invalid;
  }

  // Improve
  while (true) {
    Model pruned = this->pruneToStrategy(f, false, false);
    auto results = pruned.getStrategy1Player();
    bool improved = false;
    for (Node n : playerNodes) {
      NodeAttributes const & nAttr = attributes.find(n)->second;
      for (Index const & t : nAttr.outEdges) {
        if (results.vals[transitions[t].destination] < results.vals[n] - 10*epsilon) {
          f[n] = transitions[t].action;
          improved = true;
          break;
        }
      }
    }
    if (!improved) {
      set<Index> notNeutral;
      for (Node n : restriction) {
        NodeAttributes const & nAttr = attributes.find(n)->second;
        unsigned inserts = 0;
        for (Index const & t : nAttr.outEdges) {
          if (nAttr.player != probPlayer &&
              fabs(results.vals[transitions[t].destination] - results.vals[n]) > 10*epsilon) {
            notNeutral.insert(t);
            inserts++;
          }
        }
        if (inserts == nAttr.outEdges.size()) {
          double minGap = INFINITY;
          for (Index const & t : nAttr.outEdges)
            if (fabs(results.vals[transitions[t].destination] - results.vals[n]) < minGap)
              minGap = fabs(results.vals[transitions[t].destination] - results.vals[n]);
          throw runtime_error("Node with all edges removed from game. Minimum gap is " + to_string(minGap) + ". Please report bug.");
        }
      }
      auto qualiResults = probMcNaughton(map<Node, Priority>(pri), restriction, notNeutral);
      bool changed = false;
      for (Node n : qualiResults.w1) {
        if (attributes.find(n)->second.player != 1)
          continue;
        if (f[n] != qualiResults.strategy[n])
          changed = true;
        f[n] = qualiResults.strategy[n];
      }
      if (!changed) {
        rv.vals = results.vals;
        for (Node n : restriction) {
          if (attributes.find(n)->second.player == 1)
            rv.strategy[n] = f[n];
          else
            rv.strategy[n] = results.strategy[n];
        }

        return rv;
      }
    }
  }
}
