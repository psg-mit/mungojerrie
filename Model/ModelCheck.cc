/** @file ModelCheck.cc

  @brief Model checking algorithms for MDPs.

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
#include "Pdriver.hh"

#if defined(HAVE_LIBORTOOLS) && defined(HAVE_ORTOOLS_LINEAR_SOLVER_LINEAR_SOLVER_H)
// Suppress warnings in OR tools when compiling with either g++ or clang++.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Woverflow"
#include "ortools/linear_solver/linear_solver.h"
#include "ortools/linear_solver/linear_solver.pb.h"
#pragma GCC diagnostic pop
#endif

using namespace std;


/**
   @brief Class to compute the (maximal) SCCs of the model.

   @details Implements Tarjan's algorithm.  Only SCCs reachable from
   some state in "init" are returned.  Only states that are in
   "restriction" and edges that are not in "forbiddenEdges" are
   visited.
*/
class Model::SccAnalyzer {
public:
  SccAnalyzer(Model const & model, vector< set<Node> > & sccs,
              set<Node> const & init, set<Node> const & restriction,
              set<Index> const & forbiddenEdges) :
    model(model), count(0), sccs(sccs), init(init), restriction(restriction),
    forbiddenEdges(forbiddenEdges) {}

  void search(void)
  {
    for (Node root : init) {
      if (!visited(root)) {
        searchScc(root);
      }
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

  using Info = unordered_map<Node,DfsNode>;

#if 1
  void searchScc(Node node) {
    // "Recursion" stack.
    using Witem = std::pair<Node,Index>;
    std::stack<Witem> work{{Witem{node,0}}};

    while (!work.empty()) {
      Witem const & edge = work.top();
      work.pop();

      if (edge.second == 0) {
        // This is the first time we look at this node.
        visit(edge.first);
      }
      DfsNode & vic = ndInfo.find(edge.first)->second;
      // Look for an unexplored edge.
      bool explore = false;
      Edges const & outEdges = model.attributes.find(edge.first)->second.outEdges;
      for (Index j = edge.second; j < outEdges.size(); ++j) {
        Index i = outEdges[j];
        // Check whether restrictions on edges and states are satisfied.
        if (forbiddenEdges.find(i) != forbiddenEdges.end()) continue;
        Node succ = model.transitions.at(i).destination;
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
  unsigned searchScc(Node node) {
    unsigned low = count;
    unsigned dfn = visit(node);
    Edges const & outEdges = model.attributes.find(node)->second.outEdges;
    for (Index i : outEdges) {
      if (forbiddenEdges.find(i) != forbiddenEdges.end()) continue;
      Node succ = model.transitions.at(i).destination;
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
    if (sccEntryPoint(dfn, low)) {
      collectScc(node);
    }
    return low;
  }
#endif

  bool visited(Node node) const
  {
    return ndInfo.find(node) != ndInfo.end();
  }

  unsigned visit(Node node)
  {
    stk.push(node);
    ndInfo.emplace(node,DfsNode{true,count,count});
    return count++;
  }

  bool sccEntryPoint(unsigned dfn, unsigned low) const
  {
    return dfn == low;
  }

  void collectScc(Node node)
  {
    sccs.emplace_back(set<Node>{});
    set<Node> & component = sccs.back();
    Node x;
    do {
      x = stk.top();
      stk.pop();
      DfsNode & ix = ndInfo.find(x)->second;
      ix.stacked = false;
      component.insert(x);
    } while (x != node);
  }

  bool inBounds(Node node) const
  {
    return restriction.find(node) != restriction.end();
  }

  Model const & model;
  stack<Node,vector<Node>> stk;
  Info ndInfo;
  unsigned count;
  vector<set<Node>> & sccs;
  set<Node> const & init;
  set<Node> const & restriction;
  set<Index> const & forbiddenEdges;
};


// Overloads of Model::getSCCs.

vector<set<Node>> Model::getSCCs(
  set<Node> const & init,
  set<Node> const & restriction,
  set<Index> const & forbiddenEdges) const
{
  vector<set<Node>> sccs;
  SccAnalyzer sa(*this, sccs, init, restriction, forbiddenEdges);
  sa.search();
  return sccs;
}


vector<set<Node>> Model::getSCCs(
  set<Node> const & init,
  set<Node> const & restriction) const
{
  return getSCCs(init, restriction, set<Index>{});
}


vector<set<Node>> Model::getSCCs(set<Node> const & restriction) const
{
  return getSCCs(restriction, restriction);
}


vector<set<Node>> Model::getSCCs(void) const
{
  set<Node> init({initialState});
  set<Node> restriction;
  for (auto const & attr : attributes) {
    restriction.insert(attr.first);
  }
  return getSCCs(init, restriction);
}


// Checks whether an SCC is trivial under the assumption
// that the states in scc actually form an SCC.
// That is, checks whether there is one state without self-loop.
bool Model::isTrivial(set<Node> const & scc) const
{
  if (scc.size() != 1) return false;
  State state = *scc.cbegin();
  Edges const & outEdges = attributes.find(state)->second.outEdges;
  for (Index i : outEdges) {
    Transition const & t = transitions.at(i);
    if (t.destination == state) {
      return false;
    }
  }
  return true;
}


unordered_map<Node, Action> Model::getStrategyWECs(vector< set<Node> > const & WECs) const
{
  unordered_map<Node, Action> strategy;
  
  for (size_t i = 0; i < WECs.size(); i++) {
    unordered_map<Node, double> vals;
    set<Node> target;
    set<Index> forbiddenEdges;
    Priority maxP = 0;
    for (Node n : WECs[i]) {
      Edges const & d = attributes.find(n)->second.outEdges;
      for (long index = (long) d.size()-1; index >= 0; index--) {
        auto id = d[index];
        Transition const & t = transitions[id];
        if (t.priority % 2 != 0 &&
            WECs[i].find(t.destination) != WECs[i].end()) {
          maxP = max(maxP, t.priority);
          break;
        }
      }
    }
    for (Node n : WECs[i]) {
      Edges const & d = attributes.find(n)->second.outEdges;
      for (long index = (long) d.size()-1; index >= 0; index--) {
        auto id = d[index];
        Transition const & t = transitions[id];
        if (t.priority > maxP &&
            WECs[i].find(t.destination) != WECs[i].end())
          forbiddenEdges.insert(id);
        else if (t.priority == maxP &&
                 WECs[i].find(t.destination) != WECs[i].end()) {
          target.insert(n);
          break;
        }
      }
    }
    for (Node n : target) {
      Edges const & d = attributes.find(n)->second.outEdges;
      for (long index = (long) d.size()-1; index >= 0; index--) {
        auto id = d[index];
        Transition const & t = transitions[id];
        if (t.priority != maxP) 
          forbiddenEdges.insert(id);
      }
    } // NOTE to use sorted nature of edges

    vals = getSSP(target, WECs[i], forbiddenEdges);
    
    for (Node n : WECs[i]) {
      NodeAttributes const & nAttr = attributes.find(n)->second;
      if (nAttr.player == probPlayer) {
        strategy.emplace(n, invalid);
        continue;
      }
      Action minAct = invalid;
      double minVal = INFINITY;
      Edges const & d = attributes.find(n)->second.outEdges;
      for (long index = 0; index < d.size(); index++) {
        auto id = d[index];
        if (transitions[id].priority > maxP)
          break;
        if (forbiddenEdges.find(id) != forbiddenEdges.end() ||
            WECs[i].find(transitions[id].destination) == WECs[i].end())
          continue;
        Transition const & t = transitions[id];
        double destVal = vals.find(t.destination)->second;
        if (minVal - destVal > epsilon) {
          minVal = destVal;
          minAct = t.action;
        }
      }
      if(minAct == invalid)
        throw logic_error("Assertion failed. WECs incorrectly calculated. Please report bug.");
      strategy.emplace(n, minAct);
    }
  }

  return strategy;
}

unordered_map<Node, Action> Model::getStrategyWECs(void) const
{
  sanityCheck();

  return getStrategyWECs(getWECs());
}

#if defined(HAVE_LIBORTOOLS) && defined(HAVE_ORTOOLS_LINEAR_SOLVER_LINEAR_SOLVER_H)

unordered_map<Node, double> Model::getSSPORTools(set<Node> const & target,
                                       set<Node> const & restriction,
                                       set<Index> const & forbiddenEdges) const
{
  using namespace operations_research;

  unordered_map<Node, double> result;
  queue<Node> q;
  set<Node> reachableSet;
  unordered_map<Node, MPVariable* const> variables;

  MPSolver::OptimizationProblemType pType = 
    MPSolver::GLOP_LINEAR_PROGRAMMING;
  MPSolver solver("ShortestPathLP", pType);
  const double infinity = solver.infinity();

  for (Node n : target) {
    variables.emplace(n, solver.MakeNumVar(0.0, 0.0, to_string(n)));
    reachableSet.insert(n);
    q.push(n);
  }
  while (!q.empty()) {
    Node n = q.front();
    q.pop();
    Edges const & d = attributes.find(n)->second.inEdges;
    for (auto i : d) {
      if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
          restriction.find(transitions[i].source) != restriction.end() &&
          reachableSet.find(transitions[i].source) == reachableSet.end()) {
        reachableSet.insert(transitions[i].source);
        q.push(transitions[i].source);
      }
    }
  }
  
  for (Node current : reachableSet) {
    NodeAttributes const & currAttr = attributes.find(current)->second;
    if (currAttr.player == probPlayer)
      continue;
    MPVariable* currVar;
    auto it = variables.find(current);
    if (it == variables.end()) {
      currVar = solver.MakeNumVar(0.0, infinity, to_string(current));
      variables.emplace(current, currVar);
    } else {
      currVar = it->second;
    }
    MPVariable* destVar;
    MPConstraint* c;
    for (auto i : currAttr.outEdges) {
      if (forbiddenEdges.find(i) != forbiddenEdges.end() ||
          reachableSet.find(transitions[i].destination) == reachableSet.end())
        continue;
      Transition const & t = transitions[i];
      NodeAttributes const & destAttr = attributes.find(t.destination)->second;
      if (destAttr.player != probPlayer) {
        if (t.destination == t.source)
          continue;
        auto it = variables.find(t.destination);
        if (it == variables.end()) {
          destVar = solver.MakeNumVar(0.0, infinity, to_string(t.destination));
          variables.emplace(t.destination, destVar);
        } else {
          destVar = it->second;
        }
        if (destVar != currVar) {
          c = solver.MakeRowConstraint(-1.0, infinity);
          c->SetCoefficient(currVar, -1);
          c->SetCoefficient(destVar, 1);
        }
      } else {
        double currVarrCoeff = -1;
        double norm = 0.0;
        for (auto id : destAttr.outEdges) {
          if (forbiddenEdges.find(id) != forbiddenEdges.end() ||
              reachableSet.find(transitions[id].destination) == reachableSet.end())
            continue;
          norm += transitions[id].probability;
        }
        c = solver.MakeRowConstraint(-1.0, infinity);
        for (auto id : destAttr.outEdges) {
          if (forbiddenEdges.find(id) != forbiddenEdges.end() ||
              reachableSet.find(transitions[id].destination) == reachableSet.end())
            continue;
          auto it = variables.find(transitions[id].destination);
          if (it == variables.end()) {
            destVar = solver.MakeNumVar(0.0, infinity, to_string(transitions[id].destination));
            variables.emplace(transitions[id].destination, destVar);
          } else {
            destVar = it->second;
          }
          if (destVar != currVar)
            c->SetCoefficient(destVar, (transitions[id].probability)/norm);
          else 
            currVarrCoeff += (transitions[id].probability)/norm;
        }
        c->SetCoefficient(currVar, currVarrCoeff);
      }
    }
  }
  
  MPObjective* const objective = solver.MutableObjective();
  for (auto it = variables.begin(); it != variables.end(); it++) {
    objective->SetCoefficient(it->second, 1);
  }
  objective->SetMaximization();

  /*
  string printMe;
  solver.ExportModelAsLpFormat(false, &printMe);
  cout << "Linear Program : \n" << printMe << endl;
  */

  const MPSolver::ResultStatus solverResult = solver.Solve();
  if (solverResult != MPSolver::OPTIMAL)
    throw logic_error("There is no optimal solution for the strategy.");

  for (Node n : restriction) {
    if (attributes.find(n)->second.player == probPlayer)
      continue;
    auto it = variables.find(n);
    if (it != variables.end()) {
      double val = it->second->solution_value();
      if (val == infinity)
        throw logic_error("Variable with infinite value.");
      result.emplace(n, val);
    } else
      result.emplace(n, INFINITY);
  }
  for (Node n : restriction) {
    if (reachableSet.find(n) == reachableSet.end()) {
      result.emplace(n, INFINITY);
      continue;
    }
    NodeAttributes const & nAttr = attributes.find(n)->second;
    if (nAttr.player == probPlayer) {
      double pVal = 0.0;
      double norm = 0.0;
      for (auto i : nAttr.outEdges) {
        if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
            reachableSet.find(transitions[i].destination) != reachableSet.end()) {
          norm += transitions[i].probability;
        }
      }
      for (auto i : nAttr.outEdges) {
        if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
            reachableSet.find(transitions[i].destination) != reachableSet.end()) {
          pVal += ((transitions[i].probability)/norm) * 
                  result.find(transitions[i].destination)->second;
        }
      }
      result.emplace(n, pVal);
    }
  }

  return result;
}     

unordered_map<Node, double> Model::getSSPPoly(std::set<Node> const & target,
                                              std::set<Node> const & restriction,
                                              std::set<Index> const & forbiddenEdges) const
{
  using namespace operations_research;

  unordered_map<Node, double> result;
  queue<Node> q;
  set<Node> reachable;
  set<Index> policy;
  unordered_map<Node, MPVariable* const> variables;

  MPSolver::OptimizationProblemType pType = 
    MPSolver::GLOP_LINEAR_PROGRAMMING;
  MPSolver solver("ShortestPathPolyLP", pType);
  const double infinity = solver.infinity();

  for (Node n : target) {
    reachable.insert(n);
    q.push(n);
  }
  while (!q.empty()) {
    Node current = q.front();
    q.pop();
    NodeAttributes const & currAttr = attributes.find(current)->second;
    for (auto i : currAttr.inEdges) {
      if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
          restriction.find(transitions[i].source) != restriction.end() &&
          reachable.find(transitions[i].source) == reachable.end()) {
        reachable.insert(transitions[i].source);
        q.push(transitions[i].source);
        NodeAttributes const & predAttr = attributes.find(transitions[i].source)->second;
        if (predAttr.player != probPlayer)
          if (!policy.insert(i).second) cout << "DUPLICATE" << endl;///NOTE
      }
    }
  }

  bool policyChanged = true;
  while (policyChanged) {

  /*
  map<Node, int> matrixVar;
  int currMatrixN = 1;
  int row = 1;
  cout << target.size() << " # of 0s" << endl;
  */

    solver.Clear();
    policyChanged = false;
    result = unordered_map<Node, double> {};
    variables = unordered_map<Node, MPVariable* const> {};
    for (Node n : target) {
      MPVariable* targetVar = solver.MakeNumVar(0.0, 0.0, to_string(n));
      variables.emplace(n, targetVar);
      /*
      int targetMVar = currMatrixN++;
      matrixVar.emplace(n, targetMVar);
      cout << row++ << " " << targetMVar << " 1" << endl;
      */
    }

    for (Node current : reachable) {
      if (target.find(current) != target.end())
        continue;
      NodeAttributes const & currAttr = attributes.find(current)->second;
      if (currAttr.player == probPlayer)
        continue;
      MPVariable* currVar;
      auto it = variables.find(current);
      if (it == variables.end()) {
        currVar = solver.MakeNumVar(0.0, infinity, to_string(current));
        variables.emplace(current, currVar);
      } else {
        currVar = it->second;
      }
      /*
      cout.precision(18);
      int currMVar;
      auto itM = matrixVar.find(current);
      if (itM == matrixVar.end()) {
        currMVar = currMatrixN++;
        matrixVar.emplace(current, currMVar);
      } else {
        currMVar = itM->second;
      }
      */
      MPVariable* destVar;
      MPConstraint* c;

      for (auto i : currAttr.outEdges) {
        if (policy.find(i) == policy.end())
          continue;
        Transition const & t = transitions[i];
        NodeAttributes const & destAttr = attributes.find(transitions[i].destination)->second;
        if (destAttr.player != probPlayer) {
          if (t.destination == current)
            continue;
          auto it = variables.find(t.destination);
          if (it == variables.end()) {
            destVar = solver.MakeNumVar(0.0, infinity, to_string(t.destination));
            variables.emplace(t.destination, destVar);
          } else {
            destVar = it->second;
          }
          c = solver.MakeRowConstraint(1.0, 1.0);
          c->SetCoefficient(currVar, 1);
          c->SetCoefficient(destVar, -1);
          /*
          int destMVar;
          auto itM = matrixVar.find(t.destination);
          if (itM == matrixVar.end()) {
            destMVar = currMatrixN++;
            matrixVar.emplace(t.destination, destMVar);
          } else {
            destMVar = itM->second;
          }
          cout << row << " " << currMVar << " 1" << endl;
          cout << row << " " << destMVar << " -1" << endl;
          row++;
          */
        } else {
          double norm = 0.0;
          double currVarCoeff = 0.0;
          for (auto id : destAttr.outEdges) {
            if (forbiddenEdges.find(id) != forbiddenEdges.end() ||
                reachable.find(transitions[id].destination) == reachable.end())
              continue;
            norm += transitions[id].probability;
            if (transitions[id].destination == current)
              currVarCoeff -= transitions[id].probability;
          }
          currVarCoeff = 1.0 + (currVarCoeff / norm);
          c = solver.MakeRowConstraint(1.0, 1.0);
          c->SetCoefficient(currVar, currVarCoeff);
          /*
          cout << row << " " << currMVar << " " << currVarCoeff << endl;
          */
          for (auto id : destAttr.outEdges) {
            if (forbiddenEdges.find(id) != forbiddenEdges.end() ||
                reachable.find(transitions[id].destination) == reachable.end() ||
                transitions[id].destination == current)
              continue;
            auto it = variables.find(transitions[id].destination);
            if (it == variables.end()) {
              destVar = solver.MakeNumVar(0.0, infinity, to_string(transitions[id].destination));
              variables.emplace(transitions[id].destination, destVar);
            } else {
              destVar = it->second;
            }
            c->SetCoefficient(destVar, -transitions[id].probability / norm);
            /*
            int destMVar;
            auto itM = matrixVar.find(transitions[id].destination);
            if (itM == matrixVar.end()) {
              destMVar = currMatrixN++;
              matrixVar.emplace(transitions[id].destination, destMVar);
            } else {
              destMVar = itM->second;
            }
            cout << row << " " << destMVar << " " << -transitions[id].probability / norm << endl;
            */
          }
          /*
          row++;
          */
        }
        break;
      }
    }

    /*
    MPObjective* const objective = solver.MutableObjective();
    for (auto it = variables.begin(); it != variables.end(); it++) {
      objective->SetCoefficient(it->second, 1);
    }
    objective->SetMaximization();
    */

    /*
    cout << "# constraints: " << solver.NumConstraints() << endl;//NOTE
    cout << "# variables: " << solver.NumVariables() << endl;//NOTE
    */

    /*
    string printMe;
    solver.ExportModelAsLpFormat(false, &printMe);
    cout << "Linear Program : \n" << printMe << endl;
    */

    const MPSolver::ResultStatus solverResult = solver.Solve();
    if (solverResult != MPSolver::OPTIMAL)
      throw logic_error("There is no optimal solution for the strategy.");

    for (Node n : reachable) {
      if (attributes.find(n)->second.player == probPlayer)
        continue;
      result.emplace(n, variables.find(n)->second->solution_value());
    }
    for (Node n : reachable) {
      NodeAttributes const & nAttr = attributes.find(n)->second;
      if (nAttr.player == probPlayer) {
        double pVal = 0.0;
        double norm = 0.0;
        for (auto i : nAttr.outEdges) {
          if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
              reachable.find(transitions[i].destination) != reachable.end()) {
            norm += transitions[i].probability;
          }
        }
        for (auto i : nAttr.outEdges) {
          if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
              reachable.find(transitions[i].destination) != reachable.end()) {
            pVal += ((transitions[i].probability)/norm) * 
                    result.find(transitions[i].destination)->second;
          }
        }
        result.emplace(n, pVal);
      }
    }

    set<Index> newPolicy;
    for (Node n : reachable) {
      if (target.find(n) != target.end())
        continue;
      NodeAttributes const & nAttr = attributes.find(n)->second;
      if (nAttr.player != probPlayer) {
        double minVal = INFINITY;
        Index maxI = -1;
        for (auto i : nAttr.outEdges) {
          if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
              reachable.find(transitions[i].destination) != reachable.end()) {
            double val = result.find(transitions[i].destination)->second;
            if (fabs(val - minVal) < epsilon && policy.find(i) != policy.end())
              maxI = i;
            if (minVal - val > epsilon) {
              minVal = val;
              maxI = i;
            }
          }
        }
        newPolicy.insert(maxI);
        if (!policyChanged && policy.find(maxI) == policy.end())
          policyChanged = true;
      }
    }
    policy = newPolicy;
  }
  
  for (Node n : reachable) {
    if (attributes.find(n)->second.player == probPlayer)
      continue;
    result.emplace(n, variables.find(n)->second->solution_value());
  }
  for (Node n : restriction) {
    if (reachable.find(n) == reachable.end()) {
      result.emplace(n, INFINITY);
      continue;
    }
    NodeAttributes const & nAttr = attributes.find(n)->second;
    if (nAttr.player == probPlayer) {
      double pVal = 0.0;
      double norm = 0.0;
      for (auto i : nAttr.outEdges) {
        if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
            reachable.find(transitions[i].destination) != reachable.end()) {
          norm += transitions[i].probability;
        }
      }
      for (auto i : nAttr.outEdges) {
        if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
            reachable.find(transitions[i].destination) != reachable.end()) {
          pVal += ((transitions[i].probability)/norm) * 
                  result.find(transitions[i].destination)->second;
        }
      }
      result.emplace(n, pVal);
    }
  }

  return result;
}

unordered_map<Node, double> Model::getProbReachabilityORTools(
                                              set<Node> const & target,
                                              set<Node> const & restriction,
                                              set<Index> const & forbiddenEdges) const
{
  using namespace operations_research;

  unordered_map<Node, double> vals;
  queue<Node> q;
  unordered_set<Node> reached;
  unordered_map<Node, MPVariable* const> variables;

  MPSolver::OptimizationProblemType pType = 
    MPSolver::GLOP_LINEAR_PROGRAMMING;
  MPSolver solver("MaxReachabilityLP", pType);
  const double infinity = solver.infinity();

  //Hard coded 0, NOTE
  unordered_set<Node> uTarget = getAttractorNoCheck(target, 0, restriction, forbiddenEdges);
  vector< set<Node> > MECs = getMECsNoCheck(restriction, forbiddenEdges);
  unordered_map<Node, size_t> MECsMap;
  unordered_map<size_t, MPVariable* const> mecVariables;
  for (size_t i = 0; i < MECs.size(); i++) {
    for (Node n : MECs[i]) {
      MECsMap.emplace(n, i);
    }
  }
  for (Node n : uTarget) {
    reached.insert(n);
    auto mit = MECsMap.find(n);
    if (mit != MECsMap.end()) {
      auto vit = mecVariables.find(mit->second);
      if (vit == mecVariables.end())
        mecVariables.emplace(mit->second, solver.MakeNumVar(1.0, 1.0, "MEC" + to_string(mit->second)));
    } else {
      variables.emplace(n, solver.MakeNumVar(1.0, 1.0, to_string(n)));
    }
    Edges const & in = attributes.find(n)->second.inEdges;
    for (auto i : in) {
      if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
          restriction.find(transitions[i].source) != restriction.end() &&
          uTarget.find(transitions[i].source) == uTarget.end() &&
          reached.find(transitions[i].source) == reached.end()) {
        //reached.insert(transitions[i].source); NOTE
        q.push(transitions[i].source);
      }
    }
  }

  auto setupConstraint = [&](Node current, MPVariable* const currVar) {
    NodeAttributes const & currAttr = attributes.find(current)->second;
    auto cit = MECsMap.find(current);
    for (auto i : currAttr.outEdges) {
      if (forbiddenEdges.find(i) != forbiddenEdges.end() ||
          restriction.find(transitions[i].destination) == restriction.end())
        continue;
      if (cit != MECsMap.end()) {
        auto dit = MECsMap.find(transitions[i].destination);
        if (dit != MECsMap.end() && dit->second == cit->second)
          continue;
      }
      NodeAttributes const & destAttr = attributes.find(transitions[i].destination)->second;
      if (destAttr.player != probPlayer) {
        MPVariable* destVar;
        auto it = variables.find(transitions[i].destination);
        if (it == variables.end()) {
          auto mit = MECsMap.find(transitions[i].destination);
          if (mit != MECsMap.end()) {
            auto vit = mecVariables.find(mit->second);
            if (vit == mecVariables.end()) {
              destVar = solver.MakeNumVar(0.0, infinity, "MEC" + to_string(mit->second));
              mecVariables.emplace(mit->second, destVar);
            } else {
              destVar = vit->second;
            }
          } else {
            destVar = solver.MakeNumVar(0.0, infinity, to_string(transitions[i].destination));
            variables.emplace(transitions[i].destination, destVar);
          }
        } else {
          destVar = it->second;
        }
        if (currVar != destVar) {
          MPConstraint* const c = solver.MakeRowConstraint(0.0, infinity);
          c->SetCoefficient(currVar, 1);
          c->SetCoefficient(destVar, -1);
        }
      } else {
        MPConstraint* const c = solver.MakeRowConstraint(0.0, infinity);
        double currVarCoeff = 1;
        map<MPVariable* const, double> mecVarCoeff;
        for (auto id : destAttr.outEdges) {     
          if (forbiddenEdges.find(id) != forbiddenEdges.end() ||
              restriction.find(transitions[id].destination) == restriction.end())
            continue;
          MPVariable* destVar;
          bool isMEC = false;
          auto it = variables.find(transitions[id].destination);
          if (it == variables.end()) {
            auto mit = MECsMap.find(transitions[id].destination);
            if (mit != MECsMap.end()) {
              auto vit = mecVariables.find(mit->second);
              if (vit == mecVariables.end()) {
                destVar = solver.MakeNumVar(0.0, infinity, "MEC" + to_string(mit->second));
                mecVariables.emplace(mit->second, destVar);
              } else {
                destVar = vit->second;
              }
              isMEC = true;
            } else {
              destVar = solver.MakeNumVar(0.0, infinity, to_string(transitions[id].destination));
              variables.emplace(transitions[id].destination, destVar);
            }
          } else {
            destVar = it->second;
          }
          if (destVar == currVar)
            currVarCoeff += -transitions[id].probability;
          else if (isMEC) {
            auto cit = mecVarCoeff.find(destVar);
            if (cit == mecVarCoeff.end()) {
              cit = mecVarCoeff.emplace(destVar, -transitions[id].probability).first;
            } else {
              cit->second += -transitions[id].probability;
            }
          } else
            c->SetCoefficient(destVar, -transitions[id].probability);
        }
        c->SetCoefficient(currVar, currVarCoeff);
        for (auto const & p : mecVarCoeff) {
          c->SetCoefficient(p.first, p.second);
        }
      }
    }
  };

  while (!q.empty()) {
    Node current = q.front();
    q.pop();
    if (reached.find(current) != reached.end())
      continue;
    reached.insert(current); //NOTE

    auto it = MECsMap.find(current);
    if (it != MECsMap.end()) {
      auto i = it->second;
      MPVariable* mecVar;
      auto vit = mecVariables.find(i);
      if (vit == mecVariables.end()) {
        mecVar = solver.MakeNumVar(0.0, infinity, "MEC" + to_string(i));
        mecVariables.emplace(i, mecVar);
      } else {
        mecVar = vit->second;
      }
      for (Node n : MECs[i]) {
        reached.insert(n);
        NodeAttributes const & nAttr = attributes.find(n)->second;
        for (auto id : nAttr.inEdges) {
          auto it = MECsMap.find(transitions[id].source);
          if ((it == MECsMap.end() || it->second != i) &&
              forbiddenEdges.find(id) == forbiddenEdges.end() &&
              reached.find(transitions[id].source) == reached.end() &&
              restriction.find(transitions[id].source) != restriction.end()) {
            //reached.insert(transitions[id].source); NOTE
            q.push(transitions[id].source);
          }
        }
        if (nAttr.player == probPlayer)
          continue;
        setupConstraint(n, mecVar);
      }
    } else {
      NodeAttributes const & currAttr = attributes.find(current)->second;
      for (auto i : currAttr.inEdges) {
        if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
            reached.find(transitions[i].source) == reached.end() &&
            restriction.find(transitions[i].source) != restriction.end()) {
          //reached.insert(transitions[i].source); NOTE
          q.push(transitions[i].source);
        } 
      }
      if (currAttr.player == probPlayer)
        continue;
      MPVariable* currVar;
      auto it = variables.find(current);
      if (it == variables.end()) {
        currVar = solver.MakeNumVar(0.0, infinity, to_string(current));
        variables.emplace(current, currVar);
      } else {
        currVar = it->second;
      }
      setupConstraint(current, currVar);
    }
  }

  MPObjective* const objective = solver.MutableObjective();
  for (auto it = variables.begin(); it != variables.end(); it++) {
    if (reached.find(it->first) == reached.end()) {
      MPConstraint* const c = solver.MakeRowConstraint(0.0, 0.0);
      c->SetCoefficient(it->second, 1);
    }
    objective->SetCoefficient(it->second, 1);
  }
  for (auto it = mecVariables.begin(); it != mecVariables.end(); it++) {
    if (reached.find(*((MECs[it->first]).begin())) == reached.end()) {
      MPConstraint* const c = solver.MakeRowConstraint(0.0, 0.0);
      c->SetCoefficient(it->second, 1);
    }
    objective->SetCoefficient(it->second, 1);
  }
  objective->SetMinimization();
  //cout << "Reached: " << reached << endl;//NOTE
  /*
  string printMe;
  solver.ExportModelAsMpsFormat(false, false, &printMe);
  solver.ExportModelAsLpFormat(false, &printMe);
  cout << "Linear Program : \n" << printMe << endl;
  */

  const MPSolver::ResultStatus solverResult = solver.Solve();
  if (solverResult != MPSolver::OPTIMAL)
    throw logic_error("There is no optimal solution for the strategy.");

  for (Node n : restriction) {
    auto it = variables.find(n);
    if (it != variables.end())
      vals.emplace(n, it->second->solution_value());
    else {
      NodeAttributes const & nAttr = attributes.find(n)->second;
      if (nAttr.player != probPlayer) {
        auto mit = MECsMap.find(n);
        if (mit == MECsMap.end())
          vals.emplace(n, 0.0);
        else {
          auto vit = mecVariables.find(mit->second);
          if (vit == mecVariables.end())
            vals.emplace(n, 0.0);
          else
            vals.emplace(n, vit->second->solution_value());
        }
      }
    }
  }
  for (Node n : restriction) {
    NodeAttributes const & nAttr = attributes.find(n)->second;
    if (nAttr.player == probPlayer) {
      double pVal = 0.0;
      for (auto i : nAttr.outEdges) {
        if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
            restriction.find(transitions[i].destination) != restriction.end())
          pVal += transitions[i].probability * vals.find(transitions[i].destination)->second;
      }
      vals.emplace(n, pVal);
      // result.strategy.emplace(n, invalid);
    }
  }

  ///
  if (verbosity > Verbosity::Terse) {
    Util::printElapsedTag();
    cout << " probability of satisfaction: " <<
      vals.find(initialState)->second << endl;
  }
  ///

  return vals;
}

#else

unordered_map<Node, double> Model::getSSPORTools(set<Node> const &,
                                                 set<Node> const &,
                                                 set<Index> const &) const
{
  throw logic_error("OR-Tools cannot be located on this system.");
}

unordered_map<Node, double> Model::getSSPPoly(std::set<Node> const &,
                                              std::set<Node> const &,
                                              std::set<Index> const &) const
{
  throw logic_error("OR-Tools cannot be located on this system.");
}

std::unordered_map<Node, double> Model::getProbReachabilityORTools(
                                              set<Node> const &,
                                              set<Node> const &,
                                              set<Index> const &) const
{
  throw logic_error("OR-Tools cannot be located on this system.");
}

#endif

unordered_map<Node, double> Model::getSSPIncr(set<Node> const & target,
                                              set<Node> const & restriction,
                                              set<Index> const & forbiddenEdges) const
{
  unordered_map<Node, double> result;

  for (Node n : restriction) {
    if (target.find(n) != target.end()) {
      result.emplace(n, 0.0);
    } else {
      result.emplace(n, INFINITY);
    }
  }

  queue<set<Node>> q;
  vector< set<Node> > SCCs = getSCCs(restriction, restriction, forbiddenEdges);
  for (set<Node> s : SCCs)
    q.push(s);
  queue<Node> sccQ;

  while (!q.empty()) {
    set<Node> current = q.front();
    q.pop();
    for (Node n : current) {
      if (target.find(n) == target.end())
        sccQ.push(n);
    }
    while (!sccQ.empty()) {
      Node n = sccQ.front();
      sccQ.pop();
      NodeAttributes const & nAttr = attributes.find(n)->second;
      double currVal = result.find(n)->second;
      double newVal;
      if (nAttr.player == probPlayer)
        newVal = 0.0;
      else 
        newVal = INFINITY;
      bool found = false;
      double norm = 0.0;
      Edges const & d = nAttr.outEdges;
      for (auto i : d) {
        Transition const & t = transitions[i];
        if (restriction.find(t.destination) == restriction.end())
          continue;
        double destVal = result.find(t.destination)->second;
        if (forbiddenEdges.find(i) != forbiddenEdges.end() ||
            restriction.find(t.destination) == restriction.end() || 
            std::isinf(destVal))
          continue;
        found = true;
        if (nAttr.player == probPlayer) {
          newVal += t.probability*destVal;
          norm += t.probability;
        } else if (destVal + 1 < newVal)
          newVal = destVal + 1;
      }
      if (!found) 
        newVal = INFINITY;
      else if (nAttr.player == probPlayer)
        newVal = newVal / norm;
      result[n] = newVal;
      if (( !std::isinf(newVal) && std::isinf(currVal) )||
          ( !std::isinf(newVal) && !std::isinf(currVal) && 
          fabs(newVal - currVal) > epsilon) ) {
        for (auto id : nAttr.inEdges) {
          Node p = transitions[id].source;
          if (forbiddenEdges.find(id) == forbiddenEdges.end() &&
              target.find(p) == target.end() &&
              current.find(p) != current.end())
            sccQ.push(p);
        }
      }
    }
  }

  return result;
}

unordered_map<Node, double> Model::getProbReachabilityIncr(
                                              set<Node> const & target,
                                              set<Node> const & restriction,
                                              set<Index> const & forbiddenEdges) const
{
  unordered_map<Node, double> vals;

  queue<set<Node>> q;
  vector< set<Node> > SCCs = getSCCs(restriction, restriction, forbiddenEdges);
  //Hard coded 0, NOTE
  unordered_set<Node> uTarget = getAttractorNoCheck(target, 0, restriction, forbiddenEdges);
  for (Node n : restriction) {
    if (uTarget.find(n) != uTarget.end())
      vals[n] = 1.0;
    else
      vals[n] = 0.0;
  }
  for (set<Node> & s : SCCs)
    q.push(s);
  queue<Node> sccQ;

  while (!q.empty()) {
    set<Node> current = q.front();
    q.pop();
    for (Node n : current) {
      if (uTarget.find(n) == uTarget.end())
        sccQ.push(n);
    }
    while (!sccQ.empty()) {
      Node n = sccQ.front();
      sccQ.pop();
      NodeAttributes const & nAttr = attributes.find(n)->second;
      double oldVal = vals.find(n)->second;
      double newVal = 0.0;
      for (auto i : nAttr.outEdges) {
        if (forbiddenEdges.find(i) != forbiddenEdges.end())
          continue;
        Transition const & t = transitions[i];
        double destVal = vals.find(t.destination)->second;
        if (nAttr.player == probPlayer)
          newVal += t.probability*destVal;
        else if (destVal > newVal)
          newVal = destVal;
      }
      vals[n] = newVal;
      if (fabs(newVal - oldVal) > epsilon) {
        for (auto i : nAttr.inEdges) {
          if (forbiddenEdges.find(i) != forbiddenEdges.end())
            continue;
          Node p = transitions[i].source;
          if (uTarget.find(p) == uTarget.end() && 
              current.find(p) != current.end())
            sccQ.push(p);
        }
      }
    }
  }
  
  ///
  if (verbosity > Verbosity::Terse) {
    Util::printElapsedTag();
    cout << " probability of satisfaction: " <<
      vals.find(initialState)->second << endl;
  }
  ///

  return vals;  
}

unordered_map<Node, double> Model::getProbabilityOfSat(void) const
{
  sanityCheck();

  if (isBDP()) return getBDPValueUsingORTools();

  if (!isGame()) {
    return getProbabilityOfSat1Player();
  } else {
    auto results = getStrategy2Player();
    return results.vals;
  }
}

unordered_map<Node, double> Model::getProbabilityOfSat1Player(void) const
{
  vector< set<Node> > WECs = getWECs();
  if (verbosity > Verbosity::Silent) {
    Util::printElapsedTag();
    cout << " " << WECs.size() << " WECs computed." << endl;
  }
  set<Node> target;
  for (set<Node> const & s : WECs) {
    target.insert(s.cbegin(), s.cend());
  }

  set<Node> nodes;
  for (auto const & p : attributes) {
    nodes.insert(p.first);
  }
  
  unordered_map<Node, double> vals;
  if (reachSolver == ModelOptions::ReachType::glop)
    vals = getProbReachabilityORTools(target, nodes, set<Index>());
  else
    vals = getProbReachabilityIncr(target, nodes, set<Index>());
  for (Node n : nodes) {
    if (vals[n] - 1 > 1e-6) {
      cerr << "Warning: There is a reachability probability computed that exceeds 1 by " << vals[n]-1;
      cerr << ". Consider increasing precision in the model transition probabilities." << endl;
      break;
    }
  }

  if (verbosity > Verbosity::Silent) {
    Util::printElapsedTag();
    cout << " reachability completed." << endl;
  }

  return vals;
}

Model::StrategyResults Model::getStrategyReach1Player(set<Node> const & target,
                                                      set<Node> const & restriction,
                                                      set<Index> const & forbiddenEdges) const
{
  sanityCheck();

  unordered_map<Node, double> vals;
  if (reachSolver == ModelOptions::ReachType::glop)
    vals = getProbReachabilityORTools(target, restriction, forbiddenEdges);
  else
    vals = getProbReachabilityIncr(target, restriction, forbiddenEdges);
  for (Node n : restriction) {
    if (vals[n] - 1 > 1e-6) {
      cerr << "Warning: There is a reachability probability computed that exceeds 1 by " << vals[n]-1;
      cerr << ". Consider increasing precision in the model transition probabilities." << endl;
      break;
    }
  }

  Model::StrategyResults result;
  result.vals = vals;
  
  set<Index> sspForbidden = forbiddenEdges;
  for (Node n : restriction) {
    NodeAttributes const & nAttr = attributes.find(n)->second;
    if (nAttr.player == probPlayer)
      continue;
    auto it = result.vals.find(n);
    double currVal = it->second;
    for (auto i : nAttr.outEdges) {
      if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
          (restriction.find(transitions[i].destination) == restriction.end() ||
           fabs(currVal - result.vals.find(transitions[i].destination)->second) > 8*epsilon))
        sspForbidden.insert(i);
    }
  }
  
  unordered_map<Node, double> ssp;
  ssp = getSSP(target, restriction, sspForbidden);

  for (Node n : restriction) {
    NodeAttributes const & nAttr = attributes.find(n)->second;
    if (nAttr.player == probPlayer ||
        fabs(result.vals.find(n)->second) <= epsilon ||
        target.find(n) != target.end()) {
      result.strategy.emplace(n, invalid);
      continue;
    }
    Action maxA = invalid;
    double minSSPVal = INFINITY;
    double currVal = result.vals.find(n)->second;
    for (auto i : nAttr.outEdges) {
      if (forbiddenEdges.find(i) != forbiddenEdges.end() &&
          restriction.find(transitions[i].destination) == restriction.end())
        continue;
      Transition const & t = transitions[i];
      double destVal = result.vals.find(t.destination)->second;
      if (fabs(currVal - destVal) <= 8*epsilon) { // NOTE
        double destSSPVal = ssp.find(t.destination)->second;
        if (minSSPVal - destSSPVal > epsilon) {
          minSSPVal = destSSPVal;
          maxA = t.action;
        }
      }
    }
    result.strategy.emplace(n, maxA);
  }

  return result;
}

Model::StrategyResults Model::getStrategy1Player(void) const
{
  // Compute the target state set as the union of all WECs.
  vector< set<Node> > WECs = getWECs();
  if (verbosity > Verbosity::Terse && !isGame()) {
    Util::printElapsedTag();
    cout << " " << WECs.size() << " WECs computed." << endl;
  }
  set<Node> target;
  for (set<Node> const & s : WECs) {
    target.insert(s.cbegin(), s.cend());
  }

  set<Node> nodes;
  for (auto const & p : attributes) {
    nodes.insert(p.first);
  }

  // The strategy is divided into strategy to reach some WEC and strategy
  // inside each WEC.
  Model::StrategyResults result = getStrategyReach1Player(target, nodes, set<Index>());
  if (verbosity > Verbosity::Terse) {
    Util::printElapsedTag();
    cout << " reachability completed." << endl;
  }
  unordered_map<Node, Action> strategyWECs = getStrategyWECs(WECs);
  for (auto it = strategyWECs.cbegin(); it != strategyWECs.cend(); it++) {
    result.strategy[it->first] = it->second;
  }

  return result;
}

Model::StrategyResults Model::getStrategy(void) const
{
  sanityCheck();
  if (isBDP()) return getBDPStrategy();
  if (!isGame())
    return getStrategy1Player();
  else 
    return getStrategy2Player();
}

unordered_map<Node, double> Model::getSSP(set<Node> const & target,
                                          set<Node> const & restriction,
                                          set<Index> const & forbiddenEdges) const
{
  unordered_map<Node, double> vals;

  if (sspSolver == ModelOptions::SSPType::glop)
    vals = getSSPORTools(target, restriction, forbiddenEdges);
  else if (sspSolver == ModelOptions::SSPType::poly)
    vals = getSSPPoly(target, restriction, forbiddenEdges);
  else
    vals = getSSPIncr(target, restriction, forbiddenEdges);

  return vals;
}                                          

vector< set<Node> > Model::getWECsNoCheck(set<Node> const & restriction,
                                          set<Index> const & forbiddenEdges) const
{
  queue< set<Node> > q;
  vector< set<Node> > rv;
  vector< set<Node> > initMECs = getMECsNoCheck(restriction, forbiddenEdges);
  for (set<Node> & s : initMECs)
    q.push(s);

  while (!q.empty()) {
    set<Node> current = q.front();
    q.pop();
    set<Index> currentForbiddenEdges = forbiddenEdges;
    while (1) {
      Priority highestP = 0;
      for (Node n : current) {
        Edges const & d = attributes.find(n)->second.outEdges;
        for (long index = (long) d.size()-1; index >= 0; index--) {
          auto i = d[index];
          Transition const & t = transitions[i];
          if(currentForbiddenEdges.find(i) != currentForbiddenEdges.end() ||
             current.find(t.destination) == current.end())
                continue;
          highestP = max(highestP, t.priority);
          break;
        }
      }
      if (highestP % 2) {
        rv.push_back(current);
        break;
      } else {
        if (highestP == 0)
          break;
        auto it = current.begin();
        while (it != current.end()) {
          Node n = *it;
          NodeAttributes const & nAttributes = attributes.find(n)->second;
          int nOutgoingEdges = 0;
          for (long index = (long) nAttributes.outEdges.size()-1; index >= 0; index--) {
            auto i = nAttributes.outEdges[index];
            Transition const & t = transitions[i];
            if (currentForbiddenEdges.find(i) != currentForbiddenEdges.end() ||
                current.find(t.destination) == current.end())
                  continue;
            if (t.priority == highestP) {
              currentForbiddenEdges.insert(i);
            } else {
              nOutgoingEdges++;
              break;
            }
          }
          if (nOutgoingEdges == 0) {
            for (auto i : nAttributes.outEdges) {
              if (currentForbiddenEdges.find(i) != currentForbiddenEdges.end())
                currentForbiddenEdges.erase(i);
            }
            it = current.erase(it);
          } else {
            it++;
          }
        }
        vector< set<Node> > newMECs = getMECsNoCheck(current, currentForbiddenEdges);
        if (newMECs.size() == 0) break;
        current = newMECs[0];
        for (size_t i = 1; i < newMECs.size(); i++)
          q.push(newMECs[i]);
      }
    }
  }
  return rv;
}                                        

vector< set<Node> > Model::getWECs(set<Node> const & restriction,
                                   set<pair<Node, Action>> const & forbiddenEdges) const
{
  set<Index> forbidden;

  size_t forbiddenEdgesSize = forbiddenEdges.size();
  for (Node n : restriction) {
    auto nit = attributes.find(n);
    if (nit == attributes.end())
      throw logic_error("Node " + to_string(n) + " in restriction does not exist.");
    Edges const & d = nit->second.outEdges;
    for (auto i : d)
      if (forbiddenEdges.find(make_pair(n, transitions[i].action)) !=
          forbiddenEdges.end()) {
            forbiddenEdgesSize--;
            forbidden.insert(i);
      }
    }
  if (forbiddenEdgesSize != 0)
    throw logic_error("Edge in forbiddenEdges does not exist in restriction.");

  return getWECsNoCheck(restriction, forbidden);
}

vector< set<Node> > Model::getWECs(set<Node> const & restriction) const
{
  return getWECs(restriction, set<pair<Node, Action>>());
}

vector< set<Node> > Model::getWECs(void) const
{
  set<Node> nodes;
  for (auto const & p : attributes) {
    nodes.insert(p.first);
  }
  return getWECs(nodes, set<pair<Node, Action>>());
}

vector< set<Node> > Model::getMECsNoCheck(set<Node> const & restriction,
                                          set<Index> const & forbiddenEdges) const
{
  // Each MEC is contained in an SCC of the given sub-MDP.  Hence, the
  // work queue is initialized to the SCCs of the sub-MDP defined by
  // paramters restriction and forbiddenEdges. 
  queue< set<Node> > q;
  set<Node> toDelete;
  vector< set<Node> > rv;
  vector< set<Node> > init = getSCCs(restriction, restriction, forbiddenEdges);
  for (set<Node> & s : init) {
    q.push(s);
  }

  while (!q.empty()) {
    set<Node> current = q.front();
    q.pop();
    auto it = current.begin();
    bool firstIter = true;
    while (it != current.end()) {
      Node n = *it++;
      if (toDelete.find(n) != toDelete.end()) {
        current.erase(n);
        toDelete.erase(n);
        firstIter = false;
      }
    }
    while (1) {
      // A set of size 0 is not an MEC.
      if (current.size() == 0)
        break;
      // A set of size 1 is an MEC only if it is nontrivial.
      if (current.size() == 1) {
        Node n = *(current.begin());
        auto const & d = attributes.find(n)->second.outEdges;
        bool isTrivial = true;
        for (auto i : d) {
          if (transitions[i].destination == n) {
            if (forbiddenEdges.find(i) == forbiddenEdges.end())
              isTrivial = false;
            break;
          }
        }
        if (!isTrivial) {
          rv.push_back(current);
          unordered_set<Node> randAttractor = getAttractorNoCheck(current, -1, restriction, forbiddenEdges);
          for (Node n : randAttractor)
            if(current.find(n) == current.end())
              toDelete.insert(n);
        }
        break;
      }
      // Break this set into SCCs, select the first one as current
      // and queue up the others.
      if (!firstIter) {
        vector< set<Node> > newSCCs = getSCCs(current, current, forbiddenEdges);
        if (newSCCs.size() == 0) break;
        current = newSCCs[0];
        for (size_t i = 1; i < newSCCs.size(); i++)
          q.push(newSCCs[i]);
      }
      // Process this sub-MDP.
      firstIter = false;
      bool isMEC = true;
      set<Node> attrRestriction;
      for (Node n : current) {
        NodeAttributes const & nAttributes = attributes.find(n)->second;
        bool isAttractedOut = false;
        if (nAttributes.player == probPlayer) {
          for (auto i : nAttributes.outEdges) {
            if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
                current.find(transitions[i].destination) == current.end()) {
              isAttractedOut = true;
              break;
            }
          }
        } else {
          isAttractedOut = true;
          for (auto i : nAttributes.outEdges) {
            if (forbiddenEdges.find(i) == forbiddenEdges.end() &&
                current.find(transitions[i].destination) != current.end()) {
              isAttractedOut = false;
              break;
            }
          }
        }
        if (isAttractedOut) {
          attrRestriction.insert(n);
          isMEC = false;
        }        
      }
      if (isMEC) {
        rv.push_back(current);
        unordered_set<Node> randAttractor = getAttractorNoCheck(current, -1, restriction, forbiddenEdges);
        for (Node n : randAttractor)
          if(current.find(n) == current.end())
            toDelete.insert(n);
        break;
      } else {
        unordered_set<Node> randAttractor = getAttractorNoCheck(attrRestriction, -1, current, forbiddenEdges);
        //getAttractor(attrRestriction, -1, current);//, forbiddenEdges); //NOTE
        for (Node n : randAttractor) {
          current.erase(n);
        }
      }
    }
  }
  return rv;
}

vector< set<Node> > Model::getMECs(set<Node> const & restriction,
                                      set<pair<Node, Action>> const & forbiddenEdges) const
{
  set<Index> forbidden;

  size_t forbiddenEdgesSize = forbiddenEdges.size();
  for (Node n : restriction) {
    auto nit = attributes.find(n);
    if (nit == attributes.end())
      throw logic_error("Node " + to_string(n) + " in restriction does not exist.");
    Edges const & outgoing = nit->second.outEdges;
    for (auto i : outgoing)
        if (forbiddenEdges.find(make_pair(n, transitions[i].action)) !=
            forbiddenEdges.end()) {
              forbiddenEdgesSize--;
              forbidden.insert(i);
            }
    }
  if (forbiddenEdgesSize != 0)
    throw logic_error("Edge in forbiddenEdges does not exist in restriction.");

  return getMECsNoCheck(restriction, forbidden);
}

vector< set<Node> > Model::getMECs(set<Node> const & restriction) const
{
  return getMECs(restriction, set<pair<Node, Action>>());
}

vector< set<Node> > Model::getMECs(void) const
{
  set<Node> nodes;
  for (auto const & p : attributes) {
    nodes.insert(p.first);
  }

  return getMECs(nodes, set<pair<Node, Action>>());
}

unordered_set<Node> Model::getAttractorNoCheck(set<Node> const & target, 
                                      Player targetPlayer, 
                                      set<Node> const & restriction,
                                      set<Index> const & forbiddenEdges) const
{
  unordered_set<Node> rv;
  queue<Node> q;
  unordered_map<Node, size_t> m;

  for (Node n : target) {
    rv.insert(n);
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
        if(predAttr.player == targetPlayer)
          it = m.emplace(p, 1).first;
        else {
          size_t tbe = 0;
          for (auto pI : predAttr.outEdges)
            if (forbiddenEdges.find(pI) == forbiddenEdges.end())
              tbe++;
          it = m.emplace(p, tbe).first;
        }
      }
      size_t & toBeExamined = it->second;
      if (toBeExamined > 0) {
        toBeExamined--;
        if (toBeExamined == 0) {
          rv.insert(p);
          q.push(p);
        } 
      }
    }
  }

  return rv;
}

unordered_set<Node> Model::getAttractor(set<Node> const & target, 
                                 Player targetPlayer, 
                                 set<Node> const & restriction,
                                 set<pair<Node, Action>> const & forbiddenEdges) const
{
  set<Index> forbidden;

  if (targetPlayer < 0) targetPlayer = probPlayer;
  else {
    auto it = counts.find(targetPlayer);
    if (it == counts.end() || it->second == 0)
      throw logic_error("Target player does not exist.");
  }

  size_t forbiddenEdgesSize = forbiddenEdges.size();
  size_t targetSize = target.size();
  for (Node n : restriction) {
    auto nit = attributes.find(n);
    if (nit == attributes.end())
      throw logic_error("Node " + to_string(n) + " in restriction does not exist.");
    Edges const & outgoing = nit->second.outEdges;
    for (auto i : outgoing) {
      if (forbiddenEdges.find(make_pair(n, transitions[i].action)) !=
          forbiddenEdges.end()) {
        forbiddenEdgesSize--;
        forbidden.insert(i);
      }
    }
    if (target.find(n) != target.end()) {
      targetSize--;
    }
  }
  if (forbiddenEdgesSize != 0)
    throw logic_error("Edge in forbiddenEdges does not exist in restriction.");
  if (targetSize != 0)
    throw logic_error("Node in target does not exist in restriction.");

  return getAttractorNoCheck(target, targetPlayer, restriction, forbidden);
}

unordered_set<Node> Model::getAttractor(set<Node> const & target, 
                                 Player targetPlayer, 
                                 set<Node> const & restriction) const
{
  return getAttractor(target, targetPlayer, restriction, set<pair<Node, Action>>{});
}

unordered_set<Node> Model::getAttractor(set<Node> const & target, 
                                 Player targetPlayer) const
{
  set<Node> nodes;
  for (auto const & p : attributes) {
    nodes.insert(p.first);
  }
  
  return getAttractor(target, targetPlayer, nodes, set<pair<Node, Action>>{});
}


#if defined(HAVE_LIBORTOOLS) && defined(HAVE_ORTOOLS_LINEAR_SOLVER_LINEAR_SOLVER_H)

Model::StrategyResults Model::getBDPStrategy() const {
  //Policy relevant only for existential Nodes. Will given arbitrary action otherwise.
  unordered_map<Node, double> vals = getBDPValueUsingORTools();
  Model::StrategyResults result;

  if (vals.size() == 0) return result; 

  result.vals = vals;

  std::vector<Node> keys;
  keys.reserve(attributes.size());
  for (auto const & it : attributes) {
    keys.push_back(it.first);
  }
  std::sort(keys.begin(), keys.end());

  set<Node> decisionNodes; // set of decision nodes
  for (auto const & na : attributes) {
    if (na.second.player != probPlayer) {
      decisionNodes.insert(na.first);
    }
  }

  for (Node node : decisionNodes) {
    auto ait = attributes.find(node);
    int minVal = numeric_limits<int>::max(); // Replace by infinity 
    Action minAction = -1;
    Edges const &outEdges = ait->second.outEdges;
     
    for (Index i : outEdges) {
      Transition const &t = transitions.at(i);
      Node destination = t.destination;
      int currVal = t.reward;

      if (decisionNodes.find(destination) != decisionNodes.end()) { // no probabilistic branch
        currVal = currVal + vals[destination];
      } else {
        auto pit = attributes.find(destination);
        Edges const &pOutEdges = pit->second.outEdges;
        for (Index j : pOutEdges) {
          Transition const &pt = transitions.at(j);
          currVal = currVal + pt.probability * vals[pt.destination];
        }
      }
      if (minVal > currVal) { 
        minVal = currVal;
        minAction = transitions.at(i).action;
      }
    }
    if (minVal != numeric_limits<int>::max()) {
      result.strategy.insert(make_pair(node, minAction));
      // cout << "Node: " << prettyPrintState(node) << "Val: " << minVal << endl;
    }
    else {
      throw logic_error("Serious error");
    }
  }
  return result;
}

unordered_map<Node, double> Model::getBDPValueUsingORTools() const {
  using namespace operations_research;
  int c_num = 0;
  
  MPSolver solver("linear_programming_examples", MPSolver::GLOP_LINEAR_PROGRAMMING);
  const double infinity = solver.infinity();

  unordered_map<Node, MPVariable* const> variables; // map of GLOP variables 

  std::vector<Node> keys; //set of all nodes including probabilistic 
  keys.reserve(attributes.size());
  for (auto const & it : attributes) {
    keys.push_back(it.first);
  }
  std::sort(keys.begin(), keys.end());

  for (auto const & n : keys) {    //Make non-probabilsitic nodes as variables.
    if ( attributes.at(n).player != probPlayer) {
      // variables.emplace(n, solver.MakeNumVar(0.0, 10, to_string(n)));
      variables.emplace(n, solver.MakeNumVar(0.0, infinity, to_string(n)));
 
      if (verbosity > Verbosity::Terse) {
        cout << "var " << prettyPrintState(n) << ";" << endl;
      }
    }
  }
 
 if (verbosity > Verbosity::Terse) 
        cout << "# number of variable " << variables.size() << ";" << endl;

  set<Node> decisionNodes; // set of decision nodes
  for (auto const & na : attributes) {
    if (na.second.player != probPlayer) {
      decisionNodes.insert(na.first);
    }
  }

  for (Node node : decisionNodes) {
    auto ait = attributes.find(node);

   
    if (ait->second.player == 1) { // Universal Player
    // No reward is assumed on the universal branches! 
    // 
      unordered_map<Node, double> constraint;  // \sum xi.wi 
      for (Node n: decisionNodes) constraint[n] = 0;

      constraint[node] = 1;  
      Edges const &outEdges = ait->second.outEdges;
  
      for (Index i : outEdges) { //For all outgoing edges
        Transition const &t = transitions.at(i);
        Node destination = t.destination; 

        if (decisionNodes.find(destination) != decisionNodes.end()) { // no probabilistic branch
          constraint[destination] = constraint[destination] - 1;
        }
        else throw logic_error("Universal Player with probabilsitic transition in BDPs is not implemented. Please report bug.");
      }

      // Add constraint 

    bool vacuous = true; // check if constraint is of the form x - x  = 0
      for (Node n: decisionNodes) {
        if (!vacuous) break;
        if (constraint[n] != 0) vacuous = false;
      }

    if (!vacuous)  {
      // MPConstraint *kk_le = solver.MakeRowConstraint(-infinity, 0); // <= 0 part
      // MPConstraint *kk_ge = solver.MakeRowConstraint(0, infinity); 
      MPConstraint *kk = solver.MakeRowConstraint(0, 0); //= 0 part
      if (verbosity > Verbosity::Terse) {
        cout << " subject to C" << c_num++ << " : " << endl;
      }
      for (Node n: decisionNodes) {
        auto dst_it = variables.find(n);
        double count = constraint[n];
        if (count != 0) {
          kk->SetCoefficient(dst_it->second, count);
          // kk_le->SetCoefficient(dst_it->second, count);
          // kk_ge->SetCoefficient(dst_it->second, count);
          if (verbosity > Verbosity::Terse) {
             if (count < 0) cout << count <<"*" << prettyPrintState(dst_it->first); 
             else cout << " + " << count <<"*" << prettyPrintState(dst_it->first); 
          }
        }
      }

      if (verbosity > Verbosity::Terse) cout << " = 0; " << endl; ///here
    }
    }
    else { 
      // node is the current node and ait is the attribute pointer 
      // Existential Player
      Edges const &outEdges = ait->second.outEdges;
      for (Index i : outEdges) {
        Transition const &t = transitions.at(i);
        Node destination = t.destination;

        if (decisionNodes.find(destination) != decisionNodes.end()) { // no probabilistic branch
           if (destination != node) {

            if (verbosity > Verbosity::Terse) {
              cout << " subject to C" << c_num++ << " : " << endl;
            }
            MPConstraint *kk = solver.MakeRowConstraint(-infinity, t.reward);  // One constraint per successor
            
            auto src_it = variables.find(node);
            auto dst_it = variables.find(destination);

            if ((src_it == variables.end()) || (dst_it == variables.end())) 
              throw logic_error("Decision Node should be in the list. Please report bug.");
            else {
              kk->SetCoefficient(src_it->second, 1);
              kk->SetCoefficient(dst_it->second, -1);
              if (verbosity > Verbosity::Terse) 
                cout << prettyPrintState(node) << " - " <<  prettyPrintState(destination) <<" <=" << t.reward  <<";" << endl; // remove
            }
           }
        }
        else {
          unordered_map<Node, double> constraint;  // \sum xi.wi 
          for (Node n: decisionNodes) constraint[n] = 0;
          constraint[node] = 1;

          auto pit = attributes.find(destination);
          Edges const &pOutEdges = pit->second.outEdges;
          for (Index j : pOutEdges) {
            Transition const &pt = transitions.at(j);
            constraint[pt.destination] += -1*pt.probability;
          }
           // Add constraint 

          bool vacuous = true; // check if constraint is of the form x - x  = 0
            for (Node n: decisionNodes) {
              if (!vacuous) break;
              if (constraint[n] != 0) vacuous = false;
            }

          if (!vacuous)  {

            if (verbosity > Verbosity::Terse) {
              cout << " subject to C" << c_num++ << " : " << endl;
            }
            MPConstraint *kk = solver.MakeRowConstraint(-infinity, t.reward);  // One constraint per successor
            for (Node n: decisionNodes) {
              auto dst_it = variables.find(n);
              double count = constraint[n];
              if (count != 0) {
                kk->SetCoefficient(dst_it->second, count);
                if (verbosity > Verbosity::Terse) {
                  if (count < 0) cout << count <<"*" << prettyPrintState(dst_it->first); 
                  else cout << " + " << count <<"*" << prettyPrintState(dst_it->first); 
                }
              }
            }
            if (verbosity > Verbosity::Terse) cout << " <=  "<< t.reward << ";" << endl; ///here
          }
        }
      }

      // Terminal node---can only be a existential node for now.
      std::stringstream buffer;
      buffer << ait->second.propositions;

      if (buffer.str().find("final") != string::npos) {
        MPConstraint *kk = solver.MakeRowConstraint(0, 0); 

        auto src_it = variables.find(node);
        if (src_it == variables.end()) throw logic_error("Decision Node should be in the list. Please report bug.");
        else {
          kk->SetCoefficient(src_it->second, 1);
          if (verbosity > Verbosity::Terse) cout <<"subject to c" << c_num++ << " :" << prettyPrintState(src_it->first) << " = 0;" << endl; //remove
        }
      }
    }
  } // Finishes adding constraints

   if (verbosity > Verbosity::Terse) {
     cout << "# Number of constraints = " << solver.NumConstraints() << endl;
     cout << "# Number of variables  = " << solver.NumVariables() << endl;

   }

  MPObjective* const objective = solver.MutableObjective();

if (verbosity > Verbosity::Terse) {
   cout <<"maximize z: " << endl << "    "; 
}
  for (auto const & n : keys) {  
    if   (attributes.at(n).player != probPlayer) {
      auto it = variables.find(n);
      if (it == variables.end()) {  
          cout << "Error1: node should be present" << endl; 
      } 
      else {
        objective->SetCoefficient(it->second, 1);
        if (verbosity > Verbosity::Terse) cout << "+" << prettyPrintState(it->first); 
      } 
    }
  } 
  if (verbosity > Verbosity::Terse) cout << ";" << endl << "end;"<<endl; 
  objective->SetMaximization();


 if (verbosity > Verbosity::Verbose) { 
    string printMe;
    solver.ExportModelAsMpsFormat(false, false, &printMe);
    cout << "Linear Program set to GLOP: \n" << printMe << endl;
  }


const MPSolver::ResultStatus result_status = solver.Solve();
// Check that the problem has an optimal solution.

unordered_map<Node, double> result;

if (result_status != MPSolver::OPTIMAL) {
  cout << "The problem does not have an optimal solution!" << endl;
  cout << result_status << endl;
  return result;
}
else {
  if (verbosity > Verbosity::Terse) {
    cout << "Solution:";
    cout << "Optimal objective value = " << objective->Value() <<endl;
  }
  for (auto const & n : keys) {    
    if ( attributes.at(n).player != probPlayer) {
      auto it = variables.find(n);
      if (it == variables.end()) {  
            cout << "Error: node should be present" << endl; // STOP
      } else {
        result.insert(make_pair(n, it->second->solution_value()));
         if (verbosity > Verbosity::Terse) {
           cout<< prettyPrintState(it->first) <<" = " << it->second->solution_value() <<endl;
         }
      } 
    }
  } 
  return result;
}
}

#else 

Model::StrategyResults Model::getBDPStrategy() const {
  throw logic_error("No GLOP, no fun!");
}

unordered_map<Node, double> Model::getBDPValueUsingORTools() const {
  throw logic_error("No GLOP, no fun!");
}

#endif
