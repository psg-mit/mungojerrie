/** @file Stree.cc

  @brief Safra trees.

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

#include <stdexcept>
#include <sstream>
#include <iostream>
#include <algorithm>
#include "Util.hh"
#include "Stree.hh"

using namespace std;

constexpr decltype(Stree::undef) Stree::undef;

Stree::Stree(void) : numNodes(0) {}


void Stree::addLastChild(int par, set<State> const &childLabel)
{
  int child = parent.size();
  if (isDefined(par) && par >= child) {
    throw out_of_range("Trying to add child to nonexisting node");
  }

  label.push_back(set<State>(childLabel.begin(), childLabel.end()));
  parent.push_back(par);
  elder.push_back(isDefined(par) ? lastChild.at(par) : +undef);
  younger.push_back(+undef);
  lastChild.push_back(+undef);

  if (isDefined(par)) {
    if (isDefined(lastChild.at(par))) {
      younger[lastChild.at(par)] = child;
    }
    lastChild[par] = child;
  }
  numNodes++;

} // Stree::addLastChild


void Stree::removeSubtree(int node)
{
  if (!isDefined(node) || (size_t) node >= lastChild.size()) {
    throw out_of_range("Trying to remove nonexisting node");
  }
  if (isDefined(parent.at(node))) {
    if (lastChild.at(parent.at(node)) == node) 
      lastChild[parent.at(node)] = elder.at(node);
  } else if (node != 0) {
    // Already removed.
    return;
  }
  while (isDefined(lastChild.at(node))) {
    removeSubtree(lastChild.at(node));
  }
  if (isDefined(elder.at(node))) {
    younger[elder.at(node)] = younger.at(node);
  }
  if (isDefined(younger.at(node))) {
    elder[younger.at(node)] = elder.at(node);
  }
  label[node] = set<State>();
  parent[node] = undef;
  younger[node] = undef;
  elder[node] = undef;
  lastChild[node] = undef;
  numNodes--;

} // Stree::removeSubtree


void Stree::makeContiguous(void)
{
  size_t cnt = 0;
  for (size_t i = 0; cnt < numNodes; ++i) {
    if (!label.at(i).empty()) {
      if (i > cnt) {
        label[cnt] = label.at(i);
        label[i] = set<State>();
        if (isDefined(younger.at(i))) {
          elder[younger.at(i)] = cnt;
          younger[cnt] = younger.at(i);
          younger[i] = undef;
        }
        if (isDefined(elder.at(i))) {
          younger[elder.at(i)] = cnt;
          elder[cnt] = elder.at(i);
          elder[i] = undef;
        }
        if (isDefined(parent.at(i))) {
          parent[cnt] = parent.at(i);
          if (lastChild[parent.at(i)] == (int) i) {
            lastChild[parent.at(i)] = cnt;
            parent[i] = undef;
          }
        }
        if (isDefined(lastChild.at(i))) {
          lastChild[cnt] = lastChild.at(i);
          int j = lastChild.at(i);
          while (isDefined(j)) {
            parent[j] = cnt;
            j = elder[j];
          }
          lastChild[i] = undef;
        }
      }
      ++cnt;
    }
  }
  parent.resize(numNodes);
  elder.resize(numNodes);
  younger.resize(numNodes);
  lastChild.resize(numNodes);
  label.resize(numNodes);

} // Stree::makeContiguous


void Stree::restrictLabels(void)
{
  if (numNodes == 0) return;
  set<State> lbl = label.at(0);
  int j = lastChild.at(0);
  if (isDefined(j))
    restrictLabelsRecur(j, lbl);

} // Stree::restrictLabels


/**
 * The label of node must be pruned of what is not in subset or is
 * in the labels of elder siblings.  We wait until the elders are done.
 * They subtract from subset the states they have.  Then we intersect
 * the label of this node with what is left of subset.  Finally, we
 * recur on the children passing a copy of the updated label.
 */
void Stree::restrictLabelsRecur(int node, set<State> & subset)
{
  int j = elder.at(node);
  if (isDefined(j))
    restrictLabelsRecur(j, subset);
  set<State> & lbl = label.at(node);
  lbl &= subset;
  j = lastChild.at(node);
  if (isDefined(j)) {
    set<State> childSubset(lbl);
    restrictLabelsRecur(j, childSubset);
  }
  if (isDefined(younger.at(node))) {
    subset -= lbl;
  }

} // Stree::restrictLabelsRecur


bool Stree::isGreen(int node) const
{
  set<State> lbl = label.at(node);
  if (lbl.empty()) return false;
  int j = lastChild.at(node);
  // Early termination only if the oldest children have empty labels.
  // Let's not bother yet.
  while (isDefined(j)) {
    lbl -= label.at(j);
    j = elder.at(j);
  }
  return lbl.empty();

} // Stree::isGreen


void Stree::removeDescendants(int node)
{
  int j = lastChild.at(node);
  while (isDefined(j)) {
    removeSubtree(j);
    j = lastChild.at(node);
  }
} // Stree::removeDescendants


void Stree::preorder(vector<int> &order) const
{
  order.clear();
  if (numNodes != 0) {
    preorderRecur(order, 0);
  }
} // Stree::preorder


void Stree::preorderRecur(vector<int> &order, int node) const
{
  int j = elder.at(node);
  if (isDefined(j)) {
    preorderRecur(order, j);
  }
  order.push_back(node);
  j = lastChild.at(node);
  if (isDefined(j)) {
    preorderRecur(order, j);
  }

} // Stree::preorderRecur


// Compares compact trees.
bool Stree::isIsomorphic(Stree const &other, int root, int otherRoot) const
{
  if (numNodes != parent.size())
    throw logic_error("Non-compact tree.");
  if (other.numNodes != other.parent.size())
    throw logic_error("Non-compact other tree.");
  if (numNodes != other.numNodes)
    return false;
  if (numNodes == 0)
    return true;
  if (label.at(root) != other.label.at(otherRoot))
    return false;

  int j = lastChild.at(root);
  int k = other.lastChild.at(otherRoot);

  while (isDefined(j) && isDefined(k)) {
    if (!isIsomorphic(other, j, k))
      return false;
    j = elder.at(j);
    k = elder.at(k);
  }
  return !isDefined(j) && !isDefined(k);

} // Stree::isIsomorphic


string Stree::toString(void) const
{
  ostringstream oss;
  if (numNodes != 0)
    toStringRecur(0, oss);
  return oss.str();

} // Stree::toString


ostream & operator<<(ostream & os, Stree const & t)
{
  os << t.toString();
  return os;
}


void Stree::toStringRecur(int node, ostringstream & oss) const
{
  int j = elder.at(node);
  if (isDefined(j)) {
    toStringRecur(j, oss);
  }
  set<State> const & lbl(label.at(node));
  oss << "(" << node << ": " << lbl << " ";
  j = lastChild.at(node);
  if (isDefined(j)) {
    toStringRecur(j, oss);
  }
  oss << ")";

} // Stree::toStringRecur


/**
 * If subgraph is nonempty, create a cluster instead of a top-level graph.
 * Use subgraph as the cluster name and as prefix to the node names for
 * disambiguation.
 */ 
string Stree::toDot(string graphname, string subgraph) const
{
  Util::replaceAll(graphname, "\n", "\\n");
  string clustername = subgraph;
  Util::replace(clustername, "t", "cluster");
  ostringstream oss;
  if (!subgraph.empty()) {
    oss << "subgraph " << clustername << " {\nnode [style=filled,color=yellow];\n"
        << "style=filled;\n" << "color=lightgrey;\n" << "label=\""
        << graphname << "\";\n";
    subgraph.append("_");
  } else {
    oss << "digraph " << graphname << " {\nnode [shape=ellipse];\n"
        << "size = \"7.5,10\";\ncenter = true;\n"
        << "\"title\" [label=\"" << graphname << "\",shape=plaintext];\n";
  }
  int i = 0;
  size_t cnt = 0;
  while (cnt < numNodes) {
    set<State> const & lbl(label.at(i));
    if (!lbl.empty()) {
      oss << subgraph << i << " [label=\"" << i << "\\n" << lbl  << "\"];\n";
      int j = lastChild.at(i);
      while (isDefined(j)) {
        oss << subgraph << i << " -> " << subgraph << j << ";\n";
        j = elder.at(j);
      }
      ++cnt;
    }
    ++i;
  }
  oss << "}";
  return oss.str();
      
} // Stree::toDot
