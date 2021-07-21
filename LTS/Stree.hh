#ifndef STREE_H_
#define STREE_H_

/** @file Stree.hh

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

#include <vector>
#include <set>
#include <string>
#include <sstream>
#include "State.hh"

/**
 * @brief Class of compact Safra trees used for Piterman's NBW to DPW procedure.
 *
 * @details These are multiway branching trees where each child points to its
 * parent and each parent points to the youngest of its children.  Each node
 * also points to younger and elder sibling.
 */
class Stree {
public:
  /// Constructor.
  Stree(void);
  /// Returns the number of nodes in the tree.
  int size(void) const { return numNodes; }
  /// Returns the set of states that label a node of the tree.
  std::set<State> getLabel(int node) const {return label[node]; }
  /// Sets the label of a node in the tree.
  void setLabel(int node, std::set<State> newlabel) { label[node] = newlabel; }
  /// Adds a node to the tree as last child of "parent."
  void addLastChild(int parent, std::set<State> const & childLabel);
  /// Removes the subtree rooted at "node."
  void removeSubtree(int node);
  /// Gives contiguous numbers to the nodes of the tree.
  void makeContiguous(void);
  /// Restricts the label of a node to help restore the tree invariant.
  void restrictLabels(void);
  /// Checks whether node of tree "flashes green."
  bool isGreen(int node) const;
  /// Removes the descendants of a node from the tree.
  void removeDescendants(int node);
  /// Returns the nodes of the tree in preorder.
  void preorder(std::vector<int> &order) const;
  /// Compares two trees.
  bool isIsomorphic(Stree const &other, int root = 0, int otherRoot = 0) const;
  /// Returns a string describing the tree.
  std::string toString(void) const;
  /// Converts the tree to dot format.
  std::string toDot(std::string graphname = std::string("tree"),
                    std::string subgraph = std::string("")) const;
  /// Index of an undefined node.
  static int constexpr undef = -1;
private:
  bool isDefined(int node) const { return node != undef; }
  void restrictLabelsRecur(int node, std::set<State> & subset);
  void preorderRecur(std::vector<int> &order, int node = 0) const;
  void toStringRecur(int node, std::ostringstream & oss) const;
  size_t numNodes;
  std::vector< std::set<State> > label;
  std::vector<int> parent;
  std::vector<int> elder;
  std::vector<int> younger;
  std::vector<int> lastChild;
};

/// Stream insertion operator for Safra trees.
std::ostream & operator<<(std::ostream & os, Stree const & t);

#endif
