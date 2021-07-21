/** @file testProduct.cc

  @brief Test program for model-objective product construction.

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
#include <stdexcept>
#include <cstring>
#include <chrono>
#include "Util.hh"
#include "Model.hh"
#include "Parity.hh"

using namespace std;

struct Options {
  Options() : verbosity(Verbosity::Silent), echo(false) {}
  Verbosity::Level verbosity;
  bool echo;
};

Parity buildDPW(Options const & options, Cudd const & mgr, BDD const & p);
Model buildModel(Options const & options, Cudd const & mgr, BDD const & p);
Model buildFreeModel(Options const & options, Cudd const & mgr, BDD const & p);
Model buildProduct(Options const & options, Model const & model, Parity const & automaton);
void usage(char const * name);
void parseArgs(int argc, char const * const argv[], Options & options);

int main(int argc, char ** argv)
{
  Options options;
  try {
    parseArgs(argc, argv, options);
  } catch (exception const & e) {
    usage(argv[0]);
    cerr << "Abnormal end: " << e.what() << endl;
    return 1;
  }

  try {
    try {
      Cudd mgr;
      BDD p = mgr.bddVar();
      Model mdp = buildModel(options, mgr, p);
      Model fmdp = buildFreeModel(options, mgr, p);
      Parity dpw = buildDPW(options, mgr, p);
      Model product = buildProduct(options, mdp, dpw);
      Model fproduct = buildProduct(options, fmdp, dpw);
      if(options.verbosity){
        set<Node> U;
        set<Node> restriction;
        set<pair<Node, Action>> edgeRestrict;
        Player player = -1;
        U.insert(9);
        for (int i = 0; i <= 12; i++) {
          if (i != 4)
            restriction.insert(i);
        }
        edgeRestrict.insert(make_pair(7, 1));
        unordered_set<Node> randAttractor = product.getAttractor(U, player, restriction, edgeRestrict);
        cout << "Attractor to set :\n";
        for(Node n : U){
          cout << "  " << product.getNodeName(n) << endl;
        }
        cout << "with player " << player << " is set:\n";
        for(Node n : randAttractor){
          cout << "  " << product.getNodeName(n) << " #: " << n <<endl;
        }
        cout << "\n=================================\n\n";
        cout << "MECs of product are:\n";
        set<Node> restrictionM;
        for(int i = 10; i <= 12; i++)
          restrictionM.insert(i);
        set<pair<Node, Action>> edgeRestrictM;
        edgeRestrictM.insert({10, 1});
        vector< set<Node> > MECs = product.getMECs(restrictionM, edgeRestrictM);
        for(set<Node> v : MECs){
          cout << "MEC:\n";
          for(Node n : v){
            cout << "  " << product.getNodeName(n) << " #: " << n <<endl;
          }
        }
        cout << "\n=================================\n\n";
        cout << "WECs of product are:\n";
        set<Node> restrictionW;
        restrictionW.insert(10);
        set<pair<Node, Action>> edgeRestrictW;
        edgeRestrictW.insert({10, 0});
        vector< set<Node> > WECs = product.getWECs(restrictionW, edgeRestrictW);
        for(set<Node> v : WECs){
          cout << "WEC:\n";
          for(Node n : v){
            cout << "  " << product.getNodeName(n) << " #: " << n << endl;
          }
        }
      }
    } catch(logic_error const & e) {
      cerr << "Logic error: " << e.what() << endl;
    }
  } catch(invalid_argument const & e) {
    cerr << "Invalid argument: " << e.what() << endl;
  }

  return 0;
}

Model buildProduct(Options const & options, Model const & model, Parity const & automaton)
{
  auto startTime = chrono::system_clock::now();
  Model product = Model(model, automaton);
  product.printDot("product");
  if (options.verbosity) {
    auto lapTime = chrono::system_clock::now();
    auto elapsed =
      chrono::duration_cast<chrono::milliseconds>(lapTime - startTime);
    cout << "Run time " << elapsed.count() << "ms" << endl;
  }
  return product;
}

Parity buildDPW(Options const & options, Cudd const & mgr, BDD const & p)
{
  auto startTime = chrono::system_clock::now();
  Parity npw(mgr);
  npw.setVerbosity(options.verbosity);
  npw.addState(0);
  npw.addState(1);
  npw.makeInitial(0);
  npw.addAtomicProposition(p, "p");
  npw.addTransition(0,0,mgr.bddOne(),0);
  npw.addTransition(0,1,p,1);
  npw.addTransition(1,1,p,1);
  if (options.echo) {
    npw.printDot("npw");
    cout << "#-----------------------------------------------\n";
    cout << "# The NPW is "
         << (npw.isDeterministic() ? "already " : "non")
         << "deterministic\n";
    cout << "#-----------------------------------------------" << endl;
  }
  if (options.verbosity) {
    cout << "SCCSs: " << npw.getSCCs() << endl;
    vector<State> transient;
    npw.findTransient(transient);
    cout << "Transient states: " << transient << endl;
  }
  npw.printDot("npw");
  Parity dpw(npw);
  if (!npw.isDeterministic()) {
    dpw = npw.determinize();
  }
  unsigned maxp = dpw.minimumIndex();
  if (options.verbosity) {
    cout << "Maximum parity is " << maxp << endl;
  }
  dpw.stateMinimization();
  if (!dpw.isDeterministic()) {
    throw logic_error("DPW is not deterministic");
  }
  if (options.verbosity) {
    cout << "SCCSs: " << dpw.getSCCs() << endl;
    vector<State> transient;
    dpw.findTransient(transient);
    cout << "Transient states: " << transient << endl;
  }
  dpw.printDot("dpw");
  auto lapTime = chrono::system_clock::now();
  auto elapsed =
    chrono::duration_cast<chrono::milliseconds>(lapTime - startTime);
  if (options.verbosity) {
    cout << "Run time " << elapsed.count() << "ms" << endl;
  }
  return dpw;
}

Model buildModel(Options const & options, Cudd const & mgr, BDD const & p)
{
  auto startTime = chrono::system_clock::now();
  Model mdp(mgr);
  mdp.addAtomicProposition(p, "p");

  mdp.setVerbosity(options.verbosity);
  mdp.addDecisionNode(0, std::string("n0"), 0);
  mdp.addDecisionNode(1, std::string("n1"), 0);
  mdp.addDecisionNode(2, std::string("n2"), 0);
  mdp.addDecisionNode(3, std::string("n3"), 0);
  mdp.makeInitial(0);
  mdp.addLabel(0, p);
  mdp.addLabel(3, p);
  mdp.addProbabilisticNode(4, std::string("p4"));
  mdp.addProbabilisticNode(5, std::string("p5"));
  mdp.addProbabilisticNode(6, std::string("p6"));
  mdp.addProbabilisticNode(7, std::string("p7"));
  mdp.addProbabilisticNode(8, std::string("p8"));
  mdp.addDecisionTransition(0, 4, 0);
  mdp.addDecisionTransition(1, 5, 0);
  mdp.addDecisionTransition(1, 6, 1);
  mdp.addDecisionTransition(3, 7, 0);
  mdp.addDecisionTransition(3, 3, 1);
  mdp.addDecisionTransition(2, 8, 0);
  mdp.addProbabilisticTransition(4, 0, 0.5, 0.5);
  mdp.addProbabilisticTransition(4, 1, 0.5, 0.5);
  mdp.addProbabilisticTransition(5, 1, 0.5, 0.5);
  mdp.addProbabilisticTransition(5, 3, 0.5, 0.5);
  mdp.addProbabilisticTransition(6, 0, 0.5, 0.5);
  mdp.addProbabilisticTransition(6, 2, 0.5, 0.5);
  mdp.addProbabilisticTransition(7, 2, 0.5, 0.5);
  mdp.addProbabilisticTransition(7, 3, 0.5, 0.5);
  mdp.addProbabilisticTransition(8, 2, 0.5, 0.5);
  mdp.addProbabilisticTransition(8, 3, 0.5, 0.5);
  mdp.sortEdgesByPriority();
  mdp.sanityCheck();
  mdp.printDot("mdp");
  if (options.echo) {
    cout << "#-----------------------------------------------\n";
    cout << mdp;
  }
  if (options.verbosity) {
    cout << "#-----------------------------------------------\n";
    cout << "Atomic propositions: " << mdp.getPropositionName(p)
         << " " << mdp.getPropositionName(p) << "\n";
    cout << "SCCSs: " << mdp.getSCCs() << endl;
    Node initial = mdp.getInitial();
    cout << "Initial node: " << mdp.getNodeName(initial)
         << ", with label " << mdp.getNodeLetter(initial) << "\n";
  }
  if (options.verbosity) {
    auto lapTime = chrono::system_clock::now();
    auto elapsed =
      chrono::duration_cast<chrono::milliseconds>(lapTime - startTime);
    cout << "Run time " << elapsed.count() << "ms" << endl;
  }
  return mdp;
}

Model buildFreeModel(Options const & options, Cudd const & mgr, BDD const & p)
{
  auto startTime = chrono::system_clock::now();
  Model mdp(mgr);
  mdp.addAtomicProposition(p, "p");

  mdp.setVerbosity(options.verbosity);
  mdp.addDecisionNode(0, std::string("n0"), 0);
  mdp.addDecisionNode(1, std::string("n1"), 1);
  mdp.addDecisionNode(2, std::string("n2"), 2);
  mdp.addDecisionNode(3, std::string("n3"), 0);
  mdp.makeInitial(0);
  mdp.addLabel(0, p);
  mdp.addLabel(3, p);
  mdp.addProbabilisticNode(4, std::string("p4"));
  mdp.addProbabilisticNode(5, std::string("p5"));
  mdp.addProbabilisticNode(6, std::string("p6"));
  mdp.addProbabilisticNode(7, std::string("p7"));
  mdp.addProbabilisticNode(8, std::string("p8"));
  mdp.addDecisionTransition(0, 1, 0);
  mdp.addDecisionTransition(0, 6, 1);
  mdp.addDecisionTransition(1, 4, 0);
  mdp.addDecisionTransition(1, 6, 1);
  mdp.addDecisionTransition(2, 2, 0);
  mdp.addDecisionTransition(2, 7, 0);
  mdp.addProbabilisticTransition(4, 0, 0.5, 0.5);
  mdp.addProbabilisticTransition(4, 2, 0.5, 0.5);
  mdp.addProbabilisticTransition(5, 1, 0.5, 0.5);
  mdp.addProbabilisticTransition(5, 3, 0.5, 0.5);
  mdp.addProbabilisticTransition(6, 1, 0.5, 0.5);
  mdp.addProbabilisticTransition(6, 2, 0.5, 0.5);
  mdp.addProbabilisticTransition(7, 0, 0.5, 0.5);
  mdp.addProbabilisticTransition(7, 1, 0.5, 0.5);
  mdp.addProbabilisticTransition(8, 2, 0.75, 0.5);
  mdp.addProbabilisticTransition(8, 3, 0.25, 0.5);
  mdp.printDot("fmdp");
  if (options.echo) {
    cout << "#-----------------------------------------------\n";
    cout << mdp;
  }
  if (options.verbosity) {
    cout << "#-----------------------------------------------\n";
    cout << "Atomic propositions: " << mdp.getPropositionName(p)
         << " " << mdp.getPropositionName(p) << "\n";
    cout << "SCCSs: " << mdp.getSCCs() << endl;
    Node initial = mdp.getInitial();
    cout << "Initial node: " << mdp.getNodeName(initial)
         << ", with label " << mdp.getNodeLetter(initial) << "\n";
  }
  if (options.verbosity) {
    auto lapTime = chrono::system_clock::now();
    auto elapsed =
      chrono::duration_cast<chrono::milliseconds>(lapTime - startTime);
    cout << "Run time " << elapsed.count() << "ms" << endl;
  }
  return mdp;
}

// Option processing.
void usage(char const * name) {
  cout << "Usage: " << name
       << " [-e] [-v] [-h] <file>\n"
       << "  Build a test MDP\n"
       << "  Options:\n"
       << "    -e         : echo input\n"
       << "    -v         : produce verbose output\n"
       << "    -h         : print this help message\n" << endl;
}

void parseArgs(int argc, char const * const argv[],
               Options & options)
{
  // Parse command line, skipping argv[0].
  for (int i = 1; i != argc; ++i) {
    if (strcmp("-h", argv[i]) == 0) {
      usage(argv[0]);
    } else if (strcmp("-e", argv[i]) == 0) {
      options.echo = true;
    } else if (strcmp("-v", argv[i]) == 0) {
      options.verbosity = Verbosity::Terse;
    } else { // unrecognized argument
      throw invalid_argument("Unrecognized argument.");
    }
  }
}
