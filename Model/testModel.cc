/** @file testModel.cc

  @brief Test program for model construction.

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

using namespace std;

struct Options {
  Options() : verbosity(Verbosity::Silent), echo(false) {}
  Verbosity::Level verbosity;
  bool echo;
};

void buildModel(Options const & options);
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
    buildModel(options);
  } catch(invalid_argument const & e) {
    cerr << "Invalid argument: " << e.what() << endl;
  }

  return 0;
}


void buildModel(Options const & options)
{
  auto startTime = chrono::system_clock::now();
  Cudd mgr;
  Model mdp(mgr);
  BDD p = mgr.bddVar();
  mdp.addAtomicProposition(p, "p");
  BDD q = mgr.bddVar();
  mdp.addAtomicProposition(q, "q");

  mdp.setVerbosity(options.verbosity);
  mdp.addDecisionNode(0, std::string("n0"), 0);
  mdp.makeInitial(0);
  mdp.addLabel(0, p);
  mdp.addLabel(0, q);
  mdp.addDecisionNode(1, std::string("n1"), 1);
  mdp.addDecisionNode(2, std::string("n2"), 1);
  mdp.addLabel(2, q);
  mdp.addProbabilisticNode(3, std::string("p3"));
  mdp.addProbabilisticNode(4, std::string("p4"));
  mdp.addProbabilisticNode(5, std::string("p5"));
  mdp.addDecisionTransition(0, 3, 0, 0.5);
  mdp.addProbabilisticTransition(3, 1, 0.4, 0.75);
  mdp.addProbabilisticTransition(3, 2, 0.6, 1.25);
  mdp.addDecisionTransition(1, 4, 0);
  mdp.addDecisionTransition(2, 5, 0);
  mdp.addProbabilisticTransition(4, 1, 0.5, 0.5);
  mdp.addProbabilisticTransition(4, 2, 0.5, 0.5);
  mdp.addProbabilisticTransition(5, 1, 0.5, 1.5);
  mdp.addProbabilisticTransition(5, 2, 0.5, 1.5);
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
         << " " << mdp.getPropositionName(q) << "\n";
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
}


// Option processing.
void usage(char const * name) {
  cout << "Usage: " << name
       << " [-e] [-v] [-h] <file>\n"
       << "  Build a test MDP\n"
       << "  Options:\n"
       << "    -e         : echo input model\n"
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
