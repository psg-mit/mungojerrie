/** @file testLTS.cc

  @brief Test program for LTS and Parity.

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
#include "Parity.hh"

using namespace std;

// Maximum index for predefined automaton.
int const MAX_N = 28;
int const MAX_G = 2;

struct Options {
  Options() : verbosity(0), echo(false), trim(true), only(-1), game(-1), filename("") {}
  int verbosity;
  bool echo;
  bool trim;
  int only;
  int game;
  string filename;
};

// Custom exception.
struct help_exception : std::exception {
  using std::exception::exception;
};

void testParity(Options const & options);
void processAutomaton(Parity & npw, Options const & options, string const & selectString);
void usage(char const * name);
void parseArgs(int argc, char const * const argv[], Options & options);
void buildPredefinedNPW(Parity & NPW, int select);
void buildNPW0(Parity & NPW);
void buildNPW1(Parity & NPW);
void buildNPW2(Parity & NPW);
void buildNPW3(Parity & NPW);
void buildNPW4(Parity & NPW);
void buildNPW5(Parity & NPW);
void buildNPW6(Parity & NPW);
void buildNPW7(Parity & NPW);
void buildNPW8(Parity & NPW);
void buildNPW9(Parity & NPW);
void buildNPW10(Parity & NPW);
void buildNPW11(Parity & NPW);
void buildNPW12(Parity & NPW);
void buildNPW13(Parity & NPW);
void buildNPW14(Parity & NPW);
void buildNPW15(Parity & NPW);
void buildNPW16(Parity & NPW);
void buildNPW17(Parity & NPW);
void buildNPW18(Parity & NPW);
void buildNPW19(Parity & NPW);
void buildNPW20(Parity & NPW);
void buildNPW21(Parity & NPW);
void buildNPW22(Parity & NPW);
void buildNPW23(Parity & NPW);
void buildNPW24(Parity & NPW);
void buildNPW25(Parity & NPW);
void buildNPW26(Parity & NPW);
void buildNPW27(Parity & NPW);
void buildNPW28(Parity & NPW);
void testOSPGame(Options const & options);
StateSet buildPredefinedGame(Parity & game, int select);
StateSet buildGame0(Parity & NPW); 
StateSet buildGame1(Parity & NPW); 
StateSet buildGame2(Parity & NPW); 

int main(int argc, char ** argv)
{
  Options options;
  try {
    parseArgs(argc, argv, options);
  } catch (invalid_argument const & e) {
    usage(argv[0]);
    cerr << "Abnormal end: " << e.what() << endl;
    return 1;
  } catch (help_exception const & e) {
    usage(argv[0]);
    return 0;
  }

  try {
    testParity(options);
  } catch (invalid_argument const & e) {
    cerr << "Invalid argument: " << e.what() << endl;
    return 1;
  } catch (logic_error const & e) {
    cerr << "Logic error: " << e.what() << endl;
    return 1;
  }

  try {
    testOSPGame(options);
  } catch (logic_error const & e) {
    cerr << "Logic error: " << e.what() << endl;
    return 1;
  }

  return 0;
}


void testParity(Options const & options)
{
  Util::stopwatch();
  if (options.filename != string{""}) {
    Cudd mgr;
    Parity npw{mgr,options.filename,Verbosity::Silent,true};
    npw.setVerbosity(options.verbosity);
    // Extract automaton name from path.
    size_t dotposn = options.filename.rfind(".hoa");
    if (dotposn == string::npos) {
      throw invalid_argument("filename has no .hoa extension");
    }
    size_t separposn = options.filename.rfind('/');
    string name{string{"-"} + options.filename.substr(separposn+1, dotposn - separposn - 1)};
    processAutomaton(npw, options, name);
  } else {
    for (int select = 0; select != MAX_N+1; ++select) {
      if (options.only >= 0 && select != options.only) continue;
      if (options.verbosity) {
        cout << "############ DPW" << select << " ############" << endl;
      }
      Cudd mgr;
      Parity npw{mgr};
      npw.setVerbosity(options.verbosity);
      buildPredefinedNPW(npw, select);
      processAutomaton(npw, options, to_string(select));
    }
  }
  if (options.verbosity) {
    cout << "Run time " << Util::getElapsedTime() << " s" << endl;
  }
}


void processAutomaton(Parity & npw, Options const & options, string const & selectString)
{
  Cudd const & mgr = npw.getManager();
  if (options.trim) {
    npw.trim();
  }
  if (options.echo) {
    npw.printDot("npw" + selectString);
  }
  bool deterministic = npw.isDeterministic();
  cout << "#-----------------------------------------------\n";
  cout << "# The NPW is "
       << (deterministic ? "already " : "non")
       << "deterministic\n";
  cout << "#-----------------------------------------------" << endl;
  if (options.verbosity) {
    cout << "SCCs: " << npw.getSCCs() << endl;
    vector<State> transient;
    npw.findTransient(transient);
    cout << "Transient states: " << transient << endl;
    StateSet ntraps = npw.getTrapStates();
    cout << "Trap states: " << ntraps << endl;
    if (npw.isTerminal()) {
      cout << "Terminal automaton" << endl;
    }
  }
  Parity complete{npw};
  complete.makeComplete();
  if (complete.leadSimulates()) {
    cout << "NPW" << selectString << " lead simulates itself" << endl;
  } else {
    cout << "NPW" << selectString << " doesn't lead simulate itself" << endl;
  }
  if (!npw.hasEpsilonTransitions()) {
    Parity dpw{npw};
    if (!deterministic) {
      Parity nsafe{mgr}, nlive{mgr};
      npw.safetyLiveness(nsafe, nlive);
      nsafe.printDot("nsafe" + selectString);
      nlive.printDot("nlive" + selectString);

      LTS<ParityTransition> prod{nsafe.product(nlive)};
      prod.printDot("prod" + selectString);
      cout << "Prod SCCs: " << prod.getSCCs() << endl;

      if (options.verbosity) {
        cout << "=== Determinizing ===" << endl;
      }
      dpw = npw.determinize();
    }
    dpw.setVerbosity(options.verbosity);
    if (options.verbosity) {
      cout << dpw << endl;
    }
    unsigned maxp = dpw.minimumIndex();
    if (options.verbosity) {
      cout << "After index minimization, maximum priority is " << maxp << endl;
      cout << dpw << endl;
    }
    dpw.stateMinimization();
    if (!dpw.isDeterministic()) {
      throw logic_error("DPW is not deterministic");
    }
    if (options.verbosity) {
      cout << "SCCs: " << dpw.getSCCs() << endl;
      vector<State> transient;
      dpw.findTransient(transient);
      cout << "Transient states: " << transient << endl;
      StateSet traps = dpw.getTrapStates();
      cout << "Trap states: " << traps << endl;
      if (dpw.isTerminal()) {
        cout << "Terminal automaton" << endl;
      }
    }
    dpw.printHOA("dpw" + selectString);
    dpw.printDot("dpw" + selectString);
    if (dpw.getMaxPriority() < 2 && complete.fairSimulates(dpw)) {
      cout << "NPW" << selectString << " is good for games" << endl;
    } else {
      cout  << "NPW" << selectString << " is not good for games" << endl;
    }
    if (complete.directSimulates(dpw)) {
      cout << "NPW" << selectString << " direct-simulates DPW"
           << selectString << endl;
    } else {
      cout  << "NPW" << selectString << " does not direct-simulate DPW"
            << selectString << endl;
    }
    if (!dpw.directSimulates(dpw)) {
      throw logic_error("DPW" + selectString + " doesn't direct-simulate itself!");
    }
    Parity cdpw = dpw.complement();
    cdpw.printDot("complement" + selectString);
    Parity safety{mgr}, liveness{mgr};
    dpw.safetyLiveness(safety, liveness);
    safety.printDot("safety" + selectString);
    liveness.printDot("liveness" + selectString);

    Parity nondet = dpw.toLDPW();
    nondet.printDot("LDPW" + selectString);
  }
}


// Option processing.
void usage(char const * name)
{
  cout << "Usage: " << name
       << " [-e] [-o] [-t] [-v] [-h]\n"
       << "  Build test NPWs and determinize them\n"
       << "  Options:\n"
       << "    -e         : echo input automaton\n"
       << "    -f filename: read HOA automaton from \"filename\"\n"
       << "    -g number  : process only game \"number\"\n"
       << "    -o number  : process only automaton \"number\"\n"
       << "    -t         : do not trim automaton before determinization\n"
       << "    -v         : produce verbose output\n"
       << "    -h         : print this help message\n" << endl;
}

void parseArgs(int argc, char const * const argv[],
               Options & options)
{
  // Parse command line, skipping argv[0].
  for (int i = 1; i != argc; ++i) {
    if (strcmp("-h", argv[i]) == 0) {
      throw help_exception();
    } else if (strcmp("-e", argv[i]) == 0) {
      options.echo = true;
    } else if (strcmp("-f", argv[i]) == 0) {
      ++i;
      if (argc == i) { // missing <filename>
        throw invalid_argument("Missing filename.");
      }
      options.filename = argv[i];
    } else if (strcmp("-g", argv[i]) == 0) {
      ++i;
      if (argc == i) { // missing <number>
        throw invalid_argument("Missing automaton number.");
      }
      // Convert the next argument into options.only.
      int numRead = sscanf(argv[i], "%d", &options.game);
      // numRead != 1 if the argument isn't an int.
      if (numRead != 1 || options.game < 0 || options.game > MAX_G) {
        throw invalid_argument("Invalid or out of range game number.");
      }
    } else if (strcmp("-o", argv[i]) == 0) {
      ++i;
      if (argc == i) { // missing <number>
        throw invalid_argument("Missing automaton number.");
      }
      // Convert the next argument into options.only.
      int numRead = sscanf(argv[i], "%d", &options.only);
      // numRead != 1 if the argument isn't an int.
      if (numRead != 1 || options.only < 0 || options.only > MAX_N) {
        throw invalid_argument("Invalid or out of range automaton number.");
      }
    } else if (strcmp("-t", argv[i]) == 0) {
      options.trim = false;
    } else if (strcmp("-v", argv[i]) == 0) {
      options.verbosity++;
    } else { // unrecognized argument
      throw invalid_argument("Unrecognized argument.");
    }
  }
}

void buildPredefinedNPW(Parity & NPW, int select)
{
  switch (select) {
  case 0: buildNPW0(NPW);
    break;
  case 1: buildNPW1(NPW);
    break;
  case 2: buildNPW2(NPW);
    break;
  case 3: buildNPW3(NPW);
    break;
  case 4: buildNPW4(NPW);
    break;
  case 5: buildNPW5(NPW);
    break;
  case 6: buildNPW6(NPW);
    break;
  case 7: buildNPW7(NPW);
    break;
  case 8: buildNPW8(NPW);
    break;
  case 9: buildNPW9(NPW);
    break;
  case 10: buildNPW10(NPW);
    break;
  case 11: buildNPW11(NPW);
    break;
  case 12: buildNPW12(NPW);
    break;
  case 13: buildNPW13(NPW);
    break;
  case 14: buildNPW14(NPW);
    break;
  case 15: buildNPW15(NPW);
    break;
  case 16: buildNPW16(NPW);
    break;
  case 17: buildNPW17(NPW);
    break;
  case 18: buildNPW18(NPW);
    break;
  case 19: buildNPW19(NPW);
    break;
  case 20: buildNPW20(NPW);
    break;
  case 21: buildNPW21(NPW);
    break;
  case 22: buildNPW22(NPW);
    break;
  case 23: buildNPW23(NPW);
    break;
  case 24: buildNPW24(NPW);
    break;
  case 25: buildNPW25(NPW);
    break;
  case 26: buildNPW26(NPW);
    break;
  case 27: buildNPW27(NPW);
    break;
  case 28: buildNPW28(NPW);
    break;
  default:
    throw out_of_range("Argument too large.");
  }
}

/* Predefined automata. */

// This is the classic automaton for FG p.
void buildNPW0(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,p,1);
  NPW.addTransition(1,1,p,1);
}

// This is for GF p -> GF q.
void buildNPW1(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,0,~q,0);
  NPW.addTransition(0,1,q,1);
  NPW.addTransition(0,2,~p,1);
  NPW.addTransition(1,0,~q,0);
  NPW.addTransition(1,1,q,1);
  NPW.addTransition(1,2,~p,1);
  NPW.addTransition(2,2,~p,1);
}

// This is from Kozen's Theory of Computation, p. 165.
void buildNPW2(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(0,2,mgr.bddOne(),0);
  NPW.addTransition(1,1,mgr.bddOne(),1);
  NPW.addTransition(2,2,mgr.bddOne(),0);
}

// This is automaton A in Safra's thesis.
void buildNPW3(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,mgr.bddOne(),1);
}

// This is automaton B in Safra's thesis.
void buildNPW4(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(1,0,mgr.bddOne(),1);
}

// This is automaton A1 in Safra's thesis.
void buildNPW5(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(1,1,mgr.bddOne(),1);
}

// This is automaton A2 in Safra's thesis.
void buildNPW6(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(1,1,mgr.bddOne(),1);
  NPW.addTransition(1,0,~p,1);
}

// This is from Perrin and Pin, Figure 9.5 of Chapter I
void buildNPW7(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,~p,1);
  NPW.addTransition(1,1,p,1);
}

// This is from Perrin and Pin, Figure 9.15 of Chapter I
// We encode a as ~p, b as p&~q and c as p&q.
void buildNPW8(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,0,~p|~q,1);
  NPW.addTransition(0,1,p,0);
  NPW.addTransition(1,1,p,0);
  NPW.addTransition(1,0,~p,1);
}

// This is from Perrin and Pin, Figure 9.21 of Chapter I
// We encode a as ~p, b as p&~q and c as p&q.
void buildNPW9(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,0,~p|~q,1);
  NPW.addTransition(0,1,~p,1);
  NPW.addTransition(1,1,p,0);
  NPW.addTransition(1,0,p,0);
}

// This is an unambiguous automaton for FG p.
void buildNPW10(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,0,~p,0);
  NPW.addTransition(0,1,p,0);
  NPW.addTransition(0,2,p,0);
  NPW.addTransition(1,0,~p,0);
  NPW.addTransition(1,1,p,0);
  NPW.addTransition(2,2,p,1);
}

void buildNPW11(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,0,~p|~q,1);
  NPW.addTransition(0,1,~p,0);
  NPW.addTransition(1,1,p,0);
  NPW.addTransition(1,0,~p,1);
}

// Michel's automaton for n=1
void buildNPW12(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,1,p,0);
  NPW.addTransition(0,2,~p,0);
  NPW.addTransition(1,1,mgr.bddOne(),0);
  NPW.addTransition(2,2,mgr.bddOne(),0);
  NPW.addTransition(1,0,p,1);
}

// Michel's automaton for n=2
void buildNPW13(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.addState(3);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,1,p&~q,0);
  NPW.addTransition(0,2,p&q,0);
  NPW.addTransition(0,3,~p,0);
  NPW.addTransition(1,1,mgr.bddOne(),0);
  NPW.addTransition(2,2,mgr.bddOne(),0);
  NPW.addTransition(3,3,mgr.bddOne(),0);
  NPW.addTransition(1,0,p&~q,1);
  NPW.addTransition(2,0,p&q,1);
}

// Michel's automaton for n=3
void buildNPW14(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.addState(3);
  NPW.addState(4);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,4,~p&~q,0);
  NPW.addTransition(0,1,~p&q,0);
  NPW.addTransition(0,2,p&~q,0);
  NPW.addTransition(0,3,p&q,0);
  NPW.addTransition(1,1,mgr.bddOne(),0);
  NPW.addTransition(2,2,mgr.bddOne(),0);
  NPW.addTransition(3,3,mgr.bddOne(),0);
  NPW.addTransition(4,4,mgr.bddOne(),0);
  NPW.addTransition(1,0,~p&q,1);
  NPW.addTransition(2,0,p&~q,1);
  NPW.addTransition(3,0,p&q,1);
}

// This is an NBW for p U q.
void buildNPW15(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,0,p,0);
  NPW.addTransition(0,1,q,0);
  NPW.addTransition(0,2,mgr.bddOne(),0);
  NPW.addTransition(1,1,mgr.bddOne(),1);
  NPW.addTransition(2,2,mgr.bddOne(),0);
}

// This is an NBW for F(p & X p).
void buildNPW16(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,p,0);
  NPW.addTransition(1,2,p,1);
  NPW.addTransition(2,2,mgr.bddOne(),1);
}

// Liveness component of NPW9.
void buildNPW17(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.addState(3);
  NPW.addState(4);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,1,~p|~q,1);
  NPW.addTransition(0,2,~p,1);
  NPW.addTransition(0,3,p&~q,0);
  NPW.addTransition(0,4,p&q,1);
  NPW.addTransition(1,1,~p|~q,1);
  NPW.addTransition(1,2,~p,1);
  NPW.addTransition(2,2,p,0);
  NPW.addTransition(2,1,p,0);
  NPW.addTransition(3,3,p&~q,0);
  NPW.addTransition(3,4,p&q,1);
  NPW.addTransition(4,4,mgr.bddOne(),1);
}

// This is an NBW for FG(~p -> X p).
void buildNPW18(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,p,1);
  NPW.addTransition(1,1,p,1);
  NPW.addTransition(1,2,~p,1);
  NPW.addTransition(2,1,p,1);
}

// A bad-for-MDPs automaton for GF p
void buildNPW19(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,mgr.bddOne(),1);
  NPW.addTransition(1,0,p,0);
}

// A bad-for-MDPs automaton for true
void buildNPW20(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,0,mgr.bddOne(),0);
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(1,0,p,1);
  NPW.addTransition(0,2,mgr.bddOne(),0);
  NPW.addTransition(2,0,~p,1);
}

// A forgiving automaton for FG p | GF q
void buildNPW21(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,0,~q,0);
  NPW.addTransition(0,0,q,1);
  NPW.addTransition(0,1,p&~q,1);
  NPW.addTransition(1,0,q,1);
  NPW.addTransition(1,1,p&~q,1);
  NPW.addTransition(1,2,~p&~q,0);
  NPW.addTransition(0,2,q,1);
  NPW.addTransition(2,2,~q,0);
}

// A good-for-MDPs automaton for p & FG p
void buildNPW22(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,1,p,0);
  NPW.addTransition(1,1,mgr.bddOne(),0);
  NPW.addTransition(1,2,p,1);
  NPW.addTransition(2,2,p,1);
}

// A bad-for-MDPs automaton
void buildNPW23(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,1,p,0);
  NPW.addTransition(1,0,~p,1);
  NPW.addTransition(1,1,mgr.bddOne(),0);
  NPW.addTransition(1,2,p,1);
  NPW.addTransition(2,2,p,1);
}

// A good-for-MDPs automaton for (p & FG p) | (~p & FG ~p)
void buildNPW24(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.addState(3);
  NPW.addState(4);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,1,~p,0);
  NPW.addTransition(0,2,p,0);
  NPW.addTransition(1,1,mgr.bddOne(),0);
  NPW.addTransition(1,3,~p,1);
  NPW.addTransition(3,3,~p,1);
  NPW.addTransition(2,2,mgr.bddOne(),0);
  NPW.addTransition(2,4,p,1);
  NPW.addTransition(4,4,p,1);
}

// A bad-for-MDPs automaton for (FG p) | (FG ~p)
void buildNPW25(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.addState(3);
  NPW.addState(4);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(0,2,mgr.bddOne(),0);
  NPW.addTransition(1,1,mgr.bddOne(),0);
  NPW.addTransition(1,3,~p,1);
  NPW.addTransition(3,3,~p,1);
  NPW.addTransition(2,2,mgr.bddOne(),0);
  NPW.addTransition(2,4,p,1);
  NPW.addTransition(4,4,p,1);
}

// A bad-for-MDPs automaton for (GF p & q) | (GF p & ~q)
void buildNPW26(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(0,2,mgr.bddOne(),0);
  NPW.addTransition(1,1,~p|q,0);
  NPW.addTransition(1,1,p&~q,1);
  NPW.addTransition(2,2,p&~q,0);
  NPW.addTransition(2,2,p&q,1);
}

// A good-for-MDPs automaton for (GF p & q) | (GF p & ~q)
void buildNPW27(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  BDD q = mgr.bddVar();
  NPW.addAtomicProposition(q, "q");
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(0,2,mgr.bddOne(),0);
  NPW.addTransition(1,1,~p|q,0);
  NPW.addTransition(1,1,p&~q,1);
  NPW.addTransition(1,0,p&q,1);
  NPW.addTransition(2,2,p&~q,0);
  NPW.addTransition(2,2,p&q,1);
}

void buildNPW28(Parity & NPW) {
  NPW.addState(0);
  NPW.addState(1);
  NPW.addState(2);
  NPW.addState(3);
  NPW.makeInitial(0);
  Cudd mgr = NPW.getManager();
  BDD p = mgr.bddVar();
  NPW.addAtomicProposition(p, "p");
  NPW.addTransition(0,1,mgr.bddOne(),0);
  NPW.addTransition(0,2,mgr.bddOne(),0);
  NPW.addTransition(1,1,~p,0);
  NPW.addTransition(1,1,p,1);
  NPW.addTransition(2,2,p,0);
  NPW.addTransition(2,3,~p,0);
  NPW.addTransition(3,2,p,0);
  NPW.addTransition(3,2,~p,1);
}

// Test one-Streett-pair game solver.
void testOSPGame(Options const & options)
{
  for (int select = 0; select != MAX_G+1; ++select) {
    if (options.game >=0 && select != options.game) continue;
    if (options.verbosity) {
      cout << "############ Game" << select << " ############" << endl;
    }
    Cudd mgr;
    Parity game{mgr};
    game.setVerbosity(options.verbosity);
    StateSet spoiler = buildPredefinedGame(game,select);
    StateSet dwinning;
    StateMap dstrategy, sstrategy;
    game.solveOneStreettPairGame(spoiler, dwinning, dstrategy, sstrategy);
    if (options.verbosity) {
      Parity::DotAttributes attributes;
      for (State s : spoiler) {
        attributes.nodeShapes.emplace(s,"box");
        if (dwinning.find(s) == dwinning.end()) {
          attributes.nodeColors.emplace(s,"red");
        }
      }
      for (auto const & edge : sstrategy) {
        attributes.edgeColors.emplace(edge,"red");
      }
      for (auto const & edge : dstrategy) {
        attributes.edgeColors.emplace(edge,"blue");
      }
      game.printDot("game" + to_string(select), attributes);
      cout << "Spoiler's states: " << spoiler << endl;
      cout << "Duplicator's strategy: " << dstrategy << endl;
      cout << "Spoiler's strategy: " << sstrategy << endl;
    }
    bool dwins = dwinning.find(game.getInitial()) != dwinning.end();
    cout << (dwins ? "Duplicator" : "Spoiler") << " wins game " << select << endl;
  }
  if (options.verbosity) {
    cout << "Run time " << Util::getElapsedTime() << " s" << endl;
  }

}

StateSet buildPredefinedGame(Parity & game, int select)
{
  switch (select) {
  case 0: return buildGame0(game);
    break;
  case 1: return buildGame1(game);
    break;
  case 2: return buildGame2(game);
    break;
  default:
    throw out_of_range("Argument too large.");
  }
}

// An example game.
StateSet buildGame0(Parity & game)
{
  game.addState(0);
  game.addState(1);
  game.addState(2);
  game.addState(3);
  game.makeInitial(0);
  BDD one = game.getManager().bddOne();
  game.addTransition(0,1,one,2);
  game.addTransition(0,3,one,2);
  game.addTransition(1,0,one,1);
  game.addTransition(1,2,one,3);
  game.addTransition(2,1,one,2);
  game.addTransition(2,3,one,2);
  game.addTransition(3,0,one,1);
  game.addTransition(3,2,one,3);
  return StateSet{1,3};
}

// Another example game.
StateSet buildGame1(Parity & game)
{
  game.addState(0);
  game.addState(1);
  game.addState(2);
  game.addState(3);
  game.addState(4);
  game.addState(5);
  game.makeInitial(0);
  BDD one = game.getManager().bddOne();
  game.addTransition(0,1,one,2);
  game.addTransition(1,0,one,3);
  game.addTransition(1,2,one,2);
  game.addTransition(2,1,one,2);
  game.addTransition(2,3,one,2);
  game.addTransition(3,4,one,1);
  game.addTransition(3,2,one,3);
  game.addTransition(4,3,one,2);
  game.addTransition(4,5,one,1);
  game.addTransition(5,4,one,1);
  return StateSet{1,3,5};
}

// An example game coming from fair simulation minimization.
StateSet buildGame2(Parity & game)
{
  game.addState(0);
  game.addState(1);
  game.addState(2);
  game.addState(3);
  game.addState(4);
  game.addState(5);
  game.addState(6);
  game.addState(7);
  game.addState(8);
  game.addState(9);
  game.makeInitial(0);
  BDD one = game.getManager().bddOne();
  game.addTransition(0,1,one,1);
  game.addTransition(0,2,one,1);
  game.addTransition(0,3,one,1);
  game.addTransition(1,4,one,3);
  game.addTransition(2,5,one,3);
  game.addTransition(3,0,one,1);
  game.addTransition(4,7,one,1);
  game.addTransition(4,8,one,1);
  game.addTransition(4,9,one,1);
  game.addTransition(5,7,one,1);
  game.addTransition(5,9,one,1);
  game.addTransition(6,2,one,1);
  game.addTransition(6,3,one,1);
  game.addTransition(7,0,one,1);
  game.addTransition(8,4,one,3);
  game.addTransition(9,5,one,3);
  return StateSet{0,4,5,6};
}
