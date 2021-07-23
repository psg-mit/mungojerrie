/** @file Options.cc

  @brief Definition and processing of command-line options.

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
#include "config.h"
#include "CommandLineOptions.hh"

using namespace std;
using namespace boost::program_options;

void checkAutomatonType(std::string const & type);
void checkReachSolver(std::string const & solver);
void checkSSPSolver(std::string const & solver);
void checkLearner(std::string const & learner);
void checkRewardType(std::string const & type);

/**
 * Constructor adds the desired command line options
 */
CommandLineOptions::CommandLineOptions() : visible("mungojerrie Usage: \
  ./mungojerrie <Input File> [OPTIONS] \nOptions") {

#if defined(HAVE_SPOT) || defined(HAVE_OWL)
#define HAVE_LTL 1
#else
#undef HAVE_LTL
#endif

#ifdef HAVE_LTL
#if defined(HAVE_SLIM)
  string const dflt_auto_type = "slim";
#elif defined(HAVE_OWL)
  string const dflt_auto_type = "ldba";
#elif defined(HAVE_SPOT)
  string const dflt_auto_type = "dpa";
#endif
#endif

  //Add the input file option as a positional option (i.e., last argument
  //passed which doesn't need a --option_name)
  posOpt.add("input-file", -1);

  //Add the options
  visible.add_options()
    ("help,h", "Print this help message")

    ("version,V", "Print version information")

    ("verbosity,v",
     value<int>(&verbosityLevel)->value_name("LEVEL")->default_value(1),
     "Set verbosity level (0-4)")

    ("define,D",
     value<vector<string>>()->value_name("CONST=VAL")->default_value({}, "")
     ->composing(),
     "Give (new) values to model constants")

    ("parity,p",
     value<string>()->value_name("FILE"),
     "Parity automaton file")

    ("to-ldba", "Convert DPA to LDBA")

    ("to-dpa", "Determinize NBA")

#ifdef HAVE_LTL
    ("ltl,L",
     value<string>()->value_name("formula"),
     "LTL formula in the syntax of the chosen translator")

    ("ltl-file,f",
     value<string>()->value_name("filename"),
     "LTL formula file")

    ("automaton-type,a",
     value<string>()->value_name("formula")->default_value(dflt_auto_type)
     ->notifier(&checkAutomatonType),
     "Target automaton type for LTL formula translation [ldba, dpa, slim, semidet]")
#endif
    ;

  options_description printing("Printing");
  printing.add_options()  
    ("dot-model",
     value<std::string>()->value_name("filename"),
     "Output model in dot format to [filename]. "
     "Use - for output to cout.")
    
    ("dot-automaton",
     value<std::string>()->value_name("filename"),
     "Output automaton in dot format to [filename]. "
     "Use - for output to cout.")
     
    ("dot-product",
     value<std::string>()->value_name("filename"),
     "Output synchronous product between model and automaton "
     "in dot format to [filename]. "
     "Use - for output to cout.")

    ("dot-mc",
     value<std::string>()->value_name("filename"),
     "Output Markov chain for strategy in dot format to [filename]. "
     "Use - for output to cout.")

    ("dot-learn",
     value<std::string>()->value_name("filename"),
     "Output learned pruned product graph in dot format to [filename]. "
     "Use - for output to cout.")

    ("prism-model",
     value<std::string>()->value_name("filename"),
     "Output model in PRISM format to [filename]. "
     "Use - for output to cout.")
       
    ("prism-product",
     value<std::string>()->value_name("filename"),
     "Output synchronous product between model and automaton "
     "in PRISM format to [filename]. "
     "Use - for output to cout.")

    ("prism-mc",
     value<std::string>()->value_name("filename"),
     "Output Markov chain for the strategy in PRISM format to [filename]. "
     "Use - for output to cout.")

     ("prism-learn",
     value<std::string>()->value_name("filename"),
     "Output learned pruned product graph in PRISM format to [filename]. "
     "Use - for output to cout.")

    ("hoa-automaton",
     value<std::string>()->value_name("filename"),
     "Output automaton in HOA format to [filename]. "
     "Use - for output to cout.")

    ;

  options_description modelchecking("Model checking");
  modelchecking.add_options()
    ("model-check,m", "Enable model checking")

    ("reach-with",
     value<std::string>()->value_name("method")->default_value("iter")
     ->notifier(&checkReachSolver),
      "Select method for probabilistic reachability [iter, glop]")

    ("ssp-with",
     value<std::string>()->value_name("method")->default_value("iter")
     ->notifier(&checkSSPSolver),
      "Select method for stochastic shortest path [iter, glop, poly]")

    ("epsilon",
     value<double>()->default_value(1e-9, "1e-9"),
     "Tolerance for probability computations")

    ("tran-epsilon",
     value<double>()->default_value(1e-12, "1e-12"),
     "Tolerance for sum of transistion probabilities to be considered 1")

    ("save-mc-strategy",
     value<std::string>()->value_name("filename"),
     "Saves final pure strategy of model checker as CSV to [filename] "
     "for all visited states.  Use - for output to cout.")

    ("save-all-mc-strategy",
     "If specified, the model checker strategy is output for all states, not "
     "just reachable states.")
    ;

  options_description learning("Learning");
  learning.add_options()
    ("learn,l", 
     value<std::string>()->default_value("none")->notifier(&checkLearner),
     "Select learner [none, Q, DQ, SL]. Algorithms are Q-learning, "
     "Double Q-learning, and Sarsa(lambda).")

    ("est-pac-probability-num-samples",
     value<unsigned int>()->default_value(0),
     "Number of samples for estimating PAC bound, where <=0 means don't estimate.")

    ("reward-type",
     value<std::string>()->default_value("default-type")->notifier(&checkRewardType),
     "Select reward function [default-type, prism, zeta-reach, zeta-acc, zeta-discount, "
     "reward-on-acc, multi-discount, parity, pri-tracker]. "
     "See documentation for GymOptions::GymRewardTypes in Gym.hh for details.")

    ("ep-number",
     value<unsigned int>()->default_value(20000),
     "Number of learning episodes")

    ("ep-length",
     value<unsigned int>()->default_value(30),
     "Length of each learning episode")

    ("zeta",
     value<double>()->default_value(0.99, "0.99"),
     "Probability that a learning agent will stay out of the sink at each "
     "accepting transition")

    ("explore",
     value<double>()->default_value(0.1, "0.1"),
     "Probability that a random action is chosen during learning")

    ("alpha",
     value<double>()->default_value(0.1, "0.1"),
     "Learning rate")

    ("discount",
     value<double>()->default_value(0.99999, "0.99999"),
     "Discount for learning algorithms")

    ("gammaB",
     value<double>()->default_value(0.99, "0.99"),
     "Second discount for multi-discount learning")

    ("linear-lr-decay",
     value<double>()->default_value(-1, "-1 (off)"),
     "Decays the learning rate linearly to specified value over the course of learning. "
     "Negative values turns decay off.")

    ("linear-explore-decay",
     value<double>()->default_value(-1, "-1 (off)"),
     "Decays learning epsilon linearly to specified value over the course of learning. "
     "Negative values turns decay off.")

    ("init-value",
     value<double>()->default_value(0.0),
     "Value initial Q-values are set to for learning")

    ("lambda",
     value<double>()->default_value(0.9, "0.9"),
     "Value of lambda for Sarsa(lambda)")

    ("replacing-trace", "Use replacing traces with Sarsa(lambda)")

    ("tolerance",
     value<vector<double>>()->default_value(vector<double>{0.01}, "[0.01]"),
     "Actions with a Q-value within tolerance times the optimal Q-value "
     "are all merged into a mixed strategy during learning verification.")

    ("pri-epsilon",
     value<double>()->default_value(0.001, "0.001"),
     "Probability used to advance priority tracker during learning verfication.")

    ("learn-stats",
     "Prints statistics for the learning agent")

    ("progress-bar",
     "Turns on progress bar for learning.")

    ("save-q",
     value<std::string>()->value_name("filename")->default_value(""),
     "Saves Q-table(s) to [filename]. Format is Boost text archive.")

    ("load-q",
     value<std::string>()->value_name("filename")->default_value(""),
     "Loads Q-table(s) from [filename]. May cause issues if loading "
     "Q-table(s) for a different model. Format is Boost text archive.")

    ("checkpoint-freq",
     value<double>()->default_value(0.0),
     "Frequency to save Q-table(s) during training as a fraction of training "
     "length (values in [0,1]).\n0 turns checkpointing off")

    ("save-learner-strategy",
     value<std::string>()->value_name("filename")->default_value(""),
     "Saves final mixed strategy of learner as CSV file to [filename] "
     "for all visited states.  Use - for output to cout.")

    ("no-reset-on-acc",
     "Turns off resetting episode step when an accepting edge is passed "
     "for zeta-reach and zeta-acc")

    ("terminal-update-on-time-limit",
     "Treat end of episodes that end due to episode limit as "
     "transitioning to zero value sink")

    ("player-1-not-strategic",
     "Does not allow player 1 to change their strategy to the optimal counter-strategy "
     "to player 0 during the verification of the learned strategies. Instead, player 1 "
     "uses learned strategy.")

    ("seed",
     value<unsigned>(),
     "Pseudorandom number generator seed")

    ;
  
  visible.add(printing).add(modelchecking).add(learning);
  hidden.add_options()
    ("input-file", value<string>(&inputFileName), "Input file");
  cmdline_options.add(visible).add(hidden);
}


int CommandLineOptions::parseCommandLineOptions(int argc, char * argv[]) {

  // Set-up.
  try {
    store(command_line_parser(argc, argv).options(
            cmdline_options).positional(posOpt).run(), varMap);
  }
  catch(exception const & e) {
    cerr << e.what() << endl;
    return 1;
  }

  notify(varMap);

  // Process options that cause immediate return.
  if(varMap.count("help")) {
    cout << visible << endl;
    return 1;
  }

  if (varMap.count("version")) {
    cout << PACKAGE_STRING << endl;
    return 1;
  }

  // Check remaining options.
  int ret = 0;

  if (varMap.count("verbosity")) {
    if (verbosityLevel < 0 || verbosityLevel > 4) {
      cout << "Error: Verbosity level (" << verbosityLevel 
           << ") out of range (0-4)" << endl;
      ret |= 1;
    }
  }

  if(varMap.count("input-file")) {
    if (inputFileName == "") {
      cout << "Error: Empty file name" << endl;
      ret |= 1;
    } else {
      if (verbosityLevel > 0)
        cout << "Input File is " << inputFileName << endl;
    }
  } else {
    cout << "Error: Input file missing" << endl << endl; 
    ret |= 1;
  }

  if (varMap.count("parity") + varMap.count("ltl") + varMap.count("ltl-file") > 1) {
    cout << "Error: Only one of --parity, --ltl, and --ltl-file is allowed" << endl;
    ret |= 1;
  }

#if !defined(HAVE_BOOST_PROCESS) || !defined(HAVE_SLIM)
  if (varMap.count("ltl") || varMap.count("ltl-file")) {
    if (varMap["automaton-type"].as<string>() == "slim") {
      cout << "Error: translation of LTL to slim automaton not enabled in this build"
           << endl;
      ret |= 1;
    } else if (varMap["automaton-type"].as<string>() == "semidet") {
      cout << "Error: translation of LTL to semi-deterministic automaton not enabled in this build"
           << endl;
      ret |= 1;
    }
  }
#endif

#if !defined(HAVE_BOOST_PROCESS) || !defined(HAVE_OWL)
  if ((varMap.count("ltl") || varMap.count("ltl-file")) &&
      varMap["automaton-type"].as<string>() == "ldba") {
    cout << "Error: translation of LTL to ldba not enabled in this build" << endl;
    ret |= 1;
  }
#endif

#if !defined(HAVE_BOOST_PROCESS) || !defined(HAVE_SPOT)
  if ((varMap.count("ltl") || varMap.count("ltl-file")) &&
      varMap["automaton-type"].as<string>() == "dpa") {
    cout << "Error: translation of LTL to dpa not enabled in this build" << endl;
    ret |= 1;
  }
#endif

  if (ret != 0) {
    cout << visible << endl;
    return ret;
  }

  return 0;
}

void CommandLineOptions::fillModelOptions(ModelOptions & m) const
{
  m.verbosity = verbosity();
  m.reachSolver = reachSolver();
  m.sspSolver = sspSolver();
  m.epsilon = options()["epsilon"].as<double>();
  m.tranEpsilon = options()["tran-epsilon"].as<double>();

  vector<string> defines = options()["define"].as<vector<string>>();

  for (auto & def : defines) {
    // Remove whitespace and break into name and value at the equal sign.
    def.erase(remove_if(def.begin(), def.end(), ::isspace), def.end());
    size_t pos = def.find('=');
    if (pos == string::npos) {
      throw validation_error(validation_error::invalid_option_value,
                             "define", def);
    }
    string const & constName = def.substr(0,pos);
    string const & constVal = def.substr(pos+1);
    m.defines.push_back({constName, constVal});
  }
}

void checkAutomatonType(std::string const & type)
{
  if (type != "dpa" && type != "ldba" && type != "slim" && type != "semidet") {
    throw validation_error(validation_error::invalid_option_value,
                           "automaton-type", type);
  }
}

void checkReachSolver(std::string const & solver)
{
#ifdef HAVE_LIBORTOOLS
  bool noGlop = solver != "glop";
#else
  bool noGlop = true;
#endif
    
  if (solver != "iter" && noGlop) {
    throw validation_error(validation_error::invalid_option_value,
                           "reach-with", solver);
  }
}

void checkSSPSolver(std::string const & solver)
{
#ifdef HAVE_LIBORTOOLS
  bool hasGlop = true;
#else
  bool hasGlop = false;
#endif
    
  if ((solver != "iter" && !hasGlop) || 
      (solver != "iter" && solver != "glop" && solver != "poly")) {
    throw validation_error(validation_error::invalid_option_value,
                           "ssp-with", solver);
  }
}

void checkLearner(std::string const & learner)
{
  if (learner != "none" && learner != "Q" && learner != "DQ" && learner != "SL")
    throw validation_error(validation_error::invalid_option_value,
                           "learn", learner);
}

void checkRewardType(std::string const & type)
{
  if (type != "default-type" && type != "prism" && type != "zeta-reach" && type != "zeta-acc" && 
      type != "zeta-discount" && type != "reward-on-acc" && type != "multi-discount" && 
      type != "parity" && type != "pri-tracker")
    throw validation_error(validation_error::invalid_option_value, "reward-type", type);
}
