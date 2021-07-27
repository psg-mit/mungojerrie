/** @file main.cc

  @brief Main function and its helpers.

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

#include <fstream>
#include <stdexcept>
#include <omp.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#pragma GCC diagnostic pop
#include "CommandLineOptions.hh"
#include "Model.hh"
#include "Parity.hh"
#include "Gym.hh"
#include "Learner.hh"

using namespace std;
using namespace boost::program_options;
namespace bf = boost::filesystem;
namespace bp = boost::process;

/**
 * @brief Gets command-line options and optionally echoes command line.
 */
int getOptions(int argc, char * argv[], CommandLineOptions & options);
/**
 * @brief Gets automaton from either an HOA file or an LTL formula.
 *
 * @details The formula may be given as string on the command line or in a file.
 * The LTL options use external translators and are enabled if
 * the program was properly configured.
 */
void getAutomaton(Cudd & mgr, CommandLineOptions const & options, Parity & objective);
/**
 * @brief Synthesize an optimal strategy.
 */
void doModelChecking(CommandLineOptions const & options, Model const & prod);
/**
 * @brief Use reinforcement learning to synthesize a strategy.
 */
void doLearning(CommandLineOptions const & options, Model const & model, Parity const & objective);
/**
 * @brief Estimates the PAC bound of the specified learning algorithm.
 */
void estimatePACProbability(CommandLineOptions options, Model const & model, Parity const & objective);
/**
 * @brief Print statistics for the objective automaton.
 */
void printObjectiveStats(Parity const & automaton);

/**
 * @brief Solve Branching Decision Process using Linear Programming 
 */
// void solveBranchingDecisionProcess(CommandLineOptions const & options, Model const & bdp);


int main(int argc, char * argv[])
{
  Util::stopwatch();

  CommandLineOptions options;
  int status = getOptions(argc, argv, options);

  if (status != 0) return status;

  try {
    // Build model from PRISM file.
    Cudd mgr;
    ModelOptions modelOptions;
    options.fillModelOptions(modelOptions);
    Model model(mgr, options.inputFile(), options.verbosity,
                modelOptions);
    if (options.verbosity > Verbosity::Silent) {
      Util::printElapsedTag();

      cout << " the environment has " << model.numNodes() << " nodes, "
           << model.numDecisionNodes() << " of which are decision nodes." << endl;

      if (options.verbosity > Verbosity::Terse) {
        for (auto & nodeC : model.getNodeCounts()) {
          Util::printElapsedTag();
          cout << " player " << nodeC.first << " controls " << nodeC.second << " nodes." << endl;
        }
      }

      if (options.verbosity > Verbosity::Informative) {
        cout << model;
      }
    }
    if (options.options().count("dot-model")) {
       if (model.isGame()) model.printDot("smg", options.options()["dot-model"].as<string>());
       else model.printDot("mdp", options.options()["dot-model"].as<string>());
    }

    if (options.options().count("prism-model")) {
      model.printPrism("m", options.options()["prism-model"].as<string>());
    }

    // Build an omega automaton for the objective property.
    if (options.options().count("parity") ||
        options.options().count("ltl") ||
        options.options().count("ltl-file")) {
      Parity objective{mgr};
      getAutomaton(mgr, options, objective);
      
      // Model check.
      if (options.options().count("model-check") ||
          options.options().count("dot-product") ||
          options.options().count("prism-product") ||
          options.options().count("dot-mc") ||
          options.options().count("prism-mc")) {
        try {
          // Compose model and automaton.
          Model prod(model, objective, modelOptions);
          if (options.options().count("dot-product")) {
            prod.printDot("product", options.options()["dot-product"].as<string>());
          }
          if (options.options().count("prism-product")) {
            prod.printPrism("m", options.options()["prism-product"].as<string>());
          }
          if (options.verbosity > Verbosity::Silent) {
            Util::printElapsedTag();
            cout << " the product has " << prod.numNodes() << " nodes, "
                 << prod.numDecisionNodes() << " of which are decision nodes."
                 << endl;
            if (options.verbosity > Verbosity::Informative && !model.isGame()) {
              auto pMECs = prod.getMECs();
              auto pWECs = prod.getWECs();
              cout << "Product MECs: " << pMECs << endl;
              cout << "Product WECs: " << pWECs << endl;
            }
          }
          if (options.options().count("model-check") ||
              options.options().count("dot-mc") ||
              options.options().count("prism-mc")) {
              doModelChecking(options, prod);
          }
        } catch (exception const & e) {
          cerr << e.what() << endl;
          status = 1;
        }
      }
      if (options.learnEnabled()) {
        if (options.options()["est-pac-probability-num-samples"].as<unsigned int>() > 0) {
          estimatePACProbability(options, model, objective);
        } else {
          doLearning(options, model, objective);
        }
      } else {
        if (options.options().count("dot-learn")) {
          cerr << "Warning: --dot-learn specified without --learn. Nothing to generate." << endl;
        }
        if (options.options().count("prism-learn")) {
          cerr << "Warning: --prism-learn specified without --learn. Nothing to generate." << endl;
        }
        if (options.options()["est-pac-probability-num-samples"].as<unsigned int>() > 0) {
          cerr << "Warning: --est-pac-probability-num-samples specified without --learn. Nothing to generate." << endl;
        }
      }
    } else {
      vector<string> optionNames = {"model-check", "dot-product", "prism-product",
                                    "dot-mc", "prism-mc", "dot-learn", "prism-learn",
                                    "dot-automaton", "hoa-automaton"};
      for (string name : optionNames) {
        if (options.options().count(name)) {
          cerr << "Warning: --" << name << " was specified, but no there is no property. "
          "Nothing to generate." << endl;
        }
      }
      if (options.learnEnabled()) {
        cerr << "Warning: --learn was specified, but no there is no property. "
        "Nothing to generate." << endl;
      }
    }
     

  } catch (exception const & e) {
    cerr << e.what() << endl;
    status = 1;
  }

  if (options.verbosity > Verbosity::Silent) {
    Util::printElapsedTag();
    cout << " end" << endl;
  }
  return status;
}

int getOptions(int argc, char * argv[], CommandLineOptions & options)
{
  int status;
  try {
    status = options.parseCommandLineOptions(argc, argv);
  } catch (error const & ie) {
    cerr << ie.what() << endl;
    return 1;
  }

  if (options.verbosity > Verbosity::Terse) {
    // Echo command line.
    cout << "#";
    for (int i = 0; i != argc; ++i)
      cout << " " << argv[i];
    cout << endl;
  }
  return status;
}

void getAutomaton(Cudd & mgr, CommandLineOptions const & options, Parity & objective)
{
  if (options.options().count("parity")) {
    string parityFile = options.options()["parity"].as<string>();
    if (options.verbosity > Verbosity::Terse) {
      cout << "Parity automaton file is " << parityFile << endl;
    }
    objective = Parity(mgr, parityFile, options.verbosity);
  } else if (options.options().count("ltl") ||
             options.options().count("ltl-file")) {
    string automatonType = options.options()["automaton-type"].as<string>();
    string ltlFormula;
    if (options.options().count("ltl")) {
      ltlFormula = options.options()["ltl"].as<string>();
    } else if (options.options().count("ltl-file")) {
      ltlFormula = options.options()["ltl-file"].as<string>();
    } else {
      throw logic_error("How did we get here?");
    }
    if (automatonType == "slim") {
#ifdef HAVE_SLIM
      string epmc(SLIM_PATH);
      if (!bf::exists(bf::path{SLIM_PATH})) {
        throw runtime_error(epmc + " not found.");
      }
      string spot(SPOT_PATH);
      if (!bf::exists(bf::path{SPOT_PATH})) {
        throw runtime_error(spot + " not found.");
      }
      string command;
      if (options.options().count("ltl")) {
        command = string("--mj-input");
      } else {
        command = string("--mj-input-file");
      }
      bp::ipstream pipe_stream;
      bp::child c(bp::search_path("java"), "-jar", epmc, "formula2automaton",
                  "--automaton-spot-ltl2tgba-cmd", spot,
                  command, ltlFormula,
                  "--mj-automaton-type", "slim",
                  "--disable-greeting-message", "true", "--mj-quiet", "true",
                  bp::std_out > pipe_stream);
      objective = Parity(mgr, pipe_stream, options.verbosity);
      c.wait();
      // Check the return value just to be safe.  In case of error, 
      // normally the HOA parser throws before this point is reached.
      int err = c.exit_code();
      if (err) {
        throw runtime_error(string("Error translating LTL formula ") + ltlFormula);
      }
#else
      throw logic_error("How did we get here?");
#endif
    } else if (automatonType == "semidet") {
#ifdef HAVE_SLIM
      string epmc(SLIM_PATH);
      if (!bf::exists(bf::path{SLIM_PATH})) {
        throw runtime_error(epmc + " not found.");
      }
      string spot(SPOT_PATH);
      if (!bf::exists(bf::path{SPOT_PATH})) {
        throw runtime_error(spot + " not found.");
      }
      string command;
      if (options.options().count("ltl")) {
        command = string("--mj-input");
      } else {
        command = string("--mj-input-file");
      }
      bp::ipstream pipe_stream;
      bp::child c(bp::search_path("java"), "-jar", epmc, "formula2automaton",
                  "--automaton-spot-ltl2tgba-cmd", spot,
                  command, ltlFormula,
                  "--mj-automaton-type", "semi-deterministic",
                  "--disable-greeting-message", "true", "--mj-quiet", "true",
                  bp::std_out > pipe_stream);
      objective = Parity(mgr, pipe_stream, options.verbosity);
      c.wait();
      // Check the return value just to be safe.  In case of error, 
      // normally the HOA parser throws before this point is reached.
      int err = c.exit_code();
      if (err) {
        throw runtime_error(string("Error translating LTL formula ") + ltlFormula);
      }
#else
      throw logic_error("How did we get here?");
#endif
    } else if (automatonType == "ldba") {
#ifdef HAVE_OWL
      if (!bf::exists(bf::path{OWL_PATH})) {
        throw runtime_error(string("OWL_PATH") + " not found.");
      }
      string command;
      if (options.options().count("ltl")) {
        command = string("-i");
      } else {
        command = string("-I");
      }
      bp::ipstream pipe_stream;
      bp::child c(OWL_PATH, command, ltlFormula, bp::std_out > pipe_stream);
      objective = Parity(mgr, pipe_stream, options.verbosity);
      c.wait();
      // Check the return value just to be safe.  In case of error, 
      // normally the HOA parser throws before this point is reached.
      int err = c.exit_code();
      if (err) {
        throw runtime_error(string("Error translating LTL formula ") + ltlFormula);
      }
#else
      throw logic_error("How did we get here?");
#endif
    } else if (automatonType == "dpa") {
#ifdef HAVE_SPOT
      if (!bf::exists(bf::path{SPOT_PATH})) {
        throw runtime_error(string(SPOT_PATH) + " not found.");
      }
      string command;
      if (options.options().count("ltl")) {
        command = string("--formula");
      } else {
        command = string("--file");
      }
      bp::ipstream pipe_stream;
      bp::child c(SPOT_PATH, command, ltlFormula,
                  "--colored-parity=max odd", "--deterministic", "--hoaf=t",
                  bp::std_out > pipe_stream);
      objective = Parity(mgr, pipe_stream, options.verbosity);
      c.wait();
      // Check the return value just to be safe.  In case of error, 
      // normally the HOA parser throws before this point is reached.
      int err = c.exit_code();
      if (err) {
        throw runtime_error(string("Error translating LTL formula ") + ltlFormula);
      }
#else
      throw logic_error("How did we get here?");
#endif
    }
  }
  if (options.options().count("to-dpa") && !objective.hasEpsilonTransitions() &&
      !objective.isDeterministic()) {
    objective = objective.determinize();
    if (options.verbosity > Verbosity::Terse) {
      Util::printElapsedTag();
      cout << " after determinization: " << objective.numStates()
           << " states" << endl;
    }
    unsigned maxPriority = objective.minimumIndex();
    if (options.verbosity > Verbosity::Terse) {
      Util::printElapsedTag();
      cout << " after index reduction: " << maxPriority + 1
           << " priorities" << endl;
    }
    objective.stateMinimization();
    if (options.verbosity > Verbosity::Terse) {
      Util::printElapsedTag();
      cout << " after state minimization: " << objective.numStates()
           << " states" << endl;
    }
  }
  if (options.options().count("to-ldba")) {
    objective = objective.toLDPW();
  }
  if (options.verbosity > Verbosity::Silent) {
    Util::printElapsedTag();
    printObjectiveStats(objective);
    if (options.verbosity > Verbosity::Informative) {
      cout << objective << endl;
    }
  }
  if (options.options().count("dot-automaton")) {
        objective.printDot("automaton", options.options()["dot-automaton"].as<string>()); 
  }
  if (options.options().count("hoa-automaton")) {
    objective.printHOA("automaton", options.options()["hoa-automaton"].as<string>());
  }
}

void doModelChecking(CommandLineOptions const & options, Model const & prod)
{
  if (options.options().count("dot-mc") || 
      options.options().count("prism-mc") ||
      options.options().count("save-mc-strategy") ||
      options.verbosity > Verbosity::Informative) {
      auto strategyResult = prod.getStrategy();
      
      if (strategyResult.strategy.size() == 0) return;
      // Problem does not have an optimal solution

    if (options.verbosity > Verbosity::Silent) {
      Util::printElapsedTag();
      cout << " strategy computed." << endl;
      Util::printElapsedTag();
      if (prod.isBDP())
      {
        cout << " Total cost from the initial state: " <<
          strategyResult.vals.find(prod.getInitial())->second << endl;
      } else {
        cout << " probability of satisfaction: " <<
          strategyResult.vals.find(prod.getInitial())->second << endl;
      }
    }

    Model prunedProd = prod.pruneToStrategy(strategyResult.strategy, true, true);
    
    if (options.options().count("save-mc-strategy")) {
      if (options.options().count("save-all-mc-strategy")) {
        prod.printStrategyCSV(strategyResult.strategy, options.options()["save-mc-strategy"].as<string>());
      } else {
        prunedProd.printActionsCSV(options.options()["save-mc-strategy"].as<string>());
      }
    }

    if (options.options().count("dot-mc")) {
      prunedProd.printDot("Pruned Product", options.options()["dot-mc"].as<string>());
    }
    if (options.options().count("prism-mc")) {
      prunedProd.printPrism("m", options.options()["prism-mc"].as<string>());
    }
    if (options.verbosity > Verbosity::Informative) {
      cout << "Vals: " << strategyResult.vals << endl;
      cout << "Strategy: " << strategyResult.strategy << endl;
      set<Node> pTarget;
      auto prunedWECs = prunedProd.getWECs();
      for (auto s : prunedWECs)
        for (Node n : s)
          pTarget.insert(n);
      auto pvAttractor = prunedProd.getAttractor(pTarget, 0);
      set<Node> pAttractor(pvAttractor.begin(), pvAttractor.end());
      if (pAttractor.find(prunedProd.getInitial()) != pAttractor.end())
        cout << "Initial node included in player attractor to WECs." << endl;
    }
  } else {
    if (options.verbosity > Verbosity::Silent) {
      
      auto prob = prod.getProbabilityOfSat();
      
      if (prob.size() == 0) return; // Problem does not have an optimal solution

      Util::printElapsedTag();
      if (prod.isBDP())
      {
        cout << " Total cost from the initial state: " <<
                  prob.find(prod.getInitial())->second << endl;
      } else {
        cout << " probability of satisfaction: " <<
                  prob.find(prod.getInitial())->second << endl;
      }
    }
  }
}

void doLearning(CommandLineOptions const & options, Model const & model, Parity const & objective)
{
  if (options.options().count("seed")) {
    Util::seed_urng(options.options()["seed"].as<unsigned int>());
  } else {
    Util::seed_urng();
  }
  GymOptions gymOptions;
  gymOptions.episodeLength = options.options()["ep-length"].as<unsigned int>();
  gymOptions.zeta = options.options()["zeta"].as<double>();
  gymOptions.tolerance = options.options()["tolerance"].as<vector<double>>();
  gymOptions.priEpsilon = options.options()["pri-epsilon"].as<double>();
  gymOptions.gammaB = options.options()["gammaB"].as<double>();
  gymOptions.rewardType = options.getRewardType();
  gymOptions.noResetOnAcc = options.options().count("no-reset-on-acc");
  gymOptions.terminalUpdateOnTimeLimit = options.options().count("terminal-update-on-time-limit");
  gymOptions.p1NotStrategic = options.options().count("player-1-not-strategic");
  gymOptions.concatActionsInCSV = options.options().count("concat-actions-in-csv");
  Gym gym(model, objective, gymOptions);

  LearnerOptions learnerOptions;

  learnerOptions.isBDP = model.isBDP();
  learnerOptions.dotLearn = (bool) options.options().count("dot-learn");
  if (learnerOptions.dotLearn) learnerOptions.dotLearnFilename = options.options()["dot-learn"].as<string>();
  else learnerOptions.dotLearnFilename ="-";

  learnerOptions.prismLearn = (bool) options.options().count("prism-learn");
  if (learnerOptions.prismLearn) learnerOptions.prismLearnFilename = options.options()["prism-learn"].as<string>();
  else learnerOptions.prismLearnFilename = "-";

  learnerOptions.verbosity = options.verbosity;
  learnerOptions.statsOn = (bool) options.options().count("learn-stats");
  learnerOptions.saveQFilename = options.options()["save-q"].as<string>();
  learnerOptions.loadQFilename = options.options()["load-q"].as<string>();
  learnerOptions.checkpointFreq = options.options()["checkpoint-freq"].as<double>();
  learnerOptions.saveStratFilename = options.options()["save-learner-strategy"].as<string>();
  learnerOptions.progressBar = options.options().count("progress-bar");

  Learner learner(gym, learnerOptions);

  unsigned int episodeNumber = options.options()["ep-number"].as<unsigned int>();
  double discount = options.options()["discount"].as<double>();
  double explore = options.options()["explore"].as<double>();
  double alpha = options.options()["alpha"].as<double>();
  double initValue = options.options()["init-value"].as<double>();
  double linearAlphaDecay = options.options()["linear-lr-decay"].as<double>();
  double linearExploreDecay = options.options()["linear-explore-decay"].as<double>();

  if (options.SLEnabled()) {
    if (options.verbosity > Verbosity::Silent) {
      cout << endl << "--------Sarsa(lambda)--------" << endl;
    }
    double lambda = options.options()["lambda"].as<double>();
    bool replacingTrace = (bool) options.options().count("replacing-trace");
    learner.SarsaLambda(lambda, replacingTrace, episodeNumber, alpha, linearAlphaDecay, 
                     discount, explore, linearExploreDecay, initValue); 
  }
  if (options.QEnabled()) {
    if (options.verbosity > Verbosity::Silent) {
      cout << endl << "--------QLearning--------" << endl;
    }
    learner.QLearning(episodeNumber, alpha, linearAlphaDecay, 
                      discount, explore, linearExploreDecay, initValue);

  }
  if (options.DQEnabled()) {
    if (options.verbosity > Verbosity::Silent) {
      cout << endl << "--------DoubleQLearning--------" << endl;
    }
    learner.DoubleQLearning(episodeNumber, alpha, linearAlphaDecay, 
                            discount, explore, linearExploreDecay, initValue);
  }
}

template <typename T>
static void check_set_option(CommandLineOptions const & options, std::string field_name, T expected, T& set)
{
  if (options.options().count(field_name)) {
    cerr << "Warning: ignoring command line option " << 
    field_name << " and setting it to " << expected << endl;
  }
  set = expected;
}

void estimatePACProbability(CommandLineOptions options, Model const & model, Parity const & objective)
{
  omp_lock_t initlock;
  omp_init_lock(&initlock);
  
  std::map<double, int> within_eps_counts;

  options.verbosity = Verbosity::Silent;

  ModelOptions modelOptions;
  options.fillModelOptions(modelOptions);

  GymOptions gymOptions;
  gymOptions.episodeLength = options.options()["ep-length"].as<unsigned int>();
  gymOptions.zeta = options.options()["zeta"].as<double>();
  gymOptions.tolerance = options.options()["tolerance"].as<vector<double>>();
  gymOptions.priEpsilon = options.options()["pri-epsilon"].as<double>();
  gymOptions.gammaB = options.options()["gammaB"].as<double>();
  gymOptions.rewardType = options.getRewardType();
  gymOptions.noResetOnAcc = options.options().count("no-reset-on-acc");
  gymOptions.terminalUpdateOnTimeLimit = options.options().count("terminal-update-on-time-limit");
  gymOptions.p1NotStrategic = options.options().count("player-1-not-strategic");
  gymOptions.concatActionsInCSV = options.options().count("concat-actions-in-csv");

  LearnerOptions learnerOptions;

  learnerOptions.isBDP = model.isBDP();
  learnerOptions.dotLearn = false;
  if (learnerOptions.dotLearn) learnerOptions.dotLearnFilename = options.options()["dot-learn"].as<string>();
  else learnerOptions.dotLearnFilename ="-";

  learnerOptions.prismLearn = (bool) options.options().count("prism-learn");
  if (learnerOptions.prismLearn) learnerOptions.prismLearnFilename = options.options()["prism-learn"].as<string>();
  else learnerOptions.prismLearnFilename = "-";

  check_set_option<Verbosity::Level>(options, "verbosityLevel", 
    (Verbosity::Level) 0, learnerOptions.verbosity);
  check_set_option<bool>(options, "learn-stats", false, learnerOptions.statsOn);
  check_set_option<string>(options, "save-q", "", learnerOptions.saveQFilename);
  check_set_option<string>(options, "load-q", "", learnerOptions.loadQFilename);
  check_set_option<double>(options, "checkpoint-freq", 0, learnerOptions.checkpointFreq);
  check_set_option<string>(options, "save-learner-strategy", "", learnerOptions.saveStratFilename);
  check_set_option<bool>(options, "progress-bar", false, learnerOptions.progressBar);

  double pac_target_prob;
  {
    Model prod(model, objective, modelOptions);
    auto prob = prod.getProbabilityOfSat();
    pac_target_prob = prob.find(prod.getInitial())->second;
  }

  unsigned int num_samples = options.options()["est-pac-probability-num-samples"].as<unsigned int>();

  const double pac_epsilon = options.options()["pac_epsilon"].as<double>();

  #pragma omp parallel shared(within_eps_counts)
  {
    Cudd mgr;
    omp_set_lock(&initlock);
    Model model(mgr, options.inputFile(), Verbosity::Silent,
                modelOptions);
    Parity objective{mgr};
    getAutomaton(mgr, options, objective);
    omp_unset_lock(&initlock);

    Gym gym(model, objective, gymOptions);

    Learner learner(gym, learnerOptions);

    unsigned int episodeNumber = options.options()["ep-number"].as<unsigned int>();
    double discount = options.options()["discount"].as<double>();
    double explore = options.options()["explore"].as<double>();
    double alpha = options.options()["alpha"].as<double>();
    double initValue = options.options()["init-value"].as<double>();
    double linearAlphaDecay = options.options()["linear-lr-decay"].as<double>();
    double linearExploreDecay = options.options()["linear-explore-decay"].as<double>();
    
    if (options.options().count("seed")) {
      unsigned int tid = omp_get_thread_num();
      Util::seed_urng(options.options()["seed"].as<unsigned int>() + tid);
    } else {
      Util::seed_urng();
    }

    #pragma omp for nowait
    for(unsigned int i = 0; i < num_samples; i++) {
      auto probs = learner.QLearning(episodeNumber, alpha, linearAlphaDecay, 
        discount, explore, linearExploreDecay, initValue);
      for (auto it : probs) {
        if(abs(it.second - pac_target_prob) < pac_epsilon) {
          int& cnt = within_eps_counts[it.first];
          #pragma omp atomic
            cnt += 1;
        }
      }
    }
  }
  for (auto it : within_eps_counts) {
    std::cout << "Probability for tol " << it.first << " is: " << (((double)it.second) / num_samples) << std::endl;
  }
}


void printObjectiveStats(Parity const & objective)
{
  size_t ntraps = objective.getTrapStates().size();
  bool terminal = objective.isTerminal();
  Priority priorities = objective.getMaxPriority() + 1;
  size_t nstates = objective.numStates();
  string automatonType{objective.isDeterministic() ? "D" : "N"};
  automatonType += priorities > 2 ? "PA" : "BA";
  if (objective.hasEpsilonTransitions()) {
    automatonType += " with epsilon transitions";
  }

  cout << " the " << (terminal ? "terminal " : "") << automatonType << " has "
       << nstates << (nstates != 1 ? " states (" : " state (")
       << ntraps << " trap" << (ntraps != 1 ? "s" : "");
  if (priorities > 2) {
    cout << ") and " << priorities << " priorities." << endl;
  } else {
    cout << ")." << endl;
  }
}
