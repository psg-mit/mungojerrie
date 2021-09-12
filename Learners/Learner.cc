/** @file Learner.cc

  @brief Reinforcement learning algorithms.

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

#include "Learner.hh"

using namespace std;

Learner::Learner(Gym gym, LearnerOptions options) : gym(gym)
{
  isBDP = options.isBDP;
  dotLearn = options.dotLearn;
  dotLearnFilename = options.dotLearnFilename;
  prismLearn = options.prismLearn;
  prismLearnFilename = options.prismLearnFilename;
  verbosity = options.verbosity;
  statsOn = options.statsOn;
  saveQFilename = options.saveQFilename;
  loadQFilename = options.loadQFilename;
  checkpointFreq = options.checkpointFreq;
  saveStratFilename = options.saveStratFilename;
  progressBar = options.progressBar;

  if (checkpointFreq < 0) 
    throw logic_error("Checkpoint frequency cannot negative.");
}

// @brief Helper function to select greedy action based on sum of two Q-tables. Ties broken randomly.
GymAction getGreedyAction(GymObservation const & observation, Qtype const & Q1, Qtype const & Q2, Player player)
{
  vector<GymAction> actions;
  auto const & actionMap = Q1.find(observation)->second;
  if (player == 0) {
    double valMaxA = -INFINITY;
    for (auto const & x : actionMap) {
      double xVal = x.second + Q2.find(observation)->second.find(x.first)->second;
      if (xVal> valMaxA) {
        valMaxA = xVal;
        actions.clear();
      }
      if (xVal == valMaxA) {
        actions.push_back(x.first);
      }
    }
  } else if (player == 1) {
    double valMinA = +INFINITY;
    for (auto const & x : actionMap) {
      double xVal = x.second + Q2.find(observation)->second.find(x.first)->second;
      if (xVal < valMinA) {
        valMinA = xVal;
        actions.clear();
      }
      if (xVal == valMinA) {
        actions.push_back(x.first);
      }
    }
  } else {
    throw logic_error("Player must be either 0 or 1 for learning.");
  }
  size_t numActions = actions.size();
  if (numActions == 1)
    return actions[0];
  vector<double> dist(numActions, 1.0/numActions);
  int index = Util::sample_discrete_distribution(dist.begin(), dist.end());
  return actions[index];
}

// @brief Helper function to select epsilon-greedy action based on sum of two Q-tables.
GymAction getEpsilonGreedyAction(double epsilon, vector<GymAction> const & actions, 
                                 GymObservation const & observation, Qtype const & Q1, Qtype const & Q2, Player player)
{ 
  vector<double> epsilonDist = {epsilon, 1-epsilon};
  if (Util::sample_discrete_distribution(epsilonDist)) {
    return getGreedyAction(observation, Q1, Q2, player);
  } else {
    size_t numActions = actions.size();
    vector<double> dist(numActions, 1.0/numActions);
    return actions[Util::sample_discrete_distribution(dist)];
  }
}

// @brief Helper function to select greedy action based on Q-table. Ties broken randomly.
GymAction getGreedyAction(GymObservation const & observation, Qtype const & Q, Player player)
{
  vector<GymAction> actions;
  auto const & actionMap = Q.find(observation)->second;

  if (player == 0) {
    double valMaxA = -INFINITY;
    for (auto const & x : actionMap) {
      if (x.second > valMaxA) {
        valMaxA = x.second;
        actions.clear();
      }
      if (x.second == valMaxA) {
        actions.push_back(x.first);
      }
    }
  } else if (player == 1) {
    double valMinA = +INFINITY;
    for (auto const & x : actionMap) {
      if (x.second < valMinA) {
        valMinA = x.second;
        actions.clear();
      }
      if (x.second == valMinA) {
        actions.push_back(x.first);
      }
    }
  } else {
    throw logic_error("Player must be either 0 or 1 for learning.");
  }
  size_t numActions = actions.size();
  if (numActions == 1)
    return actions[0];
  vector<double> dist(numActions, 1.0/numActions);
  int index = Util::sample_discrete_distribution(dist.begin(), dist.end());
  return actions[index];
}

// @brief Helper function to select greedy action based on Q-table. Ties broken randomly.
GymAction getMinAction(GymObservation const & observation, Qtype const & Q, Player player)
{
  vector<GymAction> actions;
  auto const & actionMap = Q.find(observation)->second;

    double valMinA = +INFINITY;
    for (auto const & x : actionMap) {
      if (x.second < valMinA) {
        valMinA = x.second;
        actions.clear();
      }
      if (x.second == valMinA) {
        actions.push_back(x.first);
      }
    } 
  size_t numActions = actions.size();
  if (numActions == 1)
    return actions[0];
  vector<double> dist(numActions, 1.0/numActions);
  int index = Util::sample_discrete_distribution(dist.begin(), dist.end());
  return actions[index];
}

// @brief Helper function to select epsilon-greedy action based on Q-table.
GymAction getEpsilonMinAction(double epsilon, vector<GymAction> const & actions, 
                                 GymObservation const & observation, Qtype const & Q, Player player)
{ 
  vector<double> epsilonDist = {epsilon, 1-epsilon};
  if (Util::sample_discrete_distribution(epsilonDist)) {
    return getMinAction(observation, Q, player);
  } else {
    size_t numActions = actions.size();
    vector<double> dist(numActions, 1.0/numActions);
    return actions[Util::sample_discrete_distribution(dist)];
  }
}

// @brief Helper function to select epsilon-greedy action based on Q-table.
GymAction getEpsilonGreedyAction(double epsilon, vector<GymAction> const & actions, 
                                 GymObservation const & observation, Qtype const & Q, Player player)
{ 
  vector<double> epsilonDist = {epsilon, 1-epsilon};
  if (Util::sample_discrete_distribution(epsilonDist)) {
    return getGreedyAction(observation, Q, player);
  } else {
    size_t numActions = actions.size();
    vector<double> dist(numActions, 1.0/numActions);
    return actions[Util::sample_discrete_distribution(dist)];
  }
}

// @brief Helper function to select epsilon-greedy action based on Q-table.
GymAction getRandomAction(vector<GymAction> const & actions, 
                                 GymObservation const & observation, Qtype const & Q, Player player)
{ 
    size_t numActions = actions.size();
    vector<double> dist(numActions, 1.0/numActions);
    return actions[Util::sample_discrete_distribution(dist)];
}

std::map<double, double> Learner::SarsaLambda(double lambda, bool replacingTrace, unsigned int numEpisodes,
                       double alpha, double linearAlphaDecay, double kktAlphaDecay, double discount, 
                       double epsilon, double linearExploreDecay, double initValue)
{
  if (isBDP)
    throw logic_error("SarsaLambda does not support BDPs.");

  Qtype Q;

  if (!(discount >= 0 && discount <= 1))
    throw logic_error("Value of discount factor must be within bounds 0 <= discount <= 1.");
  if (!(epsilon >= 0 && epsilon <= 1))
    throw logic_error("Value of explore rate must be within bounds 0 <= explore <= 1.");
  if (!(alpha >= 0 && alpha <= 1))
    throw logic_error("Value of learning rate must be within bounds 0 <= alpha <= 1.");
  if (linearAlphaDecay > alpha)
    throw logic_error("Value of linearAlphaDecay must be less than alpha.");
  if (linearExploreDecay > epsilon)
    throw logic_error("Value of linearExploreDecay must be less than epsilon.");

  double alphaDecay;
  if (linearAlphaDecay < 0) {
    alphaDecay = 0.0;
  } else {
    alphaDecay = (alpha - linearAlphaDecay)/numEpisodes;
  }

  double exploreDecay;
  if (linearExploreDecay < 0) {
    exploreDecay = 0.0;
  } else {
    exploreDecay = (epsilon - linearExploreDecay)/numEpisodes;
  }

  if (loadQFilename != "") {
    gym.loadQ(Q, "SarsaLambda", loadQFilename);
  }

  unsigned int checkpointEp = 0;
  int progress = 0;

  gym.resetStats();

  // Loop for each episode 
  for (unsigned int episode = 1; episode <= numEpisodes; episode++) {
    // Initialize eligibility trace
    map<pair<GymObservation, GymAction>, double> z;

    // Initialize  S
    auto info = gym.reset();
    GymObservation S = info.observation;

    // If S is previously unseen; expand Q-table
    auto it = Q.find(S);
    if (it == Q.end()) {
      map<GymAction, double> init;
      for (GymAction act : info.actions) {
        if (info.done)
          init[act] = 0.0;
        else
          init[act] = initValue;
      }
      Q[S] = init;
    }
    
    // Choose action A from S using policy derived from Q (e.g., epsilon-greedy)
    GymAction A = getEpsilonGreedyAction(epsilon, info.actions, S, Q, info.player);

    // Loop for each step of episode 
    while (!info.done) {  // until S is terminal 
      // Take action A
      info = gym.step(A);
      GymObservation sPrime = info.observation;
      auto it = Q.find(sPrime);
      // If the observation S' is previously unexplored; expand Q-table
      if (it == Q.end()) {
        map<GymAction, double> init;
        for (GymAction act : info.actions)
          init[act] = 0.0;
        Q[sPrime] = init;
      }

      // Choose action A' from S' using using policy derived from Q (e.g., epsilon-greedy)
      GymAction aPrime = getEpsilonGreedyAction(epsilon, info.actions, sPrime, Q, info.player);
      Reward R = info.reward;

      // Compute temporal difference error
      double delta; 
      if (!info.done || info.terminationOverride) {
        if (!info.discountOverride)
          delta = R + discount*Q[sPrime][aPrime] - Q[S][A];
        else
          delta = R + info.discount*Q[sPrime][aPrime] - Q[S][A];
      } else {
        delta = R - Q[S][A];
      }
      // Update eligibility trace for S, A
      if (!replacingTrace)
        z[{S, A}] += 1.0;
      else
        z[{S, A}] = 1.0;
   
      // Update Q and eligibility trace for all S, A
      for (auto & x : z) {
        auto & Qx = Q[x.first.first][x.first.second];
        Qx = Qx + alpha*delta*x.second;
        x.second = lambda * discount * x.second;
      }

      // S <- S' and A <- A'
      S = sPrime;
      A = aPrime;
    }

    if (kktAlphaDecay > 0) {
      alpha = kktAlphaDecay / (kktAlphaDecay + episode);
    } else {
      alpha -= alphaDecay;
    }

    if (alpha < 0)
      alpha = 0.0;
    epsilon -= exploreDecay;
    if (epsilon < 0)
      epsilon = 0.0;

    if (checkpointFreq != 0 && (double)(episode - checkpointEp)/(double)numEpisodes >= checkpointFreq) {
      checkpointEp = episode;
      gym.saveQ(Q, "SarsaLambda", saveQFilename + "_ep" + to_string(checkpointEp));
    }

    if (progressBar && floor((double)(progressBarLength*episode)/(double)numEpisodes) > progress) {
      progress++;
      cout << "\r[";
      for (int i = 0; i < progressBarLength; i++) {
        if (i < floor((double)(progressBarLength*episode)/(double)numEpisodes))
          cout << "=";
        else
          cout << " ";
      }
      cout << "]" << flush;
      if (progress == progressBarLength)
        cout << endl;
    }
  }

  // Learning complete
  if (verbosity > Verbosity::Silent) {
    Util::printElapsedTag();
    cout << " learning complete." << endl;
  }
  if (statsOn) {
    auto info = gym.reset();
    if (Q.find(info.observation) == Q.end())
      cout << "Initial-state value estimate: " << initValue << endl;
    else
      cout << "Initial-state value estimate: " << Q[info.observation][(getGreedyAction(info.observation, Q, info.player))] << endl;
    gym.printStats();
  }
  if (verbosity > Verbosity::Terse) {
    cout << "Q:\n" << gym.toString(Q) << endl << endl;
  }
  if (saveStratFilename != "") {
    gym.saveStrat(Q, saveStratFilename);
  }
  if (saveQFilename != "") {
    gym.saveQ(Q, "SarsaLambda", saveQFilename);
  }
  if (dotLearn) {
    gym.printDotLearn(Q, dotLearnFilename);
  }
  if (prismLearn) {
    gym.printPrismLearn(Q, prismLearnFilename);
  }

  auto probs = gym.getProbabilityOfSat(Q, statsOn);
  if (verbosity >= Verbosity::Informative) {
    for (auto it : probs)
      cout << "Probability for tol " << it.first << " is: " << it.second << endl;
  }
  return probs;
}

std::map<double, double> Learner::DoubleQLearning(unsigned int numEpisodes, double alpha, double linearAlphaDecay, double kktAlphaDecay,
                              double discount, double epsilon, double linearExploreDecay, double initValue)
{
  if (isBDP)
    throw logic_error("DoubleQLearning does not support BDPs.");

  Qtype Q1;
  Qtype Q2;

  if (!(discount >= 0 && discount <= 1))
    throw logic_error("Value of discount factor must be within bounds 0 <= discount <= 1.");
  if (!(epsilon >= 0 && epsilon <= 1))
    throw logic_error("Value of explore rate must be within bounds 0 <= explore <= 1.");
  if (!(alpha >= 0 && alpha <= 1))
    throw logic_error("Value of learning rate must be within bounds 0 <= alpha <= 1.");
  if (linearAlphaDecay > alpha)
    throw logic_error("Value of linearAlphaDecay must be less than alpha.");
  if (linearExploreDecay > epsilon)
    throw logic_error("Value of linearExploreDecay must be less than epsilon.");

  double alphaDecay;
  if (linearAlphaDecay < 0) {
    alphaDecay = 0.0;
  } else {
    alphaDecay = (alpha - linearAlphaDecay)/numEpisodes;
  }

  double exploreDecay;
  if (linearExploreDecay < 0) {
    exploreDecay = 0.0;
  } else {
    exploreDecay = (epsilon - linearExploreDecay)/numEpisodes;
  }

  if (loadQFilename != "") {
    gym.loadQ(Q1, Q2, "DoubleQLearning", loadQFilename);
  }

  unsigned int checkpointEp = 0;
  int progress = 0;

  gym.resetStats();

  // Loop for each episode 
  for (unsigned int episode = 1; episode <= numEpisodes; episode++) {
    auto info = gym.reset();
    GymObservation S = info.observation;
    Player player = info.player;

     // If S is previously unseen; expand Q-tables
    auto it = Q1.find(S);
    if (it == Q1.end()) {
      map<GymAction, double> init;
      for (GymAction act : info.actions) {
        if (info.done)
          init[act] = 0.0;
        else
          init[act] = initValue;
      }
      Q1[S] = init;
      Q2[S] = init;
    }

    // Choose action A from  S using policy (epsilon-greedy) in Q1+Q2
    GymAction A = getEpsilonGreedyAction(epsilon, info.actions, S, Q1, Q2, player);

    // Loop for each step of episode 
    while (!info.done) { // until S is terminal 
      // Take action A; observe R, S'
      info = gym.step(A);
      GymObservation sPrime = info.observation;
      Reward R = info.reward;

     // If the observation S' is previously unexplored; expand Q-tables
      auto it = Q1.find(sPrime);
      if (it == Q1.end()) {
        map<GymAction, double> init;
        for (GymAction act : info.actions) {
          if (info.done)
            init[act] = 0.0;
          else
            init[act] = initValue;
        }
        Q1[sPrime] = init;
        Q2[sPrime] = init;
      }

      // With equal probability pick Q1 or Q2 to update
      int sample = Util::sample_discrete_distribution({0.5, 0.5});

      if (sample) {
        // Compute a = argopt_a Q1(S', a)  
        GymAction maxminAPrime = getGreedyAction(sPrime, Q1, info.player);

        // Q1(S, A) =  Q1(S, A) + alpha * [ R + discountFactor * Q2(S', a) - Q1(S, A) ]
        double delta; // temporal difference error
        if (!info.done || info.terminationOverride) {
          if (!info.discountOverride)
            delta = R + discount*Q2[sPrime][maxminAPrime] - Q1[S][A];
          else
            delta = R + info.discount*Q2[sPrime][maxminAPrime] - Q1[S][A];
        } else {
          delta = R - Q1[S][A];
        }
        Q1[S][A] = Q1[S][A] + alpha*delta;
      } else {
        // Compute a = argopt_a Q2(S', a)  
        GymAction maxminAPrime = getGreedyAction(sPrime, Q2, info.player);

        // Q2(S, A) =  Q2(S, A) + alpha * [ R + discountFactor * Q1(S', a) - Q2(S, A) ]
        double delta; // temporal difference error
        if (!info.done || info.terminationOverride) {
          if (!info.discountOverride)
            delta = R + discount*Q1[sPrime][maxminAPrime] - Q2[S][A];
          else
            delta = R + info.discount*Q1[sPrime][maxminAPrime] - Q2[S][A];
        } else {
          delta = R - Q2[S][A];
        }
        Q2[S][A] = Q2[S][A] + alpha*delta;
      }

      // Choose action A from next state using policy (epsilon-greedy) in Q1+Q2
      A = getEpsilonGreedyAction(epsilon, info.actions, sPrime, Q1, Q2, info.player);
      S = sPrime;
    }

    if (kktAlphaDecay > 0) {
      alpha = kktAlphaDecay / (kktAlphaDecay + episode);
    } else {
      alpha -= alphaDecay;
    }

    if (alpha < 0)
      alpha = 0.0;
    epsilon -= exploreDecay;
    if (epsilon < 0)
      epsilon = 0.0;

    if (checkpointFreq != 0 && (double)(episode - checkpointEp)/(double)numEpisodes >= checkpointFreq) {
      checkpointEp = episode;
      gym.saveQ(Q1, Q2, "DoubleQLearning", saveQFilename + "_ep" + to_string(checkpointEp));
    }

    if (progressBar && floor((double)(progressBarLength*episode)/(double)numEpisodes) > progress) {
      progress++;
      cout << "\r[";
      for (int i = 0; i < progressBarLength; i++) {
        if (i < floor((double)(progressBarLength*episode)/(double)numEpisodes))
          cout << "=";
        else
          cout << " ";
      }
      cout << "]" << flush;
      if (progress == progressBarLength)
        cout << endl;
    }
  }

  // Learning complete
  if (verbosity > Verbosity::Silent) {
    Util::printElapsedTag();
    cout << " learning complete." << endl;
  }
  Qtype Qsum;
  for (auto const & x : Q1) {
    for (auto const & y : x.second) {
      Qsum[x.first][y.first] = 0.5*(y.second + Q2[x.first][y.first]);
    }
  }
  if (statsOn) {
    auto info = gym.reset();
    if (Q1.find(info.observation) == Q1.end()) {
      cout << "Initial-state value estimate: " << initValue << endl;
    } else {
      cout << "Initial-state value estimate Q1: " << Q1[info.observation][(getGreedyAction(info.observation, Q1, info.player))] << endl;
      cout << "Initial-state value estimate Q2: " << Q2[info.observation][(getGreedyAction(info.observation, Q2, info.player))] << endl;
      cout << "Initial-state value estimate: " << Qsum[info.observation][(getGreedyAction(info.observation, Qsum, info.player))] << endl;
    }
    gym.printStats();
  }
  if (verbosity > Verbosity::Terse) {
    cout << "Q:\n" << gym.toString(Qsum) << endl << endl;
  }
  if (saveStratFilename != "") {
    gym.saveStrat(Qsum, saveStratFilename);
  }
  if (saveQFilename != "") {
    gym.saveQ(Q1, Q2, "DoubleQLearning", saveQFilename);
  }
  if (dotLearn) {
    gym.printDotLearn(Qsum, dotLearnFilename);
  }
  if (prismLearn) {
    gym.printPrismLearn(Qsum, prismLearnFilename);
  }
  
  auto probs = gym.getProbabilityOfSat(Qsum, statsOn);
  if (verbosity >= Verbosity::Informative) {
    for (auto it : probs)
      cout << "Probability for tol " << it.first << " is: " << it.second << endl;
  }
  return probs;
}

std::map<double, double> Learner::QLearning(unsigned int numEpisodes, double alpha, double linearAlphaDecay, double kktAlphaDecay,
                        double discount, double epsilon, double linearExploreDecay, double initValue)
{
  Qtype Q;

  if (!(discount >= 0 && discount <= 1))
    throw logic_error("Value of discount factor must be within bounds 0 <= discount <= 1.");
  if (!(epsilon >= 0 && epsilon <= 1))
    throw logic_error("Value of explore rate must be within bounds 0 <= explore <= 1.");
  if (!(alpha >= 0 && alpha <= 1))
    throw logic_error("Value of learning rate must be within bounds 0 <= alpha <= 1.");
  if (linearAlphaDecay > alpha)
    throw logic_error("Value of linearAlphaDecay must be less than alpha.");
  if (linearExploreDecay > epsilon)
    throw logic_error("Value of linearExploreDecay must be less than epsilon.");

  double alphaDecay;
  if (linearAlphaDecay < 0) {
    alphaDecay = 0.0;
  } else {
    alphaDecay = (alpha - linearAlphaDecay)/numEpisodes;
  }

  double exploreDecay;
  if (linearExploreDecay < 0) {
    exploreDecay = 0.0;
  } else {
    exploreDecay = (epsilon - linearExploreDecay)/numEpisodes;
  }

  if (loadQFilename != "") {
    gym.loadQ(Q, "QLearning", loadQFilename);
  }
 
  unsigned int checkpointEp = 0;
  int progress = 0;

  gym.resetStats();

  // Loop for each episode 
  for (unsigned int episode = 1; episode <= numEpisodes; episode++) {
    // Initialize  S
    auto info = gym.reset();
    GymObservation S = info.observation;
    Player player = info.player;

     // If S is previously unseen; expand Q-table
    auto it = Q.find(S);
    if (it == Q.end()) { 
      map<GymAction, double> init;
      for (GymAction act : info.actions) {
        if (info.done)
          init[act] = 0.0;
        else
          init[act] = initValue;
      }
      Q[S] = init;
    }

    // Choose action A from  S using policy derived from Q (e.g., epsilon-greedy)
    GymAction A;

    if (isBDP) {
      if (info.player == 1) {
         A = getRandomAction(info.actions, S, Q, player); 
      } else {
        A = getEpsilonMinAction(epsilon, info.actions, S, Q, player); 
      }
    } else {
      A = getEpsilonGreedyAction(epsilon, info.actions, S, Q, player); 
    }
    
    // if (isBDP && info.player == 1) 
    //   A = getRandomAction(info.actions, S, Q, player); 
    // else 
    //   A = getEpsilonGreedyAction(epsilon, info.actions, S, Q, player);

    // Loop for each step of episode 
    while (!info.done) { // until S is terminal 
      // Take action A; observe R, S'
      info = gym.step(A);
      GymObservation sPrime = info.observation;
      Reward R = info.reward;

     // If the observation S' is previously unexplored; expand Q-table
      auto it = Q.find(sPrime);
      if (it == Q.end()) {
        map<GymAction, double> init;
        for (GymAction act : info.actions) {
          if (info.done)
            init[act] = 0.0;
          else
            init[act] = initValue;
        }
        Q[sPrime] = init;
      }

    // Update vPrime
    double vPrime = 0;
    
    if (isBDP) {
      if (info.player == 1) {
        for (GymAction act : info.actions) {
          vPrime += Q[sPrime][act];
        }      
      } else {
        vPrime = Q[sPrime][getMinAction(sPrime, Q, info.player)];
      }
    } else {
      vPrime = Q[sPrime][getGreedyAction(sPrime, Q, info.player)];
    }

      // double vPrime = 0;
      // if (isBDP && info.player == 1) {
      //   for (GymAction act : info.actions) {
      //     vPrime += Q[sPrime][act];
      //   }
      // }
      // else 
      //   vPrime = Q[sPrime][getGreedyAction(sPrime, Q, info.player)];

      // Q(S, A) =  Q(S, A) + alpha * [ R + discountFactor * (vPrime) - Q(S. A) ]
      double delta; // temporal difference error
      if (!info.done || info.terminationOverride) {
        if (!info.discountOverride) 
          delta = R + discount*vPrime - Q[S][A];
        else
          delta = R + info.discount*vPrime - Q[S][A];
      } else {
        delta = R - Q[S][A];
      }
      Q[S][A] = Q[S][A] + alpha*delta;
      // Choose action A from  S using policy derived from Q (e.g.,
      // epsilon-greedy)
      if (isBDP) {
        if (info.player == 1) {
          A = getRandomAction(info.actions, sPrime, Q, info.player);
        } else {
          A = getEpsilonMinAction(epsilon, info.actions, sPrime, Q, info.player);
        }
      } else {
        A = getEpsilonGreedyAction(epsilon, info.actions, sPrime, Q, info.player);
      }      


      // if (isBDP && info.player == 1) 
      //   A = getRandomAction(info.actions, sPrime, Q, player);
      // else 
      //   A = getEpsilonGreedyAction(epsilon, info.actions, sPrime, Q, player);

      // S <- S'
      S = sPrime; 
    }
    
    if (kktAlphaDecay > 0) {
      alpha = kktAlphaDecay / (kktAlphaDecay + episode);
    } else {
      alpha -= alphaDecay;
    }

    if (alpha < 0)
      alpha = 0.0;
    epsilon -= exploreDecay;
    if (epsilon < 0)
      epsilon = 0.0;
    
    if (checkpointFreq != 0 && (double)(episode - checkpointEp)/(double)numEpisodes >= checkpointFreq) {
      checkpointEp = episode;
      gym.saveQ(Q, "QLearning", saveQFilename + "_ep" + to_string(checkpointEp));
    }

    if (progressBar && floor((double)(progressBarLength*episode)/(double)numEpisodes) > progress) {
      progress++;
      cout << "\r[";
      for (int i = 0; i < progressBarLength; i++) {
        if (i < floor((double)(progressBarLength*episode)/(double)numEpisodes))
          cout << "=";
        else
          cout << " ";
      }
      cout << "]" << flush;
      if (progress == progressBarLength)
        cout << endl;
    }
  }
  // Learning complete

  if (verbosity > Verbosity::Silent) {
    Util::printElapsedTag();
    cout << " learning complete." << endl;
  }
  
  if (statsOn) {
    auto info = gym.reset();
    if (Q.find(info.observation) == Q.end())
      cout << "Initial-state value estimate: " << initValue << endl;
    else {
      if (isBDP) {
        cout << "Initial-state value estimate: " << Q[info.observation][(getMinAction(info.observation, Q, info.player))] << endl;
      } else {
        cout << "Initial-state value estimate: " << Q[info.observation][(getGreedyAction(info.observation, Q, info.player))] << endl;
      }
    }
    gym.printStats();
  }

  if (verbosity > Verbosity::Terse) {
    cout << "Q:\n" << gym.toString(Q) << endl << endl;
  }
  
  if (saveStratFilename != "") {
    if (isBDP) 
      gym.saveStratBDP(Q, saveStratFilename);
    else 
      gym.saveStrat(Q, saveStratFilename);
  }
  if (saveQFilename != "") {
    gym.saveQ(Q, "QLearning", saveQFilename);
  }

  if (dotLearn) {
    gym.printDotLearn(Q, dotLearnFilename);
  }
  if (prismLearn) {
    gym.printPrismLearn(Q, prismLearnFilename);
  }

  if (isBDP) {
    cout << "Total cost (Q-value) from the initial state is: ";
    auto info = gym.reset();
    if (Q.find(info.observation) == Q.end())
      cout << initValue << endl;
    else
      cout << Q[info.observation][(getMinAction(info.observation, Q, info.player))] << endl;
    return {}; // TODO: what to do here?
  } else {
    auto probs = gym.getProbabilityOfSat(Q, statsOn);
    if (verbosity >= Verbosity::Informative) {
      for (auto it : probs)
        cout << "Probability for tol " << it.first << " is: " << it.second << endl;
    }
    return probs;
  }
}
