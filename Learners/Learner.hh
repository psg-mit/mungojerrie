#ifndef LEARNER_
#define LEARNER_

/** @file Learner.hh

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

#include "Gym.hh"

struct LearnerOptions {
  LearnerOptions() {
    saveQFilename = "";
    loadQFilename = "";
    checkpointFreq = 0;
    saveStratFilename = "";
  }
  bool isBDP;
  bool dotLearn;
  std::string dotLearnFilename;
  bool prismLearn;
  std::string prismLearnFilename;
  Verbosity::Level verbosity;
  bool statsOn;
  std::string saveQFilename;
  std::string loadQFilename;
  double checkpointFreq;
  std::string saveStratFilename;
  bool progressBar;
};

class Learner {
public:
  friend std::ostream & operator<<(std::ostream & os, Qtype const & Q);
  /** @brief Constructor for learner. */
  Learner(Gym gym, LearnerOptions options);
  /** @brief Runs the Sarsa(\f$\lambda\f$) algorithm. 
   *  @param lambda The lambda parameter in Sarsa(\f$\lambda\f$). 
   *  @param replacingTrace Indicates the use of replacing traces. 
   *  If it is false, then accumulating traces are used. 
   *  @param numEpisodes The number of episodes to train for.
   *  @param alpha The learning rate. 
   *  @param linearAlphaDecay The final value of alpha to decay 
   *  linearly to over the course of learning. Negative values indicate
   *  no decay. 
   *  @param discount The discount factor. 
   *  @param epsilon The epsilon parameter used in the epsilon 
   *  greedy action selection. 
   *  @param linearExploreDecay The final value epsilon to decay linearly 
   *  to over the course of learning. Negative values indicate no decay. 
   *  @param initValue The value to initialize the Q-table to.
   */
  void SarsaLambda(double lambda, bool replacingTrace, unsigned int numEpisodes,
                double alpha, double linearAlphaDecay, double discount, 
                double epsilon, double linearExploreDecay, double initValue);
  /** @brief Runs the Double Q-learning algorithm. 
   *  @param numEpisodes The number of episodes to train for.
   *  @param alpha The learning rate. 
   *  @param linearAlphaDecay The final value of alpha to decay 
   *  linearly to over the course of learning. Negative values indicate
   *  no decay. 
   *  @param discount The discount factor. 
   *  @param epsilon The epsilon parameter used in the epsilon 
   *  greedy action selection. 
   *  @param linearExploreDecay The final value epsilon to decay linearly 
   *  to over the course of learning. Negative values indicate no decay. 
   *  @param initValue The value to initialize the Q-table to.
   */
  void DoubleQLearning(unsigned int numEpisodes, double alpha, double linearAlphaDecay, 
                       double discount, double epsilon, double linearExploreDecay, double initValue);
  /** @brief Runs the Q-learning algorithm. 
   *  @param numEpisodes The number of episodes to train for.
   *  @param alpha The learning rate. 
   *  @param linearAlphaDecay The final value of alpha to decay 
   *  linearly to over the course of learning. Negative values indicate
   *  no decay. 
   *  @param discount The discount factor. 
   *  @param epsilon The epsilon parameter used in the epsilon 
   *  greedy action selection. 
   *  @param linearExploreDecay The final value epsilon to decay linearly 
   *  to over the course of learning. Negative values indicate no decay. 
   *  @param initValue The value to initialize the Q-table to.
   */
  void QLearning(unsigned int numEpisodes, double alpha, double linearAlphaDecay, 
                 double discount, double epsilon, double linearExploreDecay, double initValue);
private:
  Gym gym;
  bool isBDP;
  bool dotLearn;
  bool prismLearn;
  std::string dotLearnFilename;
  std::string prismLearnFilename;
  Verbosity::Level verbosity;
  bool statsOn;
  std::string saveQFilename;
  std::string loadQFilename;
  double checkpointFreq;
  std::string saveStratFilename;
  bool progressBar;
  static int constexpr progressBarLength = 25;
};

#endif
