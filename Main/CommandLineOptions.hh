#ifndef COMMAND_LINE_OPTIONS_HH_
#define COMMAND_LINE_OPTIONS_HH_

/** @file CommandLineOptions.hh

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

#include <boost/program_options.hpp>
#include <string>
#include <vector>

#include "Verbosity.hh"
#include "ModelOptions.hh"
#include "Gym.hh"

/**
 * @brief A class for parsing and storing command line options
 */
class CommandLineOptions {

public:

  /**
   * @brief Constructor adds the desired command line options
   */
  CommandLineOptions();

  /**
   * @brief Parses the command line options and stores them in the appropriate member variables
   */
  int parseCommandLineOptions(int argc, char * argv[]);

  /**
   * @brief Returns the boost::program_options::variables_map
   * @details The map indicates the command line options.
   */
  boost::program_options::variables_map & options() { return varMap; }

  boost::program_options::variables_map const & options() const 
  { return varMap; }

  /**
   * @brief Get the input file name
   */
  std::string const & inputFile() const { return inputFileName; }

  ModelOptions::ReachType reachSolver() const {
    if (varMap["reach-with"].as<std::string>() == std::string("glop"))
      return ModelOptions::ReachType::glop;
    if (varMap["reach-with"].as<std::string>() == std::string("iter"))
      return ModelOptions::ReachType::iter;
    throw std::out_of_range("Invalid solver type");
  }

  ModelOptions::SSPType sspSolver() const {
    if (varMap["ssp-with"].as<std::string>() == std::string("glop"))
      return ModelOptions::SSPType::glop;
    if (varMap["ssp-with"].as<std::string>() == std::string("iter"))
      return ModelOptions::SSPType::iter;
    if (varMap["ssp-with"].as<std::string>() == std::string("poly"))
      return ModelOptions::SSPType::poly;
    throw std::out_of_range("Invalid solver type");
  }

  void fillModelOptions(ModelOptions & m) const;

  bool learnEnabled() const {
    if (varMap["learn"].as<std::string>() != std::string("none"))
      return true;
    else
      return false;
  }

  bool QEnabled() const {
    if (varMap["learn"].as<std::string>() == std::string("Q"))
      return true;
    else
      return false;
  }

  bool DQEnabled() const {
    if (varMap["learn"].as<std::string>() == std::string("DQ"))
      return true;
    else
      return false;
  }

  bool SLEnabled() const {
    if (varMap["learn"].as<std::string>() == std::string("SL"))
      return true;
    else
      return false;
  }

  GymOptions::GymRewardTypes getRewardType() const {
    if (varMap["reward-type"].as<std::string>() == std::string("default-type"))
      return GymOptions::GymRewardTypes::default_type;
    else if (varMap["reward-type"].as<std::string>() == std::string("prism"))
      return GymOptions::GymRewardTypes::prism;
    else if (varMap["reward-type"].as<std::string>() == std::string("zeta-reach"))
      return GymOptions::GymRewardTypes::zeta_reach;
    else if (varMap["reward-type"].as<std::string>() == std::string("zeta-acc"))
      return GymOptions::GymRewardTypes::zeta_acc;
    else if (varMap["reward-type"].as<std::string>() == std::string("zeta-discount"))
      return GymOptions::GymRewardTypes::zeta_discount;
    else if (varMap["reward-type"].as<std::string>() == std::string("reward-on-acc"))
      return GymOptions::GymRewardTypes::reward_on_acc;
    else if (varMap["reward-type"].as<std::string>() == std::string("multi-discount"))
      return GymOptions::GymRewardTypes::multi_discount;
    else if (varMap["reward-type"].as<std::string>() == std::string("parity"))
      return GymOptions::GymRewardTypes::parity;
    else if (varMap["reward-type"].as<std::string>() == std::string("pri-tracker"))
      return GymOptions::GymRewardTypes::pri_tracker;
    else
      throw(std::runtime_error("Reward type invalid.")); 
  }

  /**
   * @brief Verbosity (integer and enum values).
   */
  union {
    int verbosityLevel;
    Verbosity::Level verbosity;
  };

private:
  boost::program_options::options_description visible;
  boost::program_options::options_description hidden;
  boost::program_options::options_description cmdline_options;
  boost::program_options::positional_options_description posOpt;
  boost::program_options::variables_map varMap;

  // Variables that reflect the command line options go here

  /**
   * @brief Input File
   */
  std::string inputFileName;


};

#endif
