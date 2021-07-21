#ifndef UTIL_H_
#define UTIL_H_

/** @file Util.hh

  @brief Utility functions.

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
#include <iostream>
#include <sstream>
#include <random>
#include <boost/random.hpp>
#include <boost/functional/hash.hpp>
#include <chrono>
#include <functional>
#include "cuddObj.hh"
#include "State.hh"

/**
   @brief Namespace of utilities.
*/
namespace Util {
  /** @brief Replace first occurrence of "from" in "str" with "to." */
  void replace(std::string & str, std::string const & from,
               std::string const & to);
  /** @brief Replace all occurrences of "from" in "str" with "to." */
  void replaceAll(std::string & str, std::string const & from,
                  std::string const & to);
  /** @brief Copy a vector of C++ strings to an array of C strings. */
  char ** cStringArrayAlloc(std::vector<std::string> const & sv);
  /** @brief Dispose of an array of C strings. */
  void cStringArrayFree(char ** sa, size_t n);

  /** @brief Replacement for std::to_string since cygwin does not have it. */
  template<typename T> std::string to_string(T const & num)
  {
    std::ostringstream oss;
    oss << num;
    return oss.str();
  }

  /**
   * @brief Class that defines a comparison operator for the BDD class.
   *
   * @details This is useful for sets of BDDs.  Note that the '<' operator
   * is already overloaded for BDDs and is not suitable as comparison
   * operator in ordered containers.
   */
  class BddCompare {
  public:
    bool operator()(BDD const & bdd1, BDD const & bdd2) const
    {
      return bdd1.getNode() < bdd2.getNode();
    }
  };

  /**
   * @brief Class that defines a comparison operator for pairs with BDDs.
   */
  template<typename T>
  class PairWithBddCompare {
  public:
    using pwbdd = std::pair<T,BDD>; 
    bool operator()(pwbdd const & p1, pwbdd const & p2) const
    {
      return std::tuple<T,DdNode *>(p1.first, p1.second.getNode()) <
        std::tuple<T,DdNode *>(p2.first, p2.second.getNode());
    }
  };

  /**
   * @brief Class that defines a hash function for pairs.
   */
  template<typename T1, typename T2>
  class PairHash {
  public:
    size_t operator()(std::pair<T1,T2> const & p) const
    {
      size_t seed = std::hash<T1>()(p.first);
      boost::hash_combine(seed, std::hash<T2>()(p.second));
      return seed;
    }
  };

  /** @brief Converts a partition map to a vector of sets. */
  template<typename T, typename Comp>
  std::vector< std::set<T, Comp> > mapToPartition(
    std::map<T,unsigned,Comp> const & m)
  {
    if (m.size() == 0) {
      return std::vector< std::set<T, Comp> >{};
    }
    unsigned maxindex = 0;
    for (auto const & e : m) {
      if (e.second > maxindex)
        maxindex = e.second;
    }
    std::vector< std::set<T, Comp> >  partition(maxindex+1);
    for (auto const & e : m) {
      partition[e.second].insert(e.first);
    }
    return partition;
  }

  using RandomEngine = std::mt19937;

  /** @brief Returns reference to uniform random number generator. */
  RandomEngine & urng();

  /** @brief Seeds the uniform random number generator. */
  void seed_urng();

  /** @brief Seeds the uniform random number generator. */
  void seed_urng(unsigned seed);

  /** @brief Return the seed for the uniform random number generator. */
  unsigned int get_urng_seed();

  /** @brief Returns a sample from the discrete distribution defined by the iterators. */
  template <typename Iterator>
  typename std::iterator_traits<Iterator>::value_type
  sample_discrete_distribution(Iterator first, Iterator last)
  {
    using Distribution = boost::random::discrete_distribution<>;
    using Parameter = Distribution::param_type;
    thread_local static Distribution dist{};
    return dist(urng(), Parameter{first, last});
  }

  /** @brief Returns a sample from discrete distribution defined by weights. */
  int sample_discrete_distribution(std::vector<double> const & weights);

  /** @brief Returns a sample from a uniform integer distribution. */
  template <typename IntType>
  IntType sample_uniform_distribution(IntType low, IntType hi)
  {
    using Distribution = boost::random::uniform_int_distribution<IntType>;
    using Parameter = typename Distribution::param_type;
    thread_local static Distribution dist{};
    return dist(urng(), Parameter{low, hi});
  }

  /** @brief Records a reference time. */
  std::chrono::time_point<std::chrono::system_clock> stopwatch();

  /** @brief Prints tag with elapsed time in seconds. */
  void printElapsedTag(std::ostream & os = std::cout);

  /** @brief Returns the elapsed time in seconds. */
  double getElapsedTime();

  /** @brief Converts an int to an array of characters in base 10. */
  char const * to_decimal(int number);

  /** @brief Returns true if pre is a prefix of str. */
  bool is_prefix(std::string const & str, std::string const & pre);

}

#endif
