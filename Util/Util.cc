/** @file Util.cc

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

#include <cstring>
#include <limits>
#include <algorithm>
#include "Util.hh"

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace std;

namespace Util {

  thread_local static unsigned int prngSeed;

  // These two are adapted from a Stack Overflow post.

  void replace(string & str, string const & from, string const & to)
  {
    size_t start_pos = str.find(from);
    if(start_pos != string::npos)
      str.replace(start_pos, from.length(), to);
  }

  void replaceAll(string & str, string const & from, string const & to)
  {
    if(from.empty())
      return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != string::npos) {
      str.replace(start_pos, from.length(), to);
      start_pos += to.length(); // In case "to" contains "from,"
                                // as when replacing "x" with "yx."
    }
  }

  char ** cStringArrayAlloc(vector<string> const & sv)
  {
    size_t nvars = sv.size();
    char **inames = new char *[nvars];
    size_t j = 0;
    for (auto v : sv) {
      char *cstr = new char[v.length() + 1];
      strcpy(cstr, v.c_str());
      inames[j] = cstr;
      ++j;
    }
    return inames;
  }

  void cStringArrayFree(char ** sa, size_t n)
  {
    for (size_t i = 0; i != n; ++i)
      delete [] sa[i];
    delete [] sa;
  }

  RandomEngine & urng()
  {
    thread_local static RandomEngine rng{};
    return rng;
  }

  void seed_urng()
  {
    using Device = random_device;
    thread_local static Device rdev{};
#ifdef _OPENMP
    prngSeed = rdev();
#else
    prngSeed = rdev() + omp_get_thread_num();
#endif
    urng().seed(prngSeed);
  }

  void seed_urng(unsigned seed)
  {
    prngSeed = seed;
    urng().seed(seed);
  }

  unsigned int get_urng_seed()
  {
    return prngSeed;
  }

  int sample_discrete_distribution(vector<double> const & weights) {
    return sample_discrete_distribution(weights.cbegin(), weights.cend());
  }

  // The first call to this function starts the stopwatch by saving the
  // current time, which is then returned by all subsequent calls.
  chrono::time_point<chrono::system_clock> stopwatch()
  {
    static auto firsttime = chrono::system_clock::now();
    return firsttime;
  }

  void printElapsedTag(ostream & os)
  {
    auto lapTime = chrono::system_clock::now();
    chrono::duration<double> elapsed = lapTime - stopwatch();
    os << "@ " << elapsed.count() << " s:";
  }

  double getElapsedTime()
  {
    auto lapTime = chrono::system_clock::now();
    chrono::duration<double> elapsed = lapTime - stopwatch();
    return elapsed.count();
  }

  namespace {
    void unsigned_to_decimal(unsigned number, char * buffer)
    {
      if (number == 0) {
        *buffer++ = '0';
      } else {
        char* p_first = buffer;
        while (number != 0) {
          *buffer++ = '0' + number % 10;
          number /= 10;
        }
        reverse(p_first, buffer);
      }
      *buffer = '\0';
    }
  }

  char const * to_decimal(int number)
  {
    using intInfo = numeric_limits<int>;
    size_t constexpr intDigits = intInfo::max_digits10;
    size_t constexpr intBufferSize = intDigits+2;
    thread_local static char buffer[intBufferSize];
    if (number < 0) {
      buffer[0] = '-';
      unsigned_to_decimal(-number, buffer + 1);
    } else {
      unsigned_to_decimal(number, buffer);
    }
    return buffer;
  }

  bool is_prefix(string const & str, string const & pre)
  {
    auto mm = mismatch(str.begin(), str.end(),
                       pre.begin(), pre.end()).second;
    return mm == pre.end();
  }
}
