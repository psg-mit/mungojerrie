/** @file testUtil.cc

  @brief Test program for utility functions.

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
#include "Util.hh"

int main()
{
  // Taking half the samples from distribution 1 and the other half
  // from distribution 2, we expect 1/4 of the samples to be 0,
  // 1/4 to be 1, 1/6 to be 2, and 1/3 to be 3.
  std::vector<double> weights1{1.0/3.0, 1.0/6.0, 1.0/6.0, 1.0/3.0};
  std::vector<int> weights2{1, 2, 1, 2};
  std::vector<int> counts(weights1.size(), 0);

  Util::seed_urng();

  for (size_t i = 0; i != 40000; ++i) {
    int s1 = Util::sample_discrete_distribution(weights1);
    counts[s1]++;
    int s2 = Util::sample_discrete_distribution(weights2.cbegin(),
                                                weights2.cend());
    counts[s2]++;
  }

  Util::seed_urng(3u);

  for (size_t i = 0; i != 10000; ++i) {
    int s1 = Util::sample_discrete_distribution(weights1);
    counts[s1]++;
    int s2 = Util::sample_discrete_distribution(weights2.cbegin(),
                                                weights2.cend());
    counts[s2]++;
  }

  std::cout << "Discrete distribution:" << std::endl;
  for (size_t v = 0; v != counts.size(); ++v) {
    std::cout << v << ": " << counts[v] << std::endl;
  }

  Util::seed_urng();

  size_t hi = 3;
  std::vector<size_t> ucounts(hi+1, 0);
  for (size_t i = 0; i != 10000; ++i) {
    int s = Util::sample_uniform_distribution<size_t>(0,hi);
    ucounts[s]++;
  }

  std::cout << "Uniform distribution:" << std::endl;
  for (size_t v = 0; v != ucounts.size(); ++v) {
    std::cout << v << ": " << ucounts[v] << std::endl;
  }
  
  return 0;
}
