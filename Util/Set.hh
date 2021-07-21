#ifndef SET_HH_
#define SET_HH_

/** @file Set.hh

  @brief Set manipulation functions.

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

#include <set>
#include <iostream>
#include <type_traits>
#include <vector>
#include <unordered_set>
#include <list>
#include <forward_list>
#include <deque>
#include <map>
#include <unordered_map>
#include <tuple>
#include <array>

/** Checks whether set1 and set2 are disjoint. */
template<typename Set1, typename Set2> 
bool disjoint(Set1 const &set1, Set2 const &set2)
{
  if (set1.empty() || set2.empty()) return true;

  typename Set1::const_iterator it1 = set1.begin(), it1End = set1.end();
  typename Set2::const_iterator it2 = set2.begin(), it2End = set2.end();

  if (*it1 > *set2.rbegin() || *it2 > *set1.rbegin()) return true;

  while (it1 != it1End && it2 != it2End) {
    if (*it1 == *it2) return false;
    if (*it1 < *it2) { ++it1; }
    else { ++it2; }
  }
  return true;
}

/** Checks whether set1 is a subset of set2. */
template<typename Set1, typename Set2>
bool subsetOf(Set1 const & set1, Set2 const & set2)
{
  if (set1.size() > set2.size()) return false;
  for (auto const & e : set1) {
    if (set2.find(e) == set2.end())
      return false;
  }
  return true;
}

/** Adds the elements of rhs to lhs. */
template<typename Set1, typename Set2>
std::set<typename Set1::value_type> & operator|=(Set1 & lhs, Set2 const & rhs)
{
  lhs.insert(rhs.cbegin(), rhs.cend());
  return lhs;
}

/** Returns the union of set1 and set2. */
template<typename Set1, typename Set2>
std::set<typename Set1::value_type> operator|(Set1 set1, Set2 const & set2)
{
  set1 |= set2;
  return set1;
}

/** Removes the elements not in rhs from lhs. */
template<typename Set1, typename Set2>
std::set<typename Set1::value_type> & operator&=(Set1 & lhs, Set2 const & rhs)
{
  typename Set1::iterator it = lhs.begin();
  while (it != lhs.end()) {
    if (rhs.find(*it) == rhs.end())
      it = lhs.erase(it);
    else
      ++it;
  }
  return lhs;
}

/** Returns the intersection of set1 and set2. */
template<typename Set1, typename Set2>
std::set<typename Set1::value_type> operator&(Set1 set1, Set2 const & set2)
{
  set1 &= set2;
  return set1;
}

/** Removes the elements of rhs from lhs. */
template<typename Set1, typename Set2>
std::set<typename Set1::value_type> & operator-=(Set1 & lhs, Set2 const & rhs)
{
  for (auto const & e : rhs) {
    lhs.erase(e);
    if (lhs.size() == 0) break;
  }
  return lhs;
}

/** Returns the set difference of set1 and set2. */
template<typename Set1, typename Set2>
std::set<typename Set1::value_type> operator-(Set1 set1, Set2 const & set2)
{
  set1 -= set2;
  return set1;
}

/** Changes lhs to the symmetric difference of lhs and rhs. */
template<typename Set1, typename Set2>
std::set<typename Set1::value_type> & operator^=(Set1 & lhs, Set2 const & rhs)
{
  for (auto const & e : rhs) {
    auto it = lhs.find(e);
    if (it != lhs.end())
      lhs.erase(it);
    else
      lhs.insert(e);
  }
  return lhs;
}

/** Returns the symmetric difference of set1 and set2. */
template<typename Set1, typename Set2>
std::set<typename Set1::value_type> operator^(Set1 set1, Set2 const & set2)
{
  set1 ^= set2;
  return set1;
}

// Overloading of operator<< for vectors, sets, unordered_sets, deques, lists,
// forward_lists.  Requires access to all members of container.  So, it does
// not work with stacks and queues.  It does not work with arrays either.
template<template<typename, typename...> class Container,
         typename T,
         typename = typename std::enable_if<std::is_same<std::vector<T>, Container<T> >::value ||
                                            std::is_same<std::set<T>, Container<T> >::value ||
                                            std::is_same<std::unordered_set<T>, Container<T> >::value ||
                                            std::is_same<std::deque<T>, Container<T> >::value ||
                                            std::is_same<std::list<T>, Container<T> >::value ||
                                            std::is_same<std::forward_list<T>, Container<T> >::value
                                            >::type,
         typename ... Rest>
std::ostream & operator<<(std::ostream & os, Container<T, Rest...> const & s)
{
  os << '{';
  char sep[] = ",";
  char * psep = sep + 1;
  for (T const & e : s) {
    os << psep << e;
    psep = sep;
  }
  os << '}';
  return os;
}

// Overloading of operator<< for pairs.
template<typename F, typename S>
std::ostream & operator<<(std::ostream & os, std::pair<F, S> const & p)
{
  os << '(' << p.first << ',' << p.second << ')';
  return os;
}

// Overloading of operator<< for maps.
template<template<typename, typename...> class Container,
         typename Key, typename Val,
         typename = typename std::enable_if<std::is_same<std::map<Key, Val>, Container<Key, Val> >::value || std::is_same<std::unordered_map<Key, Val>, Container<Key, Val> >::value>::type,
         typename ... Rest>
std::ostream & operator<<(std::ostream & os, Container<Key, Val, Rest...> const & m)
{
  os << '{';
  char sep[] = ",";
  char * psep = sep + 1;
  for (std::pair<Key, Val> const e : m) {
    os << psep << e;
    psep = sep;
  }
  os << '}';
  return os;
}

// Overloading of operator<< for std::arrays.
template<typename T, size_t N>
std::ostream & operator<<(std::ostream & os, std::array<T, N> const & a)
{
  os << '{';
  char sep[] = ",";
  char * psep = sep + 1;
  for (T const & e : a) {
    os << psep << e;
    psep = sep;
  }
  os << '}';
  return os;
}

// Overloading of operator<< for C arrays.  Arrays of chars are
// excluded to avoid ambiguity.
template<typename T,
         typename = typename std::enable_if<!std::is_same<char, T>::value>::type,
         size_t N>
std::ostream & operator<<(std::ostream & os, T const (&a)[N])
{
  os << '{';
  char sep[] = ",";
  char * psep = sep + 1;
  for (T const & e : a) {
    os << psep << e;
    psep = sep;
  }
  os << '}';
  return os;
}


// Overloading of operator<< for tuples.  Uses compile-time recursion.
template <typename T, size_t Size>
struct print_tuple_aux {
  static std::ostream & print(std::ostream & os, T const & t) {
    return print_tuple_aux<T, Size-1>::print(os, t)
      << (Size != 1 ? "," : "") << std::get<Size-1>(t);
  }
};

template <typename T>
struct print_tuple_aux<T, 0> {
  static std::ostream & print(std::ostream & os, T const &) {
    return os << '<';
  }
};

template <typename ...Args>
std::ostream & operator<<(std::ostream & os, std::tuple<Args...> const & t) {
  using T = std::tuple<Args...>;
  return print_tuple_aux<T, sizeof...(Args)>::print(os, t) << '>';
}

#endif
