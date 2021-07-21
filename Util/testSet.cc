/** @file testSet.cc

  @brief Test rpogram for set manipulation functions.

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
#include <deque>
#include <list>
#include <forward_list>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <tuple>
#include <array>
#include "Set.hh"

using namespace std;

/** @brief This struct uses template specialization for the comparison function. */
struct MyStruct {
  MyStruct(int a = 0, int b = 0) : a(a), b(b) {}
  friend ostream& operator<<(ostream & os, MyStruct const & s)
  {
    os << "(" << s.a << "," << s.b << ")";
    return os;
  }
  int a;
  int b;
};

namespace std {
  /** @brief Overloading of '<' operator. */
  template<> struct less<MyStruct> {
    bool operator()(MyStruct const & s1, MyStruct const & s2) const
    {
      return (s1.a + s1.b) < (s2.a + s2.b);
    }
  };
}


/** @brief This struct overloads the '<' operator. */
struct MyOtherStruct {
  MyOtherStruct(int a = 0, int b = 0) : a(a), b(b) {}
  bool operator<(MyOtherStruct const & other) const
  {
    return (a + b) < (other.a + other.b);
  }
  friend ostream& operator<<(ostream & os, MyOtherStruct const & s)
  {
    os << "(" << s.a << "," << s.b << ")";
    return os;
  }
  int a;
  int b;
};


/** @brief This structure relies on a function object for the comparison. */
struct YourStruct {
  YourStruct(int a = 0) : a(a) {}
  bool operator==(YourStruct const & other) const
  {
    return a == other.a;
  }
  bool operator<(YourStruct const & other) const
  {
    return a < other.a;
  }
  friend ostream& operator<<(ostream & os, YourStruct const & s)
  {
    os << s.a;
    return os;
  }
  int a;
};

/** @brief Function object for YourStruct. */
struct YourCompare {
  bool operator()(YourStruct const & s1, YourStruct const & s2) const
  {
    return s1.a < s2.a;
  }
};


/** @brief Template class that overloads operator<<. */
template<typename T1, typename T2>
struct MyPair {
  MyPair(T1 a, T2 b) : a(a), b(b) {};
  friend ostream& operator<<(ostream & os, MyPair const & p)
  {
    os << "(" << p.a << "," << p.b << ")";
    return os;
  }
  T1 a;
  T2 b;
};

void phase1(void);
void phase2(void);

int main(void)
{
  phase1();
  phase2();
  return 0;
}

void phase1(void)
{
  set<int> S;
  cout << (S.empty() ? "empty" : "not empty") << " [empty]" << endl;
  set<unsigned> T({1,2});
  set<unsigned> U(T.begin(), T.end(), std::less<unsigned>());
  cout << "U: " << U << " [{1,2}]" << endl;
  set< unsigned,less<unsigned> > V;
  V.insert(43);
  cout << (V.find(43) != V.end() ? "" : "not ") << "found [found]" << endl;
  V.clear();
  cout << (V == U ? "equal" : "different") << " [different]" << endl;
  set<unsigned>::const_pointer up = &(*(T.begin()));
  cout << "up: " << *up << " [1]" << endl;
  set<unsigned> W({3,4});
  T.swap(W);
  cout << "4 is " << (T.count(4) ? "in" : "out") << " [in]" << endl;
  cout << "W: " << W << " [{1,2}]" << endl;

  set<MyStruct> m1({MyStruct(2,3)});
  m1.insert({5,6}); // implicit conversion
  cout << "m1.count: " << m1.count(MyStruct(3,2)) << " [1]" << endl;
  cout << "m1.size: " << m1.size() << " [2]" << endl;

  set<MyOtherStruct> m2({MyOtherStruct(2,3)});
  set<MyOtherStruct> m3({MyOtherStruct(2,3), MyOtherStruct(4,2)});
  m3.insert(MyOtherStruct(6,7));
  cout << "m3.size: " << m3.size() << " [3]" << endl;
  cout << "m3: " << m3 << " [{(2,3),(4,2),(6,7)}]" << endl;

  set<YourStruct,YourCompare> y1;
  set<YourStruct,YourCompare> y2({YourStruct(5), YourStruct(11)});
  y1.insert(YourStruct(6));
  y1.erase(y1.begin());
  cout << "y1.size: " << y1.size() << " [0]" << endl;
  set<YourStruct,YourCompare> y3({YourStruct(5), YourStruct(11)});
  cout << (y2 == y3 ? "equal" : "different") << " [equal]" << endl;
  cout << (y2 != y3 ? "not equal" : "not different") << " [not different]" << endl;
  y1 = y2;
  y1.insert(7); // implicit conversion
  cout << (y2 < y1 ? "" : "not ") << "before [not before]" << endl;
  y2 = {YourStruct(45), YourStruct(33)};
  cout << "y2.size: " << y2.size() << " [2]" << endl;
  // Looping the old way.
  for (set<YourStruct,YourCompare>::const_iterator it = y2.cbegin();
       it != y2.cend(); ++it)
    cout << " " << *it;
  cout << " [33 45]" << endl;

  // Test union.
  set<int> I1({2,3,4});
  set<int> I2(set<int>({5,3,6})); // constructor that takes an std::set
  set<int> I3(I1 | I2);
  cout << "I3: " << I3 << " [{2,3,4,5,6}] I3.size: "
       << I3.size() << " [5]" << endl;
  set<int> I4(I2 & I1);
  cout << "I4.size: " << I4.size() << " [1]" << endl;
  set<int> I5(I2 - I1);
  cout << "I5.size: " << I5.size() << " [2]" << endl;
  set<int> I6(I2 ^ I1);
  cout << "I6.size: " << I6.size() << " [4]" << endl;

  I3.erase(11);
  I3.erase(3);
  cout << "I3.size: " << I3.size() << " [4]" << endl;

  I6 |= set<int>({65});
  cout << "I6: " << I6 << " [{2,4,5,6,65}]"
       << " I6.size: " << I6.size() << " [5]" << endl;
  I6 -= I3;
  cout << "I6: " << I6 << " [{65}]"  << endl;
  cout << (disjoint(I4, I5) ? "" : "not ") << "disjoint [disjoint]" << endl;
}

void phase2(void)
{
  set<int> s1({1,2,3});
  set<int> s2({3,4,5});
  set<int> s3 = s1 | s2;
  s2 |= s1;
  cout << s2 << " [{1,2,3,4,5}]" << endl;
  s2 &= s1;
  cout << s2 << " [{1,2,3}]" << endl;
  s2 -= s1;
  cout << s2 << " [{}]" << endl;
  s2 ^= s1;
  cout << s2 << " [{1,2,3}]" << endl;
  cout << "s1 " << (subsetOf(s1,s2) ? "" : "not ") << "subset of s2"
       << " [subset]" << endl;
  cout << "s1 " << (disjoint(s1,s2) ? "" : "not ")
       << "disjoint from s2" << " [not disjoint]" << endl;

  unordered_set<int> u1({1,2,3,4,5,6,7,8,9});
  cout << "u1: " << u1 << " [1..9 in some order]" << endl;

  vector<int> v1({1,3,5,6,4,2});
  cout << "v1: " << v1 << " [{1,3,5,6,4,2}]" << endl;

  set<int, greater<int> > s4({8,9,10});
  cout << "s4: " << s4 << " [{10,9,8}]" << endl;

  string st1("abc");
  cout << "st1 " << st1 << " [abc]" << endl;

  vector<char> v2({'a','b','c'});
  cout << "v2: " << v2 << " [{a,b,c}]" << endl;

  pair<int,int> p1(2,3);
  cout << "p1: " << p1 << " [(2,3)]" << endl;

  map<int,int> m1({{1,2}});
  cout << "m1: " << m1 << " [{(1,2)}]" << endl;

  tuple<int,char> t1 = make_tuple(18,'k');
  cout << "t1: " << t1 << " [<18,k>]" << endl;

  MyPair<int,char> mp1(12,'b');
  cout << "mp1: " << mp1 << " [(12,b)]" << endl;

  deque<double> dq1({1.1,1.2,1.3});
  cout << "dq1: " << dq1 << " [{1.1,1.2,1.3}]" << endl;

  list<string> ls1({"hello","world","!"});
  cout << "ls1: " << ls1 << " [{hello,world,!}]" << endl;

  forward_list<unsigned> fl1({2,3,5,7});
  cout << "fl1: " << fl1 << " [{2,3,5,7}]" << endl;

  array<int,3> ar1 = {{1,2,3}};
  cout << "ar1: " << ar1 << " [{1,2,3}]" << endl;

  int ar2[] = {4,5,6};
  cout << "ar2: " << ar2 << " [{4,5,6}]" << endl;

  char ar3[] = "zot!";
  cout << "ar3: " << ar3 << " [zot!]" << endl;
}
