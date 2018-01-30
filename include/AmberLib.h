/*
 *  Copyright (c) 2015 Vijay Ingalalli
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef AMBERLIB_H
#define AMBERLIB_H

#include <limits>
#include <numeric>
#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <algorithm>
#include <functional>
#include <sstream>
#include <set>
#include <string>
#include <cstdio>
#include <array>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#undef NDEBUG
#include <deque>
#include <limits>
#include <bitset>
#include <list>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <regex>


using namespace std;
using namespace std::placeholders;

template <typename T> // Decrementing order
vector<T> sortIndex(const vector<T> &v) {

  // initialize original index locations
  vector<T> idx(v.size());
  for (int i = 0; i != idx.size(); ++i) idx[i] = i;

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(), [&v](int i1, int i2) {return v[i1] > v[i2];});

  return idx;
}

template <typename T>
vector<int> sortIndexIncr(const vector<T> &v) {

  // initialize original index locations
  vector<int> idx(v.size());
  for (int i = 0; i != idx.size(); ++i) idx[i] = i;

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(), [&v](int i1, int i2) {return v[i1] < v[i2];});

  return idx;
}

template <typename T>
std::set<T> setDiff(const std::set<T> &a, const std::set<T> &b)
{
  std::set<T> c(a.size()+b.size());
  typename std::set<T>::iterator it;

  it=std::set_difference (a.begin(), a.end(), b.begin(), b.end(), c.begin());
  c.resize(it-c.begin());
  return c;
}

template <typename T>
std::vector<T> setMinus(std::vector<T> a, std::vector<T> b)
{
  std::vector<T> c(a.size()+b.size());
  typename std::vector<T>::iterator it;

  std::sort (a.begin(), a.end());
  std::sort (b.begin(), b.end());

  it=std::set_difference (a.begin(), a.end(), b.begin(), b.end(), c.begin());
  c.resize(it-c.begin());
  return c;
}


template <typename T>
std::vector<T> setUnion(std::vector<T> a, std::vector<T> b)
{
  std::vector<T> c(a.size()+b.size());
  typename std::vector<T>::iterator it;

  std::sort (a.begin(), a.end());
  std::sort (b.begin(), b.end());

  it=std::set_union (a.begin(), a.end(), b.begin(), b.end(), c.begin());
  c.resize(it-c.begin());
  return c;
}

template<typename S>
S union_sets(const S& s1, const S& s2)
{
  S result = s1;
  result.insert(s2.cbegin(), s2.cend());
  return result;
}

template <typename T>
std::vector<T> setIntersection(std::vector<T> a, std::vector<T> b)
{
  std::vector<T> c(a.size()+b.size());
  typename std::vector<T>::iterator it;

  std::sort (a.begin(), a.end());
  std::sort (b.begin(), b.end());

  it=std::set_intersection (a.begin(), a.end(), b.begin(), b.end(), c.begin());
  c.resize(it-c.begin());
  return c;
}

template <typename T>
std::vector<T> vecIntersection(std::vector<T> a, std::vector<T> b) // For higher speed, maintain a > b |
{
  unordered_set<T> s(a.begin(), a.end());
  std::vector<T> c;
  for (size_t i = 0; i < b.size(); ++i){
    if (s.find(b[i]) != s.end())
      c.push_back(b[i]);
  }
  return c;
}

template <typename T>
bool isSubset(const std::vector<T> &A, const std::vector<T> &B) // Used to ckeck if "B" is the SUBSET of "A" (not vice versa)
{
  std::sort(A.begin(), A.end());
  std::sort(B.begin(), B.end());
  return std::includes(A.begin(), A.end(), B.begin(), B.end());
}

template <typename T>
bool is_unique(const std::vector<T> &vec)
{
  std::sort(vec.begin(), vec.end());
  return std::unique(vec.begin(), vec.end()) == vec.end();
}


template <typename T>
std::vector<T> makeUnique(std::vector<T> vec)
{
  std::unordered_set<T> s(vec.begin(), vec.end());
  vec.assign(s.begin(), s.end());
  return vec;
}

template <typename T>
bool file_exists(const T& name)
{
  ifstream file(name);
  if(!file)            // If the file was not found, then file is 0, i.e. !file=1 or true.
    return false;    // The file was not found.
  else                 // If the file was found, then file is non-0.
    return true;     // The file was found.
}


typedef std::map< std::pair<int, int>, std::set<int>> EdgeLabelMap;
typedef std::vector<std::vector<int>> Vector2D;
typedef std::vector<std::string> VecOfStr;
typedef std::vector<std::set<int>> VecOfSet;
typedef std::vector<Vector2D> Vector3D;
typedef std::vector<std::vector<std::set<int>>> EdgeLabel;
typedef std::pair<int, int> IntPair;
typedef std::pair<std::string, std::string> StrPair;
typedef std::unordered_map<int, std::vector<int>> AttMap;
typedef std::map<std::set<int>, int> MapSet;
typedef std::vector<std::unordered_set<int>> VecUset;
typedef std::pair<int, std::set<int>> IntSetPair;
typedef std::map<int, std::vector<IntSetPair>> UriInfo; // <core-id, <vector<uri_id, multi_edges>>> |

struct SparqlMapping{
  UriInfo uriData; // <core-id, <vector<uri_id, multi_edges>>> |
  std::map<std::pair<int,int>, std::set<int>> edges;
  std::map<std::string, int> nodes;
  std::map<std::string, int> vLabels;
  std::map<int, std::set<int>> nodeAtt;
};

#endif // AMBERLIB_H
