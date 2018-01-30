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

#ifndef MATCHER_H
#define MATCHER_H

#include "AmberLib.h"
#include "IndexManager.h"
#include "Trie.h"
#include "SparqlParser.h"

const int MAX_EMB = 1000000; // The maximum number of embeddings the algorithm is allowed to output
const int MAX_ALLOWED_TIME = 600000000; // 600 seconds (60*CLOCKS_PER_SEC)

typedef std::unordered_map<int, std::unordered_set<int>> UMapSet;
typedef std::vector<std::unordered_set<int>> VecUSet;

struct InitialSolutions
{
  std::map<int, std::unordered_set<int>> attSolns;
  std::map<int, std::unordered_set<int>> loopSolns; // maintains the original node-id |
  std::map<int, std::unordered_set<int>> uriSolns;
  std::map<int, std::set<int>> repEdges; // <query node, edge labels>
};

class Matcher
{
  public:
    Matcher();
    virtual ~Matcher();
    void findStarMatches(IndexType& index, const std::vector<int>& initialMatches, InitialSolutions& constrainedSols, QueryParameter& queryGraphInfo, Vector3D& satMatches, std::vector<int>& coreMatches, Vector2D& nStarEmb, unsigned long& nEmb);
    void findInitialMatches(const QueryParameter& queryGraphInfo, InitialSolutions& constrainedSols, IndexType& index, std::vector<int>& initialMatches);
    void findAllMatches(const std::vector<int>& initialMatches, InitialSolutions&  constrainedSols, IndexType& index, QueryParameter& queryGraphInfo);
    void findComplexMatches(const std::vector<int>& initialMatches, const Vector3D& initialStarMatches, const Vector2D& nStarEmb, InitialSolutions& constrainedSols, IndexType& index, QueryParameter& decomposedQuery, QueryParameter& queryGraphInfo);
  protected:
  private:
};

#endif // MATCHER_H
