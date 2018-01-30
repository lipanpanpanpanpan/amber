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

#ifndef SOLUTIONFINDER_H
#define SOLUTIONFINDER_H

#include "AmberLib.h"
#include "IndexManager.h"
#include "Trie.h"
#include "SparqlParser.h"
#include "Matcher.h"


class SolutionExplorer
{
  public:
    SolutionExplorer();
    virtual ~SolutionExplorer();
    void findSolutions(IndexType& index, const MapSet& edgeFrequency, const SparqlMapping& sparqlMap, QueryParameter& queryGraphInfo, QueryParameter& decomposedQuery);
  private:
    void chooseFrontier(const std::vector<int>& already_m, const Vector2D& queryAdjacencyList, std::vector<int>& frontier);
    void orderPrimitively(const EdgeLabel& queryNeighbourSign, std::vector<int>& nEdges, std::vector<int>& nEdgeIndex);
    void orderComplexQueryNodes(const QueryParameter& decomposedQuery, QueryParameter& queryGraphInfo);
    void getQueryEdgeFrequency(const EdgeLabel& neighbourSign, const std::map<std::set<int>, int>& dataEdgeFrequency, std::vector<int>& edgeFreq);
    void orderQueryNodes(const std::vector<int>& sign_rank, const std::vector<int>& edgeFreq, QueryParameter& queryGraph);
    void getUriSolutions(const IntSetPair& uriEdge, IndexType& index, std::vector<int>& uriSolutions);
    void getConstrainedSolutions(const SparqlMapping& sparqlMap, const QueryParameter& queryGraphInfo, IndexType& index, InitialSolutions& constrainedSols);
  protected:
};

#endif //SOLUTIONFINDER_H
