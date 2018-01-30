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

#include "QueryHandler.h"

QueryHandler::QueryHandler()
{
  //ctor
}

QueryHandler::~QueryHandler()
{
  //dtor
}

void find2loops(QueryParameter& query, const int& nPred)
{
  for (auto it = query.eLabelMap.begin(); it != query.eLabelMap.end(); ++it) {
    bool fwd = false, rev = false;
    for (auto it_e = it->second.begin(); it_e != it->second.end(); ++it_e) {
      if (*it_e <= nPred)
        fwd = true;
      else
        rev = true;
    }
    if (fwd && rev) {
      query.loopEdges.insert(make_pair(it->first.first, it->first.second)); // Insert only the forward, since "eLabelMap" contains both the directions |
    }
  }
}

void  QueryHandler::getQueryContents(const SparqlMapping& sparqlMap, const int& nPred, const int& nVLabels, QueryParameter& queryGraphInfo)
{
  queryGraphInfo.nNodes = sparqlMap.nodes.size();
  queryGraphInfo.attributes.resize(queryGraphInfo.nNodes);
  for (auto it = sparqlMap.nodes.begin(); it != sparqlMap.nodes.end(); ++it) { // Read nodes |
    queryGraphInfo.nodes.push_back(it->second); // collect query nodes | NOT NEEDED
    auto it_a = sparqlMap.nodeAtt.find(it->second);
    if (it_a != sparqlMap.nodeAtt.end())
      queryGraphInfo.attributes[it->second] = it_a->second;
    else{
      std::set<int> tmp;
      tmp.insert(-1);
      queryGraphInfo.attributes[it->second] = tmp;
    }
  }

  queryGraphInfo.neighbourSign.resize(queryGraphInfo.nNodes);
  queryGraphInfo.adjacencyList.resize(queryGraphInfo.nNodes);

  for (auto it = sparqlMap.edges.begin(); it != sparqlMap.edges.end(); ++it) { // Read edges |
    std::set<int> temp, temp_neg;
    temp = it->second;
    for(auto it_l = temp.begin(); it_l != temp.end(); ++it_l)
      temp_neg.insert(nPred+(*it_l));
    queryGraphInfo.neighbourSign[it->first.first].push_back(temp_neg);
    queryGraphInfo.neighbourSign[it->first.second].push_back(temp);

     // Here we can handle 2-loops |
    auto it_f = queryGraphInfo.eLabelMap.find(std::make_pair(it->first.first, it->first.second));
    if (it_f == queryGraphInfo.eLabelMap.end())
      queryGraphInfo.eLabelMap.insert(std::make_pair(std::make_pair(it->first.first, it->first.second), temp));
    else
      it_f->second.insert(temp.begin(), temp.end());

    auto it_r = queryGraphInfo.eLabelMap.find(std::make_pair(it->first.second, it->first.first));
    if (it_r == queryGraphInfo.eLabelMap.end())
      queryGraphInfo.eLabelMap.insert(std::make_pair(std::make_pair(it->first.second, it->first.first), temp_neg));
    else
      it_r->second.insert(temp_neg.begin(), temp_neg.end());

    queryGraphInfo.edges[0].push_back(it->first.first);
    queryGraphInfo.edges[1].push_back(it->first.second);
  }
  for(int k = 0; k < queryGraphInfo.edges[0].size(); ++k) {
    // Self loop information is not st0red in adjacency list |
    if (queryGraphInfo.edges[0][k] != queryGraphInfo.edges[1][k]) {
      queryGraphInfo.adjacencyList[queryGraphInfo.edges[0][k]].push_back(queryGraphInfo.edges[1][k]);
      queryGraphInfo.adjacencyList[queryGraphInfo.edges[1][k]].push_back(queryGraphInfo.edges[0][k]);
    }
  }
  /// Collect the self loops
  for (size_t i = 0; i < queryGraphInfo.edges[0].size(); ++i) {
    if (queryGraphInfo.edges[0][i] == queryGraphInfo.edges[1][i]) {
      queryGraphInfo.loopNodes.insert(queryGraphInfo.edges[0][i]);
    }
  }
  /// Sort each signature within itself wrt subsignature size in  increasing order
  for(size_t i = 0; i < queryGraphInfo.nNodes; ++i) {
    std::vector<int> nNeighbours;
    VecOfSet srted_sign;
    for(auto it = queryGraphInfo.neighbourSign[i].begin(); it!=queryGraphInfo.neighbourSign[i].end(); ++it)
      nNeighbours.push_back((*it).size());
    std::vector<int> sortedNeighIndex = sortIndexIncr(nNeighbours); // sort each signature within itself wrt subsignature size |
    for(size_t j = 0; j < nNeighbours.size(); ++j)
      srted_sign.push_back(queryGraphInfo.neighbourSign[i][sortedNeighIndex[j]]);
    queryGraphInfo.neighbourSign[i] = srted_sign;
  }
  find2loops(queryGraphInfo, nPred);
}

void QueryHandler::decomposeQuery(const SparqlMapping& sparqlMap, const int nPred, QueryParameter& queryGraphInfo, QueryParameter& decomposedQuery)
{
  /// Collect core nodes
  int core_nodes = 0;
  for (size_t m = 0; m < queryGraphInfo.adjacencyList.size(); ++m) {
    if (queryGraphInfo.adjacencyList[m].size() > 1)
      ++core_nodes;
  }

  /// Query transformation happens here
  std::set<std::pair<int,int>> satEdges; // <core-satellite> : edges corresponding to satellite nodes and core nodes |
  std::set<int> satLoops; // satellite nodes that have self loops |
  if (core_nodes > 1){ // If not a pure star query |
    /// Collect all the satellite nodes and their core nodes.
    std::set<int> nodesParsed;
    for (size_t q = 0; q < queryGraphInfo.nNodes; ++q) {
      if (queryGraphInfo.adjacencyList[q].size() == 1) {// A satellite node it is |
        nodesParsed.insert(q); //satellite node added |
        nodesParsed.insert(queryGraphInfo.adjacencyList[q][0]); // corresponding core node added |
        satEdges.insert(make_pair(queryGraphInfo.adjacencyList[q][0], q));
        satEdges.insert(make_pair(q, queryGraphInfo.adjacencyList[q][0]));
        auto it = decomposedQuery.satellite.find(queryGraphInfo.adjacencyList[q][0]);
        if (it == decomposedQuery.satellite.end()) {
          std::set<int> sat_n;
          sat_n.insert(q);
          decomposedQuery.satellite.insert(make_pair(queryGraphInfo.adjacencyList[q][0], sat_n));
        }
        else
          it->second.insert(q);
      if (queryGraphInfo.loopNodes.find(q) != queryGraphInfo.loopNodes.end()) // Check if the sat_node has a self loop |
            satLoops.insert(q);
      }
    }
    if (nodesParsed.size() != queryGraphInfo.nNodes) { // => There are core nodes that have no satellite nodes
      for(size_t s = 0; s < queryGraphInfo.nNodes; ++s) {
        if (nodesParsed.find(s) == nodesParsed.end()) {
          std::set<int> sat_n; // empty satellite node set |
          decomposedQuery.satellite.insert(make_pair(s, sat_n));
        }
      }
    }
    /// Check, if (nodesParsed.empty()), makes it a complex type.
    if (decomposedQuery.satellite.empty())
      decomposedQuery.queryType = "no-star";
    else
      decomposedQuery.queryType = "complex";
  }
  else
    decomposedQuery.queryType = "star";
  queryGraphInfo.queryType = decomposedQuery.queryType;
  if (!decomposedQuery.satellite.empty()) { // There is at least one satellite node
    EdgeLabelMap eLabelMap;
    eLabelMap  = queryGraphInfo.eLabelMap;
    for (auto it = satEdges.begin(); it != satEdges.end(); ++it) {
      auto it_q = eLabelMap.find(*it);
      eLabelMap.erase(it_q);
    }
  for (auto it = satLoops.begin(); it != satLoops.end(); ++it)
    eLabelMap.erase(make_pair((*it), (*it)));

    int k = 0;
    std::set<int> newNodes;
    for (auto it = eLabelMap.begin(); it != eLabelMap.end(); ++it) {
      if (decomposedQuery.origDecompMap.find(it->first.first) == decomposedQuery.origDecompMap.end()) {
        decomposedQuery.origDecompMap.insert(make_pair(it->first.first, k));
        newNodes.insert(k);
        ++k;
      }
      if (decomposedQuery.origDecompMap.find(it->first.second) == decomposedQuery.origDecompMap.end()) {
        decomposedQuery.origDecompMap.insert(make_pair(it->first.second, k));
        newNodes.insert(k);
        ++k;
      }
    }
    decomposedQuery.nNodes = newNodes.size();
    for (auto it = eLabelMap.begin(); it != eLabelMap.end(); ++it) {
      std::pair<int, int> pair_map = make_pair(decomposedQuery.origDecompMap.find(it->first.first)->second, decomposedQuery.origDecompMap.find(it->first.second)->second);
      decomposedQuery.eLabelMap.insert(make_pair(pair_map, it->second));
    }

    decomposedQuery.attributes.resize(decomposedQuery.nNodes);
    int l = 0;
    for (auto it = decomposedQuery.origDecompMap.begin(); it != decomposedQuery.origDecompMap.end(); ++it) { // Read nodes that corresponds to the original query nodes |
      auto it_a = sparqlMap.nodeAtt.find(it->first);
      if (it_a != sparqlMap.nodeAtt.end())
        decomposedQuery.attributes[l] = it_a->second;
      else{
        std::set<int> tmp;
        tmp.insert(-1);
        decomposedQuery.attributes[l] = tmp;
      }
      ++l;
    }

    decomposedQuery.loopNodes = queryGraphInfo.loopNodes;
    for (auto it = satLoops.begin(); it != satLoops.end(); ++it) {
      decomposedQuery.loopNodes.erase(*it); // Retain only the core loop nodes |
    }

    decomposedQuery.neighbourSign.resize(decomposedQuery.nNodes);
    for (auto it = decomposedQuery.eLabelMap.begin(); it != decomposedQuery.eLabelMap.end(); ++it) { // Read edges |
      std::set<int> temp, temp_neg;
      temp = it->second;
      for(auto it_l = temp.begin(); it_l != temp.end(); ++it_l)
        temp_neg.insert(nPred+(*it_l));
      decomposedQuery.neighbourSign[it->first.first].push_back(temp_neg);
      decomposedQuery.neighbourSign[it->first.second].push_back(temp);
    }

    decomposedQuery.adjacencyList.resize(decomposedQuery.nNodes);
    std::set<std::pair<int, int>> src_dst, src_dst_map;
    for(int k = 0; k < queryGraphInfo.edges[0].size(); ++k)
      src_dst.insert(make_pair(queryGraphInfo.edges[0][k], queryGraphInfo.edges[1][k]));
    for (auto it = satEdges.begin(); it != satEdges.end(); ++it) {
      auto it_q = src_dst.find(*it);
      if (it_q != src_dst.end())
        src_dst.erase(it_q);
    }

    for (auto it = src_dst.begin(); it != src_dst.end(); ++it)
      src_dst_map.insert(make_pair(decomposedQuery.origDecompMap.find((*it).first)->second, decomposedQuery.origDecompMap.find((*it).second)->second));


    for (auto it = decomposedQuery.origDecompMap.begin(); it != decomposedQuery.origDecompMap.end(); ++it)
      decomposedQuery.decompOrigMap.insert(make_pair(it->second, it->first));

    for (auto it = src_dst_map.begin(); it != src_dst_map.end(); ++it) {
      // Self loop information is not stored in adjacency list |
      if ((*it).first != (*it).second) {
        decomposedQuery.adjacencyList[(*it).first].push_back((*it).second);
        decomposedQuery.adjacencyList[(*it).second].push_back((*it).first);
      }
    }
  }
  find2loops(decomposedQuery, nPred);

}
