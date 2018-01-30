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

#include "SolutionExplorer.h"

SolutionExplorer::SolutionExplorer()
{
  //ctor
}

SolutionExplorer::~SolutionExplorer()
{
  //dtor
}

void SolutionExplorer::chooseFrontier(const std::vector<int>& already_m, const Vector2D& queryAdjacencyList, std::vector<int>& frontier)
{
  for (size_t i = 0; i < already_m.size(); ++i) {
    if (i == 0)
      frontier = queryAdjacencyList[already_m[i]];
    else
      frontier = setUnion(frontier, queryAdjacencyList[already_m[i]]);
  }
  frontier = setMinus(frontier, already_m);
  return;
}


void getEntireQueryOrder(QueryParameter& queryGraphInfo, QueryParameter& decomposedQuery)
{
  for(size_t i = 0; i < decomposedQuery.orderedNodes.size(); ++i){
    int coreV = decomposedQuery.decompOrigMap.find(decomposedQuery.orderedNodes[i])->second;
    std::vector<int> satV;
    auto it = decomposedQuery.satellite.find(coreV);
    if (!it->second.empty())
      std::copy(it->second.begin(), it->second.end(), std::back_inserter(satV));
    queryGraphInfo.solutions.complexQueryOrder.push_back(std::make_pair(coreV, satV));
  }
}



void SolutionExplorer::orderPrimitively(const EdgeLabel& queryNeighbourSign, std::vector<int>& nEdges, std::vector<int>& nEdgeIndex)
{
  for (size_t i = 0; i < queryNeighbourSign.size(); ++i) {
    int allDim = 0;
    for (int j = 0; j < queryNeighbourSign[i].size(); ++j)
      allDim += queryNeighbourSign[i][j].size();
    nEdges.push_back(allDim);
  }
  nEdgeIndex = sortIndex(nEdges);
}

void SolutionExplorer::orderComplexQueryNodes(const QueryParameter& decomposedQuery, QueryParameter& queryGraphInfo)
{
  std::vector<int> satSize, satSizeInd; // Sort the star queries with decreasing size |
  if (decomposedQuery.satellite.size() == 1) {
    queryGraphInfo.orderedNodes.push_back(decomposedQuery.satellite.begin()->first);
    for (auto it = decomposedQuery.satellite.begin()->second.begin(); it != decomposedQuery.satellite.begin()->second.end(); ++it)
      queryGraphInfo.orderedNodes.push_back(*it);
  }
  else{
    for (auto it = decomposedQuery.satellite.begin(); it != decomposedQuery.satellite.end(); ++it)
      satSize.push_back(it->second.size());
    satSizeInd = sortIndex(satSize);
    auto it = decomposedQuery.satellite.begin();
    std::advance(it, satSizeInd[0]);
    queryGraphInfo.orderedNodes.push_back(it->first);
    for(auto it_1 = it->second.begin(); it_1 != it->second.end(); ++it_1)
      queryGraphInfo.orderedNodes.push_back(*it_1);
  }
}

void SolutionExplorer::getQueryEdgeFrequency(const EdgeLabel& neighbourSign, const std::map<std::set<int>, int>& dataEdgeFrequency, std::vector<int>& edgeFreq)
{
  for (size_t i = 0; i < edgeFreq.size(); ++i) {
    edgeFreq[i]= 0;
    for (size_t j = 0; j < neighbourSign[i].size(); ++j) {
      std::vector<int> m_s;
      std::copy(neighbourSign[i][j].begin(), neighbourSign[i][j].end(), std::back_inserter(m_s));
      for (size_t e = 0; e < m_s.size(); ++e)
        m_s[e] -= 1;
      std::set<int> orig_edges(m_s.begin(), m_s.end());
      edgeFreq[i]+=dataEdgeFrequency.find(orig_edges)->second;
    }
    edgeFreq[i] = edgeFreq[i]/neighbourSign[i].size();
  }
}

void SolutionExplorer::orderQueryNodes(const std::vector<int>& sign_rank, const std::vector<int>& edgeFreq, QueryParameter& queryGraph)
{
  /// Using 3 Rank strategy
  while (queryGraph.orderedNodes.size() != queryGraph.nNodes) {
    std::vector<int> frontier;
    chooseFrontier(queryGraph.orderedNodes, queryGraph.adjacencyList, frontier);
    if (frontier.size() == 1)
      queryGraph.orderedNodes.push_back(frontier[0]);
    else {
      std::vector<int> q_rank(frontier.size()); //, rank_1(frontier.size()), rank_2(frontier.size()), rank_3(frontier.size());
      int rank_1 = 0, rank_2 = 0, rank_3 = 0;

      // Rank vector for edge frequency |
      std::vector<int> freq_frontier;
      for (size_t k = 0; k < frontier.size(); ++k)
        freq_frontier.push_back(edgeFreq[frontier[k]]);
      std::vector<int> freq_ind = sortIndex(freq_frontier); //; Decreasing order
      std::vector<int> rank_4(frontier.size());
      for (size_t k = 0; k < frontier.size(); ++k)
        rank_4[freq_ind[k]] = k;

      for (size_t i = 0; i < frontier.size(); ++i) {
        // Rank-1
        rank_1 = setIntersection(queryGraph.orderedNodes, queryGraph.adjacencyList[frontier[i]]).size();
        // Rank-2
        for (size_t j = 0; j < queryGraph.orderedNodes.size(); ++j) {
          std::vector<int> q_s = queryGraph.orderedNodes;
          q_s.erase(q_s.begin()+j);  // remove the vertex itself |
          std::vector<int> unmatched_nbr = setMinus(queryGraph.adjacencyList[queryGraph.orderedNodes[j]], q_s);
          if (!unmatched_nbr.empty()) {
            if (!setIntersection(unmatched_nbr, queryGraph.adjacencyList[frontier[i]]).empty())
              ++rank_2;
          }
        }
        // Rank 3
        std::set<int> q_s_set(queryGraph.orderedNodes.begin(), queryGraph.orderedNodes.end());
        std::set<int> front_set(frontier.begin(), frontier.end());
        for (size_t j = 0; j < queryGraph.adjacencyList[frontier[i]].size(); ++j) {
          int u_nbr = queryGraph.adjacencyList[frontier[i]][j];
          if ( q_s_set.find(u_nbr) ==  q_s_set.end() && front_set.find(u_nbr) ==  front_set.end() )
            ++rank_3;
        }
        // Assign priorities |
        q_rank[i] =  rank_1 * 1000 + rank_4[i] * 10 + rank_2 * 1 + rank_3 * 100;
      }
      std::vector<int> q_r_sorted = sortIndex(q_rank);
      if (q_r_sorted.size() > 1) {
        if (q_rank[q_r_sorted[0]] != q_rank[q_r_sorted[1]])
          queryGraph.orderedNodes.push_back(frontier[q_r_sorted[0]]);
        else {
          std::vector<int> same_rank_1;
          same_rank_1.push_back(frontier[q_r_sorted[0]]);
          int i = 0;
          while (i < frontier.size()-1){
            if (q_rank[q_r_sorted[i]] == q_rank[q_r_sorted[i+1]])
              same_rank_1.push_back(frontier[q_r_sorted[i+1]]);
            else
              break;
            ++i;
          }
          std::vector<int> rank_2(same_rank_1.size());
          for (size_t i = 0; i < same_rank_1.size(); ++i)
            rank_2[i] = sign_rank[same_rank_1[i]];
          std::vector<int> srt_rank_2 = sortIndex(rank_2);
          queryGraph.orderedNodes.push_back(same_rank_1[srt_rank_2[0]]);
        }
      }
      else
        queryGraph.orderedNodes.push_back(frontier[q_r_sorted[0]]);
    }
  }
}


/*
void findInitialVertex(const EdgeLabel& queryNeighbourSign, const std::map<int, std::unordered_set<int>>& att_solns, const std::map<int, std::unordered_set<int>>& uri_solns, const std::map<int, std::unordered_set<int>>& loop_solns, std::vector<int>& n_of_edges, std::vector<int>& ordered_index, int& initialVertex)
{

    for (size_t i = 0; i < queryNeighbourSign.size(); ++i) {
        int allDim = 0;
        for (int j = 0; j < queryNeighbourSign[i].size(); ++j)
            allDim += queryNeighbourSign[i][j].size();
        n_of_edges.push_back(allDim);
    }
    ordered_index = sortIndex(n_of_edges); //; Decreasing order

    if (!att_solns.empty() || !uri_solns.empty() || !loop_solns.empty()) {
        std::vector<int> q_nodes, q_node_cand_size, srt_index;
        for (auto it = att_solns.begin(); it != att_solns.end(); ++it) {
            q_nodes.push_back(it->first);
            q_node_cand_size.push_back(it->second.size());
        }
        for (auto it = uri_solns.begin(); it != uri_solns.end(); ++it) {
            q_nodes.push_back(it->first);
            q_node_cand_size.push_back(it->second.size());
        }
        for (auto it = loop_solns.begin(); it != loop_solns.end(); ++it) {
            q_nodes.push_back(it->first);
            q_node_cand_size.push_back(it->second.size());
        }
//cout << "q_node_cand_size: " << q_nodes.size() << endl;
        srt_index = sortIndexIncr(q_node_cand_size);
        initialVertex = q_nodes[srt_index[0]];
    }
    else {
        initialVertex = ordered_index[0];
    }
}
*/

void SolutionExplorer::getUriSolutions(const IntSetPair& uriEdge, IndexType& index, std::vector<int>& uriSolutions)
{
  IndexManager rdfIndex;
  bool loop_2 = false, fwd = false, rev = false;
  for (auto it = uriEdge.second.begin(); it != uriEdge.second.end(); ++it) {
    if (*it <= index.nPred)
      fwd = true;
    else
      rev = true;
  }
  if (fwd && rev)
    loop_2 = true;

  if (!loop_2) { // uriEdge.second is not a 2-loop |
    if ((*uriEdge.second.begin()) <= index.nPred) // checks if multiedge is incoming or outgoing |
      rdfIndex.queryNeighTrie(index.neighborTrieIn[uriEdge.first], uriEdge.second, uriSolutions);
    else
      rdfIndex.queryNeighTrie(index.neighborTrieOut[uriEdge.first], uriEdge.second, uriSolutions);
  }
  else { // uriEdge.second forms a 2-loop |
    std::set<int> in_e_label, out_e_label;
    for (auto it = uriEdge.second.begin(); it != uriEdge.second.end(); ++it) {
      if (*it <= index.nPred)
        in_e_label.insert(*it);
      else
        out_e_label.insert(*it);
    }
    rdfIndex.queryNeighTrie(index.neighborTrieIn[uriEdge.first], in_e_label, uriSolutions);
    std::vector<int> nbr_lbl_matches;
    rdfIndex.queryNeighTrie(index.neighborTrieOut[uriEdge.first], out_e_label, nbr_lbl_matches);
    uriSolutions = vecIntersection(nbr_lbl_matches, uriSolutions);
  }
}


void SolutionExplorer::getConstrainedSolutions(const SparqlMapping& sparqlMap, const QueryParameter& queryGraphInfo, IndexType& index, InitialSolutions& constrainedSols)
{
  /// Fetching the candidates from vertex attribute indexing
  Vector2D vAtt(queryGraphInfo.attributes.size());
  VecUset vAttSet(queryGraphInfo.attributes.size());

  IndexManager rdfIndex;
  rdfIndex.queryAttHash(queryGraphInfo.attributes, index.attributeHash, vAtt, vAttSet);
  /// Fetch the candidates for the queries with attributes
  for (size_t i = 0; i < vAttSet.size(); ++i) {
    if (!vAttSet[i].empty())
      constrainedSols.attSolns.insert(make_pair(i, vAttSet[i]));
  }

  /// Fetch the candidates for the queries with self loops
  for (auto it = queryGraphInfo.loopNodes.begin(); it != queryGraphInfo.loopNodes.end(); ++it) { // fetch solutions for all the loop nodes |
    std::vector<int> nbr_lbl_matches;
    auto query_it = queryGraphInfo.eLabelMap.find(make_pair((*it), (*it)));  // an edge that corresponds to self loop |
    rdfIndex.queryNeighTrie(index.neighborTrieLoop, query_it->second, nbr_lbl_matches);
    std::unordered_set<int> solns(nbr_lbl_matches.begin(), nbr_lbl_matches.end());
    constrainedSols.loopSolns.insert(make_pair((*it), solns));
  }

  /// Fetch the candidates for query vertexes with known URI
  for (auto it = sparqlMap.uriData.begin(); it != sparqlMap.uriData.end(); ++it) {
    std::vector<int> uri_solns_single_core;
    for (size_t k = 0; k < it->second.size(); ++k) { // More than one URI might exist for a core node |
      std::vector<int> uriSolutions;
      getUriSolutions(it->second[k], index, uriSolutions);
      if (k == 0)
        uri_solns_single_core = uriSolutions;
      else
        uri_solns_single_core = vecIntersection(uriSolutions, uri_solns_single_core);
    }
    std::unordered_set<int> solns(uri_solns_single_core.begin(), uri_solns_single_core.end());
    constrainedSols.uriSolns.insert(make_pair(it->first, solns));
  }

  /// Find the query vertices that have repeated multi-edges.
  for (size_t k = 0; k < queryGraphInfo.neighbourSign.size(); ++k) {
    std::map<std::set<int>, int> edges_repeated; // <edge labels, repetition count>
    for (size_t l = 0; l < queryGraphInfo.neighbourSign[k].size(); ++l) {
      auto it = edges_repeated.find(queryGraphInfo.neighbourSign[k][l]);
      if (it == edges_repeated.end())
        edges_repeated.insert(make_pair(queryGraphInfo.neighbourSign[k][l], 1));
      else
        it->second+=1;
    }
    for(auto it = edges_repeated.begin(); it != edges_repeated.end(); ++it) {
      if (it->second > 1) { // only if a multi-edge is repeated, for a particular query vertex 'k' |
        std::set<int> rev_labels; // store the reverse labels, as they are always fetched from the other end of the edge |
        for(auto it_l = it->first.begin(); it_l != it->first.end(); ++it_l) {
          if (*it_l <= index.nPred)
            rev_labels.insert( (*it_l) + index.nPred );
          else
            rev_labels.insert( (*it_l) - index.nPred );
        }
        constrainedSols.repEdges.insert(make_pair(k, rev_labels));
      }
    }
  }
}


void SolutionExplorer::findSolutions(IndexType& index, const MapSet& edgeFrequency, const SparqlMapping& sparqlMap, QueryParameter& queryGraphInfo, QueryParameter& decomposedQuery)
{
  int coreNodes = decomposedQuery.satellite.size();
  std::vector<int> initialMatches;

  InitialSolutions constrainedSols;
  getConstrainedSolutions(sparqlMap, queryGraphInfo, index, constrainedSols);

  if (coreNodes == 1 || coreNodes == 0) { // It's a star pattern | coreNodes == 0, when only one edge is present |
    std::vector<int> nEdges;
    orderPrimitively(queryGraphInfo.neighbourSign, nEdges, queryGraphInfo.orderedNodes); // Since it is a star query |

    Matcher matcher;
    matcher.findInitialMatches(queryGraphInfo, constrainedSols, index, initialMatches);

    Vector3D initialStarMatches; // all solutions except for the initial query vertex |
    std::vector<int> initialCoreMatches; // exact solutions for the initial query vertex |
    Vector2D nStarEmb(queryGraphInfo.nNodes-1); // no of embeddings for each distinct query node |
    unsigned long nEmb = 0;
    matcher.findStarMatches(index, initialMatches, constrainedSols, queryGraphInfo, initialStarMatches, initialCoreMatches, nStarEmb, nEmb);
    queryGraphInfo.nSolutions = nEmb;
    queryGraphInfo.solutions.starSolutions = std::make_pair(initialCoreMatches, initialStarMatches);
  }
  else if (decomposedQuery.satellite.empty()) { // No star pattern found in the query graph | Thus, no query decomposition.
    std::vector<int> nEdges, nEdgeIndex, queryEdgeFreq(queryGraphInfo.nNodes);
    orderPrimitively(queryGraphInfo.neighbourSign, nEdges, nEdgeIndex);
    // findInitialVertex(queryGraphInfo.neighbourSign, constrainedSols.attSolns, constrainedSols.uriSolns, constrainedSols.loopSolns, nEdges, nEdgeIndex, initialVertex);
    queryGraphInfo.orderedNodes.push_back(nEdgeIndex[0]);
    getQueryEdgeFrequency(queryGraphInfo.neighbourSign, edgeFrequency, queryEdgeFreq);
    orderQueryNodes(nEdges, queryEdgeFreq, queryGraphInfo);

    Matcher matcher;
    matcher.findInitialMatches(queryGraphInfo, constrainedSols, index, initialMatches);
    matcher.findAllMatches(initialMatches, constrainedSols, index, queryGraphInfo);
  }
  else if (!decomposedQuery.satellite.empty()) {
    orderComplexQueryNodes(decomposedQuery, queryGraphInfo);

    Matcher matcher;
    matcher.findInitialMatches(queryGraphInfo, constrainedSols, index, initialMatches);

    Vector3D initialStarMatches; // all solutions except for the initial query vertex |
    std::vector<int> initialCoreMatches; // exact solutions for the initial query vertex |
    Vector2D nStarEmb(queryGraphInfo.orderedNodes.size()-1); // no. of embeddings for each distinct query node | initialStarMatches[0].size() =  initialMatchesExact.size() |
    unsigned long nEmb = 0;
    matcher.findStarMatches(index, initialMatches, constrainedSols, queryGraphInfo, initialStarMatches, initialCoreMatches, nStarEmb, nEmb);

    std::vector<int> nEdges, nEdgeIndex, queryEdgeFreq(decomposedQuery.nNodes);
    int coreInitialVertex = decomposedQuery.origDecompMap.find(queryGraphInfo.orderedNodes[0])->second;
    decomposedQuery.orderedNodes.push_back(coreInitialVertex);
    orderPrimitively(decomposedQuery.neighbourSign, nEdges, nEdgeIndex);
    getQueryEdgeFrequency(queryGraphInfo.neighbourSign, edgeFrequency, queryEdgeFreq);
    orderQueryNodes(nEdges, queryEdgeFreq, decomposedQuery);

    getEntireQueryOrder(queryGraphInfo, decomposedQuery);

    matcher.findComplexMatches(initialCoreMatches, initialStarMatches, nStarEmb, constrainedSols, index, decomposedQuery, queryGraphInfo);
  }
}
