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

#include "Matcher.h"

Matcher::Matcher()
{
  //ctor
}

Matcher::~Matcher()
{
  //dtor
}

void Matcher::findStarMatches(IndexType& index, const std::vector<int>& initialMatches, InitialSolutions& constrainedSols, QueryParameter& queryGraphInfo, Vector3D& satMatches, std::vector<int>& coreMatches, Vector2D& nStarEmb, unsigned long& nEmb)
{
  IndexManager rdfIndex;
  int initialVertex = queryGraphInfo.orderedNodes[0];
  int nStarNodes = queryGraphInfo.orderedNodes.size();
  for (int iV = 0; iV < initialMatches.size(); ++iV) {
    Vector2D partialEmb(nStarNodes-1);
    int pV = 1; // This accounts to initialVertex |
    int nEmbIv = 1; // Assuming that the initial vertex leads to an embedding |
    while (pV < nStarNodes) {
      std::pair<int, int> mEdge = make_pair(initialVertex, queryGraphInfo.orderedNodes[pV]);
      auto qIt = queryGraphInfo.eLabelMap.find(mEdge);
      /// Using Pointers and supersetQueryTest
      std::vector<Node*> matchPointer;
      if (queryGraphInfo.loopEdges.find(mEdge) == queryGraphInfo.loopEdges.end()) { // mEdge is not a 2-loop |
        if ((*qIt->second.begin()) <= index.nPred) // checks if multiedge is incoming or outgoing |
          rdfIndex.queryNeighTriePtr(index.neighborTrieIn[initialMatches[iV]], qIt->second, matchPointer);
        else
          rdfIndex.queryNeighTriePtr(index.neighborTrieOut[initialMatches[iV]], qIt->second, matchPointer);
      }
      else { // mEdge forms a 2-loop |
        std::set<int> eLabelIn, eLabelOut;
        for (auto it = qIt->second.begin(); it != qIt->second.end(); ++it) {
          if (*it <= index.nPred)
            eLabelIn.insert(*it);
          else
            eLabelOut.insert(*it);
        }
        std::vector<Node*> matchPointerIn, matchPointerOut;
        rdfIndex.queryNeighTriePtr(index.neighborTrieIn[initialMatches[iV]], eLabelIn, matchPointerIn);
        rdfIndex.queryNeighTriePtr(index.neighborTrieOut[initialMatches[iV]], eLabelOut, matchPointerOut);
        std::vector<int> neighMatches;
        for (size_t k = 0; k < matchPointerIn.size(); ++k) {
          std::vector<int> tm(matchPointerIn[k]->vertexId());
          partialEmb[pV-1].insert(partialEmb[pV-1].end(), tm.begin(), tm.end());
        }
        for (size_t k = 0; k < matchPointerOut.size(); ++k) {
          std::vector<int> tm(matchPointerOut[k]->vertexId());
          neighMatches.insert(neighMatches.end(), tm.begin(), tm.end());
        }
        partialEmb[pV-1] = vecIntersection(neighMatches, partialEmb[pV-1]);
      }
      if (matchPointer.empty() && partialEmb[pV-1].empty()){ // Star query structure not found. Thus, pV < q_nodes |
        break;
      }
      auto itA = constrainedSols.attSolns.find(queryGraphInfo.orderedNodes[pV]);
      if (itA != constrainedSols.attSolns.end()) {
        if (partialEmb[pV-1].empty()) {
          for (size_t k = 0; k < matchPointer.size(); ++k) {
            std::vector<int> tm(matchPointer[k]->vertexId());
            for(auto itV = tm.begin(); itV != tm.end(); ++itV)
              if (itA->second.find(*itV) != itA->second.end())
                partialEmb[pV-1].push_back(*itV);
          }
          if (partialEmb[pV-1].empty())
            break;
        }
        else {
          std::vector<int> temp;
          for(auto itV = partialEmb[pV-1].begin(); itV != partialEmb[pV-1].end(); ++itV)
            if (itA->second.find(*itV) != itA->second.end())
              temp.push_back(*itV);
          if (temp.empty())
            break;
          partialEmb[pV-1] = temp;
        }
      }

      auto itL = constrainedSols.loopSolns.find(queryGraphInfo.orderedNodes[pV]);
      if (itL != constrainedSols.loopSolns.end()) { // If initial vertex has a self loop |
        if (partialEmb[pV-1].empty()) {
          for (size_t k = 0; k < matchPointer.size(); ++k) {
            std::vector<int> tm(matchPointer[k]->vertexId());
            for(auto itV = tm.begin(); itV != tm.end(); ++itV)
              if (itL->second.find(*itV) != itL->second.end())
                partialEmb[pV-1].push_back(*itV);
          }
          if (partialEmb[pV-1].empty())
            break;
        }
        else {
          std::vector<int> temp;
          for(auto itV = partialEmb[pV-1].begin(); itV != partialEmb[pV-1].end(); ++itV)
            if (itL->second.find(*itV) != itL->second.end())
              temp.push_back(*itV);
          if (temp.empty())
            break;
          partialEmb[pV-1] = temp;
        }
      }
      auto itU = constrainedSols.uriSolns.find(queryGraphInfo.orderedNodes[pV]);
      if (itU != constrainedSols.uriSolns.end()) {
        if (partialEmb[pV-1].empty()) {
          for (size_t k = 0; k < matchPointer.size(); ++k) {
            std::vector<int> tm(matchPointer[k]->vertexId());
            for(auto itV = tm.begin(); itV != tm.end(); ++itV)
              if (itU->second.find(*itV) != itU->second.end())
                partialEmb[pV-1].push_back(*itV);
          }
          if (partialEmb[pV-1].empty())
            break;
        }
        else {
          std::vector<int> temp;
          for(auto itV = partialEmb[pV-1].begin(); itV != partialEmb[pV-1].end(); ++itV)
            if (itU->second.find(*itV) != itU->second.end())
              temp.push_back(*itV);
          if (temp.empty())
            break;
          partialEmb[pV-1] = temp;
        }
      }

      /// In case, no vertex attribute exists
      if(partialEmb[pV-1].empty()){
        for (size_t k = 0; k < matchPointer.size(); ++k) {
          std::vector<int> tm(matchPointer[k]->vertexId());
          partialEmb[pV-1].insert(partialEmb[pV-1].end(), tm.begin(), tm.end());
        }
      }
      nEmbIv *= partialEmb[pV-1].size();
      ++pV; // Does NOT get incremented if one query vertex fails to find any solutions |
    }
//        if (queryGraphInfo.timedOut)
//            break;
    if (pV == nStarNodes) { // Entire query has been matched |
      coreMatches.push_back(initialMatches[iV]);
      satMatches.push_back(partialEmb);
      for (size_t j = 0; j < partialEmb.size(); ++j)
        nStarEmb[j].push_back(partialEmb[j].size());
      nEmb += nEmbIv;
    }
  }
}

void Matcher::findInitialMatches(const QueryParameter& queryGraphInfo, InitialSolutions& constrainedSols, IndexType& index, std::vector<int>& initialMatches)
{
  int initialVertex = queryGraphInfo.orderedNodes[0];
  auto itA = constrainedSols.attSolns.find(initialVertex);
  auto itL = constrainedSols.loopSolns.find(initialVertex);
  auto itU = constrainedSols.uriSolns.find(initialVertex);
  if (itA != constrainedSols.attSolns.end() || itL != constrainedSols.loopSolns.end() || itU != constrainedSols.uriSolns.end()) {
    if (itA != constrainedSols.attSolns.end()) {
      initialMatches.resize(itA->second.size());
      int k = 0;
      for (auto it = itA->second.begin(); it != itA->second.end(); ++it) {
        initialMatches[k] = (*it);
        ++k;
      }
      constrainedSols.attSolns.erase(itA); // erase it, since it will further reduce the map size |
    }
    if (itL != constrainedSols.loopSolns.end()) { // If initial vertex has a self loop |
      if (initialMatches.empty()) {
        initialMatches.resize(itL->second.size());
        int k = 0;
        for (auto it = itL->second.begin(); it != itL->second.end(); ++it) {
          initialMatches[k] = (*it);
          ++k;
        }
      }
      else {
        std::vector<int> temp;
        for (size_t k = 0; k < initialMatches.size(); ++k)
          if (itL->second.find(initialMatches[k]) != itL->second.end())
            temp.push_back(initialMatches[k]);
        initialMatches = temp;
      }
      constrainedSols.loopSolns.erase(itL);
    }
    if (itU != constrainedSols.uriSolns.end()) { // If initial vertex has a URI |
      if (initialMatches.empty()) {
        initialMatches.resize(itU->second.size());
        int k = 0;
        for (auto it = itU->second.begin(); it != itU->second.end(); ++it) {
          initialMatches[k] = (*it);
          ++k;
        }
      }
      else {
        std::vector<int> temp;
        for (size_t k = 0; k < initialMatches.size(); ++k)
          if (itU->second.find(initialMatches[k]) != itU->second.end())
            temp.push_back(initialMatches[k]);
        initialMatches = temp;
      }
      constrainedSols.uriSolns.erase(itU);
    }
  }
  else {
    /// Select ALL candidates using trie structure
    IndexManager rdfIndex;
    std::vector<int> initNeigh, initSyn;
    std::set<std::set<int>> uniqueMedges; // collect the distinct multiedges |
    for(int n = 0; n < queryGraphInfo.neighbourSign[initialVertex].size(); ++n)
      uniqueMedges.insert(queryGraphInfo.neighbourSign[initialVertex][n]);
    std::vector<Node*> matchPointerAll;

    if (uniqueMedges.size() == 1)
      rdfIndex.queryNeighTrie(index.multiedgeTrie, (*uniqueMedges.begin()), initNeigh);
    else {
      int i = 0;
      for (auto it = uniqueMedges.begin(); it != uniqueMedges.end(); ++it) {
        std::vector<int> tmpMatches;
        rdfIndex.queryNeighTrie(index.multiedgeTrie, (*it), tmpMatches);
        if (i == 0)
          initNeigh = tmpMatches;
        else
          initNeigh.insert(initNeigh.end(), tmpMatches.begin(), tmpMatches.end());
        ++i;
      }
    }
    /// Select the PARTIAL candidates or the initial query vertex
    if(!queryGraphInfo.neighbourSign[initialVertex].empty()) {
      IndexManager rdfIndex;
      rdfIndex.querySynTrie(queryGraphInfo.neighbourSign[initialVertex], index.synopsesTrie, index.nPred, initSyn);
    }
    else {
      cout << "Neighborhood Signature is empty!!!";
      return;
    }
    std::vector<int> initSynReal;
    if (!initSyn.empty()) {
      initSynReal = index.synSolmap.find(initSyn[0])->second;
      if (initSyn.size() > 1) {
        for (size_t s = 1; s < initSyn.size(); ++s)
          initSynReal.insert(initSynReal.end(), index.synSolmap.find(initSyn[s])->second.begin(), index.synSolmap.find(initSyn[s])->second.end());
      }
    }
    initialMatches = vecIntersection(initNeigh, initSynReal);
  }
}

void pruneCandidates(InitialSolutions& constrainedSols, const int& nextVertex, const int& pM, Vector2D& exactStack)
{
  auto itA = constrainedSols.attSolns.find(nextVertex); // check if the query node has an attribute |
  if (itA != constrainedSols.attSolns.end()) {
    std::vector<int> tmp;
    for (size_t t = 0; t < exactStack[pM-1].size(); ++t) {
      if (itA->second.find(exactStack[pM-1][t]) != itA->second.end())
        tmp.push_back(exactStack[pM-1][t]);
    }
    exactStack[pM-1] = tmp;
  }
  auto itU = constrainedSols.uriSolns.find(nextVertex); // check if the query node has an URI |
  if (itU != constrainedSols.uriSolns.end()) {
    std::vector<int> tmp;
    for (size_t t = 0; t < exactStack[pM-1].size(); ++t) {
      if (itU->second.find(exactStack[pM-1][t]) != itU->second.end())
        tmp.push_back(exactStack[pM-1][t]);
    }
    exactStack[pM-1] = tmp;
  }
  auto itL = constrainedSols.loopSolns.find(nextVertex); // check if the query node has a LOOP |
  if (itL != constrainedSols.loopSolns.end()) {
    std::vector<int> tmp;
    for (size_t t = 0; t < exactStack[pM-1].size(); ++t) {
      if (itL->second.find(exactStack[pM-1][t]) != itL->second.end())
        tmp.push_back(exactStack[pM-1][t]);
    }
    exactStack[pM-1] = tmp;
  }
}


void findIndexSolutions(QueryParameter& queryGraph, InitialSolutions& constrainedSols, IndexType& index, const int& nextVertex, const int& matchedQueryNeighbour, const int& matchedDataNeighbour, std::vector<int>& neighMatches, UMapSet& repSolns, UMapSet::iterator & itS)
{
  std::pair<int, int> mEdge = make_pair(matchedQueryNeighbour, nextVertex);
  auto qIt = queryGraph.eLabelMap.find(mEdge);
  IndexManager rdfIndex;
  if (queryGraph.loopEdges.find(mEdge) == queryGraph.loopEdges.end()) { // mEdge is not a 2-loop |
    auto it_r = constrainedSols.repEdges.find(matchedQueryNeighbour); // check if there is at least one query vertex that has repeated edges |
    if (it_r != constrainedSols.repEdges.end() && it_r->second == qIt->second) { // check if the query vertex with repeated edge exists  AND check if the query vertex has the multi-edge that is repeated |
      itS = repSolns.find(matchedDataNeighbour);
      if (itS == repSolns.end()) { // A new data vertex for the repeating instance is to be found |
        if ((*qIt->second.begin()) <= index.nPred) // checks if multiedge is incoming or outgoing |
          rdfIndex.queryNeighTrie(index.neighborTrieIn[matchedDataNeighbour], qIt->second, neighMatches);
        else
          rdfIndex.queryNeighTrie(index.neighborTrieOut[matchedDataNeighbour], qIt->second, neighMatches);
        if (!neighMatches.empty()) {
          std::unordered_set<int> uSet(neighMatches.begin(), neighMatches.end());
          repSolns.insert(make_pair(matchedDataNeighbour, uSet));
        }
     }
    }
    else { // A non-repeating edge scenario |
      if ((*qIt->second.begin()) <= index.nPred) // checks if multiedge is incoming or outgoing |
        rdfIndex.queryNeighTrie(index.neighborTrieIn[matchedDataNeighbour], qIt->second, neighMatches);
      else
        rdfIndex.queryNeighTrie(index.neighborTrieOut[matchedDataNeighbour], qIt->second, neighMatches);
    }
  }
  else { // mEdge forms a 2-loop |
    std::set<int> eLabelIn, eLabelOut;
    for (auto it = qIt->second.begin(); it != qIt->second.end(); ++it) {
      if (*it <= index.nPred)
        eLabelIn.insert(*it);
      else
        eLabelOut.insert(*it);
    }
    rdfIndex.queryNeighTrie(index.neighborTrieIn[matchedDataNeighbour], eLabelIn, neighMatches);
    std::vector<int> neighMatches2;
    rdfIndex.queryNeighTrie(index.neighborTrieOut[matchedDataNeighbour], eLabelOut, neighMatches2);
    neighMatches = vecIntersection(neighMatches2, neighMatches);
  }
}


void findSubgraphMatches(const std::vector<int>& queryOrderOld, const int& pM, const Vector2D& matchedQueryNeighbours, const std::vector<int>& matchedDataVertices, Vector2D& exactStack, IndexType& index, QueryParameter& queryGraph, InitialSolutions& constrainedSols, const VecUSet& invalidMatches, const std::set<int>& prunableCoreNodes, UMapSet& repSolns)
{
  int nextVertex = queryGraph.orderedNodes[pM];
  std::vector<int> matchedDataNeighbours(matchedQueryNeighbours[pM-1].size());
  for(size_t i = 0; i < matchedDataNeighbours.size(); ++i)
    matchedDataNeighbours[i] = matchedDataVertices[find(queryGraph.orderedNodes.begin(), queryGraph.orderedNodes.end(), matchedQueryNeighbours[pM-1][i]) - queryGraph.orderedNodes.begin()];

  bool allNeighbMatched = true;
  std::vector<int> exactMatches;
  auto itInit = repSolns.end(); // has a pointer (itS->second) to the already matched candidates |

  for(size_t i = 0; i < matchedQueryNeighbours[pM-1].size(); ++i) {
    if (matchedQueryNeighbours[pM-1].size() == 1) {
      std::vector<int> neighMatches; // maintains the new candidates in it |
      auto itS = repSolns.end(); // has a pointer (itS->second) to the already matched candidates |
      findIndexSolutions(queryGraph, constrainedSols, index, nextVertex,  matchedQueryNeighbours[pM-1][i], matchedDataNeighbours[i], neighMatches, repSolns, itS);
      if (!neighMatches.empty()) { // new candidates found |
        if (prunableCoreNodes.find(nextVertex) != prunableCoreNodes.end()) { // remove unmatchable candidates |
          for (auto it_n = neighMatches.begin(); it_n != neighMatches.end(); ++it_n)
            if (invalidMatches[nextVertex].find(*it_n) == invalidMatches[nextVertex].end())
              exactStack[pM-1].push_back(*it_n);
        }
        else
          exactStack[pM-1] = neighMatches; // retain the same candidates |
        pruneCandidates(constrainedSols, queryOrderOld[pM], pM, exactStack);
      }
      else if (itS != repSolns.end()) { // candidates were already found |
        if (prunableCoreNodes.find(nextVertex) != prunableCoreNodes.end()) { // remove unmatchable candidates |
          for (auto it_n = itS->second.begin(); it_n != itS->second.end(); ++it_n)
            if (invalidMatches[nextVertex].find(*it_n) == invalidMatches[nextVertex].end())
              exactStack[pM-1].push_back(*it_n);
        }
        else
          for (auto it_n = itS->second.begin(); it_n != itS->second.end(); ++it_n)
            exactStack[pM-1].push_back(*it_n); // retain the same candidates, but convert from set to vector |
        pruneCandidates(constrainedSols, queryOrderOld[pM], pM, exactStack);
      }
    }
    else {
//            std::unordered_set<int> exactMatches_1_set; // assigned only when i = 0;  in all other cases, it should be empty |
      if (i == 0) {
        findIndexSolutions(queryGraph, constrainedSols, index, nextVertex,  matchedQueryNeighbours[pM-1][i], matchedDataNeighbours[i], exactMatches, repSolns, itInit);
        if (exactMatches.empty() && itInit == repSolns.end()) {
          allNeighbMatched = false;
          break;
        }
      }
      else {
        if (i > 1)
          itInit = repSolns.end();
        std::vector<int> neighMatches; // maintains the new candidates in it |
        auto itS = repSolns.end(); // has a pointer (itS->second) to the already matched candidates |
        findIndexSolutions(queryGraph, constrainedSols, index, nextVertex,  matchedQueryNeighbours[pM-1][i], matchedDataNeighbours[i], neighMatches, repSolns, itS);
        if (!neighMatches.empty()) { // new matches have been found |
          if (itInit == repSolns.end()) { // =>, for i = 0, candidates 'exactMatches' are in vector form|
            std::unordered_set<int> nSet(neighMatches.begin(), neighMatches.end());
            std::vector<int> tmp;
            for (auto it_e = exactMatches.begin(); it_e != exactMatches.end(); ++it_e)
              if (nSet.find(*it_e) != nSet.end())
                tmp.push_back(*it_e);
            exactMatches = tmp;
          }
          else { // =>, for i = 0, candidates 'itInit->second' are in set form|
            for (auto it_e = neighMatches.begin(); it_e != neighMatches.end(); ++it_e)
              if (itInit->second.find(*it_e) != itInit->second.end())
                exactMatches.push_back(*it_e); // this case executes ONLY when i = 1, which makes the step very efficient |
          }
          if (exactMatches.empty()) {
            allNeighbMatched = false;
            break;
          }
        }
        else if (itS != repSolns.end()) { // matches were already found |
          if (itInit == repSolns.end()) { // =>, for i = 0, candidates 'exactMatches' are in vector form|
            std::vector<int> tmp;
            for (auto it_e = exactMatches.begin(); it_e != exactMatches.end(); ++it_e)
              if (itS->second.find(*it_e) != itS->second.end())
                tmp.push_back(*it_e);
            exactMatches = tmp;
          }
          else {
            for (auto it_e = itS->second.begin(); it_e != itS->second.end(); ++it_e)
              if (itInit->second.find(*it_e) != itInit->second.end())
                exactMatches.push_back(*it_e); // this case executes ONLY when i = 1, which makes the step very efficient |
          }
          if (exactMatches.empty()) {
            allNeighbMatched = false;
            break;
          }
        }
        else {
          allNeighbMatched = false;
          break;
        }
      }
    }
  }
  if (allNeighbMatched && matchedQueryNeighbours[pM-1].size() > 1) {
    if (prunableCoreNodes.find(nextVertex) != prunableCoreNodes.end()) {
      for (int k = 0; k < exactMatches.size(); ++k)
        if (invalidMatches[nextVertex].find(exactMatches[k]) == invalidMatches[nextVertex].end())
          exactStack[pM-1].push_back(exactMatches[k]);
    }
    else
      exactStack[pM-1] = exactMatches;
    pruneCandidates(constrainedSols, queryOrderOld[pM], pM, exactStack);
  }
}


void Matcher::findAllMatches(const std::vector<int>& initialMatches, InitialSolutions&  constrainedSols, IndexType& index, QueryParameter& queryGraphInfo)
{
  clock_t stop_time = clock();
  Vector2D matchedQueryNeighbours(queryGraphInfo.nNodes-1); // Excludes initialVertex |
  std::vector<int> q_v;
  q_v.push_back(queryGraphInfo.orderedNodes[0]);
  for(size_t i = 0; i < queryGraphInfo.nNodes-1; ++i) {
    int nextVertex = queryGraphInfo.orderedNodes[i+1];
    matchedQueryNeighbours[i] = vecIntersection(queryGraphInfo.adjacencyList[nextVertex], q_v);
    q_v.push_back(nextVertex);
  }
  std::vector<int> matchedDataVertices(queryGraphInfo.nNodes);
  VecUSet invalidMatches(queryGraphInfo.nNodes); // Not used for this case |
  std::vector<int> queryOrderOld(queryGraphInfo.nNodes); // Not used for this case |

  std::set<int> prunableCoreNodes;
  for (auto it = constrainedSols.uriSolns.begin(); it != constrainedSols.uriSolns.end(); ++it)
    prunableCoreNodes.insert(it->first);

  for (auto it = constrainedSols.loopSolns.begin(); it != constrainedSols.loopSolns.end(); ++it)
    prunableCoreNodes.insert(it->first);

  for (auto it = constrainedSols.attSolns.begin(); it != constrainedSols.attSolns.end(); ++it)
    prunableCoreNodes.insert(it->first);


  for (int it = 0; it < initialMatches.size(); ++it) {
    UMapSet repSolns; // solutions for the repeated query vertices and corresponding edges |
    int pM = 0;
    matchedDataVertices[pM] = initialMatches[it];
    ++pM;
    Vector2D exactStack(queryGraphInfo.nNodes-1);
    bool incr = true; // keeps checking if we are moving forward in search tree space |

    while (pM != 0 ) {
      if (pM == queryGraphInfo.orderedNodes.size()) {
        bool all_matched = true;
        for (size_t r = 1; r < queryGraphInfo.orderedNodes.size(); ++r) {
          /// Check for URI node
          auto itU = constrainedSols.uriSolns.find(queryGraphInfo.orderedNodes[r]); // check if the query node has a URI |
          if (itU != constrainedSols.uriSolns.end()) {
            if (itU->second.find(matchedDataVertices[r]) == itU->second.end()) { // Check if there is a match for the URI |
              invalidMatches[queryGraphInfo.orderedNodes[r]].insert(matchedDataVertices[r]); // For the query node - queryGraphInfo.orderedNodes[r], matchedDataVertices[r] is an invalid match. Hence never match it |
              all_matched = false;
              break; // Think about this break !!!
            }
          }
          /// Check for self loop
          auto itL = constrainedSols.loopSolns.find(queryGraphInfo.orderedNodes[r]); // check if the query node has a URI |
          if (itL != constrainedSols.loopSolns.end()) {
            if (itL->second.find(matchedDataVertices[r]) == itL->second.end()) { // Check if there is a match for the loop node |
              invalidMatches[queryGraphInfo.orderedNodes[r]].insert(matchedDataVertices[r]); // For the query node - queryGraphInfo.orderedNodes[r], matchedDataVertices[r] is an invalid match. Hence never match it |
              all_matched = false;
              break; // Think about this break !!!
            }
          }
          /// Check for vertex attribute
          auto itA = constrainedSols.attSolns.find(queryGraphInfo.orderedNodes[r]); // check if the query node has a URI |
          if (itA != constrainedSols.attSolns.end()) {
            if (itA->second.find(matchedDataVertices[r]) == itA->second.end()) { // Check if there is a match for the attribute node |
              invalidMatches[queryGraphInfo.orderedNodes[r]].insert(matchedDataVertices[r]); // For the query node - queryGraphInfo.orderedNodes[r], matchedDataVertices[r] is an invalid match. Hence never match it |
              all_matched = false;
              break; // Think about this break !!!
            }
          }
        }
        if (all_matched) {
          queryGraphInfo.solutions.normalSolutions.push_back(matchedDataVertices);
          ++queryGraphInfo.nSolutions;
        }
        --pM;
        incr = false;
      }
      else {
        if (incr) {
          findSubgraphMatches(queryOrderOld, pM, matchedQueryNeighbours, matchedDataVertices, exactStack, index, queryGraphInfo, constrainedSols, invalidMatches, prunableCoreNodes, repSolns);
          if (double(clock() - stop_time) > MAX_ALLOWED_TIME) {
            queryGraphInfo.timedOut = true;
            break;
          }
        }
      }

      if (!exactStack[pM-1].empty()) {
        matchedDataVertices[pM] = exactStack[pM-1].back();
        exactStack[pM-1].pop_back();
        ++pM;
        incr = true;
      }
      else {
        --pM;
        incr = false;
      }
    }
    if (queryGraphInfo.timedOut) {
      queryGraphInfo.timedOut = true;
      break;
    }
  }
}

void Matcher::findComplexMatches(const std::vector<int>& initialMatches, const Vector3D& initialStarMatches, const Vector2D& nStarEmb, InitialSolutions& constrainedSols, IndexType& index, QueryParameter& decomposedQuery, QueryParameter& queryGraphInfo)
{
  clock_t stop_time = clock();
  /// Sort "satellite" with decreasing #satellite nodes.
  std::vector<int> srtedInd, satSize(decomposedQuery.satellite.size());
  int k = 0;
  for(auto it = decomposedQuery.satellite.begin(); it != decomposedQuery.satellite.end(); ++it){
    if (decomposedQuery.origDecompMap.find(it->first)->second == decomposedQuery.orderedNodes[0])
      satSize[k] = it->second.size() + 1; // add 1 to ensure that this always tops the sorted vector srtedInd |
    else
      satSize[k] = it->second.size();
    ++k;
  }
  srtedInd = sortIndex(satSize);

  std::map<int,int> qSeqIdMap; // map the new query sequence and the corresponding positions |
  for(size_t i = 0; i < decomposedQuery.orderedNodes.size(); ++i)
    qSeqIdMap.insert(std::make_pair(decomposedQuery.orderedNodes[i], i));

  Vector2D matchedQueryNeighbours(decomposedQuery.nNodes-1); // Excludes core_initialVertex |
  std::vector<int> q_v;
  q_v.push_back(decomposedQuery.orderedNodes[0]); // core initial vertex |
  for(size_t i = 0; i < decomposedQuery.nNodes-1; ++i) {
    int nextVertex = decomposedQuery.orderedNodes[i+1];
    matchedQueryNeighbours[i] = vecIntersection(decomposedQuery.adjacencyList[nextVertex], q_v);
    q_v.push_back(nextVertex);
  }
  std::vector<int> matchedDataVertices(decomposedQuery.nNodes);
  VecUSet invalidMatches(decomposedQuery.nNodes);
  std::vector<int> queryOrderOld(decomposedQuery.nNodes);
  for (size_t k = 0; k < decomposedQuery.orderedNodes.size(); ++k)
    queryOrderOld[k] = decomposedQuery.decompOrigMap.find(decomposedQuery.orderedNodes[k])->second;

  std::set<int> prunableCoreNodes;
  /// satellite prunable
  for(auto it = decomposedQuery.satellite.begin(); it != decomposedQuery.satellite.end(); ++it) {
    int coreNode = decomposedQuery.origDecompMap.find(it->first)->second;
    if (coreNode != decomposedQuery.orderedNodes[0] && !it->second.empty()) // except the initial chosen vertex |
      prunableCoreNodes.insert(coreNode);
  }
  /// URI prunable
  for (auto it = constrainedSols.uriSolns.begin(); it != constrainedSols.uriSolns.end(); ++it)
    prunableCoreNodes.insert(decomposedQuery.origDecompMap.find(it->first)->second);

  /// Loop prunable
  for (auto it = constrainedSols.loopSolns.begin(); it != constrainedSols.loopSolns.end(); ++it)
    prunableCoreNodes.insert(decomposedQuery.origDecompMap.find(it->first)->second);

  /// Attribute prunable
  for (auto it = constrainedSols.attSolns.begin(); it != constrainedSols.attSolns.end(); ++it)
    prunableCoreNodes.insert(decomposedQuery.origDecompMap.find(it->first)->second);


  for (int iV = 0; iV < initialMatches.size(); ++iV) {
    UMapSet repSolns; // solutions for the repeated query vertices and corresponding edges |
    int pM = 0;
    matchedDataVertices[pM] = (initialMatches[iV]);
    ++pM;
    Vector2D exactStack(decomposedQuery.nNodes-1);
    bool incr = true; // keeps checking if we are moving forward in search tree space |
    while (pM != 0 ) {
      if (pM == decomposedQuery.orderedNodes.size()) { // The core graph is matched|
        bool all_matched = true;
        for (size_t r = 1; r < decomposedQuery.orderedNodes.size(); ++r) {
          /// Check for URI node
          int q_node = decomposedQuery.decompOrigMap.find(decomposedQuery.orderedNodes[r])->second; // (improve:) have this information passed instead of computing here?
          auto itU = constrainedSols.uriSolns.find(q_node); // check if the query node has a URI |
          if (itU != constrainedSols.uriSolns.end()) {
            if (itU->second.find(matchedDataVertices[r]) == itU->second.end()) { // Check if there is a match for the URI |
              invalidMatches[decomposedQuery.orderedNodes[r]].insert(matchedDataVertices[r]); // For the query node - decomposedQuery.orderedNodes[r], matchedDataVertices[r] is an invalid match. Hence never match it |
              all_matched = false;
              break; // Think about this break !!!
            }
          }
          /// Check for self loop
          auto itL = constrainedSols.loopSolns.find(q_node); // check if the query node has a URI |
          if (itL != constrainedSols.loopSolns.end()) {
            if (itL->second.find(matchedDataVertices[r]) == itL->second.end()) { // Check if there is a match for the loop node |
              invalidMatches[decomposedQuery.orderedNodes[r]].insert(matchedDataVertices[r]); // For the query node - decomposedQuery.orderedNodes[r], matchedDataVertices[r] is an invalid match. Hence never match it |
              all_matched = false;
              break; // Think about this break !!!
            }
          }
          /// Check for vertex attribute
          auto itA = constrainedSols.attSolns.find(q_node); // check if the query node has a URI |
          if (itA != constrainedSols.attSolns.end()) {
            if (itA->second.find(matchedDataVertices[r]) == itA->second.end()) { // Check if there is a match for the attribute node |
              invalidMatches[decomposedQuery.orderedNodes[r]].insert(matchedDataVertices[r]); // For the query node - decomposedQuery.orderedNodes[r], matchedDataVertices[r] is an invalid match. Hence never match it |
              all_matched = false;
              break; // Think about this break !!!
            }
          }
        }

        ComplexSoln sparqlSolutions;
        sparqlSolutions.push_back(std::make_pair(initialMatches[iV], initialStarMatches[iV])); // fetch the solutions of intial core vertex and corresponding satellite vertices.

        /// Match the rest of the constellations.
        int no_sat_cores = 1; // One core (initial) is already matched |
        Vector3D all_sat_solns; // Store the solutions for all the satellite node for each constellation |
        unsigned long all_sat_n_cnt = 1;
        while (no_sat_cores != decomposedQuery.satellite.size() && !decomposedQuery.satellite.empty()) {
          auto it = decomposedQuery.satellite.begin(); // points to the 1st core vertex in the 'satellite' data structure |
          std::advance(it, srtedInd[no_sat_cores]); // advance linearly one by one in the decreasing satellite-node size;
          std::vector<int> q_seq_sat;
          q_seq_sat.push_back(it->first); // Choose all the satellites of core node, except for the initial as it has been already considered |
          for (auto it_1 = it->second.begin(); it_1 != it->second.end(); ++it_1)
            q_seq_sat.push_back(*it_1);
          queryGraphInfo.orderedNodes = q_seq_sat;
          std::vector<int> initialCoreMatch; // This is always just one |
          int nxt_q_id = decomposedQuery.origDecompMap.find(q_seq_sat[0])->second;
          initialCoreMatch.push_back(matchedDataVertices[qSeqIdMap.find(decomposedQuery.origDecompMap.find(it->first)->second)->second]);

          Vector3D satMatches; // all solutions except for the initial query vertex |
          std::vector<int> coreMatches; // exact solutions for the initial query vertex |
          Vector2D nStarEmb(q_seq_sat.size()-1); // no of embeddings for each distinct query node |
          int q_nodes = q_seq_sat.size();
          int pV = 1;
          unsigned long nEmb = 0;
          findStarMatches(index, initialCoreMatch, constrainedSols, queryGraphInfo, satMatches, coreMatches, nStarEmb, nEmb);

          if (!coreMatches.empty()) {
            sparqlSolutions.push_back(std::make_pair(coreMatches[0], satMatches[0]));
            all_sat_n_cnt*=nEmb; // Since we need combinations of >=2 constellations |
          }
          else {
            if (pV < q_nodes) // This condition checks if the vertex "initialCoreMatch[0]" has no STAR matches|
              invalidMatches[nxt_q_id].insert(initialCoreMatch[0]);
              all_matched = false;
              break;
          }
          ++no_sat_cores;
        }
        ///#
        if (all_matched) { // All star constellations are matched, and hence a valid set of complete embedding are found |
          queryGraphInfo.solutions.complexSolutions.push_back(sparqlSolutions);
          if (!decomposedQuery.satellite.empty()) {
            unsigned long i_star_matched = 1;
            for (size_t k = 0; k < nStarEmb.size(); ++k)
             i_star_matched*=nStarEmb[k][iV];
            queryGraphInfo.nSolutions+=all_sat_n_cnt*i_star_matched;
          }
          else
            ++queryGraphInfo.nSolutions;
        }
        --pM;
        incr = false;
      }
      else {
        if (incr) {
          findSubgraphMatches(queryOrderOld, pM, matchedQueryNeighbours, matchedDataVertices, exactStack, index, decomposedQuery, constrainedSols, invalidMatches, prunableCoreNodes, repSolns);
          if (double(clock() - stop_time) > MAX_ALLOWED_TIME) {
            queryGraphInfo.timedOut = true;
            break;
          }
        }
      }

      if (!exactStack[pM-1].empty()) {
        matchedDataVertices[pM] = exactStack[pM-1].back();
        exactStack[pM-1].pop_back();
        ++pM;
        incr = true;
      }
      else {
        --pM;
        incr = false;
      }
    } // Full depth is searched for all the solutions |
    if (queryGraphInfo.timedOut) {
      queryGraphInfo.timedOut = true;
      break;
    }
  }
}
