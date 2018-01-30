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

#include "IndexManager.h"


IndexManager::IndexManager()
{
  //ctor
}

IndexManager::~IndexManager()
{
  //dtor
}

BoundingBox IndexManager::bounds(std::vector<short> synopses)
{
	BoundingBox bb;
  for (size_t i = 0; i < synopses.size(); ++i) {
    bb.edges[i].first  = 0;
    bb.edges[i].second = synopses[i];
  }
	return bb;
}

void IndexManager::buildRdfIndexes(GraphParameter& dataGraphInfo, const RdfMapping& rdfMap, IndexType& index)
{
	index.nPred = dataGraphInfo.nPred;
  sortSignature(dataGraphInfo); // sort the neighborhood signature with decreasing size of the number of neighbors |
  buildAttributeIndex(dataGraphInfo, rdfMap, index.attributeHash);
  buildSynopsesTrie(dataGraphInfo, index);
  buildMultiedgeTrie(dataGraphInfo, index.multiedgeTrie);
  buildNeighborTrie(dataGraphInfo, index);
}

void IndexManager::buildAttributeIndex(const GraphParameter& dataGraphInfo, const RdfMapping& rdfMap, AttMap& attributeHash)
{
  cout << "Building hash map for vertex attributes..." << endl;
  for (size_t i = 0; i < dataGraphInfo.attributes.size(); ++i){
    if ( !(dataGraphInfo.attributes[i].size() == 1 && (*dataGraphInfo.attributes[i].begin()) == -1) ) { // map only vertexes that have labels |
      for (auto itD = dataGraphInfo.attributes[i].begin(); itD != dataGraphInfo.attributes[i].end(); ++itD) {
        auto itA = attributeHash.find((*itD));
        int vrtx = rdfMap.vIdMap.find(i)->second; // map from rdf-id to amber-id |
        if (itA == attributeHash.end()) {
          std::vector<int> attTemp;
          attTemp.push_back(vrtx);
          attributeHash.insert(make_pair((*itD), attTemp));
        }
        else
          itA->second.push_back(vrtx);
      }
    }
  }
}

void IndexManager::buildSynopsesTrie(const GraphParameter& dataGraphInfo, IndexType& index)
{
  cout << "Constructing the index for edge dimensions..." << endl;
  std::map<std::vector<short>, std::vector<int>> synMap; // < synopses, data_vertices > |
  std::vector<short> zero(SYN_SIZE);
  for (size_t i = 0; i < dataGraphInfo.neighbourSign.size(); ++i) {
    std::vector<short> syn = createSynopses(dataGraphInfo.neighbourSign[i], dataGraphInfo.nPred);
    auto it = synMap.find(syn);
    if (it == synMap.end()) {
      std::vector<int> tempV;
      tempV.push_back(i);
      if (dataGraphInfo.neighbourSign[i].empty())
        synMap.insert(make_pair(zero, tempV));
      else
        synMap.insert(make_pair(syn, tempV));
    }
    else
      it->second.push_back(i);
  }
  int i = 0;
  for (auto it = synMap.begin(); it != synMap.end(); ++it) {
    index.synSolmap.insert(make_pair(i, it->second));
    index.synopsesTrie.Insert(i, bounds(it->first));
    ++i;
  }
}

std::vector<short> IndexManager::createSynopses(const std::vector<std::set<int>>& signature, const int nP)
{
  std::set<int> uniqueDimIn, uniqueDimOut;
  std::vector<int> itemSizeIn, itemSizeOut;
  for (int j=0; j<signature.size(); ++j) {
    for(auto it = signature[j].begin(); it != signature[j].end(); ++it) {
      if ( (*it) < nP )
        uniqueDimIn.insert((*it));
      else
        uniqueDimOut.insert((*it));
    }
    if (*signature[j].begin() < nP)
      itemSizeIn.push_back(signature[j].size());
    else
      itemSizeOut.push_back(signature[j].size());
  }

  std::vector<short> synopses(SYN_SIZE) ;
  // 4 synopses for incoming edges
  if (!uniqueDimIn.empty()){
    synopses[0] = *std::max_element(std::begin(itemSizeIn), std::end(itemSizeIn))+1;
    synopses[1] = uniqueDimIn.size()+1;
    synopses[2] = *uniqueDimIn.rbegin()+1;
    synopses[3] = *uniqueDimIn.begin()+1;
  }
  else{
    synopses[0] = 1; synopses[1] = 1; synopses[2] = 1; synopses[3] = nP;
  }
  // 4 synopses for outgoing edges
  if (!uniqueDimOut.empty()) {
    synopses[4] = uniqueDimOut.size()+1;
    synopses[5] = *uniqueDimOut.rbegin()+1;
    synopses[6] = *uniqueDimOut.begin()+1;
    synopses[7] = *std::max_element(std::begin(itemSizeOut), std::end(itemSizeOut))+1;
  }
  else{
    synopses[4] = 1; synopses[5] = 1; synopses[6] = 2*nP; synopses[7] = 1;
  }
  return synopses;
}

std::vector<short> IndexManager::createSynNoIncr(const std::vector<std::set<int>>& signature, const int nP)
{
  std::set<int> uniqueDimIn, uniqueDimOut;
  std::vector<int> itemSizeIn, itemSizeOut;
  for (int j=0; j<signature.size(); ++j) {
    for(auto it = signature[j].begin(); it != signature[j].end(); ++it) {
      if ( (*it) <= nP )
        uniqueDimIn.insert((*it));
      else
        uniqueDimOut.insert((*it));
    }
    if (*signature[j].begin() <= nP)
      itemSizeIn.push_back(signature[j].size());
    else
      itemSizeOut.push_back(signature[j].size());
  }

  std::vector<short> synopses(SYN_SIZE) ;
  // 4 synopses for incoming edges
  if (!uniqueDimIn.empty()) {
    synopses[0] = *std::max_element(std::begin(itemSizeIn), std::end(itemSizeIn));
    synopses[1] = uniqueDimIn.size()+1;
    synopses[2] = *uniqueDimIn.rbegin();
    synopses[3] = *uniqueDimIn.begin();
  }
  else {
    synopses[0] = 1; synopses[1] = 1; synopses[2] = 1; synopses[3] = nP+1;
  }
  // 4 synopses for outgoing edges
  if (!uniqueDimOut.empty()) {
    synopses[4] = uniqueDimOut.size()+1;
    synopses[5] = *uniqueDimOut.rbegin();
    synopses[6] = *uniqueDimOut.begin();
    synopses[7] = *std::max_element(std::begin(itemSizeOut), std::end(itemSizeOut));
  }
  else {
    synopses[4] = 1; synopses[5] = 1; synopses[6] = 2*nP+1; synopses[7] = 1;
  }
  return synopses;
}

void IndexManager::buildMultiedgeTrie(const GraphParameter& dataGraphInfo, Trie* mT)
{
  cout << "Building multiedge trie..." << endl;
  for(int m = 0; m < dataGraphInfo.neighbourSign.size(); ++m) {
    std::set<std::set<int>> uniqueMultidges;
    for(int n = 0; n < dataGraphInfo.neighbourSign[m].size(); ++n)
      uniqueMultidges.insert(dataGraphInfo.neighbourSign[m][n]);
    for(auto it = uniqueMultidges.begin(); it != uniqueMultidges.end(); ++it)
      mT->addSignatureDim((*it), m); //Add the multi-edge and its corresponding vertex id to the trie;
  }
  mT->updateHashTable(mT->Root(), mT->LabelMap); // Update the hash table with edges as keys and corresponding neighborhood vertices as the values |
  std::map<int, Node*> myMapInt;
  mT->linkSameLabels(mT->Root(), mT->Root(), myMapInt);
  mT->assignParent(mT->Root());
  myMapInt.clear();
}

void IndexManager::buildNeighborTrie(const GraphParameter& dataGraphInfo, IndexType& index)
{
  cout << "Building Neighbourhood Index Trie..." << endl;
  index.neighborTrieIn.resize(dataGraphInfo.nNodes);
  index.neighborTrieOut.resize(dataGraphInfo.nNodes);
  bool loopExists = false;

  for(int m = 0; m < dataGraphInfo.nNodes; ++m) {
    Trie *trieSignatureIn = new Trie();
    Trie *trieSignatureOut = new Trie();
    for(int n = 0; n < dataGraphInfo.adjacencyList[m].size(); ++n) {
      auto itD = dataGraphInfo.eLabelMap.find(make_pair(m, dataGraphInfo.adjacencyList[m][n]));
      if (m == dataGraphInfo.adjacencyList[m][n]) {
        index.neighborTrieLoop->addSignatureDim(itD->second, m);
        loopExists = true;
      }
      std::set<int> outLabels, inLabels; // Separate the labels that are both outgoing and incoming | This happens since there are 2-loops in the data graph. When there is a 2-loop, an edge can have both outgoing and incoming edges |
      for (auto it = itD->second.begin(); it != itD->second.end(); ++it) {
        if (*it < dataGraphInfo.nPred)
          inLabels.insert(*it);
        else
          outLabels.insert(*it);
      }
      if (!inLabels.empty())
        trieSignatureIn->addSignatureDim(inLabels, dataGraphInfo.adjacencyList[m][n]); //Add the POSITIVE multi-edge and its corresponding vertex id to the trie;
      if (!outLabels.empty())
        trieSignatureOut->addSignatureDim(outLabels, dataGraphInfo.adjacencyList[m][n]); //Add the NEGATIVE multi-edge and its corresponding vertex id to the trie;
    }
    trieSignatureIn->updateHashTable(trieSignatureIn->Root(), trieSignatureIn->LabelMap); // Update the hash table with edges as keys and corresponding neighborhood vertices as the values |

    std::map<int, Node*> myMapInt;
    trieSignatureIn->linkSameLabels(trieSignatureIn->Root(), trieSignatureIn->Root(), myMapInt);
    trieSignatureIn->assignParent(trieSignatureIn->Root());
    index.neighborTrieIn[m] = trieSignatureIn;
    myMapInt.clear();
    trieSignatureOut->updateHashTable(trieSignatureOut->Root(), trieSignatureOut->LabelMap); // Update the hash table with edges as keys and corresponding neighborhood vertices as the values |

    trieSignatureOut->linkSameLabels(trieSignatureOut->Root(), trieSignatureOut->Root(), myMapInt);
    trieSignatureOut->assignParent(trieSignatureOut->Root());
    index.neighborTrieOut[m] = trieSignatureOut;
    myMapInt.clear();
  }
  std::map<int, Node*> myMapInt;
  if (loopExists) { // Post process the self loop index |
    index.neighborTrieLoop->linkSameLabels(index.neighborTrieLoop->Root(), index.neighborTrieLoop->Root(), myMapInt);
    index.neighborTrieLoop->assignParent(index.neighborTrieLoop->Root());
    myMapInt.clear();
  }
}

/*
void IndexManager::queryAttHash(const VecOfSet& queryAtt, const AttMap& attributeHash, VecOfSet&  attMatches)
{
  for (size_t i = 0; i < queryAtt.size(); ++i) {
    if ( !(queryAtt[i].size() == 1 && (*queryAtt[i].begin()) == -1) ) { // only if the vertex has labels |
      size_t k = 0;
      for (auto it_q = queryAtt[i].begin(); it_q != queryAtt[i].end(); ++it_q) {
        if (k == 0)
          attMatches[i] = attributeHash.find((*it_q))->second;
        else {
//                    attMatches[i] = setIntersection(attMatches[i], attributeHash.find((*it_q))->second);
          std::set<int> intersect;
          set_intersection(attMatches[i].begin(),attMatches[i].end(),attributeHash.find((*it_q))->second.begin(),attributeHash.find((*it_q))->second.end(), std::inserter(intersect,intersect.begin()));
          attMatches[i] = intersect;
        }
        ++k;
      }
    }
  }
}
*/

void IndexManager::queryAttHash(const std::vector<std::set<int>>& queryAtt, const AttMap& vAttMap, Vector2D& vAtt, std::vector<std::unordered_set<int>>&  vAttSet)
{
  for (size_t q = 0; q < queryAtt.size(); ++q) {
    if ( !(queryAtt[q].size() == 1 && (*queryAtt[q].begin()) == -1) ) {
      size_t k = 0;
      for (auto it_q = queryAtt[q].begin(); it_q != queryAtt[q].end(); ++it_q) {
        if (k == 0)
        	vAtt[q] = vAttMap.find((*it_q))->second;
        else
        	vAtt[q] = setIntersection(vAtt[q], vAttMap.find((*it_q))->second);
        ++k;
        }
    }
  }
  for (size_t i = 0; i < vAtt.size(); ++i) {
    std::unordered_set<int> s(vAtt[i].begin(), vAtt[i].end());
    vAttSet[i] = s;
  }
}

void IndexManager::querySynTrie(const std::vector<std::set<int>>& initSignature, RTree& synopsesTrie, const int& nPred, std::vector<int>& initialMatches)
{
  std::vector<short> qSynopses = createSynNoIncr(initSignature, nPred);
  BoundingBox bound = bounds(qSynopses);
  Visitor x = synopsesTrie.Query(RTree::AcceptEnclosing(bound), Visitor());
  if (!x.edgeIndices.empty())
    initialMatches = x.edgeIndices;
}


void IndexManager::queryNeighTrie(Trie* t, const std::set<int>& multiEdge, std::vector<int>& MatchedIds)
{
  /// Check if all the elements of multiedges are found in this tree
  for(auto it = multiEdge.begin(); it != multiEdge.end(); ++it)
    if (t->LabelMap.find(*it) == t->LabelMap.end())
      return; // if any edge is not present in the tree, quit |

  auto it = multiEdge.rbegin();
  std::vector<Node *> matches = t->LabelMap.find(*it)->second; // get all the pointers for the end character of multiedge |

  for(size_t i = 0; i < matches.size(); ++i){
    Node* currentNode = matches[i]->nodeParent();
    if(multiEdge.size() > 1) {
      int mtch = 0;
      auto rit = multiEdge.rbegin();
      std::advance (rit,1); // skip the matching of the last element in 'multiEdge' as it is already done |
      for (; rit != multiEdge.rend(); rit++) {
        bool found = false;
        while(currentNode->contentInt() != 0) { // Trace back until root is reached |
          if(currentNode->contentInt() == (*rit)+1){
            currentNode = currentNode->nodeParent();
            ++mtch;
            found = true;
            break;
          }
          else
            currentNode = currentNode->nodeParent();
        }
        if(!found)
          break;
      }
      if(mtch == multiEdge.size()-1) {
        std::vector<int> tm(matches[i]->vertexId());
        MatchedIds.insert(MatchedIds.end(), tm.begin(), tm.end());
      }
    }
    else {
      std::vector<int> tm(matches[i]->vertexId());
      MatchedIds.insert(MatchedIds.end(), tm.begin(), tm.end());
    }
  }
}

void IndexManager::queryNeighTriePtr(Trie* t, const std::set<int>& multiEdge, std::vector<Node*>& matchPointer)
{
  /// Check if all the elements of multiedges are found in this tree
  for(auto it = multiEdge.begin(); it != multiEdge.end(); ++it)
    if (t->LabelMap.find(*it) == t->LabelMap.end())
      return; // if any edge is not present in the tree, quit |

  auto it = multiEdge.rbegin();
  std::vector<Node *> matches = t->LabelMap.find(*it)->second; // get all the pointers for the end character of multiedge |
  for(size_t i = 0; i < matches.size(); ++i){
    Node* currentNode = matches[i]->nodeParent();
    if(multiEdge.size() > 1) {
      int mtch = 0;
      auto rit = multiEdge.rbegin();
      std::advance (rit,1); // skip the matching of the last element in 'multiEdge' as it is already done |
      for (; rit != multiEdge.rend(); rit++) {
        bool found = false;
        while(currentNode->contentInt() != 0) { // Trace back until root is reached |
          if(currentNode->contentInt() == (*rit)+1){
            currentNode = currentNode->nodeParent();
            ++mtch;
            found = true;
            break;
          }
          else
            currentNode = currentNode->nodeParent();
        }
        if(!found)
          break;
      }
      if(mtch == multiEdge.size()-1)
        matchPointer.push_back(matches[i]);
    }
    else
      matchPointer.push_back(matches[i]);
  }
}

void IndexManager::sortSignature(GraphParameter& dataGraphInfo)
{
  std::vector<int> nEdges(dataGraphInfo.nNodes);
  for(size_t i = 0; i < dataGraphInfo.nNodes; ++i) {
    nEdges[i] = 0;
    std::vector<std::set<int>> sortedSign;
    std::vector<int> nNeighbours;
    for(auto it = dataGraphInfo.neighbourSign[i].begin(); it!=dataGraphInfo.neighbourSign[i].end(); ++it) {
      nEdges[i] += (*it).size();
      nNeighbours.push_back((*it).size());
    }
    std::vector<int> sortedNeighIndex = sortIndex(nNeighbours); // sort each signature within itself wrt subsignature size |
    for(size_t j = 0; j < nNeighbours.size(); ++j)
      sortedSign.push_back(dataGraphInfo.neighbourSign[i][sortedNeighIndex[j]]);
    dataGraphInfo.neighbourSign[i] = sortedSign;
  }
  dataGraphInfo.orderedNodes = sortIndex(nEdges); // sort all the data vertices wrt data adjaceny list of each vertex |
}
