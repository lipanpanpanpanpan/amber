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

#ifndef INDEXMANAGER_H
#define INDEXMANAGER_H

#include "AmberLib.h"
#include "FileManager.h"
#include "rtree/RStarTree.h"
#include "Trie.h"

#define SYN_SIZE 8 // Number of fields in the Synopses vector

typedef RStarTree<int, SYN_SIZE, 32, 64> RTree;
typedef RTree::BoundingBox BoundingBox;

struct IndexType
{
  AttMap attributeHash;
  RTree synopsesTrie;
  std::map<int, std::vector<int>> synSolmap;
  std::vector<Trie*> neighborTrieIn;
  std::vector<Trie*> neighborTrieOut;
  Trie* neighborTrieLoop = new Trie();
  Trie* multiedgeTrie = new Trie();
  int nPred;
};

struct Visitor
{
	int cnt;
	bool ContinueVisiting;
	std::vector<int> edgeIndices;
	Visitor() : cnt(0), ContinueVisiting(true) {};
	void operator()(const RTree::Leaf * const leaf){
	  edgeIndices.push_back(leaf->leaf);
	}
};


class IndexManager
{
  public:
    IndexManager();
    virtual ~IndexManager();
    void sortSignature(GraphParameter& dataGraphInfo);
    void buildRdfIndexes(GraphParameter& dataGraphInfo, const RdfMapping& rdfMap, IndexType& index);
    void buildAttributeIndex(const GraphParameter& dataGraphInfo, const RdfMapping& rdfMap,  AttMap& attributeHash);
    void buildSynopsesTrie(const GraphParameter& dataGraphInfo, IndexType& index);
    void buildMultiedgeTrie(const GraphParameter& dataGraphInfo, Trie* mT);
    void buildNeighborTrie(const GraphParameter& dataGraphInfo, IndexType& index);
    std::vector<short> createSynNoIncr(const std::vector<std::set<int>>& signature, const int nP);
    // void queryAttHash(const VecOfSet& queryAtt, const AttMap& attributeHash, VecOfSet&  attMatches);
    void queryAttHash(const std::vector<std::set<int>>& queryAtt, const AttMap& vAttMap, Vector2D& vAtt, std::vector<std::unordered_set<int>>&  vAttSet);
    void querySynTrie(const std::vector<std::set<int>>& initSignature, RTree& synopsesTrie, const int& nPred, std::vector<int>& initialMatches);
    void queryNeighTrie(Trie* t, const std::set<int>& multiEdge, std::vector<int>& MatchedIds);
    void queryNeighTriePtr(Trie* t, const std::set<int>& multiEdge, std::vector<Node*>& MatchedIds);
    BoundingBox bounds(std::vector<short> synopses);
  protected:
    std::vector<short> createSynopses(const std::vector<std::set<int>>& signature, const int nP);
  private:
};

#endif // INDEXMANAGER_H
