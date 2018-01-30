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

#ifndef TRIE_H
#define TRIE_H

#include <map>
#include <vector>
#include <set>
#include <iostream>

using namespace std;


class Node
{
  public:
    Node()
    {
        mContentInt = 0;
        mOccurrences = 0;
        mMarker = false;
    }
    ~Node()
    {
    delete mNodeParent;
    delete mLabelLink;
    for(std::vector<Node*>::iterator it = mChildren.begin(); it != mChildren.end(); ++it)
        delete (*it);
    }
    int contentInt(){
    	return mContentInt;
    }
    void setContentInt(int c){
        mContentInt = c;
    }
    bool wordMarker(){
        return mMarker;
    }
    void setWordMarker(){
        mMarker = true;
    }
    Node* findChildInt(int c);
    void appendChild(Node* child){
        mChildren.push_back(child);
    }
    std::vector<Node*> children(){
        return mChildren;
    }
    std::vector<int> vertexId(){
        return mVertexId;
    }
    void setVertexId(int id){
        mVertexId.push_back(id);
    }
    Node* nodeParent(){
        return mNodeParent;
    }
    void setNodeParent(Node* nP){
        mNodeParent = nP;
    }
    Node* labelLink(){
        return mLabelLink;
    }
    void setLabelLink(Node* link){
        mLabelLink = link;
    }
    std::set<int> Multiedge(){
        return mMultiedge;
    }
    void assignMultiedge(std::set<int> edge){
        mMultiedge = edge;
    }
    int occurrences(){
        return mOccurrences;
    }
    void countOccurrences(){
        ++mOccurrences;
    }
  private:
    Node* mNodeParent;
    Node* mLabelLink;
    int mContentInt;
    int mOccurrences;
    std::set<int> mMultiedge;
    bool mMarker;
    std::vector<Node*> mChildren;
    std::vector<int> mVertexId;
};

typedef std::map<int, std::vector<Node *>> LabelHash;

class Trie
{
  public:
    Trie();
    ~Trie();
    Node* Root() { return root; };
    LabelHash LabelMap;

    void addSignatureDim(const std::set<int>& s, const int& id);
    void updateHashTable(Node* trieNode, LabelHash& LabelMap);
    void linkSameLabels(Node* newValue, Node* rt, std::map<int, Node*>& myMapInt);
    void assignParent(Node* n);
    void addMultiedges(const std::set<int>& multiedge);
    void addNonFreqEdges(const std::set<int>& multiedge);
    bool edgeExists(Node* n, std::set<int> s, int pos, bool& found);
    void preOrder(Node* n);
    void postOrder(Node* n);
    void DestroyTrie(Node* n);
  protected:
  private:
    Node* root;
};

#endif // TRIE_H
