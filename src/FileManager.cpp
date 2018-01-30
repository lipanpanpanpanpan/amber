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

#include "FileManager.h"

FileManager::FileManager()
{
  //ctor
}

FileManager::~FileManager()
{
  //dtor
}

void FileManager::splitString(const std::string& str, char chr, std::vector<std::string>& strs)
{
  std::string::const_iterator first = str.cbegin();
  std::string::const_iterator second = std::find(first+1, str.cend(), chr);
  while(second != str.cend()){
    strs.emplace_back(first, second);
    first = second+1;
    second = std::find(second+1, str.cend(), chr);
  }
  strs.emplace_back(first, str.cend());
}

void FileManager::readContents(const std::string& dataPath, GraphParameter& graphInfo)
{
  /// Read node file
  std::string nodeFile = dataPath + "nodes.txt";
  graphInfo.nodeLabelsExist = false; // Do not consider node labels
  const char * nF = nodeFile.c_str();
  std::ifstream nFile (nF);
  if (nFile.is_open()) {
    std::string nodeAtt;
    while (getline(nFile, nodeAtt)) {
      std::vector<std::string> nodeAttAll;
      splitString(nodeAtt, ',', nodeAttAll);
      std::set<int> setAtt;
      for (size_t i = 0; i < nodeAttAll.size(); ++i)
        setAtt.insert(stoi(nodeAttAll[i]));
      graphInfo.attributes.push_back(setAtt);
    }
    nFile.close();
    graphInfo.nNodes = graphInfo.attributes.size();
  }
  else{
      graphInfo.nodeLabelsExist = false;
      std::cout << "Unable to open node file!" << std::endl;
  }
  /// Read edge file
  std::string edgeFile = dataPath + "edges.txt";
  const char * eF = edgeFile.c_str();
  std::ifstream eFile (eF);
  std::map<std::string, int> nodeMap; // < original_id, mapped_id >
  if (eFile.is_open()) {
    std::string multiEdge;
    int n = 0;
    std::vector<std::vector<int>> adjListTemp(graphInfo.nNodes);
    EdgeLabel edgeLabels(graphInfo.nNodes);
    while (getline(eFile, multiEdge)) {
      std::vector<std::string> edgeContent;
      splitString(multiEdge, ' ', edgeContent);
      int node1; int node2;
      auto it = nodeMap.find(edgeContent.at(0));
      if (it == nodeMap.end()) {
        nodeMap.insert(make_pair(edgeContent.at(0), n));
        node1 = n;
        ++n;
      }
      else
        node1 = it->second;
      it = nodeMap.find(edgeContent.at(1));
      if (it == nodeMap.end()) {
        nodeMap.insert(make_pair(edgeContent.at(1), n));
        node2 = n;
        ++n;
      }
      else
        node2 = it->second;

      std::vector<std::string> labels;
      splitString(edgeContent.at(2), ',', labels);
      std::set<int> labelIn, labelOut;
      for(auto it = labels.begin(); it != labels.end(); ++it){
        labelIn.insert(stoi((*it)));
        labelOut.insert(graphInfo.nPred+stoi((*it)));
      }

      edgeLabels[node1].push_back(labelOut); // NEGATIVE labels for outgoing edge
      edgeLabels[node2].push_back(labelIn); // POSITIVE labels for incoming edge

      // Frequency of multiedges |
      auto it_e = graphInfo.edgeFrequency.find(labelIn);
      if (it_e == graphInfo.edgeFrequency.end())
        graphInfo.edgeFrequency.insert(make_pair(labelIn, 1));
      else
        it_e->second++;

      std::pair<int, int> nodePair = std::make_pair(node1,node2);
      auto it_1 = graphInfo.eLabelMap.find(nodePair);
      if (it_1 == graphInfo.eLabelMap.end())
        graphInfo.eLabelMap.insert(std::make_pair(nodePair, labelIn));
      else
        for (auto it = labelIn.begin(); it != labelIn.end(); ++it)
          it_1->second.insert(*it);

      std::pair<int, int> nodePairRev = std::make_pair(node2,node1);
      auto it_2 = graphInfo.eLabelMap.find(nodePairRev);
      if (it_2 == graphInfo.eLabelMap.end())
        graphInfo.eLabelMap.insert(std::make_pair(nodePairRev, labelOut));
      else
        for (auto it = labelOut.begin(); it != labelOut.end(); ++it)
          it_2->second.insert(*it);

      if (find(adjListTemp[node1].begin(), adjListTemp[node1].end(), node2) == adjListTemp[node1].end())
        adjListTemp[node1].push_back(node2);
      if (find(adjListTemp[node2].begin(), adjListTemp[node2].end(), node1) == adjListTemp[node2].end())
        adjListTemp[node2].push_back(node1);
    }
    graphInfo.adjacencyList = adjListTemp;
    graphInfo.neighbourSign = edgeLabels;
  }
  else
    std::cout << "Unable to open edge file!" << std::endl;
}


void FileManager::readMappings(const std::string& dPath, RdfMapping& rdfMap)
{
  std::map<int, std::string> nodeMap; // S/O -> an integer for vertex id |
  std::map<int, std::string> edgeMap; // P -> an integer for edge dimension |
  std::map<int, std::pair<std::string, std::string>> labelMap; // P:O pair -> an integer for vertex label |
  std::map<std::string, int> RdfAmberMap; // < sumgra_id, rdf_id > |

  ifstream vertexFile (dPath + "vertex_map.txt");
  if (vertexFile.is_open()) {
    std::string line;
    while (getline(vertexFile, line)) {
      std::vector<std::string> strs;
      splitString(line, ' ', strs);
      int node_id = stoi(strs.back());
      strs.pop_back();
      std::string node = strs[0];
      for (int j = 0; j < strs.size()-1; ++j)
        node = node + " " + strs[j+1];
      nodeMap.insert(make_pair(node_id, node));
    }
    vertexFile.close();
  }
  else
    cout << "Unable to open file" << endl;
  ifstream edgeFile (dPath + "edge_map.txt");
  if (edgeFile.is_open()) {
    std::string line;
    while (getline(edgeFile, line)) {
      std::vector<std::string> strs;
      splitString(line, ' ', strs);
      edgeMap.insert(make_pair(stoi(strs[1]), strs[0]));
    }
    edgeFile.close();
  }
  else
    cout << "Unable to open file" << endl;
    ifstream attributeFile (dPath + "attribute_map.txt");
  if (attributeFile.is_open()) {
    std::string line;
    while (getline(attributeFile, line)) {
      std::vector<std::string> line_contents_tmp;
      splitString(line, ' ', line_contents_tmp);
      std::vector<std::string> line_contents(3);
      if (line_contents_tmp.size() > 3) {
        line_contents[0] = line_contents_tmp.front();
        line_contents[1] = line_contents_tmp[1];
        line_contents[2] = line_contents_tmp.back();
        for (int j = 0; j < line_contents_tmp.size()-3; ++j)
          line_contents[1] = line_contents[1] + " " + line_contents_tmp[j+2];
      }
      else if (line_contents_tmp.size() == 3)
        line_contents = line_contents_tmp;
      labelMap.insert(make_pair(stoi(line_contents[2]), make_pair(line_contents[0], line_contents[1])));
    }
    attributeFile.close();
  }
  else
    cout << "Unable to open file" << endl;
  ifstream mapFile (dPath + "vertex_map_algo.txt"); // Read sumgra <-> rdf node mappinigs |
  if (mapFile.is_open()) {
    std::string line;
    while (getline(mapFile, line)) {
      VecOfStr strs;
      splitString(line, ' ', strs);
      int n_id = std::stoi(strs[1]);
      RdfAmberMap.insert(make_pair(strs[0], n_id));
    }
  }
  else
      cout << "cannot open file: " << dPath + "vertex_map_algo.txt" << endl;
  mapFile.close();

  for (auto it = edgeMap.begin(); it != edgeMap.end(); ++it)
    rdfMap.eLabels.insert(make_pair(it->second, it->first));

  for (auto it = labelMap.begin(); it != labelMap.end(); ++it)
    rdfMap.vLabels.insert(make_pair(it->second, it->first));

  for (auto it = nodeMap.begin(); it != nodeMap.end(); ++it)
    rdfMap.nodes.insert(make_pair(it->second, it->first));

  for (auto it = RdfAmberMap.begin(); it != RdfAmberMap.end(); ++it)
    rdfMap.vIdMap.insert(make_pair(it->second, std::stoi(it->first)));
}


void dfsTraverse(const Vector2D& solutions, const std::string& fPath, const std::map<int, std::string> rdfNodes)
{
  // Vector2D vertexPermutations;
  std::ofstream outSols;
  const char * pTFile = fPath.c_str();
  outSols.open(pTFile, std::ios_base::app);

  for(size_t i = 0; i < solutions[0].size(); ++i){
    std::vector<int> singlePermutation;
    singlePermutation.push_back(solutions[0][i]); /// Add elements to this vector while Incrementing and remove while Decrementing.
    // Recursively find all the permutations |
    int depth = 1;
    Vector2D pStack(solutions.size()-1);
    bool incr = true; // keeps checking if we are moving forward in search tree space |
    while (depth != 0 ) {
      if (singlePermutation.size() == solutions.size()) {
        for(size_t j = 0; j < singlePermutation.size(); ++j)
          outSols << rdfNodes.find(singlePermutation[j])->second << "\t|\t";
        outSols << endl;
        // vertexPermutations.push_back(singlePermutation);
        singlePermutation.pop_back();
        --depth; // this restores the 'depth' value going out of bound |
        pStack[depth-1].pop_back(); // remove the stack element only when entire length is matched |
        incr = false;
      }
      else {
        if (incr)
          pStack[depth-1] = solutions[depth];
        else{
          singlePermutation.pop_back();
          pStack[depth-1].pop_back();
        }
      }
      if (!pStack[depth-1].empty()) {
        singlePermutation.push_back(pStack[depth-1].back());
        ++depth;
        incr = true;
      }
      else {
        --depth;
        incr = false;
      }
    }
  }
}


void FileManager::printSolutions(const QueryParameter& queryGraphInfo, const RdfMapping& rdfMap, const SparqlMapping& sparqlMap, const std::string& fPath)
{
  /// Get reverse mppings for sparql and rdf nodes
  std::map<int, std::string> rdfNodes;
  std::map<int, std::string> sparqlNodes;
  for(auto it = rdfMap.nodes.begin(); it != rdfMap.nodes.end(); ++it)
    rdfNodes.insert(make_pair(it->second, it->first));
  for(auto it = sparqlMap.nodes.begin(); it != sparqlMap.nodes.end(); ++it)
    sparqlNodes.insert(make_pair(it->second, it->first));

  /// Get ordered query vertices
  std::vector<int> queryOrder;
  if(queryGraphInfo.queryType == "complex"){
    ComplexOrder query = queryGraphInfo.solutions.complexQueryOrder;
    for(size_t i = 0; i < query.size(); ++i){
      queryOrder.push_back(query[i].first);
      if(!query[i].second.empty())
        queryOrder.insert(queryOrder.end(), query[i].second.begin(), query[i].second.end());
    }
  }
  else{ // For both star and no-star type
    queryOrder = queryGraphInfo.orderedNodes;
  }

  /// Print ordered query vertices
  std::ofstream outVars;
  const char * pTFile = fPath.c_str();
  outVars.open(pTFile);
  for(size_t i = 0; i < queryOrder.size(); ++i)
    outVars << sparqlNodes.find(queryOrder[i])->second << "\t|\t";
  outVars << endl << endl;

  if(queryGraphInfo.queryType == "complex"){
    std::vector<ComplexSoln> allSolutions = queryGraphInfo.solutions.complexSolutions;
    for(size_t i  = 0; i < allSolutions.size(); ++i){
      Vector2D solutions;
      for(size_t j = 0; j < allSolutions[i].size(); ++j){
        std::vector<int> sol(1);
        sol[0] = allSolutions[i][j].first;
        solutions.push_back(sol);
        int starSols = 1;
        for (size_t k = 0; k < allSolutions[i][j].second.size(); ++k) {
          starSols*=allSolutions[i][j].second[k].size();
          solutions.push_back(allSolutions[i][j].second[k]);
        }
      }
      dfsTraverse(solutions, fPath, rdfNodes);
    }
  }
  else if(queryGraphInfo.queryType == "star"){
    std::pair<std::vector<int>, Vector3D> allSolutions = queryGraphInfo.solutions.starSolutions;
    for(size_t i = 0; i < allSolutions.first.size(); ++i){
      Vector2D solutions;
      std::vector<int> sol;
      sol.push_back(allSolutions.first[i]);
      solutions.push_back(sol);
      for(size_t j = 0; j < allSolutions.second[i].size(); ++j)
        solutions.push_back(allSolutions.second[i][j]);
      dfsTraverse(solutions, fPath, rdfNodes);
    }
  }
  else{
    Vector2D solutions = queryGraphInfo.solutions.normalSolutions;
    std::ofstream outSols;
    const char * pTFile = fPath.c_str();
    outSols.open(pTFile, std::ios_base::app);
    for(size_t i = 0; i < solutions.size(); ++i){
      for(size_t j = 0; j < queryOrder.size(); ++j)
        outSols << rdfNodes.find(solutions[i][j])->second << "\t|\t";
      outSols << endl;
    }
    outSols.close();
  }
}


void FileManager::printStatistics(const QueryParameter& queryGraphInfo, const std::string& fPath, int& nQ, double& queryTime)
{
  std::ofstream outTime;
  const char * pTFile = fPath.c_str();
  outTime.open(pTFile, std::ios_base::app);
  if (!queryGraphInfo.timedOut)
    outTime << nQ << "\t" << queryGraphInfo.nSolutions  << "\t" << (queryTime)*1000  <<  endl;
  else
    outTime << nQ << "\t-\t-" << endl;
}
