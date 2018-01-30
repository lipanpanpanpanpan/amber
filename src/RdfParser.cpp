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

#include "RdfParser.h"

RdfParser::RdfParser()
{
  //ctor
}

RdfParser::~RdfParser()
{
  //dtor
}

void writeToDatabase(std::map<std::string, int>& nodeMap, std::map<std::string, int>& predicateMap, Vector2D& mappedRdf, std::string& path)
{
  ofstream outV;
  outV.open(path + "vertex_map.txt");
  for (auto it = nodeMap.begin(); it != nodeMap.end(); ++it)
    outV << (it->first) << " " << (it->second) << endl;
  outV.close();
  ofstream outE;
  outE.open(path + "edge_map.txt");
  for (auto it = predicateMap.begin(); it != predicateMap.end(); ++it)
    outE << (it->first) << " " << (it->second) << endl;
  outE.close();
  // ofstream outA;
  // outA.open(path + "attribute_map.txt");
  // for (auto it = attributeMap.begin(); it != attributeMap.end(); ++it)
  //     outA << (it->first.first) << " " << (it->first.second) << " " << (it->second) << endl;
  // outA.close();

  std::map<std::pair<int,int>, std::vector<int>> mappedEdges;
  int multiEdges;
  std::set<int> authorities, predicates;
  for (auto it = nodeMap.begin(); it != nodeMap.end(); ++it)
    authorities.insert(it->second); // collect all unique authorities (subject/object);

  for (size_t i = 0; i < mappedRdf.size(); ++i) {
    predicates.insert(mappedRdf[i][2]);
    auto it = mappedEdges.find(make_pair(mappedRdf[i][0], mappedRdf[i][1]));
    std::vector<int> multiLabels;
    multiLabels.push_back(mappedRdf[i][2]);
    if (it == mappedEdges.end()) {
      mappedEdges.insert(make_pair(make_pair(mappedRdf[i][0], mappedRdf[i][1]), multiLabels));
    }
    else {
      ++multiEdges;
      multiLabels.insert(multiLabels.end(), it->second.begin(), it->second.end());
      mappedEdges.erase(it);
      mappedEdges.insert(make_pair(make_pair(mappedRdf[i][0], mappedRdf[i][1]), multiLabels));
    }
  }
  cout << "# edges: " << mappedEdges.size() << endl;
  cout << "# nodes: " << authorities.size() << endl;
  cout << "size of hash map: " << mappedRdf.size() << endl;

  // cout << "# vertex labels: " << attributeMap.size() << endl;
  cout << multiEdges << " multi edges exist!" <<  endl;
  int selfLoops = 0;
  ofstream outF;
  outF.open(path + "edges.txt");
  for (auto it = mappedEdges.begin(); it != mappedEdges.end(); ++it) {
    outF << it->first.first << " " << it->first.second << " ";
    for (size_t i = 0; i < it->second.size()-1; ++i)
      outF << it->second[i] << ",";
    outF << it->second.back() << endl;
    if (it->first.first == it->first.second)
      ++selfLoops;
  }
  outF.close();
  cout << selfLoops << " self loops exist!" << endl;
  std::map<int, int> node_m;  // < rdf_id, sumgra_id >
  int n = 0;
  for (auto it_r = mappedEdges.begin(); it_r != mappedEdges.end(); ++it_r) {
    auto it = node_m.find(it_r->first.first);
    if (it == node_m.end()) {
      node_m.insert(make_pair(it_r->first.first, n));
      ++n;
    }
    it = node_m.find(it_r->first.second);
    if (it == node_m.end()) {
      node_m.insert(make_pair(it_r->first.second, n));
      ++n;
    }
  }
  // ofstream map_out; // < sumgra_id, rdf_id>
  outF.open(path + "vertex_map_algo.txt");
  for (auto it = node_m.begin(); it != node_m.end(); ++it)
    outF << it->second << " " << it->first << endl;
  outF.close();


  // ofstream out_file_3; // write the node labels to the output file;
  outF.open(path + "nodes.txt");
  for (auto it_n = authorities.begin(); it_n != authorities.end(); ++it_n) {
    // auto it = nodeAttributes.find((*it_n));
    // if (it == nodeAttributes.end())
    outF << -1 << endl; // no labels attached and hence assign the unique value (highest possible number or -1) |
    // else {
    //     for (int j = 0; j < it->second.second.size()-1; ++j)
    //         outF << it->second.second[j] << ",";
    //     outF << it->second.second[it->second.second.size()-1] << endl;
    // }
  }
  outF.close();

  if (authorities.size()-1 != (*authorities.rbegin())) {
    ofstream outFn;
    outFn.open(path + "node_ids.txt");
    for (auto it = authorities.begin(); it != authorities.end(); ++it)
      outFn << (*it) << endl;
    outFn.close();
  }

  ofstream outFM;
  outFM.open(path + "metadata.txt");
  outFM << "# nodes: " << authorities.size() << endl;
  outFM << "# edges: " << mappedEdges.size() << endl;
  outFM << "# multi edges: " << multiEdges <<  endl;
  outFM << "# unique dim: " << predicates.size() << endl;
  outFM << "# self loops: " << selfLoops << endl;
  // outFM << "# vertex labels: " << attributeMap.size() << endl;
  outFM << "# rdf triples: " << mappedRdf.size() << endl;

  // if (authorities.size()-1 == (*authorities.rbegin()))
  //   outFM << "It's a connected graph!" << endl;
  // else
  //   outFM << "The RDF graph is disconnected!" << endl;

  outFM.close();
}

/// Read RDF data by considering literals as vertex labels;

    /// File Preprocessing
      // Replace Tab or white space characters by only whitespace using bash:
      // usage: translate ['existing delimiter'] ['required delimiter'] [infile] [outfile]
      // tr ' \t ' ' ' < in.txt > out.txt

    /// File cleaning
      // tr -d '\r' < infile.txt > outfile.txt // to remove carriage returns that come from windows machines

void RdfParser::createGraphDatabase(VecOfStr inArgs) // code execution: ./output d "/home/vijay/Phd/MULTIGRAPHS/trunk/dataset/rdf/" freebase-wiki
{
  clock_t load_time = clock();

  std::string inFile = inArgs[0] + inArgs[1];
  ifstream file_stream (inFile);
  std::map<std::string, int> nodeMap; // S/O -> an integer for vertex id |
  std::map<std::string, int> predicateMap; // P -> an integer for edge dimension |
  // std::map<std::pair<std::string, std::string>, int> attributeMap; // P:O pair -> an integer for vertex label |
  // std::map<int, pair<std::vector<std::pair<std::string, std::string>>, std::vector<int>>> nodeAttributes; // <node, <n_lable, n_label_id>>
  int nNodes = 0, nEdges = 0;
  Vector2D mappedRdf;
  int cnt = 0;
  FileManager textLine;

  if (file_stream.is_open()) {
    std::string line;
    while(getline(file_stream, line)) {
      if (line.at(line.length()-1) == '.')
        line.pop_back(); // Remove the dot |
      else {
        cout << "The RDF data is missing a dot at the end of a triple" << endl;
        exit(0);
      }
      if (line.at(line.length()-1) == ' ')
        line.pop_back(); // Remove the white space |

      VecOfStr strs;
      textLine.splitString(line, ' ', strs);
      std::vector<std::string> line_contents(3);
      if (strs.size() > 3) {
        line_contents[0] = strs[0];
        line_contents[1] = strs[1];
        line_contents[2] = strs[2];
        for (int i = 0; i < strs.size()-3; ++i)
          line_contents[2] = line_contents[2] + " " + strs[i+3];
      }
      else if (strs.size() == 3)
        line_contents = strs;
      else
        cout << "Not a valid triple!" << endl;

      std::vector<int> line_rdf(3);
      // Check if the item is an "OBJECT" or a "LITERAL"
      bool edge_exists = true;
//            if (line_contents[2].at(0) == '<' || line_contents[2].at(0) == '_') { // if "O" is an Object, execute this code;
      if (line_contents[0].at(0) == '<') {  // "S" is a valid subject |

//                if (line_contents[0].at(0) == '<' && line_contents[2].at(0) == '<') { // if "O" is an Object, and "S" is a valid subject |
        auto it = nodeMap.find(line_contents[0]); // ADD SUBJECT:- node:source
        if (it != nodeMap.end())
          line_rdf[0] = it->second;
        else {
          nodeMap.insert(make_pair(line_contents[0], nNodes));
          line_rdf[0] = nNodes;
          ++nNodes;
        }
        it = nodeMap.find(line_contents[2]); // ADD OBJECT:- node:destination
        if (it != nodeMap.end())
          line_rdf[1] = it->second;
        else {
          nodeMap.insert(make_pair(line_contents[2], nNodes));
          line_rdf[1] = nNodes;
          ++nNodes;
        }
        it = predicateMap.find(line_contents[1]); // ADD PREDICATE:- edge
        if (it != predicateMap.end())
          line_rdf[2] = it->second;
        else {
          predicateMap.insert(make_pair(line_contents[1], nEdges));
          line_rdf[2] = nEdges;
          ++nEdges;
        }
        mappedRdf.push_back(line_rdf); // < node:source, node:destination, edge >;
      }
      else {
        cout << "Invalid SUBJECT format for line no: " << cnt << endl;
        cout << line_contents[0] << endl;
      }
      if (cnt % 1000000 == 0)
        cout << cnt << endl;
      ++cnt;
    }
  }
  else {
    cout << "Could not open the file!" << endl;
    return;
  }

  writeToDatabase(nodeMap, predicateMap, mappedRdf, inArgs[0]);
  double l_time = double(clock() - load_time)/CLOCKS_PER_SEC;
  cout << "Database construction time: " << l_time << " seconds" << endl;
}
