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

#include "SparqlParser.h"

SparqlParser::SparqlParser()
{
  //ctor
}

SparqlParser::~SparqlParser()
{
  //dtor
}

void SparqlParser::parseQuery(const std::string& qFile, const RdfMapping& rdfMap, const int& nPred, SparqlMapping& sparqlMap)
{
  ifstream sparqlFile(qFile);
  int q_n = 0;
  if (sparqlFile.is_open()) {
    std::string line;
    while(getline(sparqlFile, line)){
      VecOfStr strs;
      if ((line.at(0) == '<' || line.at(0) == '?' || line.at(0) == '$')) { // If the line is a triple, check its format |
        if (line.at(line.length()-1) == '.')
          line.pop_back(); // Remove the dot |
        else {
          cout << "The SPARQL query is missing a dot at the end of triplet" << endl;
            exit(0);
        }
        if (line.at(line.length()-1) == ' ')
          line.pop_back(); // Remove the white space |
      }
      FileManager operation;
      operation.splitString(line, ' ', strs);
      if ((strs[0].at(0) == '<' || strs[0].at(0) == '?' || strs[0].at(0) == '$')) { // Process the line if it is an rdf triplet |
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
          cout << "Not a valid triplet!" << endl;
  //                    if (line_contents[2].back() == '.')
  //                        line_contents[2].pop_back(); // the 'dot' at the end is removed, if it is NOT followed by a space " " |
        if ( (line_contents[2].at(0) == '?' && line_contents[0].at(0) == '?') || (line_contents[2].at(0) == '$' && line_contents[0].at(0) == '$') ) { // An edge with edge_label exists |
          int src, dst;
          auto it = sparqlMap.nodes.find(line_contents[0]);
          if (it == sparqlMap.nodes.end()) {
            sparqlMap.nodes.insert(make_pair(line_contents[0], q_n));
            src = q_n;
            ++q_n;
          }
          else
            src = it->second;
          // if (line_contents[0] == line_contents[2]) // Performed in QueryHandler.
          //     sparqlMap.loopNodes.insert(src); // collect the self-loop nodes |
          it = sparqlMap.nodes.find(line_contents[2]);
          if (it == sparqlMap.nodes.end()) {
            sparqlMap.nodes.insert(make_pair(line_contents[2], q_n));
            dst = q_n;
            ++q_n;
          }
          else
            dst = it->second;
          std::pair<int,int> edge = make_pair(src,dst);
          auto it_e = sparqlMap.edges.find(edge);
          int e_label = rdfMap.eLabels.find(line_contents[1])->second + 1; // Add 1 for correct index querying |
          if (it_e == sparqlMap.edges.end()) {
            std::set<int> tmp;
            tmp.insert(e_label);
            sparqlMap.edges.insert(make_pair(edge, tmp));
          }
          else
            it_e->second.insert(e_label);
        }
        else if ( (line_contents[2].at(0) == '?' || line_contents[2].at(0) == '$') && line_contents[0].at(0) == '<' ) { // Subject is an URI and Object is an unknown variable |
          int core_node;
          auto it = sparqlMap.nodes.find(line_contents[2]);
          if (it == sparqlMap.nodes.end()) {
            sparqlMap.nodes.insert(make_pair(line_contents[2], q_n));
            core_node = q_n;
            ++q_n;
          }
          else
            core_node = it->second;

          int uri_id; // Fetch the URI-id from the data graph.
          auto it_q = sparqlMap.vLabels.find(line_contents[0]); // Fetch all the distinct labels, which are RDF constants |
          if (it_q == sparqlMap.vLabels.end()) {
            int uri_id_tmp = rdfMap.nodes.find(line_contents[0])->second;
            uri_id = rdfMap.vIdMap.find(uri_id_tmp)->second; // Map the node from RDF to Amber  AND Map to a retrievable space ( + n_v_labels) |
            sparqlMap.vLabels.insert(make_pair(line_contents[0], uri_id));
          }
          else
            uri_id = it_q->second;

          int e_label = rdfMap.eLabels.find(line_contents[1])->second + 1; // Get the corresponding edge. It is positive since the core node gets this edge | AND add 1 for correct index querying |

          auto it_c = sparqlMap.uriData.find(core_node);
          if (it_c == sparqlMap.uriData.end()) { // a new core i_d; make all fresh entries |
            std::vector<std::pair<int, std::set<int>>> uri_vals;
            std::pair<int, std::set<int>> uri_pair;
            std::set<int> e_labels;
            e_labels.insert(e_label);
            uri_pair = make_pair(uri_id, e_labels);
            uri_vals.push_back(uri_pair);
            sparqlMap.uriData.insert(make_pair(core_node, uri_vals)); // insert everything new |
          }
          else {
            bool same_uri_id = false;
            for (size_t k = 0; k < it_c->second.size(); ++k) {
              if(it_c->second[k].first == uri_id) {
                it_c->second[k].second.insert(e_label); // only insert a new edge label |
                same_uri_id = true;
                break;
              }
            }
            if (!same_uri_id) {
              std::pair<int, std::set<int>> uri_pair;
              std::set<int> e_labels;
              e_labels.insert(e_label);
              uri_pair = make_pair(uri_id, e_labels);
              it_c->second.push_back(uri_pair); // insert a new pair |
            }
          }
        }
  //                    else if ( (line_contents[0].at(0) == '?' || line_contents[0].at(0) == '$') && line_contents[2].at(0) == '<' ) { // Subject is an Unknown variable, Object is an URI |
        else if ( (line_contents[0].at(0) == '?' || line_contents[0].at(0) == '$') && (line_contents[2].at(0) != '?' || line_contents[2].at(0) != '$') ) { // Subject is an Unknown variable, Object is an URI/Constant |
          int core_node;
          auto it = sparqlMap.nodes.find(line_contents[0]);
          if (it == sparqlMap.nodes.end()) {
            sparqlMap.nodes.insert(make_pair(line_contents[0], q_n));
            core_node = q_n;
            ++q_n;
          }
          else
            core_node = it->second;

          int uri_id; // Fetch the URI-id from the data graph.
          auto it_q = sparqlMap.vLabels.find(line_contents[2]); // Fetch all the distinct labels, which are RDF constants |
          if (it_q == sparqlMap.vLabels.end()) {
            int uri_id_tmp = rdfMap.nodes.find(line_contents[2])->second;
            uri_id = rdfMap.vIdMap.find(uri_id_tmp)->second; // Map the node from RDF to Amber  AND Map to a retrievable space ( + n_v_labels) |
            sparqlMap.vLabels.insert(make_pair(line_contents[2], uri_id));
          }
          else
            uri_id = it_q->second;

          int e_label = rdfMap.eLabels.find(line_contents[1])->second + nPred + 1; // Get the corresponding edge; It is negative since the core node looses this edge | AND add 1 for correct index querying |

          auto it_c = sparqlMap.uriData.find(core_node);
          if (it_c == sparqlMap.uriData.end()) { // a new core i_d; make all fresh entries |
            std::vector<std::pair<int, std::set<int>>> uri_vals;
            std::pair<int, std::set<int>> uri_pair;
            std::set<int> e_labels;
            e_labels.insert(e_label);
            uri_pair = make_pair(uri_id, e_labels);
            uri_vals.push_back(uri_pair);
            sparqlMap.uriData.insert(make_pair(core_node, uri_vals)); // insert everything new |
          }
          else {
            bool same_uri_id = false;
            for (size_t k = 0; k < it_c->second.size(); ++k) {
              if(it_c->second[k].first == uri_id) {
                it_c->second[k].second.insert(e_label); // only insert a new edge label |
                same_uri_id = true;
                break;
              }
            }
            if (!same_uri_id) {
              std::pair<int, std::set<int>> uri_pair;
              std::set<int> e_labels;
              e_labels.insert(e_label);
              uri_pair = make_pair(uri_id, e_labels);
              it_c->second.push_back(uri_pair); // insert a new pair |
            }
          }
        }
      }
    }
  }
  else {
    cout << "Unable to open query file: " << qFile << endl;
  }
}
