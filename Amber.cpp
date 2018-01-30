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

#include "include/AmberLib.h"
#include "include/FileManager.h"
#include "include/IndexManager.h"
#include "include/SparqlParser.h"
#include "include/QueryHandler.h"
#include "include/SolutionExplorer.h"
#include "include/RdfParser.h"

void queryingSparql(VecOfStr inArgs)
{

  std::string dataPath = inArgs[0];
  std::string queryPath = inArgs[1];
  std::string resPath = inArgs[2];
  std::string nQueries = inArgs[3];
  std::string printSols = inArgs[4];

  /// Read RDF graph data
  cout << "Loading RDF graph database..." << endl;
  FileManager dataGraph;
  RdfMapping rdfMap;
  dataGraph.readMappings(dataPath, rdfMap);
  GraphParameter dataGraphInfo;
  dataGraphInfo.nPred = rdfMap.eLabels.size();
  dataGraph.readContents(dataPath, dataGraphInfo);

  /// Build indexes
  IndexType index;
  IndexManager graphIndex;
  graphIndex.buildRdfIndexes(dataGraphInfo, rdfMap, index);

  int nQ = 0;
  while(nQ < std::stoi(nQueries)) {
    /// Read SPARQL query
    clock_t start = clock();
    std::string qyeryFile = queryPath + "q_" + std::to_string(nQ) + ".txt"; // for our queries |
    SparqlParser sparqlParser;
    SparqlMapping sparqlMap;
    sparqlParser.parseQuery(qyeryFile, rdfMap, dataGraphInfo.nPred, sparqlMap);
    QueryParameter queryGraphInfo;
    QueryHandler queryHandler;
    queryHandler.getQueryContents(sparqlMap, dataGraphInfo.nPred, rdfMap.vLabels.size(), queryGraphInfo);
    QueryParameter decomposedQuery;
    queryHandler.decomposeQuery(sparqlMap, dataGraphInfo.nPred, queryGraphInfo, decomposedQuery);

    /// Perform querying
    SolutionExplorer querySolution;
    querySolution.findSolutions(index, dataGraphInfo.edgeFrequency, sparqlMap, queryGraphInfo, decomposedQuery);
    double queryTime = double(clock() - start) / CLOCKS_PER_SEC;

    /// Output results
    // cout << queryGraphInfo.queryType << endl;
    if(!queryGraphInfo.timedOut)
        cout << "q_" << nQ << ":\t" << queryGraphInfo.nSolutions << " (#Emb)\t" << (queryTime)*1000 << " (time[ms])" << endl;
    else
        cout << "q_" << nQ << ":\tTimed out!" << endl;

    FileManager results;
    std::string pT = resPath + "time.txt";
    results.printStatistics(queryGraphInfo, pT, nQ, queryTime);

    std::string pS = resPath + "solutions_" + std::to_string(nQ) + ".txt";
    if (printSols == "-yes" && !queryGraphInfo.timedOut)
      results.printSolutions(queryGraphInfo, rdfMap, sparqlMap, pS);
    ++nQ;
  }
}

int main(int argc, char* argv[])
{
  char * parameter = argv[1];
  VecOfStr inArgs(argc-2);
  for (int i = 0; i < argc-2; ++i)
    inArgs[i] = argv[i+2];

  switch(*parameter) {
    case 'd' :{
      RdfParser rdf;
      rdf.createGraphDatabase(inArgs);
    } break;
    case 'q' :
      queryingSparql(inArgs);
      break;
    default :
      cout << "Invalid parameter!" << endl;
  }
  return 0;
}
