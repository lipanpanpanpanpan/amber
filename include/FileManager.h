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

#ifndef FILEMANAGER_H
#define FILEMANAGER_H

#include "AmberLib.h"

typedef std::vector<std::pair<int, Vector2D>> ComplexSoln;
typedef std::vector<std::pair<int, std::vector<int>>> ComplexOrder;

struct SparqlSolutions
{
  unsigned long  nSolutions = 0;
  Vector2D normalSolutions;
  std::vector<ComplexSoln> complexSolutions;
  std::pair<std::vector<int>, Vector3D> starSolutions;
  std::vector<int> normalQueryOrder;
  std::pair<int, std::vector<int>> starQueryOrder;
  ComplexOrder complexQueryOrder;
};

struct GraphParameter
{
  int nNodes;
  int nPred; // # distinct edges
  std::vector<int> nodes;
  VecOfSet attributes;
  bool nodeLabelsExist;
  GraphParameter() : edges(2) {}
  Vector2D edges; // Seems unused
  EdgeLabel neighbourSign;
  Vector2D adjacencyList;
  EdgeLabelMap eLabelMap;
  std::vector<int> orderedNodes; // useless
  MapSet edgeFrequency; // needed for the data graph |
};

struct QueryParameter : public GraphParameter
{
  std::string queryType;
  std::set<int> loopNodes;
  std::set<std::pair<int, int>> loopEdges; // edges that form a 2 loop |
  std::map<int, std::set<int>> satellite; // <core node, satellite nodes> |
  std::map<int,int> origDecompMap, decompOrigMap; // original-query-node, new-query-node ids |
  SparqlSolutions solutions;
  unsigned long  nSolutions = 0;
  bool timedOut = false;
};

struct RdfMapping
{
  std::map<std::string, int> nodes; // Used as vertex attribute index |
  std::map<std::pair<std::string, std::string>, int> vLabels;
  std::map<std::string, int> eLabels;
  std::map<int, int> vIdMap; // <input_id, amber_id > | // is it NECESSARY?
};


class FileManager
{
  public:
    FileManager();
    virtual ~FileManager();
    void readMappings(const std::string& dPath, RdfMapping& rdfMap);
    void readContents(const std::string& dataPath, GraphParameter& graphInfo);
    void splitString(const std::string& str, char chr, std::vector<std::string>& strs);
    void printSolutions(const QueryParameter& queryGraphInfo, const RdfMapping& rdfMap, const SparqlMapping& sparqlMap, const std::string& fPath);
    void printStatistics(const QueryParameter& queryGraphInfo, const std::string& fPath, int& nQ, double& queryTime);
  protected:
  private:
};

#endif // FILEMANAGER_H
