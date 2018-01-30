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

#ifndef QUERYHANDLER_H
#define QUERYHANDLER_H

#include "AmberLib.h"
#include "FileManager.h"
#include "SparqlParser.h"

class QueryHandler
{
  public:
    QueryHandler();
    virtual ~QueryHandler();
    void getQueryContents(const SparqlMapping& sparqlMap, const int& nPred, const int& nVLabels, QueryParameter& queryGraphInfo);
    void decomposeQuery(const SparqlMapping& sparqlMap, const int nPred, QueryParameter& queryGraphInfo, QueryParameter& decomposedQuery);
  private:
  protected:
};

#endif // QUERYHANDLER_H
