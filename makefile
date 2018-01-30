INCLUDES= -I include/
CC=g++ -std=c++11
#CFLAGS=-c -O3 -fopenmp
CFLAGS=-c -O3

all: Amber.o RdfParser.o FileManager.o IndexManager.o Trie.o SparqlParser.o QueryHandler.o SolutionExplorer.o Matcher.o
		$(CC) -O3 -o amber Amber.o RdfParser.o FileManager.o IndexManager.o Trie.o SparqlParser.o QueryHandler.o SolutionExplorer.o Matcher.o 

Amber.o: Amber.cpp
	$(CC) $(CFLAGS) $(INCLUDES) Amber.cpp

RdfParser.o: src/RdfParser.cpp include/RdfParser.h
	$(CC) $(CFLAGS) $(INCLUDES) src/RdfParser.cpp
	
FileManager.o: src/FileManager.cpp include/FileManager.h
	$(CC) $(CFLAGS) $(INCLUDES) src/FileManager.cpp
	
IndexManager.o: src/IndexManager.cpp include/IndexManager.h
	$(CC) $(CFLAGS) $(INCLUDES) src/IndexManager.cpp

Trie.o: src/Trie.cpp include/Trie.h
	$(CC) $(CFLAGS) $(INCLUDES) src/Trie.cpp
	
SparqlParser.o: src/SparqlParser.cpp include/SparqlParser.h
	$(CC) $(CFLAGS) $(INCLUDES) src/SparqlParser.cpp

QueryHandler.o: src/QueryHandler.cpp include/QueryHandler.h
	$(CC) $(CFLAGS) $(INCLUDES) src/QueryHandler.cpp

SolutionExplorer.o: src/SolutionExplorer.cpp include/SolutionExplorer.h
	$(CC) $(CFLAGS) $(INCLUDES) src/SolutionExplorer.cpp

Matcher.o: src/Matcher.cpp include/Matcher.h
	$(CC) $(CFLAGS) $(INCLUDES) src/Matcher.cpp
								

clean:
	rm -f *.o
	rm amber
