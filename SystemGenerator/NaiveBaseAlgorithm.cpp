//
//  NaiveBaseAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/14/13.
//
//

#include "NaiveBaseAlgorithm.h"
#include <numeric>
#include <sstream>
#include <wordexp.h>

// basic implementation of constructor and destructor for subclasses to build on
NaiveAlgorithm::NaiveAlgorithm(int maxGenerations, int maxStagnation, float p_m, float p_c)
:p_m(p_m),
p_c(p_c),
maxStagnation(maxStagnation),
maxGenerations(maxGenerations)
{
    generations = 0;
};

NaiveAlgorithm::~NaiveAlgorithm()
{
    currentLogFile.close();
}

long NaiveAlgorithm::getNumberOfIterations()
{
    return generations;
}

 char * NaiveAlgorithm::inputDescription()
{
    return "";
}

 char * NaiveAlgorithm::outputDescription()
{
    return "";

}

void NaiveAlgorithm::logPopulationStatistics()
{
    if (!currentLogFile.is_open()) {
        time_t now = time(NULL);
        std::stringstream s;
        wordexp_t directory;
        wordexp("~/temp/machines/", &directory, 0);
        s << directory.we_wordv[0];
        s << "log" << now << ".log";
        std::string filename = s.str();
        
        currentLogFile.open(filename.c_str());
    }
    
    // write generation #: <list of fitnesses>
    currentLogFile << generations << ": ";
    std::vector<SystemInfo *>::iterator popIter = population.begin();
    while (popIter != population.end()) {
        currentLogFile << (*popIter)->fitness << " ";
        popIter++;
    }
    currentLogFile << "\n";
    currentLogFile.flush();
}