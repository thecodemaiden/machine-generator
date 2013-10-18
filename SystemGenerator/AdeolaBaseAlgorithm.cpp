//
//  AdeolaBaseAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/14/13.
//
//

#include "AdeolaBaseAlgorithm.h"
#include <numeric>

// basic implementation of constructor and destructor for subclasses to build on
AdeolaAlgorithm::AdeolaAlgorithm(int maxGenerations, int maxStagnation, float p_m, float p_c)
:p_m(p_m),
p_c(p_c),
maxStagnation(maxStagnation),
maxGenerations(maxGenerations)
{
    generations = 0;
};



long AdeolaAlgorithm::getNumberOfIterations()
{
    return generations;
}

 char * AdeolaAlgorithm::inputDescription()
{
    return "";
}

 char * AdeolaAlgorithm::outputDescription()
{
    return "";

}