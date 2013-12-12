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

std::vector<SystemInfo *> NaiveAlgorithm::spawnChildren()
{
    // choose random pairs from the population to swap parts or attachments
    std::vector<SystemInfo *> toCombine = std::vector<SystemInfo *>(population);
    std::vector<SystemInfo *> children;
    while (toCombine.size() > 1) {
        int parent1Index = arc4random_uniform(toCombine.size());
        int parent2Index = parent1Index;
        while (parent2Index == parent1Index) {
            parent2Index = arc4random_uniform(toCombine.size());
        }
        MachineSystem *child1 = new MachineSystem(*toCombine[parent1Index]->system);
        MachineSystem *child2 = new MachineSystem(*toCombine[parent2Index]->system);
        
        float selector = (float)rand()/RAND_MAX;
        if (selector < p_c/2) {
            // swap attachment
            cpVect firstPos1, firstPos2, secondPos1, secondPos2;
            child1->getRandomAttachment(NULL, &firstPos1, &firstPos2);
            child2->getRandomAttachment(NULL, &secondPos1, &secondPos2);
            child1->swapAttachmentsBetweenParts(child2, secondPos1, secondPos2, firstPos1, firstPos2);
        } else if (selector < p_c) {
            // swap part
            cpVect firstPos, secondPos;
            child1->getRandomPartPosition(&firstPos);
            child2->getRandomPartPosition(&secondPos);
            child1->swapPartsAtPositions(child2, secondPos, firstPos);
        }
        
        // then mutate each
        selector = (float)rand()/RAND_MAX;
        if (selector <p_m) {
            MachineSystem *oldChild = child1;
            child1 = mutateSystem(child1);
            delete oldChild;
        }
        selector = (float)rand()/RAND_MAX;
        if (selector < p_m) {
            MachineSystem *oldChild = child2;
            child2 = mutateSystem(child2);
            delete oldChild;
        }
        
        
        // we have to erase in order, later then earlier, to keep list correct
        int first = parent1Index < parent2Index ? parent1Index : parent2Index;
        int second = parent1Index < parent2Index ? parent2Index : parent1Index;
        
        toCombine.erase(toCombine.begin()+second);
        toCombine.erase(toCombine.begin()+first);
        
        SystemInfo *spawned1 = new SystemInfo(simSteps);
        spawned1->system = child1;
        
        SystemInfo *spawned2 = new SystemInfo(simSteps);
        spawned2->system = child2;
        
        children.push_back(spawned1);
        children.push_back(spawned2);
        
    }
    return children;
}

MachineSystem *NaiveAlgorithm::mutateSystem(MachineSystem *original)
{
    int selector = arc4random_uniform(5);
    switch (selector) {
        case 0:
            return attachmentAnchorMutator2(original);
            break;
        case 1:
            return attachmentAnchorMutator(original);
            break;
        case 2:
            return attachmentMutator1(original);
            break;
        case 3:
            return attachmentMutator2(original);
            break;
        case 4:
        default:
            return attachmentMutator3(original);
            break;
    }
}


static bool isMoreFit(SystemInfo * s1, SystemInfo *s2)
{
    return s1->fitness > s2->fitness;
}

bool NaiveAlgorithm::tick()
{
    cpFloat bestFitness = -INFINITY; // we want zero fitness
    cpFloat worstFitness = INFINITY;
    
    cpFloat lastBestFitness = bestIndividual ? bestIndividual->fitness : bestFitness;

    std::vector<SystemInfo *>nextGen = spawnChildren();
    
    // add old population
    for (size_t popIter = 0; popIter <population.size(); popIter++) {
        // reset the individual... by copying!
        SystemInfo *individual = population[popIter];
        MachineSystem *original = individual->system;
        MachineSystem *copy = new MachineSystem(*original);
        individual->system = copy;
        delete original;
        
        nextGen.push_back(individual);
    }
    
    for (size_t popIter = 0; popIter <nextGen.size(); popIter++) {
        SystemInfo *individual = nextGen[popIter];
 
        stepSystem(individual);
        individual->fitness = evaluateSystem(individual);
    }
    
   
    // cut the population back down to size
    std::sort(nextGen.begin(), nextGen.end(), isMoreFit);
    
    bestIndividual = nextGen.front();
    bestFitness = bestIndividual->fitness;
    
    worstFitness = nextGen.back()->fitness;
    
    std::vector<SystemInfo *>::iterator deleteIt = nextGen.begin()+population.size();
    while (deleteIt != nextGen.end()) {
        SystemInfo *unfit = *deleteIt;
        assert(unfit != bestIndividual);
        delete  unfit;
        deleteIt = nextGen.erase(deleteIt);
    }
    
    if (bestFitness > allTimeBestFitness)
        allTimeBestFitness = bestFitness;
    
    
    assert(nextGen.size() == population.size());
    
    population = nextGen;
    
    fprintf(stderr, "BEST FITNESS: %f \n", bestFitness);
    fprintf(stderr, "WORST FITNESS: %f \n", worstFitness);
    
    if (lastBestFitness == bestFitness)
        stagnantGenerations++;
    else
        stagnantGenerations = 0;
    
    bool stop =  (generations >= maxGenerations) || goodEnoughFitness(bestFitness) || (stagnantGenerations >= maxStagnation);
    
    generations++;
    logPopulationStatistics();
    
    if (stop) {
        fprintf(stderr, "ALL TIME BEST FITNESS: %f\n", allTimeBestFitness);
    }
    return  stop;
}

MachineSystem *NaiveAlgorithm::bestSystem()
{
    return bestIndividual->system;
}

bool NaiveAlgorithm::goodEnoughFitness(cpFloat bestFitness)
{
    return false;
}

void NaiveAlgorithm::logPopulationStatistics()
{
    if (!currentLogFile.is_open()) {
        time_t now = time(NULL);
        std::stringstream s;
        wordexp_t directory;
        memset(&directory, 0, sizeof(wordexp_t));
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