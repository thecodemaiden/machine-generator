//
//  NEATAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/4/13.
//
//

#ifndef __SystemGenerator__NEATAlgorithm__
#define __SystemGenerator__NEATAlgorithm__

#include <vector>
#include "Algorithm.h"

// each generation the members are cleared out, repopulated, and the representative is updated
struct NEATSpecies {
    MachineSystem *representative;
    std::vector<SystemInfo *>members;
    double totalSharedFitness;
    
    ~NEATSpecies() {
        delete representative;
    }
};

class NEATAlgorithm {
protected:
    std::vector<AttachmentInnovation> newConnections;
    std::vector<SystemInfo *> population;
    std::vector<NEATSpecies *>speciesList;

    // recombination probability
    cpFloat p_c;
    
    // mutation probabilities
    cpFloat p_m_node;
    cpFloat p_m_conn;
    
    // threshold distance - a difference bigger than this will exclude an individual from a species
    
    cpFloat d_threshold = 1.0;
    
    // weights for excess, disjoint, and average matching (attachment) gene distance
    // how do we define distance between 2 different attachments?
        // propose:
        // start at 4.0 if they are different types,
        // else start with sum of ratios between attributes (i.e. smaller damping/smaller damping, smaller gearRatio/larger gearRatio etc
        // and add ratio between attachment lengths?
    cpFloat w_excess = 0.5;
    cpFloat w_disjoint = 0.25;
    cpFloat w_matching = 0.25;

    int simSteps = 10;
    int populationSize;
    int stagnantGenerations;
    int maxStagnation;
    int maxGenerations;
    
    long generations;
    SystemInfo *bestIndividual;
    cpFloat allTimeBestFitness;
    
    int nextInnovationNumber;
    bool insertRandomAttachments = true;   // if true, make up attachments when we insert a new node between two nodes
                                            // default is to use the original attachment type

    virtual void mutateSystem(MachineSystem *original); // does not make a copy
    MachineSystem *combineSystems(MachineSystem *sys1, MachineSystem *sys2); // assumes the fitter individual is first
    
    // functions to override in subclasses
    virtual void spawnNextGeneration(); // usually recombine to get enough children then mutate them alllllll - could also add the original population to the list
    virtual MachineSystem *createInitialSystem();
    virtual void stepSystem(SystemInfo *individual);
    
    virtual cpFloat evaluateSystem(SystemInfo *sys);
    virtual bool goodEnoughFitness(cpFloat bestFitness);
    
    virtual cpFloat genomeDistance(MachineSystem *sys1, MachineSystem *sys2);
    
public:
    NEATAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_c=0.5, float p_m_node=0.5, float p_m_conn=0.5);
    ~NEATAlgorithm();
    
    bool tick(); // should not need to be overridden
    
    // override if needed 
    virtual MachineSystem *bestSystem(); // please override according to the representation for your algorithm
    virtual long getNumberOfIterations();
    
    // almost certainly override
    virtual  char* inputDescription();
    virtual  char* outputDescription();
    
private:
    void selectParents(SystemInfo **parent1, SystemInfo **parent2, cpFloat fitnessSum);
    void speciate(); // divide everything into species
};

#endif /* defined(__SystemGenerator__NEATAlgorithm__) */
