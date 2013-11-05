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

// the members are cleared out, repopulated, and the representative is updated from the new generation
struct NEATSpecies {
    std::vector<AttachmentInnovation> representativeGenome;
    std::vector<SystemInfo *>members;
};

class NEATAlgorithm {
protected:
    std::vector<AttachmentInnovation> newConnections;
    std::vector<SystemInfo *> population;
    

    // recombination probability
    cpFloat p_c;
    
    // mutation probabilities
    cpFloat p_m_node;
    cpFloat p_m_conn;
    
    // weights for excess, disjoint, and average matching (attachment) gene distance
    // how do we define distance between 2 different attachments?
        // propose:
        // start at 4.0 if they are different types,
        // else start with sum of ratios between attributes (i.e. smaller damping/smaller damping, smaller gearRatio/larger gearRatio etc
        // and add ratio between attachment lengths?
    cpFloat w_excess;
    cpFloat w_disjoint;
    cpFloat w_matching;

    int simSteps = 10;
    int populationSize;
    int stagnantGenerations;
    int maxStagnation;
    int maxGenerations;
    
    long generations;
    SystemInfo *bestIndividual;
    cpFloat allTimeBestFitness;
    
    int nextInnovationNumber;
    bool insertRandomAttachments = false;   // if true, make up attachments when we insert a new node between two nodes
                                            // default is to use the original attachment type

    virtual void mutateSystem(MachineSystem *original); // does not make a copy
    MachineSystem *combineSystems(MachineSystem *sys1, MachineSystem *sys2); // assumes the fitter individual is first
    
    // functions to override in subclasses
    virtual void spawnNextGeneration(); // usually recombine to get enough children then mutate them alllllll - could also add the original population to the list
    virtual MachineSystem *createInitialSystem();
    virtual void stepSystem(SystemInfo *individual);
    
    virtual cpFloat evaluateSystem(SystemInfo *sys);
    virtual bool goodEnoughFitness(cpFloat bestFitness);
    
    void evaluatePopulationFitnesses();

public:
    NEATAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_c=0.5, float p_m_node=0.5, float p_m_conn=0.5);
    ~NEATAlgorithm();
    
    bool tick();
    
    // override if needed 
    virtual MachineSystem *bestSystem(); // please override according to the representation for your algorithm
    virtual long getNumberOfIterations();
    
    // almost certainly override
    virtual  char* inputDescription();
    virtual  char* outputDescription();
    
private:
    void selectParents(SystemInfo **parent1, SystemInfo **parent2, cpFloat fitnessSum);
};

// add a method to MachineSystem?

#endif /* defined(__SystemGenerator__NEATAlgorithm__) */
