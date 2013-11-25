//
//  NEATMOS.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/21/13.
//
//

#ifndef __SystemGenerator__NEATMOS__
#define __SystemGenerator__NEATMOS__

#include "Algorithm.h"

// If I used templates on NEATAlgorithm I could avoid this re-writing for a multi objective version,
// but I'm trying to avoid using too many C++ idioms

struct ObjectiveData {
    std::vector<double> observedValues;
    double fitness;
};

struct MOSystemInfo {
    ObjectiveData xDisplacement;
    ObjectiveData yDisplacement;
    ObjectiveData angle;
    MachineSystem *system;
    int frontRank;
    double fitness;
};

// each generation the members are cleared out, repopulated, and the representative is updated
struct NEATMOSpecies {
    MachineSystem *representative;
    std::vector<MOSystemInfo *>members;
    double totalSharedFitness;
    
    ~NEATMOSpecies() {
        delete representative;
    }
};

class NEATMOS {
protected:
    std::vector<AttachmentInnovation> newConnections;
    std::vector<MOSystemInfo *> population;
    std::vector<NEATMOSpecies *>speciesList;
    
    // recombination probability
    cpFloat p_c;
    
    // mutation probabilities
    cpFloat p_m_node; // how likely we are to add a new node
    cpFloat p_m_conn;   // how likely we are to add a new connection
    cpFloat p_m_attach; // how likely are we to mutate each existing attachment
    
    // threshold distance - a difference bigger than this will exclude an individual from a species
    
    cpFloat d_threshold = 1.0;
    
    // weights for excess, disjoint, and average matching (attachment) gene distance
    // how do we define distance between 2 different attachments?
    // propose:
    // start at 4.0 if they are different types,
    // else start with sum of ratios between attributes (i.e. smaller damping/smaller damping, smaller gearRatio/larger gearRatio etc
    // and add differences between attachment points
    cpFloat w_excess = 0.5;
    cpFloat w_disjoint = 0.25;
    cpFloat w_matching = 0.25;
    
    int simSteps;
    int populationSize;
    int stagnantGenerations;
    int maxStagnation;
    int maxGenerations;
    int sys_w;
    int sys_h;
    
    long generations;
    MOSystemInfo *bestIndividual;
    cpFloat allTimeBestFitness;
    cpFloat lastBestFitness; // before sharing
    
    bool insertRandomAttachments;   // if true, make up attachments when we insert a new node between two nodes
    // default is false to use the original attachment type
    
    void mutateSystem(MachineSystem *original); // does not make a copy
    void spawnNextGeneration(); // recombine species to get enough children then mutate each one
    
    void addGeneFromParentSystem(MachineSystem *parent, AttachmentInnovation gene, MachineSystem *newChild); // used in combineSystems
#pragma mark - Override these functions in NEAT variants only
    virtual void assignInnovationNumberToAttachment(Attachment *att, AttachmentInnovation info);
    virtual MachineSystem *combineSystems(MachineSystem *sys1, MachineSystem *sys2); // assumes the fitter individual is first
    virtual cpFloat genomeDistance(MachineSystem *sys1, MachineSystem *sys2);
    virtual void selectParents(MOSystemInfo **parent1, MOSystemInfo **parent2, cpFloat fitnessSum);
    
#pragma mark - Override these functions any time you need
    // you can override this to do attachment mutation differently
    virtual void mutateAttachmentWeight(MachineSystem *sys, const AttachmentInnovation &attachmentInfo);
    
    virtual void stepSystem(MOSystemInfo *individual);
    
    virtual cpFloat evaluateSystem(MOSystemInfo *sys);
    virtual bool goodEnoughFitness(cpFloat bestFitness);
    
    // overriden functions cannot be called in constructors, so this is called on the first tick();
    virtual void prepareInitialPopulation();
    
    virtual MachineSystem *createInitialSystem();
    
public:
    NEATMOS(int populationSize, int maxGenerations, int maxStagnation, float p_c, float p_m_attach, float p_m_node, float p_m_conn, int systemWidth=5, int systemHeight=5);
    ~NEATMOS();
    
    // notice that this cannot be overriden.
    bool tick();
    
    MachineSystem *bestSystem();
    long getNumberOfIterations();
    
#pragma mark - Please override for your algorithm
    
    virtual  char* inputDescription();
    virtual  char* outputDescription();
    
private:
    int nextInnovationNumber;
    void speciate(); // divide everything into species
    
};
#endif /* defined(__SystemGenerator__NEATMOS__) */
