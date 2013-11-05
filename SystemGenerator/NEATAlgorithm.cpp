//
//  NEATAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/4/13.
//
//

#include "NEATAlgorithm.h"
#include <algorithm>

NEATAlgorithm::NEATAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_c, float p_m_node, float p_m_conn)
:populationSize(populationSize), maxStagnation(maxStagnation), maxGenerations(maxGenerations), p_c(p_c), p_m_node(p_m_node), p_m_conn(p_m_conn)
{
    bestIndividual = NULL;
    nextInnovationNumber = 1;
    allTimeBestFitness = -FLT_MAX;
    generations = 0;
    
}

NEATAlgorithm::~NEATAlgorithm()
{
    std::vector<SystemInfo *>::iterator it = population.begin();
    while (it != population.end()) {
        delete *it;
        it++;
    }
}

// return true if sys1 comes before sys2
static bool compareIndividuals(const SystemInfo *sys1, const SystemInfo *sys2)
{
    return sys1->fitness > sys2->fitness;
}

MachineSystem *NEATAlgorithm::combineSystems(MachineSystem *sys1, MachineSystem *sys2)
{
    MachineSystem *newChild = new MachineSystem(300, 300, 5, 5, cpvzero);
    std::vector<AttachmentInnovation> genome1 = sys1->attachmentGenome();
    std::vector<AttachmentInnovation> genome2 = sys2->attachmentGenome();
    
    int i = 0; // position in genome1 we're examining
    int j = 0; // the position in genome2 we're examning
    while (i < genome1.size()) {
        AttachmentInnovation a1 = genome1[i];
        
        cpVect pos1 = a1.pos1;
        cpVect pos2 = a1.pos2;
        
        MachineSystem *parent = sys1;

        // we never examine things at the end of a2 that aren't in a1
        if (j < genome2.size()) {
            AttachmentInnovation a2 = genome2[j];
            
            if (a1.innovationNumber == a2.innovationNumber) {
                
                // randomly choose 1 to add to the child
                float selector = (cpFloat)rand()/RAND_MAX;
               
                if (selector <= p_c) {
                    parent = sys2;
                    pos1 = a2.pos1;
                    pos2 = a2.pos2;
                }
                
                // move both to next
                j++;
                i++;
            } else if (a1.innovationNumber < a2.innovationNumber) {
                //disjoint, belongs to a1, no need to increment a2 pointer yet - add to our child
                // move a1 along - maybe a2 will match up
                i++;
            } else if (a1.innovationNumber > a2.innovationNumber) {
                // skip this one of a2, don't evaluate a1 yet - we don't know if it's disjoint or if there is a match later in a2
                j++;
                continue;
            }
        } else {
            // more left in a1 not in a2? excess - add to child as well
            // move along a1
            i++;
        }
        
        if (!newChild->partAtPosition(pos1)) {
            MachinePart *newPart = new MachinePart(*parent->partAtPosition(pos1), newChild->getSpace());
            Attachment *wallAttachment = Attachment::copyAttachment(parent->attachmentToWall(pos1));
            newChild->addPart(newPart, wallAttachment, pos1);
        }
        
        if (!newChild->partAtPosition(pos2)) {
            MachinePart *newPart = new MachinePart(*parent->partAtPosition(pos2), newChild->getSpace());
            Attachment *wallAttachment = Attachment::copyAttachment(parent->attachmentToWall(pos2));
            newChild->addPart(newPart, wallAttachment, pos2);
        }
        
        // now attach
        Attachment *attachmentToCopy = parent->attachmentBetween(pos1, pos2);
        newChild->attachMachines(pos1, pos2, Attachment::copyAttachment(attachmentToCopy));
        
        // child needs input and ouput part... random?
        cpVect pos;
        newChild->getRandomPartPosition(&pos);
        newChild->inputMachinePosition = pos;
        do {
            newChild->getRandomPartPosition(&pos);
        } while (cpveql(pos, newChild->inputMachinePosition));
        newChild->outputMachinePosition = pos;
    }
    
    return newChild;
}

//fitness proportionate selection
void  NEATAlgorithm::spawnNextGeneration()
{
    // sort the population by fitness
    std::sort(population.begin(), population.end(), compareIndividuals);
    
    cpFloat fitnessSum = 0.0;
    std::vector<SystemInfo *>::iterator it = population.begin();
    while (it != population.end()) {
        fitnessSum += (*it)->fitness;
        it++;
    }
    
    // create population/2 children through recombination
    std::vector<SystemInfo *>newGeneration;

    do {
        SystemInfo *p1 = NULL;
        SystemInfo *p2 = NULL;
        selectParents(&p1, &p2, fitnessSum);
        
        MachineSystem *child = combineSystems(p1->system, p2->system);
        // mutate the new individual (maybe)
        mutateSystem(child);
        
        SystemInfo *i = new SystemInfo(simSteps);
        i->system = child;
        newGeneration.push_back(i);
    } while (newGeneration.size() < populationSize);
    
    // add the original population too
    population.insert(population.end(), newGeneration.begin(), newGeneration.end());
}

void NEATAlgorithm::selectParents(SystemInfo **parent1, SystemInfo **parent2, cpFloat fitnessSum)
{
    if (!parent1 || !parent2)
        return; // don't waste my time
    
    // Assume sorted population
    float selector = ((cpFloat)rand()/RAND_MAX);
    
    //pick parent 1
    std::vector<SystemInfo *>::iterator it = population.begin();
    SystemInfo *chosen = population.back();
    cpFloat cumulativeFitness = 0.0;
    while (it != population.end()) {
        cumulativeFitness += (*it)->fitness/fitnessSum;
        if (cumulativeFitness >= selector) {
            chosen = *it;
            break;
        }
        it++;
    }
    
    *parent1 = chosen;
    
    selector = ((cpFloat)rand()/RAND_MAX);
    
    //pick parent 2
     it = population.begin();
    chosen = population.back();
     cumulativeFitness = 0.0;
    while (it != population.end()) {
        cumulativeFitness += (*it)->fitness/fitnessSum;
        if (cumulativeFitness >= selector) {
            chosen = *it;
            break;
        }
        it++;
    }
    *parent2 = chosen;
}

void NEATAlgorithm::evaluatePopulationFitnesses() {
    cpFloat bestFitness = -INFINITY; // we want zero fitness
    cpFloat worstFitness = INFINITY;
    

    size_t bestPos = -1;
    size_t worstPos = -1;
    
    
    for (size_t popIter = 0; popIter <population.size(); popIter++) {
        SystemInfo *individual = population[popIter];
        
        stepSystem(individual);
        individual->fitness = evaluateSystem(individual);
        
        if (individual->fitness > bestFitness) {
            bestFitness = individual->fitness;
            bestIndividual = individual;
            bestPos = popIter;
        }
        
        if (individual -> fitness <= worstFitness) {
            worstFitness = individual->fitness;
            worstPos = popIter;
        }
    }
    
    if (bestFitness > allTimeBestFitness)
        allTimeBestFitness = bestFitness;
    
    fprintf(stderr, "BEST FITNESS: %f (%lu)\n", bestFitness, bestPos);
    fprintf(stderr, "WORST FITNESS: %f (%lu)\n", worstFitness, worstPos);
}

bool NEATAlgorithm::tick()
{
    cpFloat lastBestFitness = bestIndividual ? bestIndividual->fitness : allTimeBestFitness;
    
    spawnNextGeneration();
    evaluatePopulationFitnesses();
    
    // sort by fitness again before we truncate
    std::sort(population.begin(), population.end(), compareIndividuals);
    
    population.resize(populationSize);
    cpFloat bestFitness = bestIndividual->fitness;
    
    if (lastBestFitness == bestFitness)
        stagnantGenerations++;
    else
        stagnantGenerations = 0;
    
    bool stop =  (generations >= maxGenerations) || goodEnoughFitness(bestFitness) || (stagnantGenerations >= maxStagnation);
    
    generations++;
    newConnections.clear();

    if (stop) {
        fprintf(stderr, "ALL TIME BEST FITNESS: %f\n", allTimeBestFitness);
    }
    
    return  stop;
}

MachineSystem *NEATAlgorithm::bestSystem()
{
    return bestIndividual->system;
}

 long NEATAlgorithm::getNumberOfIterations()
{
    return generations;
}

  char* NEATAlgorithm::inputDescription()
{
    return "";
}

char* NEATAlgorithm::outputDescription()
{
    return "";
}

MachineSystem *NEATAlgorithm::createInitialSystem(){
    MachineSystem *newSystem = new MachineSystem(300, 300, 5, 5, cpvzero);
    neatGenerator(newSystem);
    return newSystem;
}

 void NEATAlgorithm::mutateSystem(MachineSystem *original)
{
    // mutate attachment or add node - or both
    float selector1 = (cpFloat)rand()/RAND_MAX;
    if (selector1 < p_m_conn) {
        // insert a new connection 
        cpVect pos1;
        cpVect pos2;
        original->getRandomDisjointParts(&pos1, &pos2);
        
        if (pos1.x != -1) {
            // join them!
            Attachment *a = Attachment::createAttachmentOfType((AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX));
            original->attachMachines(pos1, pos2, a);
            
            // update the innovation number
            AttachmentInnovation created = AttachmentInnovation();
            created.pos1 = pos1;
            created.pos2 = pos2;
            
            // have we already created this 'innovation' in this generation?
            std::vector<AttachmentInnovation>::iterator it = std::find(newConnections.begin(), newConnections.end(), created);
            if (it!= newConnections.end()) {
                a->innovationNumber = (*it).innovationNumber;
            } else {
                a->innovationNumber = nextInnovationNumber;
                created.innovationNumber = nextInnovationNumber++;
                newConnections.push_back(created);
            }
        }
    }
    
    float selector2 = (cpFloat)rand()/RAND_MAX;
    if (selector2 < p_m_node) {
        // add a node somewhere if possible
        cpVect newPosition;
        original->getRandomEmptySpot(&newPosition);
        if (newPosition.x != -1) {
            // split up two other attachments and insert this part 'between' them
            Attachment *a = NULL;
            cpVect pos1;
            cpVect pos2;
            
            original->getRandomAttachment(&a, &pos1, &pos2);
            if (!a) {
                int x = 1;
                original->getRandomAttachment(&a, &pos1, &pos2);
            }
            //assert(a);
            if (a) {
                Attachment *new1;
                Attachment *new2;

                if (insertRandomAttachments) {
                    new1 = Attachment::createAttachmentOfType((AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX));
                    new2 = Attachment::createAttachmentOfType((AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX));
                } else {
                    new1 = Attachment::createAttachmentOfType(a->attachmentType());
                    new2 = Attachment::createAttachmentOfType(a->attachmentType());
                }
                AttachmentType t;
                do {
                    t=(AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
                } while (t==ATTACH_GEAR);
                
                MachinePart *newPart = randomPart(original->getSpace(), original->getSpacing());
                original->detachMachines(pos1, pos2);
                
                original->addPart(newPart, Attachment::createAttachmentOfType(t), newPosition);
                original->attachMachines(pos1, newPosition, new1);
                original->attachMachines(pos2, newPosition, new2);
                
                // now update innovation numbers
                AttachmentInnovation created = AttachmentInnovation();
                created.pos1 = pos1;
                created.pos2 = newPosition;
                
                // have we already created this 'innovation' in this generation?
                std::vector<AttachmentInnovation>::iterator it = std::find(newConnections.begin(), newConnections.end(), created);
                if (it != newConnections.end()) {
                    new1->innovationNumber = it->innovationNumber;
                } else {
                    new1->innovationNumber = nextInnovationNumber;
                    created.innovationNumber = nextInnovationNumber++;
                    newConnections.push_back(created);
                }
                
                // now the second attachment
                created.pos1 = pos2;
                created.pos2 = newPosition;

                it = std::find(newConnections.begin(), newConnections.end(), created);
                if (it != newConnections.end()) {
                    new2->innovationNumber = it->innovationNumber;
                } else {
                    new2->innovationNumber = nextInnovationNumber;
                    created.innovationNumber = nextInnovationNumber++;
                    newConnections.push_back(created);
                }
                
            }
        }
    }
 
}

 void NEATAlgorithm::stepSystem(SystemInfo *individual)
{
    
}

 cpFloat NEATAlgorithm::evaluateSystem(SystemInfo *sys)
{
    return 1.0;
}

 bool NEATAlgorithm::goodEnoughFitness(cpFloat bestFitness)
{
    return false;
}