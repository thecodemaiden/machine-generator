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
    stagnantGenerations = 0;
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

//static int indexOfInnovationNumberInGenome(std::vector<AttachmentInnovation> genome, int in, int start) {
//    int loc = -1;
//    
//    std::vector<AttachmentInnovation>::iterator it = genome.begin()+start;
//    
//    while (it != genome.end()) {
//        if (it->innovationNumber == in) {
//            loc = int(it - genome.begin());
//            break;
//        }
//        it++;
//    }
//    
//    return loc;
//}

static bool compareInnovationNumbers(const AttachmentInnovation a1, const AttachmentInnovation a2)
{
    return a1.innovationNumber < a2.innovationNumber;
}

static void addGeneFromParentSystem(MachineSystem *parent, AttachmentInnovation gene, MachineSystem *newChild)
{
    cpVect pos1 = gene.pos1;
    cpVect pos2 = gene.pos2;
    
    
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
    Attachment *copy = Attachment::copyAttachment(attachmentToCopy);
    copy->innovationNumber = gene.innovationNumber;
    newChild->attachMachines(pos1, pos2, copy);
}

MachineSystem *NEATAlgorithm::combineSystems(MachineSystem *sys1, MachineSystem *sys2)
{
    MachineSystem *newChild = new MachineSystem(300, 300, 5, 5, cpvzero);
    std::vector<AttachmentInnovation> genome1 = sys1->attachmentGenome();
    std::vector<AttachmentInnovation> genome2 = sys2->attachmentGenome();
    
    
    // holy crap, c++ has some good standard methods, like *set difference*
    std::vector<AttachmentInnovation> matchingGenes1;
    std::vector<AttachmentInnovation> matchingGenes2;
    std::vector<AttachmentInnovation> disjointandExcess;
    
    // set intersection takes from the first range given - I do it twice so I have the matches from parent 1 and from parent 2.
    std::set_intersection(genome1.begin(), genome1.end(), genome2.begin(), genome2.end(), std::back_inserter(matchingGenes1), compareInnovationNumbers);
    std::set_intersection(genome2.begin(), genome2.end(), genome1.begin(), genome1.end(),std::back_inserter(matchingGenes2), compareInnovationNumbers);
    
    // difference takes things in the first range that are not in the second - perfect if we assume parent 1 is more fit
    std::set_difference(genome1.begin(), genome1.end(), genome2.begin(), genome2.end(), std::back_inserter(disjointandExcess), compareInnovationNumbers);
    

    // first we handle the matching genes
    for (int i=0; i<matchingGenes1.size(); i++){
            AttachmentInnovation gene = matchingGenes1[i];
            MachineSystem *parent = sys1;
                // randomly choose 1 to add to the child
                float selector = (cpFloat)rand()/RAND_MAX;
            
            if (selector <= p_c) {
                parent = sys2;
                gene = matchingGenes2[i];
            }
        addGeneFromParentSystem(parent, gene, newChild);
    }
    
    // then the disjoint genes
    for (int i=0; i<disjointandExcess.size(); i++) {
        addGeneFromParentSystem(sys1, disjointandExcess[i], newChild);
    }
    
                // child needs input and ouput part... copy from parent 1
    newChild->inputMachinePosition = sys1->inputMachinePosition;
    newChild->outputMachinePosition = sys1->outputMachinePosition;
    
//        cpVect pos;
//        newChild->getRandomPartPosition(&pos);
//        newChild->inputMachinePosition = pos;
//        do {
//            newChild->getRandomPartPosition(&pos);
//        } while (cpveql(pos, newChild->inputMachinePosition));
//        newChild->outputMachinePosition = pos;
    
    
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

void NEATAlgorithm::speciate()
{
    std::vector<SystemInfo *>::iterator populationIter = population.begin();
    
    while (populationIter != population.end()) {
        // assign to species
        bool added= false;
        
        std::vector<NEATSpecies *>::iterator speciesIterator = speciesList.begin();
        while (speciesIterator != speciesList.end()) {
            double distance = genomeDistance((*populationIter)->system, (*speciesIterator)->representative);
            if (fabs(distance) < d_threshold) {
                added = true;
                (*speciesIterator)->members.push_back(*populationIter);
                break;
            }
            speciesIterator++;
        }
      
        if (!added) {
            // new species!!
            NEATSpecies *s = new NEATSpecies();
            s->representative = new MachineSystem(*(*populationIter)->system);
            s->members.push_back(*populationIter);
            speciesList.push_back(s);
        }
        
        populationIter++;
    }
    fprintf(stderr, "%ld species\n", speciesList.size());
    
    // for each species, copy a random current member to be the representative for the next generation, and adjust the fitnesses for sharing
    // kill off a species with no members
    std::vector<NEATSpecies *>::iterator speciesIterator = speciesList.begin();
    while (speciesIterator != speciesList.end()) {
        // pick a new rep
        std::vector<SystemInfo *> members = (*speciesIterator)->members;
        if (members.size() == 0) {
            NEATSpecies *extinctSpecies = *speciesIterator;
            speciesIterator = speciesList.erase(speciesIterator);
            delete extinctSpecies;
        } else {
            
            int index = arc4random_uniform(members.size());
            MachineSystem *rep = new MachineSystem(*(members[index]->system));
            delete (*speciesIterator)->representative;
            (*speciesIterator)->representative = rep;
            
           // fprintf(stderr, "\tSpecies %d: %ld members\n", ++i, members.size());
            
            // update the fitnesses
            double totalFitness = 0.0;
            std::vector<SystemInfo *>::iterator memberIterator = members.begin();
            while (memberIterator != members.end()) {
                (*memberIterator)->fitness /= members.size();
                totalFitness += (*memberIterator)->fitness;
                memberIterator++;
            }
            (*speciesIterator)->totalSharedFitness = totalFitness;
            speciesIterator++;
        }
    }
}

static double normalize(double x1, double x2)
{
    if (x1==0 && x2== 0)
        return 0.0;
    return (x1-x2)/(x1+x2);
}

static double smallerToLarger(double x1, double x2)
{
    if (x1 > x2)
        return x2/x1;
    return x1/x2;
}

// compute difference
// propose:
// start at 4.0 if they are different types,
// else start with sum of ratios between attributes
// and add ratio between attachment lengths?
static double attachmentDifference(Attachment *a1, Attachment *a2)
{

    double diff = normalize(a1->attachmentLength, a2->attachmentLength);
    
    if (a1->attachmentType() == a2->attachmentType()) {
        switch (a1->attachmentType()) {
            case ATTACH_GEAR:
            {
                diff += normalize(((GearAttachment *)a1)->gearRatio, ((GearAttachment *)a2)->gearRatio);
                
                diff += normalize(((GearAttachment *)a1)->phase, ((GearAttachment *)a2)->phase);
                
                break;
            }
            case ATTACH_PIVOT:
            {
                diff += normalize(((PivotAttachment *)a1)->pivotPosition, ((PivotAttachment *)a2)->pivotPosition);
                break;
            }
            case ATTACH_SLIDE:
            {
                diff += normalize(((SlideAttachment *)a1)->minDistance, ((SlideAttachment *)a2)->minDistance);
        
                diff += normalize(((SlideAttachment *)a1)->maxDistance, ((SlideAttachment *)a2)->maxDistance);
                break;
            }
            case ATTACH_SPRING:
            {
                diff += normalize(((SpringAttachment *)a1)->damping, ((SpringAttachment *)a2)->damping);
                
                diff += normalize(((SpringAttachment *)a1)->stiffness, ((SpringAttachment *)a2)->stiffness);
                break;
            }
            case ATTACH_FIXED:
                // nothing to do, fall through
            default:
                break;
        }
    } else {
        diff = diff + 4.0;
    }

    return diff;
}


cpFloat NEATAlgorithm::genomeDistance(MachineSystem *sys1, MachineSystem *sys2)
{
    std::vector<AttachmentInnovation> genome1 = sys1->attachmentGenome();
    std::vector<AttachmentInnovation> genome2 = sys2->attachmentGenome();
    
    // find the longer one
    size_t longerSize = (genome1.size() > genome2.size() ? genome1.size()  : genome2.size());
    
    size_t nDisjoint = 0;
    size_t nExcess = 0;
    size_t nMatching = 0;
    size_t matchingDiff = 0;

    // yay stdlib!
    std::vector<AttachmentInnovation> matchingGenes1;
    std::vector<AttachmentInnovation> matchingGenes2;
    std::vector<AttachmentInnovation> disjointFromSys1;
    std::vector<AttachmentInnovation> disjointFromSys2;
    

    // set intersection takes from the first range given - I do it twice so I have the matches from parent 1 and from parent 2.
    std::set_intersection(genome1.begin(), genome1.end(), genome2.begin(), genome2.end(), std::back_inserter(matchingGenes1), compareInnovationNumbers);
    std::set_intersection(genome2.begin(), genome2.end(), genome1.begin(), genome1.end(),std::back_inserter(matchingGenes2), compareInnovationNumbers);
    
    // difference takes things in the first range that are not in the second - we will have to check for excess ourself
    std::set_difference(genome1.begin(), genome1.end(), genome2.begin(), genome2.end(), std::back_inserter(disjointFromSys1), compareInnovationNumbers);
    std::set_difference(genome2.begin(), genome2.end(), genome1.begin(), genome1.end(), std::back_inserter(disjointFromSys2), compareInnovationNumbers);

    
    nMatching = matchingGenes2.size();
    // first find the distance between matching genes
    for (int i=0; i<nMatching; i++) {
        AttachmentInnovation a1 = matchingGenes1[i];
        AttachmentInnovation a2 = matchingGenes2[i];
        
        Attachment *attachment1 = sys1->attachmentBetween(a1.pos1, a1.pos2);
        Attachment *attachment2 = sys2->attachmentBetween(a2.pos1, a2.pos2);
  
        matchingDiff += attachmentDifference(attachment1, attachment2);
    }
    
    // now determine the excess vs disjoint
    // if one of the disjoint sets is empty, then it's all excess
    if (disjointFromSys2.size() == 0 || disjointFromSys1.size() == 0) {
        nExcess = disjointFromSys1.size() + disjointFromSys2.size();
    } else {
        // else chalk everything up to disjoint
        nDisjoint = disjointFromSys1.size() + disjointFromSys2.size();
        
        // then find the number of excess genes
        int last1 = disjointFromSys1.back().innovationNumber;
        int last2 = disjointFromSys2.back().innovationNumber;
        
        std::vector<AttachmentInnovation> shorter;
        std::vector<AttachmentInnovation> longer;
        
        if (last1 != last2) {
            if (last1 > last2) {
                // genome 1 has excess
                shorter = disjointFromSys2;
                longer = disjointFromSys1;
            } else if (last1 < last2) {
                // genome 2 has excess, switch last1 and last2
                shorter=disjointFromSys1;
                longer = disjointFromSys2;
                
                int temp = last1;
                last1 = last2;
                last2 = temp;
            }
            // go back from end of longer until we run into a value <= the end of shorter
            size_t i;
            for (i=longer.size(); i; i--) {
                if (longer[i].innovationNumber < shorter.back().innovationNumber)
                    break;
            }
            nExcess = i;
            nDisjoint -= nExcess;
        }
    }

    

    double d = w_disjoint*nDisjoint/longerSize + w_excess*nExcess/longerSize + w_matching*matchingDiff/nMatching;

    return d;
}

bool NEATAlgorithm::tick()
{
    cpFloat lastBestFitness = bestIndividual ? bestIndividual->fitness : allTimeBestFitness;
    
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
    
    speciate();
    
    // figure out how many of each species to save
    double sharedFitnessSum = 0.0;
    for (int i=0; i<speciesList.size(); i++) {
        sharedFitnessSum += speciesList[i]->totalSharedFitness;
    }
    
    std::vector<SystemInfo *> individualsToSave;
    
    for (int i=0; i<speciesList.size(); i++) {
        double proportionToSave = (speciesList[i]->totalSharedFitness)/sharedFitnessSum;
        int numToSave = MIN((proportionToSave*populationSize), (int)speciesList[i]->members.size());
        numToSave = MAX(numToSave, 1); // save 1 of each species
        std::sort(speciesList[i]->members.begin(), speciesList[i]->members.end(), compareIndividuals);

        fprintf(stderr, "Species %d: %d/%ld\n", i, numToSave, speciesList[i]->members.size());

        
        for (int j=0; j<numToSave; j++) {
            individualsToSave.push_back(speciesList[i]->members[j]);
        }
        
        for (int j=numToSave; j<speciesList[i]->members.size(); j++) {
            delete speciesList[i]->members[j];
        }

    }
    
    population = individualsToSave;
    
    if (lastBestFitness == bestFitness)
        stagnantGenerations++;
    else
        stagnantGenerations = 0;
    
    generations++;
    
    bool stop =  (generations >= maxGenerations) || goodEnoughFitness(bestFitness) || (stagnantGenerations >= maxStagnation);
    
    
    if (stop) {
        fprintf(stderr, "ALL TIME BEST FITNESS: %f\n", allTimeBestFitness);
    } else {
        spawnNextGeneration();
    }
    
    // empty the innovation list and clear the non-representative members from the species list
    newConnections.clear();
    
    std::vector<NEATSpecies *>::iterator speciesIterator = speciesList.begin();
    while (speciesIterator != speciesList.end()) {
       (*speciesIterator)->members.clear();
        speciesIterator++;
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