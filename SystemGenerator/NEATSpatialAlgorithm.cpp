//
//  NEATSpatialAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/15/13.
//
//

#include "NEATSpatialAlgorithm.h"

NEATSpatialAlgorithm::NEATSpatialAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_c, float p_m_attach, float p_m_node, float p_m_conn, int systemWidth, int systemHeight)
:NEATAlgorithm(populationSize, maxGenerations, maxStagnation, p_c, p_m_attach, p_m_node, p_m_conn, systemWidth, systemHeight)
{
    simSteps = 100;
}

MachineSystem * NEATSpatialAlgorithm::createInitialSystem()
{
    MachineSystem *s = new MachineSystem(300, 300, sys_w, sys_h, cpvzero);
    s->destroyAttachments = true;
    return s;
}

void NEATSpatialAlgorithm::prepareInitialPopulation()
{
    MachineSystem *initialSystem = createInitialSystem();
    neatGenerator(initialSystem);
    
    AttachmentInnovation info;
    info.pos1 = initialSystem->inputMachinePosition;
    info.pos2 = initialSystem->outputMachinePosition;
    // create one of each attachment type, allow destruction of attachments instead of disabling
    for (int i=0; i< ATTACH_TYPE_MAX; i++) {
        MachineSystem *newSys = new MachineSystem(*initialSystem);
       
        
        Attachment *newAtt = Attachment::createAttachmentOfType((AttachmentType)i);
        newSys->updateAttachmentBetween(newSys->inputMachinePosition, newSys->outputMachinePosition, newAtt);
        assignInnovationNumberToAttachment(newAtt, info);
        ExtendedSystemInfo *s = new ExtendedSystemInfo(simSteps);
        s->system = newSys;
        population.push_back(s);
    }
    delete initialSystem;
}

void NEATSpatialAlgorithm::assignInnovationNumberToAttachment(Attachment *att, AttachmentInnovation info)
{
         // create a number - the first part number + size*secondPartNumber
        int part1Num = sys_w * info.pos1.y + info.pos1.x;
        int part2Num = sys_w * info.pos2.y + info.pos2.x;
        
        int totalParts = sys_w*sys_h;
        
        int innovationNumber;
        if (part1Num > part2Num) {
            innovationNumber = part1Num * totalParts + part2Num;
        } else {
            innovationNumber = part2Num * totalParts + part1Num;
        }
        
        att->innovationNumber = info.innovationNumber = innovationNumber;
}

// return true if a1 < a2
static bool compareGenes (const AttachmentInnovation &a1, const AttachmentInnovation &a2)
{
    return a1.innovationNumber < a2.innovationNumber;
}

MachineSystem * NEATSpatialAlgorithm::combineSystems(MachineSystem *sys1, MachineSystem *sys2)
{
    
    if (sys1 == sys2) {
        // breeding with myself?
        return new MachineSystem(*sys1);
    }
    
    MachineSystem *newChild = createInitialSystem();
    std::vector<AttachmentInnovation> genome1 = sys1->attachmentGenome();
    std::vector<AttachmentInnovation> genome2 = sys2->attachmentGenome();
    
    // holy crap, c++ has some good standard methods, like *set difference*
    std::vector<AttachmentInnovation> matchingGenes1;
    std::vector<AttachmentInnovation> matchingGenes2;
    std::vector<AttachmentInnovation> disjointandExcess;
    
    // set intersection takes from the first range given - I do it twice so I have the matches from parent 1 and from parent 2.
    std::set_intersection(genome1.begin(), genome1.end(), genome2.begin(), genome2.end(), std::back_inserter(matchingGenes1), compareGenes);
    std::set_intersection(genome2.begin(), genome2.end(), genome1.begin(), genome1.end(),std::back_inserter(matchingGenes2), compareGenes);
    
    // difference takes things in the first range that are not in the second - perfect if we assume parent 1 is more fit
    std::set_difference(genome1.begin(), genome1.end(), genome2.begin(), genome2.end(), std::back_inserter(disjointandExcess), compareGenes);
    
    
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
    
    assert(newChild->partAtPosition(newChild->inputMachinePosition) != NULL);
    assert(newChild->partAtPosition(newChild->outputMachinePosition) != NULL);
    
    return newChild;
}

static bool compareInnovationNumbers(const AttachmentInnovation a1, const AttachmentInnovation a2)
{
    return a1.innovationNumber < a2.innovationNumber;
}
cpFloat NEATSpatialAlgorithm::genomeDistance(MachineSystem *sys1, MachineSystem *sys2)
{
    std::vector<AttachmentInnovation> genome1 = sys1->attachmentGenome();
    std::vector<AttachmentInnovation> genome2 = sys2->attachmentGenome();
    
    // find the longer one
    size_t longerSize = (genome1.size() > genome2.size() ? genome1.size()  : genome2.size());
    
    size_t nDisjoint = 0;
    size_t nMatching = 0;
    size_t matchingDiff = 0;
    
    // yay stdlib!
    std::vector<AttachmentInnovation> matchingGenes1;
    std::vector<AttachmentInnovation> matchingGenes2;
    std::vector<AttachmentInnovation> disjointGenes;
    
    
    // set intersection takes from the first range given - I do it twice so I have the matches from parent 1 and from parent 2.
    std::set_intersection(genome1.begin(), genome1.end(), genome2.begin(), genome2.end(), std::back_inserter(matchingGenes1), compareInnovationNumbers);
    std::set_intersection(genome2.begin(), genome2.end(), genome1.begin(), genome1.end(),std::back_inserter(matchingGenes2), compareInnovationNumbers);
    
    // difference takes things in the first range that are not in the second - we will have to check for excess ourself
    std::set_symmetric_difference(genome1.begin(), genome1.end(), genome2.begin(), genome2.end(), std::back_inserter(disjointGenes), compareInnovationNumbers);
  
    
    
    nMatching = matchingGenes2.size();
    // first find the distance between matching genes
    for (int i=0; i<nMatching; i++) {
        AttachmentInnovation a1 = matchingGenes1[i];
        AttachmentInnovation a2 = matchingGenes2[i];
        
        Attachment *attachment1 = sys1->attachmentBetween(a1.pos1, a1.pos2);
        Attachment *attachment2 = sys2->attachmentBetween(a2.pos1, a2.pos2);
        
        matchingDiff += attachmentDifference(attachment1, attachment2);
    }

        // else chalk everything up to disjoint
    nDisjoint = disjointGenes.size();
    
    
    double d = w_disjoint*nDisjoint/longerSize;
    
    if (nMatching == 0)
        d += 0; // all difference accounted for with disjoint set
    else
        d += w_matching *matchingDiff/nMatching;
    
    assert(!isUnreasonable(d));
    return d;
}
