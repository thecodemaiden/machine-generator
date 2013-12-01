//
//  NEATSpatialAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/15/13.
//
//

#ifndef __SystemGenerator__NEATSpatialAlgorithm__
#define __SystemGenerator__NEATSpatialAlgorithm__

#include "NEATAlgorithm.h"

// disjoint and excess don't make sense... only use 'disjoint'.

class NEATSpatialAlgorithm : public NEATAlgorithm {
    
    void assignInnovationNumberToAttachment(Attachment *att, AttachmentInnovation info);
    
    MachineSystem *combineSystems(MachineSystem *sys1, MachineSystem *sys2); // assumes the fitter individual is first

    
    // make one of each attachment type
    void prepareInitialPopulation();
    
    MachineSystem *createInitialSystem();
    
    cpFloat genomeDistance(MachineSystem *sys1, MachineSystem *sys2);
protected:
    cpFloat w_disjoint = 1.0;
public:
    NEATSpatialAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_c, float p_m_attach, float p_m_node, float p_m_conn, int systemWidth=5, int systemHeight=5);
    
};

#endif /* defined(__SystemGenerator__NEATSpatialAlgorithm__) */
