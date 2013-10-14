//
//  MachineSystem.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

#ifndef __SystemGenerator__MachineSystem__
#define __SystemGenerator__MachineSystem__

#include <iostream>
#include "MachinePart.h"
#include <list>
#include <vector>

class MachineSystem {
    
public:
    
    cpVect inputMachinePosition;
    cpVect outputMachinePosition;
    
    // create a new system that can attach machine components
    // each system resides in its own cpSpace (no interaction with other systems)
    MachineSystem(int width, int height, int hPegs, int vPegs, cpVect position);
    
    // deep copy!
    MachineSystem(MachineSystem &toCopy, cpVect position=cpvzero);
    
    // NOTE - don't use a GEAR attachment to attach to the wall - the peg does not rotate, so the body will not rotate
    void addPart(MachinePart *newMachine, Attachment *attachment, cpVect gridPosition);
    void removePart(cpVect gridPosition);
    
    MachinePart *partAtPosition(cpVect gridPosition);
    Attachment *attachmentBetween(cpVect machine1Pos, cpVect machine2Pos);
    Attachment *attachmentToWall(cpVect gridPosition);
    
    bool attachMachines(cpVect machine1Pos, cpVect machine2Pos, Attachment *attachment);
    bool detachMachines(cpVect machine1Pos, cpVect machine2Pos);
    
    void getRandomAttachment(Attachment **attachment, cpVect *pos1, cpVect *pos2);
    void getRandomPartPosition(cpVect *partPosition);
    
    cpSpace *getSpace();
    cpVect getSize();
    cpVect getSpacing();
    
    ~MachineSystem(); // frees machines and attachments too (should it?)
    
private:
    cpBody *body;
    cpSpace *space;
    
    std::vector<MachinePart *> parts;
    std::vector< std::vector<Attachment*> > attachments;
    cpVect gridSpacing;

    cpVect size; // n of pegs horizontally and vertically
    
    int nMachines;
    int nAttachments; //does not count attachment to walls
    
    int machinePositionToNumber(cpVect gridPosition);
    cpVect machineNumberToPosition(int machineNumber);
    cpVect gridPositionToWorld(cpVect gridPosition);
};

#endif /* defined(__SystemGenerator__MachineSystem__) */
