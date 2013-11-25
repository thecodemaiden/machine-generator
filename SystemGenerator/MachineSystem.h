//
//  MachineSystem.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

#ifndef __SystemGenerator__MachineSystem__
#define __SystemGenerator__MachineSystem__

#include "MachinePart.h"
#include <vector>


#pragma mark - For NEAT
struct AttachmentInnovation {
    cpVect pos1;
    cpVect pos2;
    int innovationNumber;
        
    // equality
    bool operator==(const AttachmentInnovation& other)const {
        return (cpveql(pos1, other.pos1) && cpveql(pos2, other.pos2)) || (cpveql(pos1, other.pos2) && cpveql(pos2, other.pos1));
    }
    
    
    bool operator!=(const AttachmentInnovation& other)const {
        return !(*this==other);
    }
    
};
#pragma mark -

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
    bool detachMachines(cpVect machine1Pos, cpVect machine2Pos); // if true, destroy the attachment, else disable
    
    void updateAttachmentToWall(cpVect gridPosition, Attachment *newAttachment);
    
    void getRandomAttachment(Attachment **attachment, cpVect *pos1, cpVect *pos2); // does not return disabled attachments
    void getRandomPartPosition(cpVect *partPosition);
    
    void getRandomDisjointParts(cpVect *pos1, cpVect *pos2);
    void getRandomEmptySpot(cpVect *partPosition);

    void updateAttachmentBetween(cpVect machine1Pos, cpVect machine2Pos, Attachment *newAttachment);
    
    cpSpace *getSpace();
    cpVect getSize();
    cpVect getSpacing();
    
    int getNumberOfParts();
    int getNumberOfAttachments();
    
    void saveToDisk(const char* filename);
    static MachineSystem * loadFromDisk(const char* filename, cpFloat wallWidth=100, cpFloat wallHeight = 100, cpVect position=cpvzero);
    
    ~MachineSystem(); // frees machines and attachments too (should it?)
    
    // for NEAT
    std::vector<AttachmentInnovation> attachmentGenome(bool sorted=true);
    bool destroyAttachments; // if true, destroy attachments instead of disabling them
    
private:
    cpBody *body;
    cpSpace *space;
    
    std::vector<MachinePart *> parts;
    std::vector< std::vector<Attachment*> > attachments;
    cpVect gridSpacing;

    cpVect size; // n of pegs horizontally and vertically
    
    int nMachines;
    int nAttachments; //does not count attachment to walls or disabled attachments
    
    
    int machinePositionToNumber(cpVect gridPosition);
    cpVect machineNumberToPosition(int machineNumber);
    cpVect gridPositionToWorld(cpVect gridPosition);
};

#endif /* defined(__SystemGenerator__MachineSystem__) */
