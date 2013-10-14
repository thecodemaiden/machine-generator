//
//  Type1Algorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/6/13.
//
//

#include "Algorithm.h"
#include <vector>

static MachinePart *randomPart(cpSpace *space, cpVect size)
{
    MachinePart *m = new MachinePart(space);
    m->machineType = (BodyType)arc4random_uniform(MACHINE_TYPE_MAX);
    
    if (m->machineType == MACHINE_BOX) {
        // will it be tall or wide?
        if (((float)random()/RAND_MAX) > 0.5) {
            m->length = size.x;
            m->height = arc4random_uniform(50) + 25;;
        } else {
            m->length = arc4random_uniform(50) + 25;;
            m->height = size.y;
        }
        
    } else {
        m->length = size.x; // large circle == trouble!!
    }
    
    return m;
}

static Attachment *randomAttachment()
{
    Attachment *a = new Attachment((AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX));
    return a;
}

void randomGenerator1(MachineSystem *sys)
{
    int nMachinesToPlace = arc4random_uniform(3)+2; // 2-4 machines to start
    boolean_t outputChosen = false;
    boolean_t inputChosen = false;
    
    cpVect lastMachinePos=cpv(-1,-1);
    
    cpVect systemSize = sys->getSize();
    
    while (nMachinesToPlace) {
        int mx = arc4random_uniform(systemSize.x);
        int my = arc4random_uniform(systemSize.y);
        cpVect mPos = cpv(mx, my);
        if (!sys->partAtPosition(mPos)) {
            Attachment *a = new Attachment();
            do {
                a->attachmentType = (AttachmentType) arc4random_uniform(ATTACH_TYPE_MAX);
            }while (a->attachmentType == ATTACH_GEAR); //can't attach to wall with gear...
            a->attachmentLength = arc4random_uniform(20);
            
            sys->addPart(randomPart(sys->getSpace(), sys->getSpacing()), a, mPos);
            
            // join to previous machine
            if (lastMachinePos.x >= 0 ) {
                Attachment *a = randomAttachment();
                a->attachmentLength = cpvlength(cpvlerp(mPos, lastMachinePos, 0.5))*cpvlength(sys->getSpacing());
                sys->attachMachines(lastMachinePos, mPos, a);
            }
            
            // choose it as input or output (possibly both...)
            if (!outputChosen) {
                if (arc4random_uniform(nMachinesToPlace) == 0) {
                    outputChosen = true;
                    sys->outputMachinePosition = mPos;
                }
            }
            
            if (!inputChosen) {
                if (arc4random_uniform(nMachinesToPlace) == 0) {
                    inputChosen = true;
                    sys->inputMachinePosition = mPos;
                }
            }
            lastMachinePos = mPos;
            nMachinesToPlace--; // one less to place
        }
    }

}

void randomGenerator2(MachineSystem *sys)
{
    int nMachinesToPlace = arc4random_uniform(4)+2; // 3-5 machines to start
    
    std::vector<cpVect> placedMachines = std::vector<cpVect>();
    
    cpVect systemSize = sys->getSize();
    
    while (nMachinesToPlace) {
        int mx = arc4random_uniform(systemSize.x);
        int my = arc4random_uniform(systemSize.y);
        cpVect mPos = cpv(mx, my);
        if (!sys->partAtPosition(mPos)) {
            Attachment *a = new Attachment();
            do {
                a->attachmentType = (AttachmentType) arc4random_uniform(ATTACH_TYPE_MAX);
            }while (a->attachmentType == ATTACH_GEAR); //can't attach to wall with gear...
          
            a->attachmentLength = arc4random_uniform(20);
            
            sys->addPart(randomPart(sys->getSpace(), sys->getSpacing()), a, mPos);
            
            // join to previous machine
            if (placedMachines.size()) {
                cpVect lastMachinePos = placedMachines.back();
                Attachment *a = randomAttachment();
                a->attachmentLength = cpvlength(cpvlerp(mPos, lastMachinePos, 0.5))*cpvlength(sys->getSpacing());
                sys->attachMachines(lastMachinePos, mPos, a);
            }
            placedMachines.push_back(mPos);
            nMachinesToPlace--; // one less to place
        }
    }
    
    // set the input and output parts to DIFFERENT parts
    int p1 = arc4random_uniform((int)placedMachines.size());
    int p2=p1;
    while (p2 == p1 )
        p2  = arc4random_uniform((int)placedMachines.size());
    
    sys->inputMachinePosition = placedMachines[p1];
    sys->outputMachinePosition = placedMachines[p2];
    
}

MachineSystem  *attachmentMutator1(MachineSystem *sys)
{
    MachineSystem *newMachine = new MachineSystem(*sys);
    
    Attachment *chosenAttachment = NULL;
    cpVect part1 = cpvzero;
    cpVect part2 = cpvzero;
    
    newMachine->getRandomAttachment(&chosenAttachment, &part1, &part2);
    
    if (chosenAttachment) {
    // break the attachment, change it, then reattach
    newMachine->detachMachines(part1, part2);
    AttachmentType newAttachmentType = (AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
    
    // don't change the length - for now
    chosenAttachment->attachmentType = newAttachmentType;
    
    newMachine->attachMachines(part1, part2, chosenAttachment);
    }
    return newMachine;
}