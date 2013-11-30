//
//  Type1Algorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/6/13.
//
//

#include "Algorithm.h"
#include <vector>

cpFloat normalize_angle(cpFloat angle)
{
    return fmodf(angle, 2*M_PI);
}

bool isUnreasonable(cpFloat n) {
    return fabs(n) == INFINITY || n != n;
}

 MachinePart *randomPart(cpSpace *space, cpVect size)
{
    MachinePart *m = new MachinePart(space);
    m->machineType = (BodyType)arc4random_uniform(MACHINE_TYPE_MAX);
    
    if (m->machineType == MACHINE_BOX) {
        // will it be tall or wide?
        if (((float)random()/RAND_MAX) > 0.5) {
            m->length = size.x*M_SQRT1_2;
            m->height = arc4random_uniform(50) + 25;;
        } else {
            m->length = arc4random_uniform(50) + 25;;
            m->height = size.y*M_SQRT1_2;
        }
        
    } else {
        m->length = size.x; // large circle == trouble!!
    }
    
    return m;
}

static Attachment *randomAttachment()
{
    Attachment *a = Attachment::createAttachmentOfType((AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX));
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
            AttachmentType t ;
            do {
                t= (AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
            } while (t==ATTACH_GEAR);
            
            Attachment *a = Attachment::createAttachmentOfType(t);
            
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
    int nMachinesToPlace = arc4random_uniform(3)+4; // 4-6 machines to start
    
    std::vector<cpVect> placedMachines = std::vector<cpVect>();
    
    cpVect systemSize = sys->getSize();
    cpVect gridSpacing = sys->getSpacing();
    while (nMachinesToPlace) {
        int mx = arc4random_uniform(systemSize.x);
        int my = arc4random_uniform(systemSize.y);
        cpVect mPos = cpv(mx, my);
        if (!sys->partAtPosition(mPos)) {
            AttachmentType t ;
            do {
                t= (AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
            } while (t==ATTACH_GEAR);
            
            Attachment *a = Attachment::createAttachmentOfType(t);
            
            a->attachmentLength = arc4random_uniform(20);
            
            sys->addPart(randomPart(sys->getSpace(), sys->getSpacing()), a, mPos);
            
            // join to previous machine
            if (placedMachines.size()) {
                cpVect lastMachinePos = placedMachines.back();
                Attachment *a = randomAttachment();
                
                cpVect truePos = cpv(mPos.x*gridSpacing.x, mPos.y*gridSpacing.y);
                cpVect trueOtherPos = cpv(lastMachinePos.x*gridSpacing.x, lastMachinePos.y*gridSpacing.y);
                
                a->attachmentLength = cpvdist(truePos, trueOtherPos);
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
    MachineSystem *newSystem = new MachineSystem(*sys);
    
    Attachment *chosenAttachment = NULL;
    cpVect part1 = cpvzero;
    cpVect part2 = cpvzero;
    
    newSystem->getRandomAttachment(&chosenAttachment, &part1, &part2);
    
    if (chosenAttachment) {
        // the wall will delete the old attachment
        Attachment *newAttachment = Attachment::createAttachmentOfType((AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX));
        newAttachment->firstAttachPoint = chosenAttachment->firstAttachPoint;
        newAttachment->secondAttachPoint = chosenAttachment->secondAttachPoint;
        newAttachment->attachmentLength = chosenAttachment->attachmentLength;
        
        newSystem->updateAttachmentBetween(part1, part2, newAttachment);
    }
    return newSystem;
}

void randomGenerator3(MachineSystem *sys)
{
    int nMachinesToPlace = arc4random_uniform(3)+3; // 3-5 machines to start
    
    std::vector<cpVect> placedMachines = std::vector<cpVect>();
    
    cpVect systemSize = sys->getSize();
    
    while (nMachinesToPlace) {
        int mx = arc4random_uniform(systemSize.x);
        int my = arc4random_uniform(systemSize.y);
        cpVect mPos = cpv(mx, my);
        if (!sys->partAtPosition(mPos)) {
            AttachmentType t ;
            do {
                t= (AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
            } while (t==ATTACH_GEAR);
            
            Attachment *a = Attachment::createAttachmentOfType(t);
            
            a->attachmentLength = arc4random_uniform(3)*cpvlength(sys->getSpacing()) + 10;
            MachinePart *newPart = randomPart(sys->getSpace(), sys->getSpacing());
            sys->addPart(newPart, a, mPos);
            
            // join to previous machine
            if (placedMachines.size()) {
                cpVect lastMachinePos = placedMachines.back();
                Attachment *a = randomAttachment();
                
                cpFloat partDistance = cpvdist(sys->partAtPosition(lastMachinePos)->body->p, newPart->body->p);
                // make attachment lengths the exact distance between parts
                a->attachmentLength = partDistance;
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


MachineSystem  *attachmentMutator2(MachineSystem *sys)
{
    MachineSystem *newSystem = new MachineSystem(*sys);
    
    cpVect partPos = cpv(-1,-1);
    newSystem->getRandomPartPosition(&partPos);
    
    if (partPos.x >= 0) {
        Attachment *wallAttachment = newSystem->attachmentToWall(partPos);
        AttachmentType oldAttachmentType = wallAttachment->attachmentType();

        AttachmentType t ;
        do {
            t= (AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
        } while (t==ATTACH_GEAR && t ==oldAttachmentType);
        
        // the wall will delete the old attachment
        Attachment *newAttachment = Attachment::createAttachmentOfType((AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX));
        newAttachment->firstAttachPoint = wallAttachment->firstAttachPoint;
        newAttachment->secondAttachPoint = wallAttachment->secondAttachPoint;
        newAttachment->attachmentLength = wallAttachment->attachmentLength;
        
        newSystem->updateAttachmentToWall(partPos, newAttachment);
    }
    
    
    return newSystem;
}

MachineSystem *attachmentMutator3(MachineSystem *sys)
{
    MachineSystem *newSystem = new MachineSystem(*sys);
    
    Attachment *chosenAttachment = NULL;
    cpVect part1 = cpvzero;
    cpVect part2 = cpvzero;
    
    newSystem->getRandomAttachment(&chosenAttachment, &part1, &part2);
    
    if (chosenAttachment) {
        Attachment *newAttachment = perturbAttachmentAttributes(chosenAttachment);
        newSystem->updateAttachmentBetween(part1, part2, newAttachment);
    }
    
    return newSystem;

}

MachineSystem  *attachmentAnchorMutator(MachineSystem *sys)
{
    MachineSystem *newSystem = new MachineSystem(*sys);
    
    cpVect partPos = cpv(-1,-1);
    newSystem->getRandomPartPosition(&partPos);
    
    if (partPos.x >= 0) {
        Attachment *wallAttachment = newSystem->attachmentToWall(partPos);
       
        cpVect displacementVector = cpv((float)rand()/(RAND_MAX/2)-1.0,  (float)rand()/(RAND_MAX/2)-1.0);
        
        if ((float)rand()/RAND_MAX > 0.5) {
            cpVect newVector = cpvadd(wallAttachment->firstAttachPoint, displacementVector);
            newVector.x = MAX(newVector.x, -1.0);
            newVector.x = MIN(newVector.x, 1.0);
            newVector.y = MAX(newVector.y, -1.0);
            newVector.y = MIN(newVector.y, 1.0);
            wallAttachment->firstAttachPoint = newVector;
            
        }
        else {
            cpVect newVector = cpvadd(wallAttachment->secondAttachPoint, displacementVector);
            newVector.x = MAX(newVector.x, -1.0);
            newVector.x = MIN(newVector.x, 1.0);
            newVector.y = MAX(newVector.y, -1.0);
            newVector.y = MIN(newVector.y, 1.0);
            wallAttachment->secondAttachPoint = newVector;
        }
        
        newSystem->updateAttachmentToWall(partPos, wallAttachment);
    }
    
    
    return newSystem;
}

MachineSystem  *attachmentAnchorMutator2(MachineSystem *sys)
{MachineSystem *newSystem = new MachineSystem(*sys);
    
    Attachment *chosenAttachment = NULL;
    cpVect part1 = cpvzero;
    cpVect part2 = cpvzero;
    
    newSystem->getRandomAttachment(&chosenAttachment, &part1, &part2);
    
    if (chosenAttachment) {
        // copy for now....
        chosenAttachment = Attachment::copyAttachment(chosenAttachment);
        
        cpVect displacementVector = cpv((float)rand()/(RAND_MAX/2)-1.0,  (float)rand()/(RAND_MAX/2)-1.0);
        
        if ((float)rand()/RAND_MAX > 0.5) {
            cpVect newVector = cpvadd(chosenAttachment->firstAttachPoint, displacementVector);
            newVector.x = MAX(newVector.x, -1.0);
            newVector.x = MIN(newVector.x, 1.0);
            newVector.y = MAX(newVector.y, -1.0);
            newVector.y = MIN(newVector.y, 1.0);
            chosenAttachment->firstAttachPoint = newVector;
            
        }
        else {
            cpVect newVector = cpvadd(chosenAttachment->secondAttachPoint, displacementVector);
            newVector.x = MAX(newVector.x, -1.0);
            newVector.x = MIN(newVector.x, 1.0);
            newVector.y = MAX(newVector.y, -1.0);
            newVector.y = MIN(newVector.y, 1.0);
            chosenAttachment->secondAttachPoint = newVector;
        }
        
        newSystem->updateAttachmentBetween(part1, part2, chosenAttachment);
    }
    
    return newSystem;
}

MachineSystem *inputMachineMutator(MachineSystem *sys)
{
    MachineSystem *newSystem = new MachineSystem(*sys);

    sys->getRandomDisjointParts(NULL, NULL);
    cpVect newInputPos;
    do {
        newSystem->getRandomPartPosition(&newInputPos);
    } while (cpveql(newInputPos, sys->inputMachinePosition) && newInputPos.x != -1);
    
    newSystem->inputMachinePosition = newInputPos;
    return  newSystem;
}

MachineSystem *outputMachineMutator(MachineSystem *sys)
{
    MachineSystem *newSystem = new MachineSystem(*sys);
    
    cpVect newOutputPos;
    do {
        sys->getRandomPartPosition(&newOutputPos);
    } while (cpveql(newOutputPos, sys->outputMachinePosition) && newOutputPos.x != -1);
    
    newSystem->outputMachinePosition = newOutputPos;
    return  newSystem;
}

MachineSystem *addAttachmentMutator(MachineSystem *sys)
{
    MachineSystem *newSystem = new MachineSystem(*sys);

    cpVect p1 = cpv(-1,-1);
    cpVect p2 = cpv(-1,-1);
    
    newSystem->getRandomDisjointParts(&p1, &p2);
    
    if (p1.x > -1) {
        Attachment *a = randomAttachment();
        newSystem->attachMachines(p1, p2, a);
    }
    
    return newSystem;
}


// ADDED FOR NEAT
// Minimalism of starting solutions is valued - systems only ever grow
void neatGenerator(MachineSystem *sys) {
    cpVect inputPos = cpv(arc4random_uniform(sys->getSize().x), arc4random_uniform(sys->getSize().y));
    cpVect outputPos;
    do {
        outputPos = cpv(arc4random_uniform(sys->getSize().x), arc4random_uniform(sys->getSize().y));
    } while (cpveql(inputPos, outputPos));
    
    // put in input machine
    
    AttachmentType t ;
    do {
        t= (AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
    } while (t==ATTACH_GEAR);
    
    Attachment *a = Attachment::createAttachmentOfType(t);
    
    a->attachmentLength = arc4random_uniform(3)*cpvlength(sys->getSpacing()) + 10;
    MachinePart *inputPart = randomPart(sys->getSpace(), sys->getSpacing());
    sys->addPart(inputPart, a, inputPos);
    
    //output machine
    
    do {
        t= (AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
    } while (t==ATTACH_GEAR);
    
    a = Attachment::createAttachmentOfType(t);
    
    a->attachmentLength = arc4random_uniform(3)*cpvlength(sys->getSpacing()) + 10;
    MachinePart *outputPart = randomPart(sys->getSpace(), sys->getSpacing());
    sys->addPart(outputPart, a, outputPos);
        
    a = randomAttachment();
    
    sys->attachMachines(inputPos, outputPos, a);
    sys->inputMachinePosition = inputPos;
    sys->outputMachinePosition = outputPos;
    
}

Attachment *changeAttachmentType(Attachment *at)
{
    AttachmentType t;
    do {
        t= (AttachmentType)arc4random_uniform(ATTACH_TYPE_MAX);
    } while (t==at->attachmentType());
    
    Attachment *newAt = Attachment::createAttachmentOfType(t);
    newAt->attachmentLength = at->attachmentLength;
    newAt->firstAttachPoint = at->firstAttachPoint;
    newAt->secondAttachPoint = at->secondAttachPoint;
    
    return newAt;
}

Attachment *perturbAttachmentAttributes(Attachment *at)
{
    cpFloat factor = (cpFloat)rand()/(RAND_MAX/2) - 1.0; // between -1 and 1
    int selector = arc4random_uniform(3);

    AttachmentType t = at->attachmentType();
    bool adjustLength = (t == ATTACH_FIXED) || selector == 2;
    at = Attachment::copyAttachment(at);
    if (!adjustLength) {
        switch (t) {
            case ATTACH_GEAR:
                // we can additionally alter ratio or phase
                if (selector == 0) {
                    ((GearAttachment *)at)->gearRatio += factor;
                } else {
                    ((GearAttachment *)at)->phase += factor;
                }
                break;
            case ATTACH_PIVOT:
            { // we can alter pivot position
                cpFloat pivotPos = ((PivotAttachment *)at)->pivotPosition;
                pivotPos += factor;
                if (pivotPos > 1.0)
                    pivotPos = 1.0;
                
                if (pivotPos < -1.0)
                    pivotPos = -1.0;
                ((PivotAttachment *)at)->pivotPosition = pivotPos;
            }
                break;
            case ATTACH_SLIDE:
                // we can alter max or min distance
                if (selector == 0) {
                    ((SlideAttachment *)at)->maxDistance += factor;
                } else {
                    ((SlideAttachment *)at)->minDistance += factor;
                }
                break;
            case ATTACH_SPRING:
                // we can alter damping or stiffness
                if (selector == 0) {
                    ((SpringAttachment *)at)->damping += factor;
                } else {
                    ((SpringAttachment *)at)->stiffness += factor;
                }
                break;
            default:
                break;
        }
    } else {
        at->attachmentLength += factor; // is this a good idea?
    }
    return at;
}

Attachment *changeAttachmentAnchorPoints(Attachment *at) {
    
    Attachment *newAttachment = Attachment::copyAttachment(at);
        cpVect displacementVector = cpv((float)rand()/(RAND_MAX/2)-1.0,  (float)rand()/(RAND_MAX/2)-1.0);
        
        if ((float)rand()/RAND_MAX > 0.5) {
            cpVect newVector = cpvadd(newAttachment->firstAttachPoint, displacementVector);
            newVector.x = MAX(newVector.x, -1.0);
            newVector.x = MIN(newVector.x, 1.0);
            newVector.y = MAX(newVector.y, -1.0);
            newVector.y = MIN(newVector.y, 1.0);
            newAttachment->firstAttachPoint = newVector;
            
        }
        else {
            cpVect newVector = cpvadd(newAttachment->secondAttachPoint, displacementVector);
            newVector.x = MAX(newVector.x, -1.0);
            newVector.x = MIN(newVector.x, 1.0);
            newVector.y = MAX(newVector.y, -1.0);
            newVector.y = MIN(newVector.y, 1.0);
            newAttachment->secondAttachPoint = newVector;
        }
        
    return newAttachment;
    
}
