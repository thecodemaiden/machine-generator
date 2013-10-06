//
//  MachineSystem.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

#import "chipmunk.h"
#include "MachineSystem.h"

#define WALL_LAYER 1>>2
#define PEG_LAYER 1>>1

#pragma mark - Helper Functions


MachineSystem::MachineSystem(int width, int height, int hPegs, int vPegs, cpVect position, cpSpace *space)
: parts(hPegs*vPegs),
  attachments((hPegs*vPegs), std::vector<Attachment *>(hPegs*vPegs, NULL)),
  space(space)
{
    
    gridSpacing = cpv((float)width/(hPegs + 1), (float)height/(vPegs + 1));
    space = space;
    size = cpv(hPegs, vPegs);
    body = cpBodyNewStatic();
    cpBodySetPos(body, position);
    
    cpShape *wallShape = cpSegmentShapeNew(body, cpv(-width/2, height/2), cpv(width/2, height/2), .5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(-width/2, -height/2), cpv(+width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(-width/2, +height/2), cpv(-width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(+width/2, +height/2), cpv(+width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
}

MachineSystem::MachineSystem(MachineSystem &original, cpVect position)
: space(original.space),
 size(original.size),
gridSpacing(original.gridSpacing)
{
    cpFloat width = (original.size.x +1)*original.gridSpacing.x;
    cpFloat height = (original.size.y +1)*original.gridSpacing.y;
   
    body = cpBodyNewStatic();
    cpBodySetPos(body, position);
    
    cpShape *wallShape = cpSegmentShapeNew(body, cpv(-width/2, height/2), cpv(width/2, height/2), .5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(-width/2, -height/2), cpv(+width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(-width/2, +height/2), cpv(-width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(+width/2, +height/2), cpv(+width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    // do the copy of the attachment and machines manually, because we need to add and attach machines
    int nPegs = size.x*size.y;

    parts.resize(nPegs, NULL);
    
    attachments.resize(nPegs);
    
    for (int x = 0; x < size.x; x++ ) {
        for (int y = 0; y < size.y; y++) {
            cpVect gridPos = cpv(x,y);
            int machineNum = machinePositionToNumber(gridPos);
            MachinePart *newPart = new MachinePart(*original.parts[machineNum]);
            Attachment *wallAttachment = new Attachment(*original.attachments[machineNum][machineNum]);
            addPart(newPart, wallAttachment, gridPos);
        }
    }
    
    for (int i=0; i<nPegs; i++) {
        attachments[i].resize(nPegs, NULL);
        for (int j=0; j<i; j++) {
            Attachment *attachmentToCopy = original.attachments[i][j];
            if (attachmentToCopy) {
                Attachment *attachmentCopy = new Attachment(*attachmentToCopy);
                cpVect machine1Pos = machineNumberToPosition(i);
                cpVect machine2Pos = machineNumberToPosition(j);
                
                attachMachines(machine1Pos, machine2Pos, attachmentCopy);
            }
        }
    }
}

cpSpace * MachineSystem::getSpace()
{
    return space;
}

cpVect MachineSystem::getSize()
{
    return size;
}
cpVect MachineSystem::getSpacing()
{
    return gridSpacing;
}

// NOTE - don't use a GEAR attachment to attach to the wall - the peg does not rotate, so the body will not rotate
void MachineSystem::addPart(MachinePart *newPart, Attachment *attachment, cpVect gridPosition)
{
    int machineNumber = machinePositionToNumber(gridPosition);
    
    MachinePart *alreadyAttached = parts[machineNumber];
    if (!alreadyAttached) {
        
        cpVect pegPosition = gridPositionToWorld(gridPosition);
        cpBody *pegBody = cpBodyNewStatic();
        cpBodySetPos(pegBody, pegPosition);
        cpShape *pegShape = cpCircleShapeNew(pegBody, 5.0, cpv(0,0));
        cpShapeSetLayers(pegShape, PEG_LAYER);
        cpSpaceAddStaticShape(space, pegShape);
        
        parts[machineNumber] = newPart;
        
        newPart->setPosition(pegPosition);
        newPart->attachToBody(attachment, pegBody);
        attachments[machineNumber][machineNumber] = attachment;
    }
}

//static void findPeg(cpBody *body, cpConstraint *c, void *data){
//    printf("X");
//
//    if (cpConstraintGetB(c) == body) {
//        cpBody *otherBody = cpConstraintGetA(c); // the peg is always body A
//        if (cpBodyGetMass(otherBody) == INFINITY)
//            *(cpBody **)data = otherBody;
//    }
//}

void MachineSystem::removePart(cpVect gridPosition)
{
    // ermagerd, free it when doneeeeee
    int machineNum = machinePositionToNumber(gridPosition);
    
    MachinePart *partToRemove = parts[machineNum];
    if (partToRemove) {
        
        cpBody *attachmentBody = partToRemove->getBody();
        __block cpBody *pegBody = NULL;
//        cpBodyEachConstraint(attachmentBody, findPeg, &pegBody);
        cpBodyEachConstraint_b(attachmentBody, ^(cpConstraint *c) {
            if (cpConstraintGetB(c) == attachmentBody) {
                cpBody *otherBody = cpConstraintGetA(c); // the peg is always body A
                if (cpBodyGetMass(otherBody) == INFINITY)
                    pegBody = otherBody;
            }
        });
        
        assert(pegBody);
        
        //remove the attachments
        int nPegs = size.x * size.y;
        // remove the attachments for this machine
        for (int i=0; i<nPegs; i++) {
            cpVect otherMachinePos = machineNumberToPosition(i);
            detachMachines(gridPosition, otherMachinePos);
        }
        
        
        partToRemove->detachFromBody(pegBody);
        cpBodyEachShape_b(pegBody, ^(cpShape *shape) {
            cpSpaceRemoveStaticShape(space, shape);
            cpShapeFree(shape);
        });
        
        cpBodyFree(pegBody);
        parts[machineNum] = NULL;
        
        // no longer allowing 'dangling' machines - everything is nailed to the wall for now
        delete partToRemove;
    }

}

MachinePart *MachineSystem::partAtPosition(cpVect gridPosition)
{
    int machineNum = machinePositionToNumber(gridPosition);
    return parts[machineNum];
}

Attachment *MachineSystem::attachmentBetween(cpVect machine1Pos, cpVect machine2Pos)
{
    int machine1Num = machinePositionToNumber(machine1Pos);
    int machine2Num = machinePositionToNumber(machine2Pos);
    
    return attachments[machine1Num][machine2Num];
}

bool MachineSystem::attachMachines(cpVect machine1Pos, cpVect machine2Pos, Attachment *attachment)
{
    bool added = false;
    if (!cpveql(machine1Pos, machine2Pos)) {
        MachinePart *machine1 = partAtPosition(machine1Pos);
        MachinePart *machine2 = partAtPosition(machine2Pos);
        if (machine1 && machine2) {
            int machine1Num = machinePositionToNumber(machine1Pos);
            int machine2Num = machinePositionToNumber(machine2Pos);
            Attachment *existingAttachment = attachments[machine1Num][machine2Num];
            if (!existingAttachment) {
                machine1->attachToBody(attachment, machine2->body);
                attachments[machine1Num][machine2Num] = attachments[machine2Num][machine1Num] = attachment;
                added = true;
            }
        }
    }
    return added;

}

bool MachineSystem::detachMachines(cpVect machine1Pos, cpVect machine2Pos)
{
    bool detached = false;
    if (!cpveql(machine1Pos, machine2Pos)) {
        MachinePart *machine1 = partAtPosition(machine1Pos);
        MachinePart *machine2 = partAtPosition(machine2Pos);
        if (machine1 && machine2) {
            int machine1Num = machinePositionToNumber(machine1Pos);
            int machine2Num = machinePositionToNumber(machine2Pos);
            Attachment *existingAttachment = attachments[machine1Num][machine2Num];
            if (existingAttachment) {
                machine1->detachFromBody(machine2->body);
                machine2->detachFromBody(machine1->body);
               
                attachments[machine1Num][machine2Num] = attachments[machine2Num][machine1Num] = NULL;
                detached = true;
            }
        }
    }
    return detached;}

MachineSystem::~MachineSystem()
{
    //detach the machinesss
    for (int x = 0; x < size.x; x++ ) {
        for (int y = 0; y < size.y; y++) {
            cpVect gridPos = cpv(x,y);
            removePart(gridPos);
        }
        
    }
    
    int nPegs = size.x * size.y;
    
    for (int i=0; i<nPegs; i++) {
        for (int j=0; j<nPegs; j++) {
            delete attachments[i][j];
        }
    }
    
    cpBodyEachShape_b(body, ^(cpShape *shape) {
        cpSpaceRemoveStaticShape(cpShapeGetSpace(shape), shape);
        cpShapeFree(shape);
    });
    
    cpBodyFree(body);
}

#pragma mark - more helpers
// to help us find edges in the adjacency matrix, we must convert (gx, gy) to an int and vice versa
int MachineSystem::machinePositionToNumber(cpVect gridPosition)
{
    int x = gridPosition.x;
    int y = gridPosition.y;
    return x + y*(int)this->size.y;
}

cpVect MachineSystem::machineNumberToPosition(int machineNumber)
{
    return cpv((machineNumber % (int)this->size.y), (machineNumber / (int)this->size.y));
}

// find the actual location of the peg for this attachment
cpVect MachineSystem::gridPositionToWorld(cpVect gridPosition) {
    int x = gridPosition.x;
    int y = gridPosition.y;
    
    cpFloat wallWidth = (this->size.x+1)*this->gridSpacing.x;
    cpFloat wallHeight = (this->size.y+1)*this->gridSpacing.y;
    
    cpVect pegPosition = cpv(this->gridSpacing.x*(x+1), this->gridSpacing.y*(y+1));
    pegPosition = cpvadd(pegPosition, cpv(-wallWidth/2, -wallHeight/2));
    pegPosition = cpvadd(pegPosition, cpBodyGetPos(this->body));
    
    return pegPosition;
}
