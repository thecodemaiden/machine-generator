//
//  MachineWall.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/25/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "MachineWall.h"

MachineWall *mgMachineWallNew(int width, int height, cpSpace *space)
{
    MachineWall *wall = (MachineWall *)calloc(1, sizeof(MachineWall));
    wall->gridSpacing = cpv((float)width/(MAX_WIDTH + 1), (float)height/(MAX_HEIGHT + 1));
    wall->space = space;
    wall->size = cpv(width, height);
    wall->body = cpBodyNew(INFINITY, INFINITY);
   // cpSpaceAddBody(space, wall->wallBody);
    cpShape *wallShape = cpBoxShapeNew(wall->body, width, height);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddShape(space, wallShape);
    
//    //add the pegs
//    int pegCount = 0;
//    for (int x=0; x<MAX_WIDTH; x++) {
//        for (int y=0; y<MAX_HEIGHT; y++) {
//            cpVect pegPosition = cpv(wall->gridSpacing.x*(x+1), wall->gridSpacing.y*(y+1));
//            pegPosition = cpvadd(pegPosition, cpv(-width/2, -height/2));
//            cpBody *staticPeg = cpBodyNew(INFINITY, INFINITY);
//                cpBodySetPos(staticPeg, pegPosition);
//            cpShape *pegShape = cpCircleShapeNew(staticPeg, 5.0, cpv(0, 0));
//            cpShapeSetLayers(pegShape, PEG_LAYER);
//            cpSpaceAddShape(space, pegShape);
//            pegCount++;
//        }
//    }
    
    return wall;
}


void mgMachineWallSetInputMachine(MachineWall *wall, Attachment *machine, cpVect gridPosition)
{
    long x = lrintf(gridPosition.x);
    long y = lrintf(gridPosition.y);
    
    Attachment *alreadyAttached = wall->attachedMachines[x][y];
    if (alreadyAttached) {
        mgMachineWallRemoveMachine(wall, gridPosition);
    }
}

void mgMachineWallSetOutputMachine(MachineWall *wall, Attachment *machine, cpVect gridPosition)
{
    
}

cpBody *mgMachineWallGetInputMachineBody(MachineWall *wall)
{
    return wall->inputMachineBody;
}

cpBody *mgMachineWallGetOutputMachineBody(MachineWall *wall)
{
    return wall->outputMachineBody;
}

void mgMachineWallAddMachine(MachineWall *wall, Attachment *newMachine, cpVect gridPosition)
{
    long x = lrintf(gridPosition.x);
    long y = lrintf(gridPosition.y);
    
    Attachment *alreadyAttached = wall->attachedMachines[x][y];
    if (!alreadyAttached) {
        cpVect pegPosition = cpv(wall->gridSpacing.x*(x+1), wall->gridSpacing.y*(y+1));
        pegPosition = cpvadd(pegPosition, cpv(-wall->size.x/2, -wall->size.y/2));
        
        
        cpBody *pegBody = cpBodyNew(INFINITY, INFINITY);
        cpBodySetPos(pegBody, pegPosition);
        cpShape *pegShape = cpCircleShapeNew(pegBody, 5.0, cpv(0,0));
        cpShapeSetLayers(pegShape, PEG_LAYER);
        cpSpaceAddShape(wall->space, pegShape);
        
        wall->attachedMachines[x][y] = newMachine;
        mgMachineAttachToBody(newMachine, pegBody, wall->space);
        
        cpSpaceReindexShape(wall->space, pegShape);
    }
}

static void getPegFromAttachment(cpBody *body, cpConstraint *c, void *data){
    
    if (cpConstraintGetB(c) == body)
        *(cpBody **)data = cpConstraintGetA(c); // the attachment is always body B of the joint
}

static void removeShapes(cpBody *body, cpShape *s, void *data)
{
    cpSpaceRemoveShape(cpShapeGetSpace(s), s);
}

void mgMachineWallRemoveMachine(MachineWall *wall, cpVect gridPosition)
{
    long x = lrintf(gridPosition.x);
    long y = lrintf(gridPosition.y);
    
    Attachment *machine = wall->attachedMachines[x][y];
    if (machine) {
        cpBody *attachmentBody = machine->machine->body;
        cpBody *pegBody = NULL;
        cpBodyEachConstraint(attachmentBody, getPegFromAttachment, &pegBody);
        mgMachineRemoveAttachmentFromSpace(attachmentBody, wall->space);
        cpBodyEachShape(pegBody, removeShapes, NULL);
        cpBodyFree(pegBody);
        wall->attachedMachines[x][y] = NULL;
    }
}

static void freeShapes(cpBody *body, cpShape *shape, void *data)
{
  //  cpSpaceRemoveShape(cpShapeGetSpace(shape), shape);
    cpShapeFree(shape);
}

void mgMachineWallFree(MachineWall *wall)
{
    cpBodyEachShape(wall->body, freeShapes, NULL);
   // cpSpaceRemoveBody(cpBodyGetSpace(wall->wallBody), wall->wallBody);
    cpBodyFree(wall->body);
    free(wall);
}