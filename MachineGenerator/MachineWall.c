//
//  MachineWall.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/25/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "MachineWall.h"

MachineWall *mgMachineWallNew(int width, int height, int hPegs, int vPegs, cpVect position, cpSpace *space)
{
    MachineWall *wall = (MachineWall *)calloc(1, sizeof(MachineWall));
    
    wall->attachedMachines = (Attachment **)calloc(hPegs*vPegs, sizeof(Attachment *));
    wall->gridSpacing = cpv((float)width/(hPegs + 1), (float)height/(vPegs + 1));
    wall->space = space;
    wall->size = cpv(hPegs, vPegs);
    wall->body = cpBodyNew(INFINITY, INFINITY);
    cpBodySetPos(wall->body, position);
    cpShape *wallShape = cpBoxShapeNew(wall->body, width, height);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddShape(space, wallShape);

    return wall;
}

void mgMachineWallSetInputMachinePosition(MachineWall *wall, cpVect gridPosition)
{
    wall->inputMachinePosition = gridPosition;
}

void mgMachineWallSetOutputMachinePosition(MachineWall *wall, cpVect gridPosition)
{
    wall->outputMachinePosition = gridPosition;
}

Attachment *mgMachineWallGetMachineAtPosition(MachineWall *wall, cpVect gridPosition)
{
    long x = lrintf(gridPosition.x);
    long y = lrintf(gridPosition.y);
    
   return wall->attachedMachines[x + y*(int)wall->size.y];
}

void mgMachineWallAddMachine(MachineWall *wall, Attachment *newMachine, cpVect gridPosition)
{
    long x = lrintf(gridPosition.x);
    long y = lrintf(gridPosition.y);
    
    Attachment *alreadyAttached = wall->attachedMachines[x + y*(int)wall->size.y];
    if (!alreadyAttached) {
        cpFloat wallWidth = (wall->size.x+1)*wall->gridSpacing.x;
        cpFloat wallHeight = (wall->size.y+1)*wall->gridSpacing.y;
        
        cpVect pegPosition = cpv(wall->gridSpacing.x*(x+1), wall->gridSpacing.y*(y+1));
        pegPosition = cpvadd(pegPosition, cpv(-wallWidth/2, -wallHeight/2));
        pegPosition = cpvadd(pegPosition, cpBodyGetPos(wall->body));
        
        cpBody *pegBody = cpBodyNew(INFINITY, INFINITY);
        cpBodySetPos(pegBody, pegPosition);
        cpShape *pegShape = cpCircleShapeNew(pegBody, 5.0, cpv(0,0));
        cpShapeSetLayers(pegShape, PEG_LAYER);
        cpSpaceAddShape(wall->space, pegShape);
        
        wall->attachedMachines[x + y*(int)wall->size.y] = newMachine;
        mgMachineAttachToBody(newMachine, pegBody, wall->space);
        
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
    
    Attachment *machine = wall->attachedMachines[x + y*(int)wall->size.y];
    if (machine) {
        cpBody *attachmentBody = machine->machine->body;
        cpBody *pegBody = NULL;
        cpBodyEachConstraint(attachmentBody, getPegFromAttachment, &pegBody);
        mgMachineDetachFromBody(attachmentBody, pegBody, wall->space);
        cpBodyEachShape(pegBody, removeShapes, NULL);
        cpBodyFree(pegBody);
        wall->attachedMachines[x + y*(int)wall->size.y] = NULL;
        
        // if the machine is not attached to any other, let's just get rid of it
        __block cpConstraint *otherConstraint = NULL;
        cpBodyEachConstraint_b(attachmentBody, ^(cpConstraint *constraint) {
           if (cpConstraintGetB(constraint) == attachmentBody)
               otherConstraint = constraint;
        });
        if (!otherConstraint)
            mgMachineDestroy(machine->machine);
    }
}

static void freeShapes(cpBody *body, cpShape *shape, void *data)
{
  //  cpSpaceRemoveShape(cpShapeGetSpace(shape), shape);
    cpShapeFree(shape);
}

void mgMachineWallFree(MachineWall *wall)
{
    free(wall->attachedMachines);
    cpBodyEachShape(wall->body, freeShapes, NULL);
   // cpSpaceRemoveBody(cpBodyGetSpace(wall->wallBody), wall->wallBody);
    cpBodyFree(wall->body);
    free(wall);
}