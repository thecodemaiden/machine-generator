//
//  MachineWall.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/25/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "MachineWall.h"

#pragma mark - Helper Functions

static void getPegFromAttachment(cpBody *body, cpConstraint *c, void *data){
    
    if (cpConstraintGetB(c) == body) {
        cpBody *otherBody = cpConstraintGetA(c); // the peg is always body A
        if (cpBodyIsStatic(otherBody))
            *(cpBody **)data = otherBody;
    }
}

static void removeShapes(cpBody *body, cpShape *s, void *data)
{
    cpSpaceRemoveShape(cpShapeGetSpace(s), s);
}


// to help us find edges in the adjacency matrix, we must convert (gx, gy) to an int and vice versa
static int machinePositionToNumber(cpVect wallSize, cpVect gridPosition)
{
    int x = gridPosition.x;
    int y = gridPosition.y;
    return x + y*(int)wallSize.y;
}

static cpVect machineNumberToPosition(cpVect wallSize, int machineNumber)
{
    return cpv((machineNumber % (int)wallSize.y), (machineNumber / (int)wallSize.y));
}

// find the actual location of the peg for this attachment
static cpVect gridPositionToWorld(MachineWall *wall, cpVect gridPosition) {
    int x = gridPosition.x;
    int y = gridPosition.y;
    
    cpFloat wallWidth = (wall->size.x+1)*wall->gridSpacing.x;
    cpFloat wallHeight = (wall->size.y+1)*wall->gridSpacing.y;
    
    
    cpVect pegPosition = cpv(wall->gridSpacing.x*(x+1), wall->gridSpacing.y*(y+1));
    pegPosition = cpvadd(pegPosition, cpv(-wallWidth/2, -wallHeight/2));
    pegPosition = cpvadd(pegPosition, cpBodyGetPos(wall->body));
    
    return pegPosition;
}

static void freeShapes(cpBody *body, cpShape *shape, void *data)
{
    //  cpSpaceRemoveShape(cpShapeGetSpace(shape), shape);
    cpShapeFree(shape);
}

#pragma  mark - Allocation and Destruction

MachineWall *mgMachineWallNew(int width, int height, int hPegs, int vPegs, cpVect position, cpSpace *space)
{
    MachineWall *wall = (MachineWall *)calloc(1, sizeof(MachineWall));
    
    int nPegs = hPegs *vPegs;
    
    wall->machines = (MachineDescription **)calloc(nPegs, sizeof (MachineDescription *));
    
    wall->attachments = (Attachment ***)calloc(nPegs, sizeof(Attachment ***));
    for (int i=0; i<=nPegs; i++) {
        wall->attachments[i] = (Attachment **)calloc(nPegs, sizeof(Attachment **));
    }
    
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

void mgMachineWallFree(MachineWall *wall)
{
    free(wall->machines);
    
    int nPegs = wall->size.x * wall->size.y;
    
    for (int i=0; i<=nPegs; i++) {
        free (wall->attachments[i]);
    }
    free(wall->attachments);
    
    cpBodyEachShape(wall->body, freeShapes, NULL);
    
    cpBodyFree(wall->body);
    free(wall);
}

#pragma mark - Machine methods


void mgMachineWallAddMachine(MachineWall *wall, MachineDescription *newMachine, Attachment *attachment, cpVect gridPosition)
{
    int machineNumber = machinePositionToNumber(wall->size, gridPosition);
    
    
    MachineDescription *alreadyAttached = wall->machines[machineNumber];
    if (!alreadyAttached) {
        
        cpVect pegPosition = gridPositionToWorld(wall, gridPosition);
        cpBody *pegBody = cpBodyNew(INFINITY, INFINITY);
        cpBodySetPos(pegBody, pegPosition);
        cpShape *pegShape = cpCircleShapeNew(pegBody, 5.0, cpv(0,0));
        cpShapeSetLayers(pegShape, PEG_LAYER);
        cpSpaceAddShape(wall->space, pegShape);
        
        wall->machines[machineNumber] = newMachine;
        
        if (!newMachine->body)
            constructBodyForDescription(newMachine, pegPosition, wall->space);
        
        mgMachineAttachToBody(attachment, newMachine->body, pegBody, wall->space);
        wall->attachments[machineNumber][machineNumber] = attachment;
        
    }
}

void mgMachineWallRemoveMachine(MachineWall *wall, cpVect gridPosition)
{
    int machineNum = machinePositionToNumber(wall->size, gridPosition);
    
    MachineDescription *machine = wall->machines[machineNum];
    if (machine) {
        cpBody *attachmentBody = machine->body;
        cpBody *pegBody = NULL;
        cpBodyEachConstraint(attachmentBody, getPegFromAttachment, &pegBody);
        mgMachineDetachFromBody(attachmentBody, pegBody, wall->space);
        cpBodyEachShape(pegBody, removeShapes, NULL);
        cpBodyFree(pegBody);
        wall->machines[machineNum] = NULL;
        
        // no longer allowing 'dangling' machines - everything is nailed to the wall for now
        mgMachineDestroy(machine);
    }
}

boolean_t mgMachineWallAttachMachines(MachineWall *wall, cpVect machine1Pos, cpVect machine2Pos, Attachment *attachment)
{
    boolean_t added = false;
    if (!cpveql(machine1Pos, machine2Pos)) {
        MachineDescription *machine1 = mgMachineWallGetMachineAtPosition(wall, machine1Pos);
        MachineDescription *machine2 = mgMachineWallGetMachineAtPosition(wall, machine2Pos);
        if (machine1 && machine2) {
            int machine1Num = machinePositionToNumber(wall->size, machine1Pos);
            int machine2Num = machinePositionToNumber(wall->size, machine2Pos);
            Attachment *existingAttachment = wall->attachments[machine1Num][machine2Num];
            if (!existingAttachment) {
                mgMachineAttachToBody(attachment, machine1->body, machine2->body, wall->space);
                wall->attachments[machine1Num][machine2Num] = wall->attachments[machine2Num][machine1Num] = attachment;
                added = true;
            }
        }
    }
    return added;
}

boolean_t mgMachineWallDetachMachines(MachineWall *wall, cpVect machine1Pos, cpVect machine2Pos)
{
    boolean_t detached = false;
    if (!cpveql(machine1Pos, machine2Pos)) {
        MachineDescription *machine1 = mgMachineWallGetMachineAtPosition(wall, machine1Pos);
        MachineDescription *machine2 = mgMachineWallGetMachineAtPosition(wall, machine2Pos);
        if (machine1 && machine2) {
            int machine1Num = machinePositionToNumber(wall->size, machine1Pos);
            int machine2Num = machinePositionToNumber(wall->size, machine2Pos);
            Attachment *existingAttachment = wall->attachments[machine1Num][machine2Num];
            if (existingAttachment) {
                mgMachineDetachFromBody(machine1->body, machine2->body, wall->space);
                wall->attachments[machine1Num][machine2Num] = wall->attachments[machine2Num][machine1Num] = NULL;
                detached = true;
            }
        }
    }
    return detached;
}

MachineDescription *mgMachineWallGetMachineAtPosition(MachineWall *wall, cpVect gridPosition)
{
    int machineNumber = machinePositionToNumber(wall->size, gridPosition);
    
    return wall->machines[machineNumber];
}

Attachment *mgMachineWallGetAttachmentBetween(MachineWall *wall, cpVect machine1Pos, cpVect machine2Pos)
{
    int machine1Num = machinePositionToNumber(wall->size, machine1Pos);
    int machine2Num = machinePositionToNumber(wall->size, machine2Pos);
    
    return wall->attachments[machine1Num][machine2Num];
}

