//
//  MachineWall.h
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/25/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "chipmunk.h"
#import "Machine.h"

#define MAX_WIDTH 20
#define MAX_HEIGHT 20

#define WALL_LAYER 1>>1 // to avoid colliding with machines
#define PEG_LAYER 1>>2 // so i can search for pegs easily when attaching machines

typedef struct  {
    Attachment *attachedMachines[MAX_HEIGHT][MAX_WIDTH]; // I feel no need to maintain a linked list - yet
    cpVect gridSpacing;
    cpVect inputMachinePosition;
    cpVect outputMachinePosition;
    cpBody *body;
    cpBody *inputMachineBody;
    cpBody *outputMachineBody;
    cpSpace *space;
    cpVect size;
} MachineWall;

MachineWall *mgMachineWallNew(int width, int height, cpSpace *space);
void mgMachineWallAddMachine(MachineWall *wall, Attachment *newMachine, cpVect gridPosition);
void mgMachineWallRemoveMachine(MachineWall *wall, cpVect gridPosition);

void mgMachineWallSetInputMachine(MachineWall *wall, Attachment *machine, cpVect gridPosition);
void mgMachineWallSetOutputMachine(MachineWall *wall, Attachment *machine, cpVect gridPosition);

cpBody *mgMachineWallGetInputMachineBody(MachineWall *wall);
cpBody *mgMachineWallGetOutputMachineBody(MachineWall *wall);

void mgMachineWallFree(MachineWall *wall);

