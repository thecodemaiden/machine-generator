//
//  MachineWall.h
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/25/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "chipmunk.h"
#import "Machine.h"


#define WALL_LAYER 1>>1 // to avoid colliding with machines
#define PEG_LAYER 1>>2 // so i can search for pegs easily when attaching machines

typedef struct  {
    Attachment **attachedMachines; // I feel no need to maintain a linked list - yet
    cpVect gridSpacing;
    cpVect inputMachinePosition;
    cpVect outputMachinePosition;
    cpBody *body;
    cpSpace *space;
    cpVect size; // n of pegs horizont
} MachineWall;

MachineWall *mgMachineWallNew(int width, int height, int hPegs, int vPegs, cpVect position, cpSpace *space);
void mgMachineWallAddMachine(MachineWall *wall, Attachment *newMachine, cpVect gridPosition);
void mgMachineWallRemoveMachine(MachineWall *wall, cpVect gridPosition);

void mgMachineWallSetInputMachinePosition(MachineWall *wall, cpVect gridPosition);
void mgMachineWallSetOutputMachinePosition(MachineWall *wall, cpVect gridPosition);

Attachment *mgMachineWallGetMachineAtPosition(MachineWall *wall, cpVect gridPosition);

void mgMachineWallFree(MachineWall *wall);

