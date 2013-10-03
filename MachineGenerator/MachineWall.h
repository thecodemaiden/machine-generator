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

// this machine wall is more like a graph of machines than the tree for single machines
// I will use an adjacency matrix: VxV array of Attachments, where matrix[B][A] exists if there is
// an attachment from B->A.


typedef struct  {
    MachineDescription **machines;
    MachineDescription ***attachments;
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

MachineDescription *mgMachineWallGetMachineAtPosition(MachineWall *wall, cpVect gridPosition);

boolean_t mgMachineWallAttachMachines(cpVect machine1Pos, cpVect machine2Pos, Attachment *attachment);

void mgMachineWallFree(MachineWall *wall);

