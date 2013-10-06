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
// an attachment from B->A. It is undirected, so matrix[A][B] should be equal

// Since machines cannot be attached to themselves, I use the diagonal for the attachment to the wall


typedef struct  {
    MachineDescription **machines;
    Attachment ***attachments;
    cpVect gridSpacing;
    cpVect inputMachinePosition;
    cpVect outputMachinePosition;
    cpBody *body;
    cpSpace *space;
    cpVect size; // n of pegs horizont
} MachineWall;

// create a new system that can attach machine components
MachineWall *mgMachineWallNew(int width, int height, int hPegs, int vPegs, cpVect position, cpSpace *space);

// don't use a GEAR attachment to attach to the wall - the peg does not rotate, so the body will not rotate
void mgMachineWallAddMachine(MachineWall *wall, MachineDescription *newMachine, Attachment *attachment, cpVect gridPosition);
void mgMachineWallRemoveMachine(MachineWall *wall, cpVect gridPosition);

// necessary if we're going to mutate and possibly discard mutation results
MachineWall *mgMachineWallCopy(MachineWall *original, cpVect position); // free when done

MachineDescription *mgMachineWallGetMachineAtPosition(MachineWall *wall, cpVect gridPosition);
Attachment *mgMachineWallGetAttachmentBetween(MachineWall *wall, cpVect machine1Pos, cpVect machine2Pos);

boolean_t mgMachineWallAttachMachines(MachineWall *wall, cpVect machine1Pos, cpVect machine2Pos, Attachment *attachment);
boolean_t mgMachineWallDetachMachines(MachineWall *wall, cpVect machine1Pos, cpVect machine2Pos);

void mgMachineWallFree(MachineWall *wall); // frees machines and attachments too (should it?)

