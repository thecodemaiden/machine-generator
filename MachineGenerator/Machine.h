//
//  Machine.h
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "chipmunk.h"

#define MAX_ATTACHMENT 3 // up to 3 attachments for now

enum {
    MACHINE_BASE, // not attached to anything
    MACHINE_GEAR,
    MACHINE_SPRING,
    MACHINE_WELD,
};

enum {
    MACHINE_BOX,
    MACHINE_WHEEL,
};


// a recursive structure
struct MachineDescription;

typedef struct Attachment
{
    cpVect parentAttachPoint;
    cpVect attachPoint;
    int attachmentType;
    cpFloat length; // how far apart are parent and child
    struct MachineDescription *machine;
} Attachment;

typedef struct MachineDescription {
    cpFloat length;
    cpFloat height; // only relevant to the bar
    int machineType;
    Attachment children[MAX_ATTACHMENT];
} MachineDescription;

cpBody *bodyFromDescription(MachineDescription *md, cpSpace *space);


