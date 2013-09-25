//
//  Machine.h
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "chipmunk.h"

#define MAX_ATTACHMENT 3 // up to 3 attachments for now
#define MASS_MULTIPLIER 0.5 // in case we want to make things more or less massive
enum {
    MACHINE_BASE, // not attached to anything
    MACHINE_GEAR, // this is kinda hard to imagine :|
    MACHINE_SPRING,
    MACHINE_FIXED, // the bodies do not move relative to each other
    MACHINE_PIVOT, // the attachment can rotate round a point on the parent
    MACHINE_ROLL, // better for wheels
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
    // length of springs/struts will be the distance between attach points
    int attachmentType;
    cpVect offset; // offset from parent center to child center
    struct MachineDescription *machine;
} Attachment;

typedef struct MachineDescription {
    cpFloat length;
    cpFloat height; // only relevant to the bar
    int machineType;
    Attachment *children[MAX_ATTACHMENT]; // we could change this to a linked list of attachments
} MachineDescription;

MachineDescription *mgMachineNew();
void mgMachineFree(MachineDescription *md);

Attachment *mgAttachmentNew();
void mgAttachmentFree(Attachment * at);

cpBody *bodyFromDescription(MachineDescription *md, cpSpace *space);


