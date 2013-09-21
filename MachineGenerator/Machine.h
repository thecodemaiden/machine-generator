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
    MACHINE_LEVER,
    MACHINE_GEAR,
    MACHINE_SPRING,
    MACHINE_WELD,
};

enum {
    MACHINE_BODY_CIRCLE,
    MACHINE_BODY_RECT,
};

// a recursive structure
struct MachineDescription;

typedef struct Attachment
{
    cpVect parentAttachPoint;
    int attachmentType;
    struct MachineDescription *bodyDesc;
} Attachment;

typedef struct MachineDescription {
    cpFloat size;
    int bodyType;
    Attachment children[MAX_ATTACHMENT];
} MachineDescription;

cpBody *bodyFromDescription(MachineDescription *md);


