//
//  Machine.h
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "chipmunk.h"

#define MAX_ATTACHMENT 3 // up to 3 attachments for now
#define MASS_MULTIPLIER 0.8 // in case we want to make things more or less massive

#define SPRING_STIFFNESS 500.0
#define SPRING_DAMPING 800.0

#define GEAR_RATIO 1.0

#define MACHINE_LAYER 1 // we do not want to collide with the wall we attach machines too
typedef enum {
    MACHINE_BASE, // not attached to anything
    MACHINE_GEAR,
    MACHINE_SPRING,
    MACHINE_FIXED, // the attachment points maintain their distance
    MACHINE_PIVOT, // the attachment can rotate round a point on the parent
} AttachmentType;

typedef enum {
    MACHINE_BOX,
    MACHINE_WHEEL,
} BodyType;


// a recursive structure
struct MachineDescription;

typedef struct Attachment
{
    cpVect parentAttachPoint;
    cpVect attachPoint;
    // length of springs/struts will be the distance between attach points
    
    AttachmentType attachmentType;
    cpVect offset; // offset from parent center to child center
    struct MachineDescription *machine;
} Attachment;

typedef struct MachineDescription {
    cpFloat length;
    cpFloat height; // only relevant to the bar
    BodyType machineType;
    Attachment *children[MAX_ATTACHMENT]; // we could change this to a linked list of attachments
    cpBody *body; // NULL until it is built or attached
} MachineDescription;

MachineDescription *mgMachineNew();
void mgMachineDestroy(MachineDescription *md); // you need to free attachments yourself
void mgMachineFree(MachineDescription *md);


Attachment *mgAttachmentNew(); // a blank attachment
void mgAttachmentFree(Attachment * at); // the attached machine is not freed

// returns true if there was space to attach the child
boolean_t mgMachineAttach(MachineDescription *parent, Attachment *attachment);

void mgMachineAttachToBody(Attachment *attachment, cpBody *body, cpSpace *space);

void mgMachineDetachFromBody(cpBody *attachmentBody, cpBody *parentBody, cpSpace *space);

cpBody *bodyFromDescription(MachineDescription *md, cpVect position, cpSpace *space);

