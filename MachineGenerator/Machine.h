//
//  Machine.h
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "chipmunk.h"

#define MASS_MULTIPLIER 0.2 // in case we want to make things more or less massive

#define SPRING_STIFFNESS 10.0
#define SPRING_DAMPING 1.0

#define GEAR_RATIO 1.0

#define MACHINE_LAYER 1 // we do not want to collide with the wall we attach machines too
typedef enum {
//    MACHINE_BASE, // not attached to anything
    ATTACH_GEAR,
    ATTACH_SPRING,
    ATTACH_FIXED, // the attachment points maintain their distance
    
    // the pivot jointis like a hinge if the attach points don't overlap
    ATTACH_PIVOT, // the attachment can rotate round a point on the parent
    
    ATTACH_SLIDE, // attachment points have a maximum distance
    ATTACH_TYPE_MAX //sentinel value for making random attachment
} AttachmentType;

typedef enum {
    MACHINE_BOX,
    MACHINE_WHEEL,
    MACHINE_TYPE_MAX //sentinel for making random machine
} BodyType;


// a recursive structure
struct MachineDescription;

typedef struct Attachment
{
    cpVect firstAttachPoint;
    cpVect secondAttachPoint;
    // length of springs/struts will be the distance between attach points
    
    AttachmentType attachmentType;
    float attachmentLength; // distance between parent and child attachment points
    
    cpConstraint *constraint; // NULL until the attachment is formed
} Attachment;

typedef struct MachineDescription {
    cpFloat length;
    cpFloat height; // only relevant to the bar
    BodyType machineType;
    cpBody *body; // NULL until it is built or attached
} MachineDescription;

// allocates space for a new machine
MachineDescription *mgMachineNew(); // free when done

// frees the physical body but not the description
void mgMachineDestroy(MachineDescription *md);

// frees the space for the machine description
void mgMachineFree(MachineDescription *md);

// allocates space and copies description but not the physical body
MachineDescription *mgMachineCopy(MachineDescription *md); // free when done

// allocates space for a new attachment
Attachment *mgAttachmentNew(); //free when done

void mgAttachmentFree(Attachment * at);

// allocates space and copies attachment info
Attachment *mgAttachmentCopy(Attachment *at); // free when done

// attach a body to another with the attachment info given
void mgMachineAttachToBody(Attachment *attachment, cpBody *machineBody, cpBody *body, cpSpace *space);

// detach two connected bodies completely
void mgMachineDetachFromBody(cpBody *machineBody, cpBody *parentBody, cpSpace *space);

// create the physical body for the machine description
void constructBodyForDescription(MachineDescription *md, cpVect position, cpSpace *space);

