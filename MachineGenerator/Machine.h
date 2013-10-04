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
    MACHINE_GEAR,
    MACHINE_SPRING,
    MACHINE_FIXED, // the attachment points maintain their distance
    MACHINE_PIVOT, // the attachment can rotate round a point on the parent
    MACHINE_SLIDE, // attachment points have a maximum distance
    MACHINE_ATTACH_MAX //sentinal value for making random attachment
} AttachmentType;

typedef enum {
    MACHINE_BOX,
    MACHINE_WHEEL,
    MACHINE_TYPE_MAX //sentinel for making random attachment
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

} Attachment;

typedef struct MachineDescription {
    cpFloat length;
    cpFloat height; // only relevant to the bar
    BodyType machineType;
    cpBody *body; // NULL until it is built or attached
} MachineDescription;

MachineDescription *mgMachineNew();
void mgMachineDestroy(MachineDescription *md); // you need to free attachments yourself
void mgMachineFree(MachineDescription *md);
MachineDescription *mgMachineCopy(MachineDescription *md);

Attachment *mgAttachmentNew();
void mgAttachmentFree(Attachment * at);
Attachment *mgAttachmentCopy(Attachment *at);


void mgMachineAttachToBody(Attachment *attachment, cpBody *machineBody, cpBody *body, cpSpace *space);

void mgMachineDetachFromBody(cpBody *machineBody, cpBody *parentBody, cpSpace *space);

void constructBodyForDescription(MachineDescription *md, cpVect position, cpSpace *space);

