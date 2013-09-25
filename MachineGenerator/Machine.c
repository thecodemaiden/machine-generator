//
//  Machine.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#include <Machine.h>

MachineDescription *mgMachineNew()
{
    
    MachineDescription *m = (MachineDescription *)calloc(1, sizeof(MachineDescription));
    m->length = m->height = 0;
    m->machineType = MACHINE_BOX;
    for (int i=0; i<MAX_ATTACHMENT; i++) {
        *(m->children + i) = NULL;
    }
    
    return m;
}

void mgMachineFree(MachineDescription *md)
{
    for (int i=0; i<MAX_ATTACHMENT; i++) {
        if (md->children[i]) {
            mgAttachmentFree(md->children[i]);
        }
    }
    free(md);
}

Attachment *mgAttachmentNew()
{
    Attachment *a = (Attachment *)calloc(1, sizeof(Attachment));
    a->attachmentType = MACHINE_BASE;
    a->attachPoint = cpv(0, 0);
    a->parentAttachPoint = cpv(0, 0);
    a->offset = cpv(0,0);
    a->machine = NULL;
    return a;
}

void mgAttachmentFree(Attachment * at)
{
    if (at->machine)
        mgMachineFree(at->machine);
    
    free(at);
}


cpBody *bodyBuildingHelper(Attachment *at, cpBody *parentBody, cpGroup parentGroup, cpSpace *s)
{
    // the location of the parent shape in space has to be added to the attachPoint
    
    cpBody *machineBody = NULL;
    cpShape *machineShape = NULL;
    MachineDescription *md = at->machine;
    cpGroup attachmentGroup = parentGroup;
    
    cpVect parentPos = parentBody ? cpBodyGetPos(parentBody) : cpv(0, 0);
    
    if (md->height == 0)
        md->height = 1;
    
    cpFloat bodyMass = 0.0;
    cpFloat bodyMoment = 0.0;
    
    if (md->machineType == MACHINE_WHEEL) {
        //  md->length = MAX(md->length, md->height);
        bodyMass = sqrt(M_PI*md->length*md->length);
        bodyMoment = cpMomentForCircle(bodyMass, 0, md->length, cpv(0, 0));
    } else if (md->machineType == MACHINE_BOX) {
        bodyMass = sqrt(md->length*md->height);
        bodyMoment = cpMomentForBox(bodyMass, md->length, md->height);
    }
    
    bodyMass *= MASS_MULTIPLIER;
    
    machineBody = cpBodyNew(bodyMass, bodyMoment);
    cpSpaceAddBody(s, machineBody);
    
    cpVect truePos = cpvadd(parentPos, at->offset);
    cpBodySetPos(machineBody, truePos);
    
    if (md->machineType == MACHINE_WHEEL) {
        machineShape = cpCircleShapeNew(machineBody, md->length, cpv(0, 0));
    } else if (md->machineType == MACHINE_BOX) {
        cpVect boxVerts[4] = {cpv(-md->length/2.0, -md->height/2.0),
            cpv(-md->length/2.0, md->height/2.0),
            cpv(md->length/2.0, md->height/2.0),
            cpv(md->length/2.0, -md->height/2.0)};
        machineShape = cpPolyShapeNew(machineBody, 4, boxVerts, cpv(0, 0));
    }
    cpSpaceAddShape(s, machineShape);
    cpShapeSetElasticity(machineShape, 1.0);
    
    //attach to parent - if we have one
    if (parentBody) {
        // find parent attach points in local coordinates
        cpVect parentAttachLocal = cpBodyLocal2World(parentBody, at->parentAttachPoint);
        parentAttachLocal = cpBodyWorld2Local(machineBody, parentAttachLocal);
        
        cpFloat attachmentLength = cpvdist(parentAttachLocal, at->attachPoint);
        
        if (at->attachmentType == MACHINE_BASE) {
            // nothing to do - this should be an error....
        } else if (at->attachmentType == MACHINE_SPRING) {
            cpConstraint *spring = cpDampedSpringNew(parentBody, machineBody, at->parentAttachPoint, at->attachPoint, attachmentLength, 3.0, 5.0);
            cpSpaceAddConstraint(s, spring);
            attachmentGroup++; // this child will collide with parent
        } else if (at->attachmentType == MACHINE_ROLL) {
            //same attachment group - will not collide with parent
            
            cpConstraint *joint = cpPinJointNew(parentBody, machineBody, at->parentAttachPoint, at->attachPoint);
            cpSpaceAddConstraint(s, joint);
        } else if (at->attachmentType == MACHINE_PIVOT) {
            // length is irrelevant
            //same attachment group - will not collide with parent
            cpConstraint *pivot = cpPivotJointNew2(parentBody, machineBody, at->parentAttachPoint, at->attachPoint);
            cpSpaceAddConstraint(s, pivot);
            
        }
    }
    
    cpShapeSetGroup(machineShape, attachmentGroup);
    
    
    int childIdx = 0;
    while (md->children[childIdx] != NULL) {
        Attachment childAttachment = *md->children[childIdx];
        bodyBuildingHelper(&childAttachment, machineBody, attachmentGroup, s);
        childIdx++;
    }
    
    
    return machineBody;
}

cpBody *bodyFromDescription(MachineDescription *md, cpSpace *space)
{
    cpBody *machineBody = NULL;
    
    cpGroup currentGroup = 1; // necessary to stop objects joined at a pivot from colliding
    // bodies don't collide with objects in the same group
    
    Attachment baseAttachment;
    baseAttachment.parentAttachPoint = cpv(0, 0);
    baseAttachment.attachPoint = cpv(0,0);
    baseAttachment.attachmentType = MACHINE_BASE;
    baseAttachment.machine = md;
    
    machineBody = bodyBuildingHelper(&baseAttachment, NULL, currentGroup, space);
    
    return machineBody;
}