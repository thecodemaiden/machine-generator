//
//  Machine.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#include <Machine.h>

    // bodies don't collide with objects in the same group
static cpGroup availableGroup = 1;

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
//    for (int i=0; i<MAX_ATTACHMENT; i++) {
//        if (md->children[i]) {
//            mgAttachmentFree(md->children[i]);
//        }
//    }
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
    a->parent = NULL;
    return a;
}

void mgAttachmentFree(Attachment * at)
{
//    if (at->machine)
//        mgMachineFree(at->machine);
//    
    free(at);
}


boolean_t mgMachineAttach(MachineDescription *parent, Attachment *attachment)
{
    int i = 0;
    while (parent->children[i])
        i++;
    if (i<MAX_ATTACHMENT) {
        attachment->parent = parent;
        parent->children[i] = attachment;
        return true;
    }
    return false;
}

static void attachmentHelper(cpBody *machineBody, cpBody *parentBody, AttachmentType attachmentType, cpVect localAttachPoint, cpVect parentAttachPoint, cpSpace *s)
{
    // find parent attach points in local coordinates
    cpVect parentAttachLocal = cpBodyLocal2World(parentBody, parentAttachPoint);
    parentAttachLocal = cpBodyWorld2Local(machineBody, parentAttachLocal);
    
    cpFloat attachmentLength = cpvdist(parentAttachLocal, localAttachPoint);
    
    if (attachmentType == MACHINE_BASE) {
        // nothing to do - this should be an error....
    }
    
    if (attachmentType == MACHINE_SPRING) {
        cpConstraint *spring = cpDampedSpringNew(parentBody, machineBody, parentAttachPoint, localAttachPoint, attachmentLength, SPRING_STIFFNESS, SPRING_DAMPING);
        cpSpaceAddConstraint(s, spring);
    }
    
    if (attachmentType == MACHINE_FIXED) {
        //same attachment group - will not collide with parent
        if (cpfabs(attachmentLength) < 1.0) {
            // zero length pin joints are bad for simulation
            attachmentType = MACHINE_PIVOT;
        } else {
            cpConstraint *joint = cpPinJointNew(parentBody, machineBody, parentAttachPoint, localAttachPoint);
            cpSpaceAddConstraint(s, joint);
        }
    }
    
    if (attachmentType == MACHINE_PIVOT) {
        // length is irrelevant
        //same attachment group - will not collide with parent
        cpConstraint *pivot = cpPivotJointNew2(parentBody, machineBody, parentAttachPoint, localAttachPoint);
        cpSpaceAddConstraint(s, pivot);
    }
    
    if (attachmentType == MACHINE_GEAR) {
        cpConstraint *gear = cpGearJointNew(parentBody, machineBody, 0.0, -GEAR_RATIO);
        cpSpaceAddConstraint(s, gear);
        
        // lash them together as well
        cpConstraint  *weldJoint = cpPinJointNew(parentBody, machineBody, parentAttachPoint, localAttachPoint);
        cpSpaceAddConstraint(s, weldJoint);
        
        cpFloat jointLength = cpvdist(parentAttachLocal, localAttachPoint);
        cpPinJointSetDist(weldJoint, jointLength);
        
    }
    
}


static void bodyBuildingHelper(Attachment *at, cpBody *parentBody, cpGroup parentGroup, cpSpace *s)
{
    // the location of the parent shape in space has to be added to the attachPoint
    
    cpBody *machineBody = NULL;
    cpShape *machineShape = NULL;
    MachineDescription *md = at->machine;
    cpVect parentAttachPoint = cpv(0,0);
    cpVect localAttachPoint = cpv(0,0);
    cpVect parentPos;
    
 
    
    // the attachpoints have components in the range -1 to 1, signifying the edges (left to right, top to bottom)
    // body coords go from -length/2 to length/2 and -height/2 to height/2
   
    if (md->height == 0)
        md->height = 1;
    
    cpFloat bodyMass = 0.0;
    cpFloat bodyMoment = 0.0;
    
    if (md->machineType == MACHINE_WHEEL) {
        md->height = md->length;
        bodyMass = sqrt(M_PI*md->length*md->length)*MASS_MULTIPLIER;
    } else if (md->machineType == MACHINE_BOX) {
        bodyMass = sqrt(md->length*md->height)*MASS_MULTIPLIER;
    }
    
    machineBody = cpBodyNew(bodyMass, 1.0); // going to set moment later
    cpSpaceAddBody(s, machineBody);
    
    if (parentBody) {
        parentPos =  cpBodyGetPos(parentBody);

    } else {
        parentPos = at->parentAttachPoint;
 
    }

    localAttachPoint = cpv(md->length*at->attachPoint.x/2, md->height*at->attachPoint.y/2);

    if (at->parent) {
        parentAttachPoint = cpv(at->parent->length*at->parentAttachPoint.x/2, at->parent->height*at->parentAttachPoint.y/2);
    }
    
    
    cpVect truePos = cpvadd(parentPos, at->offset);
    cpBodySetPos(machineBody, truePos);

    
    if (md->machineType == MACHINE_WHEEL) {
        const int numSegments = 12;
        cpVect circleVerts[numSegments];
        float theta = 2 * 3.1415926 /numSegments;
        float tangetial_factor = tanf(theta);//calculate the tangential factor
        
        float radial_factor = cosf(theta);//calculate the radial factor
        
        float x = md->length/2;//we start at angle = 0
        
        float y = 0;
        
        for(int ii = 0; ii < numSegments; ii++)
        {
            circleVerts[ii] = cpv(x, y);
            
            //calculate the tangential vector
            //remember, the radial vector is (x, y)
            //to get the tangential vector we flip those coordinates and negate one of them
            
            float tx = -y;
            float ty = x;
            
            //add the tangential vector
            
            x -= tx * tangetial_factor;
            y -= ty * tangetial_factor;
            
            //correct using the radial factor 
            
            x *= radial_factor; 
            y *= radial_factor; 
        } 
        
       // there is no rolling friction, so I approximate circles.
        machineShape = cpPolyShapeNew(machineBody, numSegments, circleVerts, cpv(0, 0));
        bodyMoment = cpMomentForPoly(bodyMass, numSegments, circleVerts, cpv(0, 0));
    } else if (md->machineType == MACHINE_BOX) {
        machineShape = cpBoxShapeNew(machineBody, md->length, md->height);
        bodyMoment = cpMomentForBox(bodyMass, md->length, md->height);
    }
    cpBodySetMoment(machineBody, bodyMoment);
    cpSpaceAddShape(s, machineShape);
    cpShapeSetElasticity(machineShape, 1.0);
    cpShapeSetLayers(machineShape, MACHINE_LAYER);
 //   cpShapeSetFriction(machineShape, 0.5);
    
    //attach to parent - if we have one
    if (parentBody) {
        AttachmentType attachmentType = at->attachmentType;
        if (attachmentType == MACHINE_SPRING || attachmentType == MACHINE_FIXED) {
            parentGroup++; // this child will collide with parent
        }
        attachmentHelper(machineBody, parentBody, attachmentType, localAttachPoint, parentAttachPoint, s);
    }
    
    cpShapeSetGroup(machineShape, parentGroup);
    md->body = machineBody;
    
    int childIdx = 0;
    while (md->children[childIdx] != NULL) {
        Attachment childAttachment = *md->children[childIdx];
        bodyBuildingHelper(&childAttachment, machineBody, parentGroup, s);
        childIdx++;
    }
    
    // we increase the availableGroup so that new machines will collide with this one
    availableGroup = MAX(availableGroup, parentGroup+1);
    
}


void mgMachineAttachToBody(Attachment *attachment, cpBody *body, cpSpace *space)
{
    attachment->parent = NULL;
    bodyBuildingHelper(attachment, body, availableGroup, space);
    
}

static void removeShapes(cpBody *body, cpShape *s, void *data)
{
    cpSpaceRemoveShape(cpShapeGetSpace(s), s);
}

static void removeAttachments(cpBody *body, cpConstraint *constraint, void *data)
{
    cpBody *childBody = cpConstraintGetB(constraint);
    cpSpace *s = cpBodyGetSpace(body);

    if (childBody != body) {
        mgMachineRemoveAttachmentFromSpace(body, s);
        cpBodyEachShape(childBody, removeShapes, NULL);
        cpSpaceRemoveBody(s, childBody);
    }
    cpSpaceRemoveConstraint(s, constraint);
}

void mgMachineRemoveAttachmentFromSpace(cpBody *attachmentBody, cpSpace *space)
{
    cpBodyEachConstraint(attachmentBody, removeAttachments, NULL);
    cpSpaceRemoveBody(space, attachmentBody);
    cpBodyEachShape(attachmentBody, removeShapes, NULL);

}

cpBody *bodyFromDescription(MachineDescription *md, cpVect position, cpSpace *space)
{
    cpBody *machineBody = NULL;
    
    Attachment baseAttachment;
    baseAttachment.parentAttachPoint = position;
    baseAttachment.attachPoint = cpv(0,0);
    baseAttachment.attachmentType = MACHINE_BASE;
    baseAttachment.machine = md;
    baseAttachment.parent = NULL;

    bodyBuildingHelper(&baseAttachment, NULL, availableGroup, space);
    
    return machineBody;
}