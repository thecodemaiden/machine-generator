//
//  Machine.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#include <Machine.h>

// bodies don't collide with objects in the same group, unless group is 0 (NO_GROUP)
static cpGroup availableGroup = 1;


static void removeShapes(cpBody *body, cpShape *s, void *data)
{

    cpSpaceRemoveShape(cpShapeGetSpace(s), s);
    cpShapeFree(s);
}

static void removeAttachments(cpBody *body, cpConstraint *constraint, void *data)
{
    cpBody *childBody = cpConstraintGetB(constraint);
    
    if (childBody == body)
        childBody = cpConstraintGetA(constraint);
    
    cpSpace *s = cpBodyGetSpace(body);
    
    if (childBody != body) {
        cpBodyEachConstraint(childBody, removeAttachments, data);
        cpBodyEachShape(childBody, removeShapes, NULL);
        cpSpaceRemoveBody(s, childBody);
    } else {
        cpSpaceRemoveConstraint(s, constraint);
        cpConstraintFree(constraint);
    }
}

MachineDescription *mgMachineNew()
{
    MachineDescription *m = (MachineDescription *)calloc(1, sizeof(MachineDescription));
    m->length = m->height = 0;
    m->machineType = MACHINE_BOX;
    
    return m;
}

MachineDescription *mgMachineCopy(MachineDescription *md)
{
    MachineDescription *copy = mgMachineNew();
    copy->length = md->length;
    copy->height = md->height;
    copy->machineType = md->machineType;
    
    return copy;
}

Attachment *mgAttachmentCopy(Attachment *at)
{
    Attachment *copy = mgAttachmentNew();
    copy->attachmentLength = at->attachmentLength;
    copy->attachmentType = at->attachmentType;
    copy->firstAttachPoint = at->firstAttachPoint;
    copy->secondAttachPoint = at->secondAttachPoint;
    
    return copy;
}

void mgMachineDestroy(MachineDescription *md){
    if (md->body) {
        cpSpace *space = cpBodyGetSpace(md->body);
        cpBodyEachConstraint(md->body, removeAttachments, NULL);
        cpBodyEachShape(md->body, removeShapes, NULL);
        cpSpaceRemoveBody(space, md->body);
        cpBodyFree(md->body);
    }
    md->body = NULL;
}

void mgMachineFree(MachineDescription *md)
{
    mgMachineDestroy(md);
    free(md);
}

Attachment *mgAttachmentNew()
{
    Attachment *a = (Attachment *)calloc(1, sizeof(Attachment));
    a->attachmentType = ATTACH_FIXED;
    a->secondAttachPoint = cpv(0, 0);
    a->firstAttachPoint = cpv(0, 0);
    a->attachmentLength = 0;
    return a;
}

void mgAttachmentFree(Attachment * at)
{
    cpSpace *s =cpConstraintGetSpace(at->constraint);
    if (s) {
        cpSpaceRemoveConstraint(s, at->constraint);
        cpConstraintFree(at->constraint);
    }
    free(at);
}



static void getShapeForBody(cpBody *body, cpShape *currentShape, void *data) {
    // assumes only one shape attached to this body - true for now
    if (data)
        *(cpShape **)data = currentShape;
}



 void constructBodyForDescription(MachineDescription *md, cpVect worldPosition, cpSpace *s)
{
    cpBody *machineBody = NULL;
    cpShape *machineShape = NULL;
    
    if (md->body) {
        mgMachineDestroy(md); // we will rebuild it
    }
    
        if (md->height == 0)
            md->height = 1;
    
        cpFloat bodyMass = 1.0;
        cpFloat bodyMoment = 0.0;
        
//        if (md->machineType == MACHINE_WHEEL) {
//            md->height = md->length;
//            bodyMass = (M_PI*md->length*md->length)*MASS_MULTIPLIER;
//        } else if (md->machineType == MACHINE_BOX) {
//            bodyMass = (md->length*md->height)*MASS_MULTIPLIER;
//        }
//        
        machineBody = cpBodyNew(bodyMass, 1.0); // going to set moment later
        cpSpaceAddBody(s, machineBody);
    
        if (md->machineType == MACHINE_WHEEL) {

            machineShape = cpCircleShapeNew(machineBody, md->length/2, cpvzero);
            bodyMoment = cpMomentForCircle(bodyMass, md->length/2, 0, cpvzero);
        } else if (md->machineType == MACHINE_BOX) {
            machineShape = cpBoxShapeNew(machineBody, md->length, md->height);
            bodyMoment = cpMomentForBox(bodyMass, md->length, md->height);
        }
        cpBodySetMoment(machineBody, bodyMoment);
        cpSpaceAddShape(s, machineShape);
        
        cpShapeSetElasticity(machineShape, 1.0);
        cpShapeSetLayers(machineShape, MACHINE_LAYER);
        //   cpShapeSetFriction(machineShape, 0.5);
        
    
        md->body = machineBody;
    
    cpBodySetPos(machineBody, worldPosition);
    
}


void mgMachineAttachToBody(Attachment *at, cpBody *machineBody, cpBody *otherBody, cpSpace *space)
{
    cpVect otherAttachPoint = cpv(0,0);
    cpVect localAttachPoint = cpv(0,0);
    
    // the attachpoints have components in the range -1 to 1, signifying the edges (left to right, top to bottom)
    // body coords go from -length/2 to length/2 and -height/2 to height/2
    
    cpShape *bodyShape = NULL;
    cpShape *otherShape = NULL;

    cpBodyEachShape(machineBody, getShapeForBody, &bodyShape);
    if (bodyShape) {
    cpBB boundingBox = cpShapeGetBB(bodyShape);
    cpFloat length = boundingBox.r - boundingBox.l;
    cpFloat height = boundingBox.t - boundingBox.b;
        localAttachPoint = cpv(length*at->secondAttachPoint.x/2, height*at->secondAttachPoint.y/2);
    }
    
    
    if (otherBody) {
        cpBodyEachShape(otherBody, getShapeForBody, &otherShape);
        if (otherShape) {
            cpBB boundingBox = cpShapeGetBB(otherShape);
            cpFloat length = boundingBox.r - boundingBox.l;
            cpFloat height = boundingBox.t - boundingBox.b;
            otherAttachPoint = cpv(length*at->firstAttachPoint.x/2, height*at->firstAttachPoint.y/2);
        }
    }
    
    // find attach point for other body in local coordinates
    cpVect otherAttachLocal = cpBodyLocal2World(otherBody, otherAttachPoint);
    otherAttachLocal = cpBodyWorld2Local(machineBody, otherAttachLocal);
    
    cpFloat bodyDistance = cpvdist(otherAttachLocal, localAttachPoint);
    
    assert(at->attachmentType != ATTACH_TYPE_MAX);
    
    cpConstraint *mainConstraint = NULL;
    
    if (at->attachmentType == ATTACH_SPRING) {
        if (at->attachmentLength == 0) {
            at->attachmentLength = bodyDistance;
        }
        
        mainConstraint = cpDampedSpringNew(otherBody, machineBody, otherAttachPoint, localAttachPoint, at->attachmentLength, SPRING_STIFFNESS, SPRING_DAMPING);
        cpSpaceAddConstraint(space, mainConstraint);
        
    }
    
    if (at->attachmentType == ATTACH_FIXED) {
        //    if (attachmentLength == 0 || attachmentLength > bodyDistance)
        at->attachmentLength = bodyDistance;
        
        if (cpfabs(at->attachmentLength) < 1.0) {
            // zero length pin joints are bad for simulation
            mainConstraint = cpPivotJointNew2(otherBody, machineBody, otherAttachPoint, localAttachPoint);
            cpSpaceAddConstraint(space, mainConstraint);
        } else {
            mainConstraint = cpPinJointNew(otherBody, machineBody, otherAttachPoint, localAttachPoint);
            cpSpaceAddConstraint(space, mainConstraint);
            cpPinJointSetDist(mainConstraint, at->attachmentLength);
        }
        
    }
    
    if (at->attachmentType == ATTACH_PIVOT) {
        // do we need this joint type?
        cpVect pivotAttachPoint = cpBodyLocal2World(machineBody, cpvlerpconst(localAttachPoint, otherAttachLocal, at->attachmentLength));
        mainConstraint = cpPivotJointNew(otherBody, machineBody, pivotAttachPoint);
        cpSpaceAddConstraint(space, mainConstraint);
        
    }
    
    if (at->attachmentType == ATTACH_GEAR) {
        //attachmentLength = bodyDistance;
        
        mainConstraint = cpGearJointNew(otherBody, machineBody, 0.0, -GEAR_RATIO);
        cpSpaceAddConstraint(space, mainConstraint);
        
        // lash them together as well
            cpConstraint  *weldJoint = cpPinJointNew(otherBody, machineBody, otherAttachPoint, localAttachPoint);
            cpSpaceAddConstraint(space, weldJoint);
            
            cpPinJointSetDist(weldJoint, at->attachmentLength);
        
    }
    
    if (at->attachmentType == ATTACH_SLIDE) {
        cpFloat bodyDistance = cpvdist(otherAttachLocal, localAttachPoint);
        
        mainConstraint = cpSlideJointNew(otherBody, machineBody, otherAttachPoint, localAttachPoint, MIN(at->attachmentLength, bodyDistance), MAX(at->attachmentLength, bodyDistance));
        cpSpaceAddConstraint(space, mainConstraint);
    }

    at->constraint = mainConstraint;
    
    cpGroup smallestGroup = MIN(cpShapeGetGroup(otherShape), cpShapeGetGroup(bodyShape));
    smallestGroup = MIN(smallestGroup, 1);
    
    if (at->attachmentType == ATTACH_PIVOT || at->attachmentType == ATTACH_GEAR) {
        // make them the same group so they don't collide
        cpShapeSetGroup(otherShape, smallestGroup);
        cpShapeSetGroup(bodyShape, smallestGroup);
    } else {
        // we increase the availableGroup so that new machines will collide with this one
        cpShapeSetGroup(bodyShape, availableGroup++);

    }
    
}


void mgMachineDetachFromBody(cpBody *attachmentBody, cpBody *otherBody, cpSpace *space)
{
    cpBodyEachConstraint_b(attachmentBody, ^(cpConstraint *constraint) {
        if ((cpConstraintGetA(constraint) == otherBody && cpConstraintGetB(constraint) == attachmentBody)) {
            cpSpaceRemoveConstraint(space, constraint);
            cpConstraintFree(constraint);
        }
    });
    

}
