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


static void removeShapes(cpBody *body, cpShape *s, void *data)
{
    cpSpaceRemoveShape(cpShapeGetSpace(s), s);
}

static void removeAttachments(cpBody *body, cpConstraint *constraint, void *data)
{
    cpBody *childBody = cpConstraintGetB(constraint);
    cpSpace *s = cpBodyGetSpace(body);
    
    if (childBody != body) {
        cpBodyEachConstraint(childBody, removeAttachments, data);
        cpBodyEachShape(childBody, removeShapes, NULL);
        cpSpaceRemoveBody(s, childBody);
    }
    cpSpaceRemoveConstraint(s, constraint);
}

MachineDescription *mgMachineNew()
{
    
    MachineDescription *m = (MachineDescription *)calloc(1, sizeof(MachineDescription));
    m->length = m->height = 0;
    m->machineType = MACHINE_BOX;
    
    return m;
}

void mgMachineDestroy(MachineDescription *md){
    cpSpace *space = cpBodyGetSpace(md->body);
    cpBodyEachConstraint(md->body, removeAttachments, NULL);
    cpSpaceRemoveBody(space, md->body);
    cpBodyEachShape(md->body, removeShapes, NULL);
    cpBodyFree(md->body);
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
    a->attachmentType = MACHINE_BASE;
    a->secondAttachPoint = cpv(0, 0);
    a->firstAttachPoint = cpv(0, 0);
    a->attachmentLength = 0;
    return a;
}

void mgAttachmentFree(Attachment * at)
{
    //    if (at->machine)
    //        mgMachineFree(at->machine);
    //
    free(at);
}



static void getShapeForBody(cpBody *body, cpShape *currentShape, void *data) {
    // assumes only one shape attached to this body
    if (data)
        *(cpShape **)data = currentShape;
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


 void constructBodyForDescription(MachineDescription *md, cpVect worldPosition, cpSpace *s)
{
    // the location of the parent shape in space has to be added to the attachPoint
    
    cpBody *machineBody = NULL;
    cpShape *machineShape = NULL;
    
    if (md->body) {
        mgMachineDestroy(md); // we will rebuild it
    }
    
        if (md->height == 0)
            md->height = 1;
        
        cpFloat bodyMass = 0.0;
        cpFloat bodyMoment = 0.0;
        
        if (md->machineType == MACHINE_WHEEL) {
            md->height = md->length;
            bodyMass = (M_PI*md->length*md->length)*MASS_MULTIPLIER;
        } else if (md->machineType == MACHINE_BOX) {
            bodyMass = (md->length*md->height)*MASS_MULTIPLIER;
        }
        
        machineBody = cpBodyNew(bodyMass, 1.0); // going to set moment later
        cpSpaceAddBody(s, machineBody);
    
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
        
    
        md->body = machineBody;
    
    cpBodySetPos(machineBody, worldPosition);
}


void mgMachineAttachToBody(Attachment *at, cpBody *machineBody, cpBody *parentBody, cpSpace *space)
{
    cpVect parentAttachPoint = cpv(0,0);
    cpVect localAttachPoint = cpv(0,0);
    
    // the attachpoints have components in the range -1 to 1, signifying the edges (left to right, top to bottom)
    // body coords go from -length/2 to length/2 and -height/2 to height/2
    
    cpShape *bodyShape = NULL;
    cpBodyEachShape(machineBody, getShapeForBody, &bodyShape);
    if (bodyShape) {
    cpBB boundingBox = cpShapeGetBB(bodyShape);
    cpFloat length = boundingBox.r - boundingBox.l;
    cpFloat height = boundingBox.t - boundingBox.b;
        localAttachPoint = cpv(length*at->secondAttachPoint.x/2, height*at->secondAttachPoint.y/2);

    }
    
    
    if (parentBody) {
        cpShape *parentShape = NULL;
        cpBodyEachShape(parentBody, getShapeForBody, &parentShape);
        if (parentShape) {
            cpBB boundingBox = cpShapeGetBB(parentShape);
            cpFloat length = boundingBox.r - boundingBox.l;
            cpFloat height = boundingBox.t - boundingBox.b;
            parentAttachPoint = cpv(length*at->firstAttachPoint.x/2, height*at->firstAttachPoint.y/2);
        }
    }
    
    attachmentHelper(machineBody, parentBody, at->attachmentType, localAttachPoint, parentAttachPoint, cpBodyGetSpace(machineBody));

    cpShapeSetGroup(bodyShape, availableGroup);

    /*
     // we increase the availableGroup so that new machines will collide with this one
     availableGroup = MAX(availableGroup, parentGroup+1);
     */
}


void mgMachineDetachFromBody(cpBody *attachmentBody, cpBody *parentBody, cpSpace *space)
{
    __block cpConstraint *constraintToDelete;
    
    cpBodyEachConstraint_b(attachmentBody, ^(cpConstraint *constraint) {
        if ((cpConstraintGetA(constraint) == parentBody && cpConstraintGetB(constraint) == attachmentBody) ||
            (cpConstraintGetA(constraint) == attachmentBody && cpConstraintGetB(constraint) == parentBody))
            constraintToDelete = constraint;
    });
    
    if (constraintToDelete) {
        cpSpaceRemoveConstraint(space, constraintToDelete);
    }
    
}
