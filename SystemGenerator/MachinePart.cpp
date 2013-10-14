//
//  MachinePart.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

#include "MachinePart.h"

#pragma  mark - Attachments

Attachment::Attachment(AttachmentType attachmentType, cpVect firstAttachPoint, cpVect secondAttachPoint, cpFloat attachmentLength)
: attachmentType(attachmentType),
firstAttachPoint(firstAttachPoint),
secondAttachPoint(secondAttachPoint),
attachmentLength(attachmentLength)
{
    
}

Attachment::Attachment(const Attachment &toCopy)
:attachmentType(toCopy.attachmentType),
attachmentLength(toCopy.attachmentLength),
firstAttachPoint(toCopy.firstAttachPoint),
secondAttachPoint(toCopy.secondAttachPoint)
{
    
}

static void deleteCallback(cpSpace *space, void *key, void *data)
{
    cpConstraint *constraint = *(cpConstraint **)data;
    cpSpaceRemoveConstraint(space, constraint);
    cpConstraintFree(constraint);
}

Attachment::~Attachment()
{
    cpSpace *s =cpConstraintGetSpace(constraint);
    if (s) {
        if (cpSpaceIsLocked(s)) {
            cpSpaceAddPostStepCallback(s, &deleteCallback, this, (void *)&constraint);
        } else {
            deleteCallback(s, (void *)this, (void *)&constraint);
        }
    }
}

#pragma mark - Machine Parts

static int availableGroup = 1;

// constructors - do not build the body immediately
MachinePart::MachinePart(cpSpace *space, cpVect position)
:position(position),
space(space),
machineType(MACHINE_BOX),
body(NULL)
{
    
}

MachinePart::MachinePart(BodyType machineType, cpVect position, cpFloat length, cpFloat height, cpSpace *space)
:machineType(machineType),
position(position),
length(length),
height(height),
space(space),
body(NULL)
{
    
}

MachinePart::MachinePart(const MachinePart &other, cpSpace *space, cpVect position)
:machineType(other.machineType),
position(position),
length(other.length),
height(other.height),
space(space ?: other.space),
body(NULL)
{
    
}

cpVect MachinePart::getPosition()
{
    if (!body)
        return position;
    
    return cpBodyGetPos(body);
}

void MachinePart::setPosition(cpVect position)
{
    this->position = position;
    
    if (body)
        cpBodySetPos(body, position);
}

void MachinePart::removeFromSpace()
{
    // if you do this directly on a machine in a machine system, you'll have a Bad Time
    if (body)
        
        cpBodyEachConstraint_b(body, ^(cpConstraint *constraint) {
            cpSpaceRemoveConstraint(space, constraint);
            cpConstraintFree(constraint);
        });
    
    cpBodyEachShape_b(body, ^(cpShape *shape) {
        cpSpaceRemoveShape(space, shape);
        cpShapeFree(shape);
    });
    
    cpSpaceRemoveBody(space, body);
    cpBodyFree(body);
    body = NULL;
}

MachinePart::~MachinePart()
{
    removeFromSpace();
}

cpBody *MachinePart::getBody()
{
    return body;
}

void MachinePart::attachToBody(Attachment *attachment, cpBody *otherBody)
{
    if (!body) {
        constructBody(position);
    }
    
    cpVect otherAttachPoint = cpv(0,0);
    cpVect localAttachPoint = cpv(0,0);
    
    // the attachpoints have components in the range -1 to 1, signifying the edges (left to right, top to bottom)
    // body coords go from -length/2 to length/2 and -height/2 to height/2
    
    __block cpShape *bodyShape = NULL;
    __block cpShape *otherShape = NULL;
    
    // we assume machine part bodies contain only one shape
    cpBodyEachShape_b(body, ^(cpShape *shape) {
        bodyShape = shape;
    });
    
    if (bodyShape) {
        cpBB boundingBox = cpShapeGetBB(bodyShape);
        cpFloat length = boundingBox.r - boundingBox.l;
        cpFloat height = boundingBox.t - boundingBox.b;
        localAttachPoint = cpv(length*attachment->secondAttachPoint.x/2, height*attachment->secondAttachPoint.y/2);
    }
    
    if (otherBody) {
        cpBodyEachShape_b(otherBody, ^(cpShape *shape) {
            otherShape = shape;
        });
        
        if (otherShape) {
            cpBB boundingBox = cpShapeGetBB(otherShape);
            cpFloat length = boundingBox.r - boundingBox.l;
            cpFloat height = boundingBox.t - boundingBox.b;
            otherAttachPoint = cpv(length*attachment->firstAttachPoint.x/2, height*attachment->firstAttachPoint.y/2);
        }
    }
    
    // find attach point for other body in local coordinates
    cpVect otherAttachLocal = cpBodyLocal2World(otherBody, otherAttachPoint);
    otherAttachLocal = cpBodyWorld2Local(body, otherAttachLocal);
    
    cpFloat bodyDistance = cpvdist(otherAttachLocal, localAttachPoint);
    
    assert(attachment->attachmentType != ATTACH_TYPE_MAX);
    
    cpConstraint *mainConstraint = NULL;
    
    if (attachment->attachmentType == ATTACH_SPRING) {
        if (attachment->attachmentLength == 0) {
            attachment->attachmentLength = bodyDistance;
        }
        
        mainConstraint = cpDampedSpringNew(otherBody, body, otherAttachPoint, localAttachPoint, attachment->attachmentLength, SPRING_STIFFNESS, SPRING_DAMPING);
        cpSpaceAddConstraint(space, mainConstraint);
        
    }
    
    if (attachment->attachmentType == ATTACH_FIXED) {
        //    if (attachmentLength == 0 || attachmentLength > bodyDistance)
        attachment->attachmentLength = bodyDistance;
        
        if (cpfabs(attachment->attachmentLength) < 1.0) {
            // zero length pin joints are bad for simulation
            mainConstraint = cpPivotJointNew2(otherBody, body, otherAttachPoint, localAttachPoint);
            cpSpaceAddConstraint(space, mainConstraint);
        } else {
            mainConstraint = cpPinJointNew(otherBody, body, otherAttachPoint, localAttachPoint);
            cpSpaceAddConstraint(space, mainConstraint);
            cpPinJointSetDist(mainConstraint, attachment->attachmentLength);
        }
        
    }
    
    if (attachment->attachmentType == ATTACH_PIVOT) {
        // do we need this joint type?
        cpVect pivotAttachPoint = cpBodyLocal2World(otherBody, otherAttachPoint);
        mainConstraint = cpPivotJointNew(otherBody, body, pivotAttachPoint);
        cpSpaceAddConstraint(space, mainConstraint);
        
    }
    
    if (attachment->attachmentType == ATTACH_GEAR) {
        //attachmentLength = bodyDistance;
        
        mainConstraint = cpGearJointNew(otherBody, body, 0.0, -GEAR_RATIO);
        cpSpaceAddConstraint(space, mainConstraint);
        
        // lash them together as well
        cpConstraint  *weldJoint = cpPinJointNew(otherBody, body, otherAttachPoint, localAttachPoint);
        cpSpaceAddConstraint(space, weldJoint);
        
        cpPinJointSetDist(weldJoint, attachment->attachmentLength);
        
    }
    
    if (attachment->attachmentType == ATTACH_SLIDE) {
        cpFloat bodyDistance = cpvdist(otherAttachLocal, localAttachPoint);
        
        mainConstraint = cpSlideJointNew(otherBody, body, otherAttachPoint, localAttachPoint, MIN(attachment->attachmentLength, bodyDistance), MAX(attachment->attachmentLength, bodyDistance));
        cpSpaceAddConstraint(space, mainConstraint);
    }
    
    attachment->constraint = mainConstraint;
    
    cpGroup smallestGroup = MIN(cpShapeGetGroup(otherShape), cpShapeGetGroup(bodyShape));
    smallestGroup = MIN(smallestGroup, 1);
    
    if (attachment->attachmentType == ATTACH_PIVOT || attachment->attachmentType == ATTACH_GEAR) {
        // make them the same group so they don't collide
        cpShapeSetGroup(otherShape, smallestGroup);
        cpShapeSetGroup(bodyShape, smallestGroup);
    } else {
        // we increase the availableGroup so that new machines will collide with this one
        cpShapeSetGroup(bodyShape, availableGroup++);
        
    }
    
}

void MachinePart::detachFromBody(cpBody *otherBody)
{
    cpBodyEachConstraint_b(otherBody, ^(cpConstraint *constraint) {
        if ((cpConstraintGetA(constraint) == otherBody && cpConstraintGetB(constraint) == body)) {
            cpSpaceRemoveConstraint(space, constraint);
            cpConstraintFree(constraint);
        }
    });
}

void MachinePart::constructBody(cpVect position)
{
    cpBody *machineBody = NULL;
    cpShape *machineShape = NULL;
    
    if (body) {
        removeFromSpace(); // we'll rebuild it
    }
    
    if (height == 0)
        height = 1;
    
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
    cpSpaceAddBody(space, machineBody);
    
    if (machineType == MACHINE_WHEEL) {
        machineShape = cpCircleShapeNew(machineBody, length/2, cpvzero);
        bodyMoment = cpMomentForCircle(bodyMass, length/2, 0, cpvzero);
    } else if (machineType == MACHINE_BOX) {
        machineShape = cpBoxShapeNew(machineBody, length, height);
        bodyMoment = cpMomentForBox(bodyMass, length, height);
    }
    cpBodySetMoment(machineBody, bodyMoment);
    cpSpaceAddShape(space, machineShape);
    
    cpShapeSetElasticity(machineShape, 1.0);
    cpShapeSetLayers(machineShape, MACHINE_LAYER);
    //   cpShapeSetFriction(machineShape, 0.5);
    
    
    body = machineBody;
    
    cpBodySetPos(machineBody, position);
}