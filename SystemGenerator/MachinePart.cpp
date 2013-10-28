//
//  MachinePart.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

#include "MachinePart.h"


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

cpVect MachinePart::getOriginalPosition()
{
    return position;
}

void MachinePart::setOriginalPosition(cpVect position)
{
    this->position = position;
}

void MachinePart::removeFromSpace()
{
    // if you do this directly on a machine in a machine system, you'll have a Bad Time
    if (body) {
        
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
    
    AttachmentType attachmentType = attachment->attachmentType();
        
    cpConstraint *mainConstraint = NULL;
    
    if (SpringAttachment *spring = dynamic_cast<SpringAttachment *>(attachment)) {
//        if (spring->attachmentLength <= 0) {
//            spring->attachmentLength = bodyDistance;
//        }
        
        mainConstraint = cpDampedSpringNew(otherBody, body, otherAttachPoint, localAttachPoint, spring->attachmentLength, spring->stiffness, spring->damping);
        cpSpaceAddConstraint(space, mainConstraint);
        
    }
    
    if (FixedAttachment *fixed = dynamic_cast<FixedAttachment *>(attachment)) {
        fixed->attachmentLength = bodyDistance;
        
        if (cpfabs(fixed->attachmentLength) < 1.0) {
            // zero length pin joints are bad for simulation
            mainConstraint = cpPivotJointNew2(otherBody, body, otherAttachPoint, localAttachPoint);
            cpSpaceAddConstraint(space, mainConstraint);
        } else {
            mainConstraint = cpPinJointNew(otherBody, body, otherAttachPoint, localAttachPoint);
            cpSpaceAddConstraint(space, mainConstraint);
            cpPinJointSetDist(mainConstraint, fixed->attachmentLength);
        }
        
    }
    
    if (PivotAttachment *pivot = dynamic_cast<PivotAttachment *>(attachment)) {
        // get the world coords of the attach points and interpolate between them
        cpVect worldAttachPoint1 = cpBodyLocal2World(body, localAttachPoint);
        cpVect worldAttachPoint2 = cpBodyLocal2World(otherBody, otherAttachPoint);
        cpVect pivotAttachPoint = cpvlerp(worldAttachPoint1, worldAttachPoint2, pivot->pivotPosition);
        mainConstraint = cpPivotJointNew(otherBody, body, pivotAttachPoint);
        cpSpaceAddConstraint(space, mainConstraint);
    }
    
    if (GearAttachment *gear = dynamic_cast<GearAttachment *>(attachment)) {
       
        // also make sure they don't move relative to each other - with a pin joint in their centers
        cpConstraint *distanceConstraint = cpPinJointNew(body, otherBody, cpvzero, cpvzero);
        cpSpaceAddConstraint(space, distanceConstraint);
        
        mainConstraint = cpGearJointNew(otherBody, body, gear->phase, gear->gearRatio);
        cpSpaceAddConstraint(space, mainConstraint);
        
    }
    
    if (SlideAttachment *slide = dynamic_cast<SlideAttachment *>(attachment)) {
        if (slide->maxDistance == 0)
            slide->maxDistance = bodyDistance;
        if (slide->minDistance > slide->maxDistance) {
            cpFloat temp = slide->minDistance;
            slide->minDistance = slide->maxDistance;
            slide->maxDistance = temp;
        }
        
        mainConstraint = cpSlideJointNew(otherBody, body, otherAttachPoint, localAttachPoint, slide->minDistance,slide->maxDistance);
        cpSpaceAddConstraint(space, mainConstraint);
    }
    
    attachment->constraint = mainConstraint;
    mainConstraint->data = attachment;
    
    cpGroup smallestGroup = MIN(cpShapeGetGroup(otherShape), cpShapeGetGroup(bodyShape));
    smallestGroup = MIN(smallestGroup, 1);
    
    if (attachmentType == ATTACH_PIVOT) {
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
            if (constraint->data) {
                Attachment *a = (Attachment *)constraint->data;
                a->constraint = NULL;
            }
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