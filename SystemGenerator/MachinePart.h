//
//  MachinePart.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

#ifndef __SystemGenerator__MachinePart__
#define __SystemGenerator__MachinePart__

extern "C" {
#include "chipmunk.h"
}

#include <iostream>

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
    
    // the pivot joint is like a hinge if the attach points don't overlap
    ATTACH_PIVOT, // the attachment can rotate round a point on the parent
    
    ATTACH_SLIDE, // attachment points have a maximum distance
    ATTACH_TYPE_MAX //sentinel value for making random attachment
} AttachmentType;

typedef enum {
    MACHINE_BOX,
    MACHINE_WHEEL,
    MACHINE_TYPE_MAX //sentinel for making random machine
} BodyType;

class Attachment
{
public:
    // attach points run {-1,1} for each component: {left,right} and {top,bottom} respectively
    // {0,0} is body center
    cpVect firstAttachPoint;
    cpVect secondAttachPoint;
    
    AttachmentType attachmentType;
    cpFloat attachmentLength; // distance between parent and child attachment points
    
    cpConstraint *constraint; // NULL until the attachment is formed
    
    // constructor
    Attachment(AttachmentType attachmentType = ATTACH_FIXED, cpVect firstAttachPoint = cpvzero, cpVect secondAttachPoint = cpvzero, cpFloat attachmentLength=0.0);
    
    // copy constructor
    Attachment (const Attachment &toCopy);
    
    ~Attachment();
};

class MachinePart {
public:
    cpFloat length;
    cpFloat height; // only relevant to the bar
    BodyType machineType;
    cpBody *body; // NULL until it is built or attached
    
    // constructor - does not build the body immediately
    MachinePart(cpSpace *space, cpVect position=cpvzero);
    MachinePart(BodyType machineType, cpVect position, cpFloat length, cpFloat height, cpSpace *space);
    
    //copy constructor
    MachinePart(const MachinePart &other, cpVect position=cpvzero);
    
    // frees the physical body but not the description
    void removeFromSpace();
    
    // frees the space for the machine description
    ~MachinePart();
   
    // attach a body to another with the attachment info given
    // constructs the body if it does not exist
    void attachToBody(Attachment *attachment, cpBody *otherBody);
    
    // detach completely
    void detachFromBody(cpBody *otherBody);
    
    cpBody *getBody();
    void setPosition(cpVect position);
    cpVect getPosition();
    
private:
    cpVect position;
    cpSpace *space;
    // create the physical body for the machine description
    void constructBody(cpVect position);
};


#endif /* defined(__SystemGenerator__MachinePart__) */
