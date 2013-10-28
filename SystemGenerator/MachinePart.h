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

#include "Attachment.h"
#include <iostream>

#define MASS_MULTIPLIER 0.2 // in case we want to make things more or less massive

typedef enum {
    MACHINE_BOX,
    MACHINE_WHEEL,
    MACHINE_TYPE_MAX //sentinel for making random machine
} BodyType;

#define MACHINE_LAYER 1 // we do not want to collide with the wall we attach machines to

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
    MachinePart(const MachinePart &other, cpSpace *space=NULL, cpVect position=cpvzero);
    
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
    void setOriginalPosition(cpVect position);
    cpVect getOriginalPosition();
    
private:
    cpVect position;
    cpSpace *space;
    // create the physical body for the machine description
    void constructBody(cpVect position);
};


#endif /* defined(__SystemGenerator__MachinePart__) */
