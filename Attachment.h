//
//  Attachment.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/27/13.
//
//

#ifndef __SystemGenerator__Attachment__
#define __SystemGenerator__Attachment__

extern "C" {
#include "chipmunk.h"
}



typedef enum {
    ATTACH_SPRING,
    ATTACH_FIXED, // the attachment points maintain their distance
    
    // the pivot joint is like a hinge if the attach points don't overlap
    ATTACH_PIVOT, // the attachment can rotate round a point on the parent
    
    ATTACH_SLIDE, // attachment points have a maximum distance
    ATTACH_GEAR, //
    ATTACH_TYPE_MAX //sentinel value for making random attachment
} AttachmentType;


// do not directly make the generic attachments. Instead make one of the ones below
class Attachment
{
public:
    // attach points run {-1,1} for each component: {left,right} and {top,bottom} respectively
    // {0,0} is body center
    cpVect firstAttachPoint;
    cpVect secondAttachPoint;
    
    cpFloat attachmentLength; // distance between parent and child attachment points
    
    cpConstraint *constraint; // NULL until the attachment is formed
    
    
    // constructor
    Attachment(cpVect firstAttachPoint = cpvzero, cpVect secondAttachPoint = cpvzero, cpFloat attachmentLength=0.0);
    
    static Attachment *createAttachmentOfType(AttachmentType t);
    static Attachment *copyAttachment(Attachment *original);

    virtual AttachmentType attachmentType()=0;
    
    virtual ~Attachment();
};

class GearAttachment : public Attachment
{
public:
    
    cpFloat gearRatio;
    cpFloat phase;
    GearAttachment(cpVect firstAttachPoint = cpvzero, cpVect secondAttachPoint = cpvzero, cpFloat attachmentLength=0.0, cpFloat ratio=-1.0, cpFloat phase=0.0)
    :Attachment(firstAttachPoint, secondAttachPoint, attachmentLength),
    gearRatio(ratio), phase(phase){};
    
    AttachmentType attachmentType(){return ATTACH_GEAR;}
};

class SpringAttachment : public Attachment
{
public:

    cpFloat damping;
    cpFloat stiffness;
    SpringAttachment(cpVect firstAttachPoint = cpvzero, cpVect secondAttachPoint = cpvzero, cpFloat attachmentLength=0.0, cpFloat d=1.0, cpFloat s=10.0)
    :Attachment(firstAttachPoint, secondAttachPoint, attachmentLength),
    damping(d), stiffness(s){};
    
    AttachmentType attachmentType(){return ATTACH_SPRING;}
};

class SlideAttachment : public Attachment
{
public:

    
    cpFloat maxDistance;
    cpFloat minDistance;
    SlideAttachment(cpVect firstAttachPoint = cpvzero, cpVect secondAttachPoint = cpvzero, cpFloat attachmentLength=0.0, cpFloat maxD=0.0, cpFloat minD=0.0)
    :Attachment(firstAttachPoint, secondAttachPoint, attachmentLength),
    maxDistance(maxD), minDistance(minD){};
    
    AttachmentType attachmentType(){return ATTACH_SLIDE;}
};

class PivotAttachment : public Attachment
{
public:
    cpFloat pivotPosition; // a number between 0 and 1
                           // the placement of the hinge point between the attachment points
    PivotAttachment(cpVect firstAttachPoint = cpvzero, cpVect secondAttachPoint = cpvzero, cpFloat attachmentLength=0.0, cpFloat pivotAt=1.0)
    :Attachment(firstAttachPoint, secondAttachPoint, attachmentLength),
    pivotPosition(pivotAt){};
    
    AttachmentType attachmentType(){return ATTACH_PIVOT;}

};

class FixedAttachment : public Attachment
{
public:
 // nothing special
    FixedAttachment(cpVect firstAttachPoint = cpvzero, cpVect secondAttachPoint = cpvzero, cpFloat attachmentLength=0.0):
    Attachment(firstAttachPoint, secondAttachPoint, attachmentLength){};

    AttachmentType attachmentType(){return ATTACH_FIXED;}
};



#endif /* defined(__SystemGenerator__Attachment__) */
