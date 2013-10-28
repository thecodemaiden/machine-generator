//
//  Attachment.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/27/13.
//
//

#include "Attachment.h"


Attachment::Attachment(cpVect firstAttachPoint, cpVect secondAttachPoint, cpFloat attachmentLength)
:firstAttachPoint(firstAttachPoint),
secondAttachPoint(secondAttachPoint),
attachmentLength(attachmentLength)
{
    
}

Attachment * Attachment::createAttachmentOfType(AttachmentType t)
{
    switch (t) {
        case ATTACH_GEAR:
            return new GearAttachment();
            break;
        case ATTACH_SPRING:
            return new SpringAttachment();
            break;
        case ATTACH_SLIDE:
            return new SlideAttachment();
            break;
        case ATTACH_PIVOT:
            return new PivotAttachment();
            break;
        case ATTACH_FIXED:
            return new FixedAttachment();
            break;
        default:
            return NULL;
            break;
    }
}

Attachment * Attachment::copyAttachment(Attachment *original)
{
    Attachment *out = NULL;
    
    if (SpringAttachment *spring = dynamic_cast<SpringAttachment *>(original)) {
        out = new SpringAttachment(spring->firstAttachPoint, spring->secondAttachPoint, spring->attachmentLength, spring->damping, spring->stiffness);
    }
    
    if (FixedAttachment *fixed = dynamic_cast<FixedAttachment *>(original)) {
        out = new FixedAttachment(fixed->firstAttachPoint, fixed->secondAttachPoint, fixed->attachmentLength);
    }
    
    if (PivotAttachment *pivot = dynamic_cast<PivotAttachment *>(original)) {
        out = new PivotAttachment(pivot->firstAttachPoint, pivot->secondAttachPoint, pivot->attachmentLength, pivot->pivotPosition);
   }
    
    if (GearAttachment *gear = dynamic_cast<GearAttachment *>(original)) {
        out = new GearAttachment(gear->firstAttachPoint, gear->secondAttachPoint, gear->attachmentLength, gear->gearRatio);
    }
    
    if (SlideAttachment *slide = dynamic_cast<SlideAttachment *>(original)) {
        out = new SlideAttachment(slide->firstAttachPoint, slide->secondAttachPoint, slide->attachmentLength, slide->maxDistance, slide->minDistance);
    }
    
    return out;
}


static void deleteCallback(cpSpace *space, void *key, void *data)
{
    cpConstraint *constraint = *(cpConstraint **)data;
    cpSpaceRemoveConstraint(space, constraint);
    cpConstraintFree(constraint);
}

Attachment::~Attachment()
{
    if (constraint) {
        cpSpace *s =cpConstraintGetSpace(constraint);
        if (s) {
            if (cpSpaceIsLocked(s)) {
                cpSpaceAddPostStepCallback(s, &deleteCallback, this, (void *)&constraint);
            } else {
                deleteCallback(s, (void *)this, (void *)&constraint);
            }
        }
    }
}

