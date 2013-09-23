//
//  Machine.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 9/19/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#include <Machine.h>


cpBody *bodyBuildingHelper(Attachment *at, cpBody *parentBody, cpVect parentOffset, cpSpace *s)
{
    // the location of the parent shape in space has to be added to the attachPoint

    cpBody *machineBody = NULL;
    cpShape *machineShape = NULL;
    MachineDescription *md = at->machine;
    
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
        
        machineBody = cpBodyNew(bodyMass, bodyMoment);
        cpSpaceAddBody(s, machineBody);
    
    cpVect truePos = cpvadd(at->parentAttachPoint, parentOffset);
    
    if (md->machineType == MACHINE_WHEEL) {
        machineShape = cpCircleShapeNew(machineBody, md->length, truePos);
    } else if (md->machineType == MACHINE_BOX) {
        cpVect boxVerts[4] = {cpv(-md->length/2.0, -md->height/2.0),
                              cpv(-md->length/2.0, md->height/2.0),
                              cpv(md->length/2.0, md->height/2.0),
                              cpv(md->length/2.0, -md->height/2.0)};
        machineShape = cpPolyShapeNew(machineBody, 4, boxVerts, truePos);
    }
    cpSpaceAddShape(s, machineShape);
    
    // do attachments later
    
    
    return machineBody;
}

cpBody *bodyFromDescription(MachineDescription *md, cpSpace *space)
{
    cpBody *machineBody = NULL;
    
    Attachment baseAttachment;
    baseAttachment.parentAttachPoint = cpv(0, 0);
    baseAttachment.attachPoint = cpv(0,0);
    baseAttachment.attachmentType = MACHINE_BASE;
    baseAttachment.machine = md;
    
    machineBody = bodyBuildingHelper(&baseAttachment, NULL, cpv(0, 0), space);
    
    return machineBody;
}