//
//  Algorithm.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 10/3/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#include "Algorithm.h"
#include <math.h>
#include "HelperFunctions.h"


static MachineDescription *randomMachine(cpFloat size)
{
    MachineDescription *m = mgMachineNew();
    m->machineType = arc4random_uniform(MACHINE_TYPE_MAX);
    
    if (m->machineType == MACHINE_BOX) {
        cpFloat dim1 = size;
        cpFloat dim2 = arc4random_uniform(50) + 25;
        
        // will it be tall or wide?
        if (random0_1() > 0.5) {
            m->length = dim1;
            m->height = dim2;
        } else {
            m->length = dim2;
            m->height = dim1;
        }

    } else {
        m->length = size; // large circle == trouble!!
    }
    
       return m;
}

static Attachment *randomAttachment()
{
    Attachment *a = mgAttachmentNew();
    a->attachmentLength = 0;
    a->attachmentType = arc4random_uniform(MACHINE_ATTACH_MAX);

    return a;
}

mgMachineWallPopulator randomGenerator1(MachineWall *wall)
{
    int nMachinesToPlace = arc4random_uniform(3)+2; // 2-4 machines to start
    boolean_t outputChosen = false;
    boolean_t inputChosen = false;
    
    cpVect lastMachinePos=cpv(-1,-1);
    
    while (nMachinesToPlace) {
        int mx = arc4random_uniform(wall->size.x);
        int my = arc4random_uniform(wall->size.y);
        cpVect mPos = cpv(mx, my);
        if (!mgMachineWallGetMachineAtPosition(wall, mPos)) {
            Attachment *a = mgAttachmentNew();
            do {
                a->attachmentType = arc4random_uniform(MACHINE_ATTACH_MAX);
            }while (a->attachmentType == MACHINE_GEAR); //can't attach to wall with gear...
            a->attachmentLength = arc4random_uniform(20);
            
            mgMachineWallAddMachine(wall, randomMachine(wall->gridSpacing.x), a, mPos);
            
            // join to previous machine
            if (lastMachinePos.x >= 0 ) {
                Attachment *a = randomAttachment();
                a->attachmentLength = cpvlength(cpvlerp(mPos, lastMachinePos, 0.5))*cpvlength(wall->gridSpacing);
                mgMachineWallAttachMachines(wall, lastMachinePos, mPos, a);
            }
            
            // choose it as input or output (possibly both...)
            if (!outputChosen) {
                if (arc4random_uniform(nMachinesToPlace) == 0) {
                    outputChosen = true;
                    wall->outputMachinePosition = mPos;
                }
            }
            
            if (!inputChosen) {
                if (arc4random_uniform(nMachinesToPlace) == 0) {
                    inputChosen = true;
                    wall->inputMachinePosition = mPos;
                }
            }
            lastMachinePos = mPos;
            nMachinesToPlace--; // one less to place
        }
    }
    printf("Done generating");
    return NULL;
}
