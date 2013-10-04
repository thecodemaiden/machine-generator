//
//  HelperFunctions.c
//  MachineGenerator
//
//  Created by Adeola Bannis on 10/4/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "HelperFunctions.h"
#include <stdlib.h>

float random0_1()
{
    float r = ((float)rand()/(float)RAND_MAX);
    return r;
}
