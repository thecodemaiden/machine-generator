//
//  Type1Algorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/6/13.
//
//

#ifndef __SystemGenerator__Type1Algorithm__
#define __SystemGenerator__Type1Algorithm__

#include "MachineSystem.h"
#include <vector>


typedef void (*spaceUpdateFunc)(cpSpace *space, long steps, cpFloat stepTime);

// example operators/generators
void randomGenerator1(MachineSystem *sys);
MachineSystem *attachmentMutator1(MachineSystem *sys);



#endif /* defined(__SystemGenerator__Type1Algorithm__) */
