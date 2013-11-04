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


typedef void (*spaceUpdateFunc)(cpSpace *space, long steps, cpFloat stepTime);

// example operators/generators
void randomGenerator1(MachineSystem *sys);
void randomGenerator2(MachineSystem *sys);
void randomGenerator3(MachineSystem *sys);

MachineSystem *attachmentMutator1(MachineSystem *sys); // change attachment between 2 machines
MachineSystem *attachmentMutator2(MachineSystem *sys); // change attachment to wall

MachineSystem *attachmentAnchorMutator(MachineSystem *sys); // change attachment position of wall attachment
MachineSystem *attachmentAnchorMutator2(MachineSystem *sys); // change attachment position between parts

//MachineSystem *inputMachineMutator(MachineSystem *sys);
//MachineSystem *outputMachineMutator(MachineSystem *sys);
//
//MachineSystem *addAttachmentMutator(MachineSystem *sys);
//MachineSystem *removeAttachmentMutator(MachineSystem *sys);
//
//MachineSystem *addPartMutator(MachineSystem *sys);
//MachineSystem *removePartMutator(MachineSystem *sys);

cpFloat normalize_angle(cpFloat angle); // convert to range (-2pi, +2pi)

#endif /* defined(__SystemGenerator__Type1Algorithm__) */
