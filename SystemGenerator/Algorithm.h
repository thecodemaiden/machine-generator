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


struct SystemInfo
{
    // a machine system and fitness info
    MachineSystem *system;
    double fitness;
    std::vector<cpFloat> outputValues;
    std::vector<cpFloat> inputValues;
    
    // we need to specify how many values this will hold for examination
    SystemInfo(int steps): outputValues(steps), fitness(0.0),
    inputValues(steps){}
    
    ~SystemInfo() {
        delete system;
    }
};

struct ExtendedSystemInfo
{
    // a machine system and fitness info
    MachineSystem *system;
    double fitness;
    std::vector<cpFloat> outputXDisplacement;
    std::vector<cpFloat> inputXDisplacement;
    
    std::vector<cpFloat> outputYDisplacement;
    std::vector<cpFloat> inputYDisplacement;
    
    std::vector<cpFloat> outputRotationAngle;
    std::vector<cpFloat> inputRotationAngle;
    
    // we need to specify how many values this will hold for examination
    ExtendedSystemInfo(int steps)
    : outputXDisplacement(steps),
      inputXDisplacement(steps),
      outputYDisplacement(steps),
      inputYDisplacement(steps),
      outputRotationAngle(steps),
      inputRotationAngle(steps),
      fitness(0.0) {}
    
    ~ExtendedSystemInfo() {
        delete system;
    }
};

typedef void (*spaceUpdateFunc)(cpSpace *space, long steps, cpFloat stepTime);

// make random parts of a certain size
MachinePart *randomPart(cpSpace *space, cpVect size);

// example operators/generators
void randomGenerator1(MachineSystem *sys);
void randomGenerator2(MachineSystem *sys);
void randomGenerator3(MachineSystem *sys);


void neatGenerator(MachineSystem *sys); // generates a minimal system of two machines - one input and one ouput


// ---- Old mutators, not really for use with NEAT -----
// Adding some basic ones to the MachineSystem

MachineSystem *attachmentMutator1(MachineSystem *sys); // change attachment between 2 machines
MachineSystem *attachmentMutator2(MachineSystem *sys); // change attachment to wall

MachineSystem *attachmentAnchorMutator(MachineSystem *sys); // change attachment position of wall attachment
MachineSystem *attachmentAnchorMutator2(MachineSystem *sys); // change attachment position between parts

// -- New mutators --
Attachment *changeAttachmentType(Attachment *at);
Attachment *perturbAttachmentAttributes(Attachment *at);
Attachment *changeAttachmentAnchorPoints(Attachment *at);
//void changeMachineShape(MachinePart *part);


cpFloat normalize_angle(cpFloat angle); // convert to range (-2pi, +2pi)

#endif /* defined(__SystemGenerator__Type1Algorithm__) */
