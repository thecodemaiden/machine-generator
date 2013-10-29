//
//  MachineSystem.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/5/13.
//
//

#import "chipmunk.h"
#include "MachineSystem.h"
#include <iostream>
#include <fstream>
#include <string>

#define WALL_LAYER 1>>2
#define PEG_LAYER 1>>1

#pragma mark - Helper Functions


MachineSystem::MachineSystem(int width, int height, int hPegs, int vPegs, cpVect position)
: parts(hPegs*vPegs),
  attachments((hPegs*vPegs), std::vector<Attachment *>(hPegs*vPegs, NULL)),
  space(cpSpaceNew()),
  nMachines(0),
  nAttachments(0)
{
    cpSpaceSetIterations(space, 20);
    gridSpacing = cpv((float)width/(hPegs + 1), (float)height/(vPegs + 1));
    size = cpv(hPegs, vPegs);
    body = cpBodyNewStatic();
    cpBodySetPos(body, position);
    
    cpShape *wallShape = cpSegmentShapeNew(body, cpv(-width/2, height/2), cpv(width/2, height/2), .5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(-width/2, -height/2), cpv(+width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(-width/2, +height/2), cpv(-width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(+width/2, +height/2), cpv(+width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
}

MachineSystem::MachineSystem(MachineSystem &original, cpVect position)
: space(cpSpaceNew()),
 size(original.size),
gridSpacing(original.gridSpacing),
inputMachinePosition(original.inputMachinePosition),
outputMachinePosition(original.outputMachinePosition),
nMachines(0),
nAttachments(0)
{
    cpFloat width = (original.size.x +1)*original.gridSpacing.x;
    cpFloat height = (original.size.y +1)*original.gridSpacing.y;
   
    body = cpBodyNewStatic();
    cpBodySetPos(body, position);
    
    cpShape *wallShape = cpSegmentShapeNew(body, cpv(-width/2, height/2), cpv(width/2, height/2), .5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(-width/2, -height/2), cpv(+width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(-width/2, +height/2), cpv(-width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    wallShape = cpSegmentShapeNew(body, cpv(+width/2, +height/2), cpv(+width/2, -height/2), 0.5);
    cpShapeSetLayers(wallShape, WALL_LAYER);
    cpSpaceAddStaticShape(space, wallShape);
    
    // do the copy of the attachment and machines manually, because we need to add and attach machines
    int nPegs = size.x*size.y;

    parts.resize(nPegs, NULL);
    
    attachments.resize(nPegs);
    
    for (int i=0; i<nPegs; i++)
        attachments[i].resize(nPegs, NULL);
    
    for (int x = 0; x < size.x; x++ ) {
        for (int y = 0; y < size.y; y++) {
            cpVect gridPos = cpv(x,y);
            int machineNum = machinePositionToNumber(gridPos);
            MachinePart *machineToCopy = original.parts[machineNum];
            if (machineToCopy) {
                MachinePart *newPart = new MachinePart(*machineToCopy, space);
                Attachment *wallAttachment = Attachment::copyAttachment(original.attachments[machineNum][machineNum]);
                addPart(newPart, wallAttachment, gridPos);
            }
        }
    }
    
    for (int i=0; i<nPegs; i++) {
        for (int j=0; j<i; j++) {
            Attachment *attachmentToCopy = original.attachments[i][j];
            if (attachmentToCopy) {
                Attachment *attachmentCopy = Attachment::copyAttachment(attachmentToCopy);
                cpVect machine1Pos = machineNumberToPosition(i);
                cpVect machine2Pos = machineNumberToPosition(j);
                
                attachMachines(machine1Pos, machine2Pos, attachmentCopy);
            }
        }
    }
}

cpSpace * MachineSystem::getSpace()
{
    return space;
}

cpVect MachineSystem::getSize()
{
    return size;
}
cpVect MachineSystem::getSpacing()
{
    return gridSpacing;
}

// NOTE - don't use a GEAR attachment to attach to the wall - the peg does not rotate, so the body will not rotate
void MachineSystem::addPart(MachinePart *newPart, Attachment *attachment, cpVect gridPosition)
{
    int machineNumber = machinePositionToNumber(gridPosition);
    
    MachinePart *alreadyAttached = parts[machineNumber];
    if (!alreadyAttached) {
        cpVect pegPosition = gridPositionToWorld(gridPosition);
        cpBody *pegBody = cpBodyNewStatic();
        cpBodySetPos(pegBody, pegPosition);
        cpShape *pegShape = cpCircleShapeNew(pegBody, 5.0, cpv(0,0));
        cpShapeSetLayers(pegShape, PEG_LAYER);
        cpSpaceAddStaticShape(space, pegShape);
        
        parts[machineNumber] = newPart;
        nMachines++;
        newPart->setOriginalPosition(pegPosition);
        newPart->attachToBody(attachment, pegBody);
        attachments[machineNumber][machineNumber] = attachment;
    }
}

void MachineSystem::removePart(cpVect gridPosition)
{
    int machineNum = machinePositionToNumber(gridPosition);
    
    MachinePart *partToRemove = parts[machineNum];
    if (partToRemove) {
        
        cpBody *attachmentBody = partToRemove->getBody();
        __block cpBody *pegBody = NULL;
        cpBodyEachConstraint_b(attachmentBody, ^(cpConstraint *c) {
            if (cpConstraintGetB(c) == attachmentBody) {
                cpBody *otherBody = cpConstraintGetA(c); // the peg is always body A
                if (cpBodyGetMass(otherBody) == INFINITY)
                    pegBody = otherBody;
            }
        });
        
        assert(pegBody);
        
        int nPegs = size.x * size.y;
        // remove the attachments for this machine
        for (int i=0; i<nPegs; i++) {
            cpVect otherMachinePos = machineNumberToPosition(i);
            detachMachines(gridPosition, otherMachinePos);
        }
        
        
        partToRemove->detachFromBody(pegBody);
        
        cpBodyEachShape_b(pegBody, ^(cpShape *shape) {
            cpSpaceRemoveStaticShape(space, shape);
            cpShapeFree(shape);
        });
        
        cpBodyFree(pegBody);
        parts[machineNum] = NULL;
        
        // delete the attachment else we'll leak memory
        delete attachments[machineNum][machineNum];
        
        attachments[machineNum][machineNum] = NULL;
        
        nMachines--;
        // no longer allowing 'dangling' machines - everything is nailed to the wall for now
        partToRemove->removeFromSpace();
    }

}

MachinePart *MachineSystem::partAtPosition(cpVect gridPosition)
{
    int machineNum = machinePositionToNumber(gridPosition);
    return parts[machineNum];
}

Attachment *MachineSystem::attachmentBetween(cpVect machine1Pos, cpVect machine2Pos)
{
    int machine1Num = machinePositionToNumber(machine1Pos);
    int machine2Num = machinePositionToNumber(machine2Pos);
    
    return attachments[machine1Num][machine2Num];
}

Attachment *MachineSystem::attachmentToWall(cpVect gridPosition)
{
    int machineNum = machinePositionToNumber(gridPosition);
    return attachments[machineNum][machineNum];
}


bool MachineSystem::attachMachines(cpVect machine1Pos, cpVect machine2Pos, Attachment *attachment)
{
    bool added = false;
    if (!cpveql(machine1Pos, machine2Pos)) {
        MachinePart *machine1 = partAtPosition(machine1Pos);
        MachinePart *machine2 = partAtPosition(machine2Pos);
        if (machine1 && machine2) {
            int machine1Num = machinePositionToNumber(machine1Pos);
            int machine2Num = machinePositionToNumber(machine2Pos);
            Attachment *existingAttachment = attachments[machine1Num][machine2Num];
            if (!existingAttachment) {
                machine1->attachToBody(attachment, machine2->body);
                attachments[machine1Num][machine2Num] = attachments[machine2Num][machine1Num] = attachment;
                added = true;
                nAttachments++;
            }
        }
    }
    return added;

}

bool MachineSystem::detachMachines(cpVect machine1Pos, cpVect machine2Pos)
{
    bool detached = false;
    if (!cpveql(machine1Pos, machine2Pos)) {
        MachinePart *machine1 = partAtPosition(machine1Pos);
        MachinePart *machine2 = partAtPosition(machine2Pos);
        if (machine1 && machine2) {
            int machine1Num = machinePositionToNumber(machine1Pos);
            int machine2Num = machinePositionToNumber(machine2Pos);
            Attachment *existingAttachment = attachments[machine1Num][machine2Num];
            if (existingAttachment) {
                machine1->detachFromBody(machine2->body);
                machine2->detachFromBody(machine1->body);
               
                
                delete attachments[machine1Num][machine2Num];
                
                attachments[machine1Num][machine2Num] = attachments[machine2Num][machine1Num] = NULL;
                detached = true;
                nAttachments--;
            }
        }
    }
    return detached;
}


void MachineSystem::updateAttachmentBetween(cpVect machine1Pos, cpVect machine2Pos, Attachment *newAttachment)
{
    if (!cpveql(machine1Pos, machine2Pos)) {
        MachinePart *machine1 = partAtPosition(machine1Pos);
        MachinePart *machine2 = partAtPosition(machine2Pos);
        if (machine1 && machine2) {
            int machine1Num = machinePositionToNumber(machine1Pos);
            int machine2Num = machinePositionToNumber(machine2Pos);
            Attachment *existingAttachment = attachments[machine1Num][machine2Num];
            if (existingAttachment) {
                detachMachines(machine1Pos, machine2Pos);
                attachMachines(machine1Pos, machine2Pos, newAttachment);
            }
        }
    }
}

void MachineSystem::updateAttachmentToWall(cpVect gridPosition, Attachment *newAttachment)
{
    
    int machineNum = machinePositionToNumber(gridPosition);
    
    MachinePart *partToUpdate = parts[machineNum];
    if (partToUpdate) {
        Attachment *existingAttachment = attachments[machineNum][machineNum];
        
        cpBody *attachmentBody = partToUpdate->getBody();
        __block cpBody *pegBody = NULL;
        cpBodyEachConstraint_b(attachmentBody, ^(cpConstraint *c) {
            if (cpConstraintGetB(c) == attachmentBody) {
                cpBody *otherBody = cpConstraintGetA(c); // the peg is always body A
                if (cpBodyGetMass(otherBody) == INFINITY)
                    pegBody = otherBody;
            }
        });
        
        assert(pegBody);
        
        partToUpdate->detachFromBody(pegBody);
        
        delete existingAttachment;
        
        partToUpdate->attachToBody(newAttachment, pegBody);
        attachments[machineNum][machineNum] = newAttachment;
    }
}

MachineSystem::~MachineSystem()
{
    //detach the machinesss
    for (int x = 0; x < size.x; x++ ) {
        for (int y = 0; y < size.y; y++) {
            cpVect gridPos = cpv(x,y);
            MachinePart *part = partAtPosition(gridPos);
            if (part) {
                removePart(gridPos);
                delete part;
            }
        }
        
    }
    
    int nPegs = size.x * size.y;
    
    for (int i=0; i<nPegs; i++) {
        for (int j=0; j<nPegs; j++) {
            // CHECK
            // may not be necessary - could have removed attachments when removing parts
            // and only wall attachments remain to be removed
            delete attachments[i][j];
        }
    }
    
    cpBodyEachShape_b(body, ^(cpShape *shape) {
        cpSpaceRemoveStaticShape(cpShapeGetSpace(shape), shape);
        cpShapeFree(shape);
    });
    
    cpBodyFree(body);
    
    cpSpaceFree(space);
}


#pragma mark - random pickers

void MachineSystem::getRandomAttachment(Attachment **attachment, cpVect *pos1, cpVect *pos2)
{
    // attachment matrix is nxn, where n is number of machines.
    // we can search above or below the diagonal to get an attachment
    // I chose below, so for machine 0 we search attachments to 1..n-1
    // for machine 1 we search attachments to 2..n-1
    // for machine n-1 we don't have to search
    
    if (!pos1 || !pos2) {
        // I'm not doing work if there's nowhere to put the result
        return;
    }
    
    cpVect p1 = cpv(-1,-1);
    cpVect p2 = cpv(-1,-1);
    Attachment *found = NULL;
    
    // we pick an existing attachment with uniform probability
    int attachmentToPick = arc4random_uniform(nAttachments);
    
    int possibleParts = size.x*size.y;
    int foundAttachments = 0;
    
    for (int i=0; i<possibleParts; i++) {
        for (int j=i+1; j<possibleParts; j++) {
            if (attachments[i][j]) {
                if (foundAttachments == attachmentToPick) {
                    p1 = machineNumberToPosition(i);
                    p2 = machineNumberToPosition(j);
                    found = attachments[i][j];
                    break;
                } else {
                    foundAttachments++;
                }
            }
        }
        if (found)
            break;
    }
    *pos1 = p1;
    *pos2 = p2;
    if (attachment)
        *attachment = found;
    
}

void MachineSystem::getRandomPartPosition(cpVect *partPosition)
{
    
    if (!partPosition) {
        // don't do work if there's nowhere to put the result
        return;
    }
    
    cpVect pos = cpv(-1,-1);
    // we pick an existing machine with uniform probability
    int partToPick = arc4random_uniform(nMachines);
    int possibleParts = size.x*size.y;
    
    int foundParts = 0;
    for (int i=0; i<possibleParts; i++) {
        if (parts[i]) {
            if (partToPick == foundParts) {
                // we found the nth existing machine
                pos = machineNumberToPosition(i);
                break;
            } else {
                foundParts++;
            }
                
        }
    }
    *partPosition = pos;
}

#pragma mark - more helpers
// to help us find edges in the adjacency matrix, we must convert (gx, gy) to an int and vice versa
int MachineSystem::machinePositionToNumber(cpVect gridPosition)
{
    int x = gridPosition.x;
    int y = gridPosition.y;
    return x + y*(int)this->size.y;
}

cpVect MachineSystem::machineNumberToPosition(int machineNumber)
{
    return cpv((machineNumber % (int)this->size.y), (machineNumber / (int)this->size.y));
}

// find the actual location of the peg for this attachment
cpVect MachineSystem::gridPositionToWorld(cpVect gridPosition) {
    int x = gridPosition.x;
    int y = gridPosition.y;
    
    cpFloat wallWidth = (this->size.x+1)*this->gridSpacing.x;
    cpFloat wallHeight = (this->size.y+1)*this->gridSpacing.y;
    
    cpVect pegPosition = cpv(this->gridSpacing.x*(x+1), this->gridSpacing.y*(y+1));
    pegPosition = cpvadd(pegPosition, cpv(-wallWidth/2, -wallHeight/2));
    pegPosition = cpvadd(pegPosition, cpBodyGetPos(this->body));
    
    return pegPosition;
}

#pragma mark - save and load



void outputAttachment(std::ofstream & o, Attachment *a, int part1, int part2)
{
    o << part1 << "\t" << part2 << "\t" << a->attachmentType() << "\t" << a->firstAttachPoint.x << "\t";
    o << a->firstAttachPoint.y << "\t" << a->secondAttachPoint.x << "\t" << a->secondAttachPoint.y << "\t";
    o << a->attachmentLength;
    switch (a->attachmentType()) {
        case ATTACH_SPRING:
            o << "\t" << ((SpringAttachment *)a)->damping << "\t" << ((SpringAttachment *)a)->stiffness <<"\n";
            break;
        case ATTACH_SLIDE:
            o  << "\t" << ((SlideAttachment *)a)->maxDistance << "\t" << ((SlideAttachment*)a)->minDistance << "\n";
            break;
        case ATTACH_PIVOT:
            o << "\t" << ((PivotAttachment *)a)->pivotPosition << "\n";
            break;
        case ATTACH_GEAR:
            o << "\t" << ((GearAttachment *)a)->gearRatio << "\t" << ((GearAttachment *)a)->phase << "\n";
            break;
        default:
            o << "\n";
            break;
    }
}

void outputMachine(std::ofstream & outputStream, MachinePart *p, int machinePos)
{
    outputStream << machinePos << "\t" << p->machineType << "\t" << p->height << "\t" << p->length << "\n";
}

void MachineSystem::saveToDisk(const char* filename)
{
    /* 1. grid size: x \t y \n
     2. n (number of defined parts)
     3. list of machines of format:
        // number \t body type \t height \t width
        // attachment (output format below)
     4. n (number of defined attachments)
     5. list of (other) attachments of format:
         machine1 \t machine2 \t attachment type \t attach point 1 x \t attach point 1 y \t attach point 2 x \t attach point 2 y \t
            attachment length [\t optional stuff]
     
         where optional stuff depends on type:
            Gear - ratio \t phase
            Slide - max distance \t min distance
            Pivot - pivot point
            Spring - damping \t stiffness
            Fixed - nothing
     6. Input and output machine position: input.x \t input.y \t output.x \t output.y \t
    */
    
    std::ofstream outputFile;
    outputFile.open(filename);
    
    //first the grid size
    outputFile << size.x << "\t" << size.y << "\n";
    outputFile << nMachines << "\n";
    for (int i=0; i<parts.size(); i++) {
        MachinePart *p = parts[i];
        if (p) {
            outputMachine(outputFile, p, i);
            outputAttachment(outputFile, attachments[i][i], i, i);
        }
    }
    outputFile << nAttachments << "\n";
    for (int i=0; i<attachments.size(); i++) {
        std::vector<Attachment *> attachmentList= attachments[i];
        for (int j=i+1; j<attachmentList.size(); j++) {
            // only really have to cover a triangle of the attachments
            Attachment *a = attachmentList[j];
            if (a) {
                outputAttachment(outputFile, a, i, j);
            }
        }
    }
    
    outputFile << inputMachinePosition.x <<"\t" << inputMachinePosition.y << "\t" << outputMachinePosition.x << "\t" << outputMachinePosition.y << "\n";
    
    outputFile.close();
}

static std::vector<std::string> splitString(std::string s, char delim) {
    std::vector<std::string> parts;

    size_t startPos = 0;
    size_t nextPos = 0;
    do {
        nextPos = s.find(delim, startPos);
        parts.push_back(s.substr(startPos, nextPos));
        startPos = nextPos+1;
    }while(nextPos != s.npos);
    
    return parts;
}

MachineSystem * MachineSystem::loadFromDisk(const char* filename, cpFloat wallWidth, cpFloat wallHeight, cpVect position)
{
    MachineSystem *sys = NULL;
    
    std::ifstream inputFile;
    inputFile.open(filename);
    
    if (!inputFile.is_open())
        return sys;
    
    // not currently checking for other errorrrssss
    int hPegs =0;
    int vPegs =0;
    
    std::vector<std::string> lineParts;
    std::string line;
    
    std::getline(inputFile, line);
    lineParts = splitString(line, '\t');
    if (lineParts.size() == 2) {
        hPegs = atoi(lineParts[0].c_str());
        vPegs = atoi(lineParts[1].c_str());
    }
    
    sys = new MachineSystem(wallWidth, wallHeight, hPegs, vPegs, position);
    cpSpace *s = sys->getSpace();
    
    // read in the machine parts and their wall attachments
    std::getline(inputFile, line);
    lineParts = splitString(line, '\t');
    
    int nParts = atoi(lineParts[0].c_str());
    for (int i=0; i<nParts; i++) {
        // read in the parts and their wall attachments
        std::getline(inputFile, line);
        lineParts = splitString(line, '\t');
        int machineNum = atoi(lineParts[0].c_str());
        
        MachinePart *p = new MachinePart((BodyType)atoi(lineParts[1].c_str()), cpvzero, atof(lineParts[3].c_str()), atof(lineParts[2].c_str()), s);
        
        // read attachmentInfo
        std::getline(inputFile, line);
        lineParts = splitString(line, '\t');
        // ignore the machine numbers, should both be this machine
        Attachment *a = NULL;
        AttachmentType t = (AttachmentType)atoi(lineParts[2].c_str());
        cpVect firstAttachPoint = cpv(atof(lineParts[3].c_str()), atof(lineParts[4].c_str()));
        cpVect secondAttachPoint = cpv(atof(lineParts[5].c_str()), atof(lineParts[6].c_str()));
        cpFloat attachmentLength = atof(lineParts[7].c_str());
        switch (t) {
            case ATTACH_FIXED:
                a = new FixedAttachment(firstAttachPoint, secondAttachPoint, attachmentLength);
                break;
            case ATTACH_GEAR:
                a = new GearAttachment(firstAttachPoint, secondAttachPoint, attachmentLength, atof(lineParts[8].c_str()), atof(lineParts[9].c_str()));
                break;
            case ATTACH_PIVOT:
                a = new PivotAttachment(firstAttachPoint, secondAttachPoint, attachmentLength, atof(lineParts[8].c_str()));
                break;
            case ATTACH_SLIDE:
                a = new SlideAttachment(firstAttachPoint, secondAttachPoint, attachmentLength, atof(lineParts[8].c_str()), atof(lineParts[9].c_str()));
                break;
            case ATTACH_SPRING:
                a = new SpringAttachment(firstAttachPoint, secondAttachPoint, attachmentLength, atof(lineParts[8].c_str()), atof(lineParts[9].c_str()));
                break;
            default:
                break;
        }
        sys->addPart(p, a, sys->machineNumberToPosition(machineNum));
    }
    
    // now read in all the other attachments
    std::getline(inputFile, line);
    lineParts = splitString(line, '\t');
    
    int nAttachments = atoi(lineParts[0].c_str());
    for (int i=0; i<nAttachments; i++) {
        // read in the attachment info
        std::getline(inputFile, line);
        lineParts = splitString(line, '\t');
        
        Attachment *a = NULL;
        AttachmentType t = (AttachmentType)atoi(lineParts[2].c_str());
        
        int part1 = atoi(lineParts[0].c_str());
        int part2 = atoi(lineParts[1].c_str());
        
        cpVect firstAttachPoint = cpv(atof(lineParts[3].c_str()), atof(lineParts[4].c_str()));
        cpVect secondAttachPoint = cpv(atof(lineParts[5].c_str()), atof(lineParts[6].c_str()));
        cpFloat attachmentLength = atof(lineParts[7].c_str());
        switch (t) {
            case ATTACH_FIXED:
                a = new FixedAttachment(firstAttachPoint, secondAttachPoint, attachmentLength);
                break;
            case ATTACH_GEAR:
                a = new GearAttachment(firstAttachPoint, secondAttachPoint, attachmentLength, atof(lineParts[8].c_str()), atof(lineParts[9].c_str()));
                break;
            case ATTACH_PIVOT:
                a = new PivotAttachment(firstAttachPoint, secondAttachPoint, attachmentLength, atof(lineParts[8].c_str()));
                break;
            case ATTACH_SLIDE:
                a = new SlideAttachment(firstAttachPoint, secondAttachPoint, attachmentLength, atof(lineParts[8].c_str()), atof(lineParts[9].c_str()));
                break;
            case ATTACH_SPRING:
                a = new SpringAttachment(firstAttachPoint, secondAttachPoint, attachmentLength, atof(lineParts[8].c_str()), atof(lineParts[9].c_str()));
                break;
            default:
                break;
        }

        sys->attachMachines(sys->machineNumberToPosition(part1), sys->machineNumberToPosition(part2), a);
    }
    
    // finally, get input and output machine positions
    std::getline(inputFile, line);
    lineParts = splitString(line, '\t');
    sys->inputMachinePosition = cpv(atoi(lineParts[0].c_str()), atoi(lineParts[1].c_str()));
    sys->outputMachinePosition = cpv(atoi(lineParts[2].c_str()), atoi(lineParts[3].c_str()));
    
    inputFile.close();
    return sys;
}
