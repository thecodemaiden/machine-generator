//
//  Algorithm1.h
//  MachineGenerator
//
//  Created by Adeola Bannis on 10/3/13.
//  Copyright (c) 2013 Adeola Bannis. All rights reserved.
//

#import "MachineWall.h"

typedef void (*mgMachineWallIterator)(MachineWall *wall, int position);
typedef void (*mgMachineWallPopulator)(MachineWall *wallToFill);

typedef struct  {
    MachineWall **population;
    mgMachineWallIterator mutationOperator;
    mgMachineWallIterator recombinationOperator;
    mgMachineWallPopulator generator;
} Type1Algorithm;


mgMachineWallPopulator randomGenerator1(MachineWall *wall);