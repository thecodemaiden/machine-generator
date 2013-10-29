//
//  AdeolaBadAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/7/13.
//
//

#include "AdeolaRotationAlgorithm.h"
#include <numeric>

AdeolaRotationAlgorithm::AdeolaRotationAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_m, float p_c)
:AdeolaAlgorithm(maxGenerations, maxStagnation, p_m, p_c)
{
    simSteps = 40;
    population.resize(populationSize);
    
    bestIndividual = NULL;
    
    allTimeBestFitness = -FLT_MAX;
    
    for (int i=0; i<populationSize; i++) {
        population[i] = new SystemInfo(simSteps);
        population[i]->system = createInitialSystem();
    }
}

 AdeolaRotationAlgorithm::~AdeolaRotationAlgorithm(){
     for (int i=0; i<population.size(); i++)
         delete population[i];
 }


static MachineSystem  *gearMutator(MachineSystem *sys)
{
    MachineSystem *newSystem = new MachineSystem(*sys);
    
    Attachment *chosenAttachment = NULL;
    cpVect part1 = cpvzero;
    cpVect part2 = cpvzero;
    
    newSystem->getRandomAttachment(&chosenAttachment, &part1, &part2);
    
    if (chosenAttachment) {
        AttachmentType t = chosenAttachment->attachmentType();
        if (t != ATTACH_GEAR) {
            // turn it into a gear!
            // don't change the length - for now
            GearAttachment *newAttachment = new GearAttachment(chosenAttachment->firstAttachPoint, chosenAttachment->secondAttachPoint, chosenAttachment->attachmentLength);
            
            newSystem->updateAttachmentBetween(part1, part2, newAttachment);
        } else  {
            // modify the gear!
            // we *add* a small factor between -1 and 1 to the gear ratio!
            // get a number between 0 and 2, then subtract 1
            cpFloat addition = (float)rand()/(RAND_MAX/2) - 1;
            cpFloat newRatio = addition + cpGearJointGetRatio(chosenAttachment->constraint);
            cpGearJointSetRatio(chosenAttachment->constraint, newRatio);
        }
    }
    return newSystem;
}

void AdeolaRotationAlgorithm::stepSystem(SystemInfo *individual)
{
    cpBody *inputBody = individual->system->partAtPosition(individual->system->inputMachinePosition)->body;
    cpBody *outputBody = individual->system->partAtPosition(individual->system->outputMachinePosition)->body;
    cpSpace *systemSpace = individual->system->getSpace();

    
    cpConstraint *motor = cpSimpleMotorNew(cpSpaceGetStaticBody(systemSpace), inputBody, M_PI);
    cpSpaceAddConstraint(systemSpace, motor);
    cpConstraintSetMaxForce(motor, 50000);

    
    for (int i=0; i<simSteps; i++) {
        individual->inputValues[i] = (cpBodyGetAngle(inputBody));
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputValues[i] = (cpBodyGetAngle(outputBody));
    }
    cpSpaceRemoveConstraint(systemSpace, motor);
    cpBodySetAngVel(outputBody, 0);
    cpBodySetAngle(inputBody, 0);
}


// (random) initializer
MachineSystem * AdeolaRotationAlgorithm::createInitialSystem()
{
    MachineSystem *sys = new MachineSystem(300, 300, 5, 5, cpvzero);
    randomGenerator2(sys);
    return sys;
}

// operators
MachineSystem *AdeolaRotationAlgorithm::mutateSystem(MachineSystem *original)
{
    // one or the other - twice the chance of gearMutator
    if (arc4random_uniform(3) == 0)
        return attachmentMutator2(original);
    else
        return gearMutator(original);
}

bool AdeolaRotationAlgorithm::tick()
{
    size_t populationSize = population.size();
    
    cpFloat bestFitness = -INFINITY; // we want zero fitness
    cpFloat worstFitness = INFINITY;
    
    cpFloat lastBestFitness = bestIndividual ? bestIndividual->fitness : bestFitness;
    
    size_t bestPos = -1;
    size_t worstPos = -1;
    
    for (size_t popIter = 0; popIter <populationSize; popIter++) {
        // possibly mutate each one
        cpFloat random = (cpFloat)rand()/RAND_MAX;
        SystemInfo *individual = population[popIter];
        SystemInfo *mutant = NULL;
        if ( fabs(random) < p_m) {
            MachineSystem *system = individual->system;
            mutant = new SystemInfo(simSteps);
            mutant->system = mutateSystem(system);
        }
        
        // reset the individual... by copying!
        MachineSystem *original = individual->system;
        MachineSystem *copy = new MachineSystem(*original);
        individual->system = copy;
        delete original;
        
        stepSystem(individual);
        individual->fitness = evaluateSystem(individual);
        
        if (mutant) {
            stepSystem(mutant);
            mutant->fitness = evaluateSystem(mutant);
            
            if (mutant->fitness > individual->fitness) {
                population[popIter] = mutant;
                delete individual;
                individual = mutant;
            } else {
                delete mutant;
            }
        }
        
        if (individual->fitness > bestFitness) {
            bestFitness = individual->fitness;
            bestIndividual = individual;
            bestPos = popIter;
        }
        
        if (individual -> fitness <= worstFitness) {
            worstFitness = individual->fitness;
            worstPos = popIter;
        }
    
        
    }
    
    if (bestFitness > allTimeBestFitness)
        allTimeBestFitness = bestFitness;
    
    if (bestPos != worstPos) {
        // replace the worst one with a random machine
        
        // fprintf(stderr, "Replacing system with fitness %f\n", worstFitness);
        SystemInfo *worst = population[worstPos];
        delete worst->system;
        worst->fitness = 0;
        worst->system = createInitialSystem();
    } else {
        fprintf(stderr, "Worst and best fitness are same - stop now?");
    }
    
    fprintf(stderr, "BEST FITNESS: %f (%lu)\n", bestFitness, bestPos);
    fprintf(stderr, "WORST FITNESS: %f (%lu)\n", worstFitness, worstPos);
    
    if (lastBestFitness == bestIndividual->fitness)
        stagnantGenerations++;
    else
        stagnantGenerations = 0;
    
    bool stop =  (generations >= maxGenerations) || goodEnoughFitness(bestFitness) || (stagnantGenerations >= maxStagnation);
    
    generations++;
    if (stop) {
        fprintf(stderr, "ALL TIME BEST FITNESS: %f\n", allTimeBestFitness);

    }
    return  stop;
}

//
// fitness evaluator
// --- SIMPLE AND DOESN'T WORK WELL ---
// find error in system
//cpFloat AdeolaRotationAlgorithm::evaluateSystem(SystemInfo *sys)
//{
//    cpFloat fitness = 0.0;
//    size_t nSteps = sys->inputValues.size();
//    
//    for (size_t i=0; i<nSteps; i++) {
//        float error = fabs(sys->inputValues[i] - sys->outputValues[i]);
//        if (error == 0) error = 1e-20; // we don't want an infinite fitness because one error was 0
//        fitness += 2*log(error); // these errors are very small at times
//    }
//    
//    return -fitness; // ideal log error -> -inf
//}

// ----- FANCY (RIDICULOUS) FITNESS FUNCTION BELOW -----
// ----- SOMETIMES PICKS EXACT OPPOSITE OF WHAT I WANT -----
// find correlation between input and ouput values
// if |correlation| -> 1, that's good (input is linearly related to output)
// I am looking for inputAngle = outputAngle, so I really want 1 (same sign)
// I also want the inputAngle to change a lot...

// let's try inputAngle:outputAngle = 2

cpFloat AdeolaRotationAlgorithm::evaluateSystem(SystemInfo *sys)
{
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputValues.size();
    
    cpFloat correlation = 0.0;
    
    cpFloat inputMean = std::accumulate(sys->inputValues.begin(), sys->inputValues.end(), 0.0)/nSteps;;
    cpFloat outputMean = std::accumulate(sys->outputValues.begin(), sys->outputValues.end(), 0.0)/nSteps;
    
    // correlation = sum[(x - mean(x))*(y - mean(y))] / sqrt(sum[(x - mean(x))^2] * sum[(y- mean(y))^2])
    
    cpFloat numerator = 0.0;
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    
    cpFloat meandInputdOutput = 0.0;
    
    for (int i=0; i<nSteps; i++) {
        float xDiff = (sys->inputValues[i]-inputMean);
        float yDiff = (sys->outputValues[i]-outputMean);
        numerator += xDiff*yDiff;
        sqrXDiffSum += xDiff*xDiff;
        sqrYDiffSum += yDiff*yDiff;
        
        if (i>0)
            meandInputdOutput += (sys->inputValues[i] - sys->inputValues[i-1])/(sys->outputValues[i]-sys->outputValues[i-1]);
    }
    correlation = numerator/sqrt(sqrXDiffSum*sqrYDiffSum);
//    
   cpFloat inputVariance = sqrXDiffSum/nSteps;
    cpFloat outputVariance = sqrYDiffSum/nSteps;
//    
    if (correlation != correlation || fabs(correlation) == INFINITY) {
        // the mean output value, and all output values, were zero -> correlation = NaN
        // otherwise the mean input value, and all input values, were zero -> correlation = +/-inf
        correlation = 0;
    }
    
    if (meandInputdOutput != meandInputdOutput || fabs(meandInputdOutput) == INFINITY) {
        meandInputdOutput = 0;
    } else {
        meandInputdOutput /= nSteps;
    }
    
        fitness = correlation;
    
    if (outputVariance > 0) {
        cpFloat distance = fabs (2.0 -meandInputdOutput);
        fitness /= (distance+0.1)*(distance+0.1);
    }

    
    return fitness*inputVariance*outputVariance; // 'ideal' fitness -> inf
}



bool AdeolaRotationAlgorithm::goodEnoughFitness(cpFloat bestFitness)
{
    return bestFitness > 300;
}

MachineSystem *AdeolaRotationAlgorithm::bestSystem()
{
    return bestIndividual->system;
}

 char* AdeolaRotationAlgorithm::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;

    snprintf(buffer, 100, "Input angle : %.3f", cpBodyGetAngle(inputBody));
    
    return buffer;
}

 char* AdeolaRotationAlgorithm::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *outputBody = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition)->body;
    
    snprintf(buffer, 100, "Output angle : %.3f", cpBodyGetAngle(outputBody));
    
    return buffer;
}
