//
//  NEATDisplacementToX.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/4/13.
//
//

#include "NEATDisplacementToX.h"
#include <numeric>

NEATDisplacementToX::NEATDisplacementToX(int populationSize, int maxGenerations, int maxStagnation, float p_c, float p_m_attach, float p_m_node, float p_m_conn)
:NEATAlgorithm(populationSize, maxGenerations, maxStagnation, p_c, p_m_node, p_m_conn),
p_m_attach(p_m_attach)
{ 
    MachineSystem *initialSystem = createInitialSystem();
    // mutate the initial system to get an initial population
    while (population.size() < populationSize) {
        MachineSystem *newSystem = new MachineSystem(*initialSystem);
        mutateSystem(newSystem);
        SystemInfo *info = new SystemInfo(simSteps);
        info->system = newSystem;
        population.push_back(info);
    }
}

void NEATDisplacementToX::stepSystem(SystemInfo *individual)
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
    cpBodySetAngVel(inputBody, 0);
    cpBodySetAngVel(outputBody, 0);
}

cpFloat NEATDisplacementToX::evaluateSystem(SystemInfo *sys)
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
    
    fitness = 1.0+correlation;
    
    if (outputVariance > 0) {
        cpFloat distance = fabs (2.0 -meandInputdOutput);
        fitness /= (distance+0.1)*(distance+0.1);
    }

    fitness = fitness*inputVariance*outputVariance;
    if (fitness == 0)
        fitness = 1e-8;
    
    return fitness;
}

bool NEATDisplacementToX::goodEnoughFitness(cpFloat bestFitness)
{
    return false;
}

void NEATDisplacementToX::mutateSystem(MachineSystem *original)
{
    // change an attachment to another type
    float selector = (cpFloat)rand()/RAND_MAX;
    if (selector < p_m_attach) {
        Attachment *old = NULL;
        cpVect p1;
        cpVect p2;
        original->getRandomAttachment(&old, &p1, &p2);
        if (old) {
            Attachment *newAt;
            if (selector < p_m_attach/2) {
                 newAt = changeAttachmentType(old);
            } else {
                newAt = perturbAttachmentAttributes(old);
            }
            original->updateAttachmentBetween(p1, p2, newAt);
        }
    }
    
    NEATAlgorithm::mutateSystem(original);
}

char* NEATDisplacementToX::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;
    
    snprintf(buffer, 100, "Input angle : %.3f", cpBodyGetAngle(inputBody));
    
    return buffer;
}

char* NEATDisplacementToX::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *outputBody = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition)->body;
    
    snprintf(buffer, 100, "Output angle : %.3f", cpBodyGetAngle(outputBody));
    
    return buffer;
}