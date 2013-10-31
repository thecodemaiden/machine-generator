//
//  AdeolaConstantToSinusoidalAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/31/13.
//
//

#include "AdeolaConstantToSinusoidalAlgorithm.h"
#include <numeric>


MachineSystem *AdeolaConstantToSinusoidalAlgorithm::mutateSystem(MachineSystem *original)
{
    return  attachmentAnchorMutator(original);
}

// ----- FANCY (RIDICULOUS) FITNESS FUNCTION BELOW -----
// ----- SOMETIMES PICKS EXACT OPPOSITE OF WHAT I WANT -----
// find correlation between input and ouput values
// if |correlation| -> 1, that's good (input is linearly related to output)
// I am looking for inputAngle = outputAngle, so I really want 1 (same sign)
// I also want the inputAngle to change a lot...

// let's try inputAngle:outputAngle = 2

cpFloat AdeolaConstantToSinusoidalAlgorithm::evaluateSystem(SystemInfo *sys)
{
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputValues.size();
    
    cpFloat correlation = 0.0;
 
    cpFloat outputMean = std::accumulate(sys->inputValues.begin(), sys->inputValues.end(), 0.0)/nSteps;;
    
    // transform the input to sin(input) and find the correlation with that
    std::transform(sys->inputValues.begin(), sys->inputValues.end(), sys->inputValues.begin(), sin);
    cpFloat inputMean = std::accumulate(sys->inputValues.begin(), sys->inputValues.end(), 0.0)/nSteps;;
    
    // correlation = sum[(x - mean(x))*(y - mean(y))] / sqrt(sum[(x - mean(x))^2] * sum[(y- mean(y))^2])
    
    cpFloat numerator = 0.0;
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    
    for (int i=0; i<nSteps; i++) {
        float xDiff = (sys->inputValues[i]-inputMean);
        float yDiff = (sys->outputValues[i]-outputMean);
        numerator += xDiff*yDiff;
        sqrXDiffSum += xDiff*xDiff;
        sqrYDiffSum += yDiff*yDiff;
        
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
    
    fitness = correlation;
    
    
    return fitness*inputVariance*outputVariance; // 'ideal' fitness -> inf
}



bool AdeolaConstantToSinusoidalAlgorithm::goodEnoughFitness(cpFloat bestFitness)
{
    return bestFitness > 300;
}