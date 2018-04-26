#ifndef BAP_DISCRETEFUNCTION_H
#define BAP_DISCRETEFUNCTION_H

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "BAP_define.h"

#define BAP_MAX_DFUNC_MINUS_SAMPLES 3
#define BAP_MAX_DFUNC_PLUS_SAMPLES  3

typedef struct BAP_DiscreteFunction_S
{
    int num_of_plus_samples;    //Max is 3, min is 1
    int num_of_minus_samples;   //Max is 3, min is 1
    float dT;
    float x;
    float x_minus[BAP_MAX_DFUNC_MINUS_SAMPLES + 1]; //offset 1
    float x_plus[BAP_MAX_DFUNC_PLUS_SAMPLES + 1];   //offset 1
}BAP_DiscreteFunction_S;

typedef struct BAP_SatFunction_S
{
    float upper_limit;
    float lower_limit;
}BAP_SatFunction_S;

BAP_RESULT_E BAP_FuncInit(BAP_DiscreteFunction_S* func, int plusnum, int minusnum, float dT);
BAP_RESULT_E BAP_FuncGetRecentSample(BAP_DiscreteFunction_S* func, float* out);
BAP_RESULT_E BAP_FuncDerivate(BAP_DiscreteFunction_S* func, int rate, float* out);
BAP_RESULT_E BAP_FuncSampleAppend(BAP_DiscreteFunction_S* func, float num);
BAP_RESULT_E BAP_FuncGetSatOutput(BAP_SatFunction_S* sat, float input, float* out);
BAP_RESULT_E BAP_FuncGetSignOutput(float input, float* out);

#endif