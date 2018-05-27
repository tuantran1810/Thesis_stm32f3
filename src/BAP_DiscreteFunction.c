#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "BAP_define.h"
#include "BAP_DiscreteFunction.h"

BAP_RESULT_E BAP_FuncInit(BAP_DiscreteFunction_S* func, int minusnum, float dT)
{
    if(func == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if(minusnum <= 0 || dT == 0)
    {
        return BAP_FAILED_WRONG_PAR;
    }

    memset(func, 0, sizeof(BAP_DiscreteFunction_S));
    func->num_of_minus_samples = minusnum;
    func->dT = dT;
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncGetRecentSample(BAP_DiscreteFunction_S* func, float* out)
{
    if(func == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    *out = func->x[0];

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncSampleAppend(BAP_DiscreteFunction_S* func, float num)
{
    if(func == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    memcpy(&(func->x[1]), &(func->x[0]), BAP_MAX_DFUNC_MINUS_SAMPLES*sizeof(float));
    func->x[0] = num;

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncDerivate(BAP_DiscreteFunction_S* func, int rate, float* out)
{
    if(rate > func->num_of_minus_samples || func->dT == 0)
    {
        return BAP_FAILED_WRONG_PAR;
    }

    if(func == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if(rate == 0)
    {
        *out = func->x[0];
    }
    else if(rate == 1)
    {
        *out = (func->x[0] - func->x[1])/func->dT;
    }
    else if(rate == 2)
    {
        *out = (func->x[0] - 2*func->x[1] + func->x[2]) / (func->dT * func->dT);
    }
    else if(rate == 3)
    {
        *out = (func->x[0] - 3*func->x[1] + 3*func->x[2] - func->x[3]) / (func->dT * func->dT * func->dT);
    }

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncGetSatOutput(BAP_SatFunction_S* sat, float input, float* out)
{
    if(sat == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if(input < sat->lower_limit)
    {
        *out = sat->lower_limit;
        return BAP_SUCCESS;
    }
    else if(input > sat->upper_limit)
    {
        *out = sat->upper_limit;
        return BAP_SUCCESS;
    }
    *out = input;

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncGetSignOutput(float input, float* out)
{
    *out = (input < 0) ? -1:1;
    return BAP_SUCCESS;
}
