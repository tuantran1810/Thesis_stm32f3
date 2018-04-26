#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "BAP_define.h"
#include "BAP_DiscreteFunction.h"

BAP_RESULT_E BAP_FuncInit(BAP_DiscreteFunction_S* func, int plusnum, int minusnum, float dT)
{
    if(func == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if(plusnum <= 0 || minusnum <= 0 || dT == 0)
    {
        return BAP_FAILED_WRONG_PAR;
    }

    memset(func, 0, sizeof(func));
    func->num_of_plus_samples = plusnum;
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

    *out = func->x;

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncSampleAppend(BAP_DiscreteFunction_S* func, float num)
{
    if(func == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if((func->num_of_minus_samples != 0) && (func->num_of_plus_samples != 0))
    {
        for(int i=func->num_of_minus_samples; i>1; i--)
        {
            func->x_minus[i] = func->x_minus[i - 1];
        }

        func->x_minus[1] = func->x;
        func->x = func->x_plus[1];

        for(int i = 1; i<func->num_of_plus_samples; i++)
        {
            func->x_plus[i] = func->x_plus[i + 1];
        }

        func->x_plus[func->num_of_plus_samples] = num;
    }
    else if((func->num_of_minus_samples != 0) && (func->num_of_plus_samples == 0))
    {
        for(int i=func->num_of_minus_samples; i>1; i--)
        {
            func->x_minus[i] = func->x_minus[i - 1];
        }
        func->x_minus[1] = func->x;
        func->x = num;
    }
    else if((func->num_of_minus_samples == 0) && (func->num_of_plus_samples != 0))
    {
        func->x = func->x_plus[1];
        for(int i = 1; i<func->num_of_plus_samples; i++)
        {
            func->x_plus[i] = func->x_plus[i + 1];
        }
        func->x_plus[func->num_of_plus_samples] = num;
    }
    else
    {
        func->x = num;
    }

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncDerivate(BAP_DiscreteFunction_S* func, int rate, float* out)
{
    if(rate > BAP_MAX_DFUNC_PLUS_SAMPLES || func->dT == 0)
    {
        return BAP_FAILED_WRONG_PAR;
    }

    if(func == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if(rate == 0)
    {
        *out = func->x;
    }
    else if(rate == 1)
    {
        *out = (func->x_plus[1] - func->x)/func->dT;
    }
    else if(rate == 2)
    {
        *out = (func->x_plus[2] - 2*func->x_plus[1] + func->x) / (func->dT * func->dT);
    }
    else if(rate == 3)
    {
        *out = (func->x_plus[3] - 3*func->x_plus[2] + 3*func->x_plus[1] - func->x) / (func->dT * func->dT * func->dT);
    }

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncGetSatOutput(BAP_SatFunction_S* sat, float input, float* out)
{
    float ret = 0;
    if(sat == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    ret = (input > sat->upper_limit)? sat->upper_limit:input;
    ret = (input < sat->lower_limit)? sat->lower_limit:input;
    *out = ret;

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_FuncGetSignOutput(float input, float* out)
{
    if(input < 0)
    {
        *out = -1;
    }
    else 
    {
        *out = 1;
    }
    return BAP_SUCCESS;
}
