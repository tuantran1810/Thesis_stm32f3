#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "BAP_define.h"
#include "BAP_DiscreteFunction.h"
#include "BAP_SMC.h"

BAP_RESULT_E BAP_SMCSliddingSurfaceCal(BAP_SMC_S* smc, float* out);

BAP_RESULT_E BAP_SMCSecondOrderLFInit(BAP_SecondOrderLF_S* lf, float dT, float a)
{
    if(lf == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if(dT == 0 || a == 0)
    {
        return BAP_FAILED_WRONG_PAR;
    }

    memset(lf, 0, sizeof(BAP_SecondOrderLF_S));

    lf->input.num_of_minus_samples = 2;
    lf->input.dT = dT;

    lf->output.num_of_minus_samples = 2;    
    lf->output.dT = dT;

    lf->a = a;
    lf->T = dT;
    lf->exp_aT = (float)exp(-(a*dT));

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCSecondOrderLFAppendInput(BAP_SecondOrderLF_S* lf, float input)
{
    BAP_FuncSampleAppend(&(lf->input), input);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCSecondOrderLFGetOutput(BAP_SecondOrderLF_S* lf, float* out)
{
    if(lf == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    float ret = 0;

    ret =   ( 2*lf->exp_aT*lf->output.x[0]
            - lf->exp_aT*lf->exp_aT*lf->output.x[1]
            + lf->output.dT*lf->exp_aT*lf->input.x[1] );

    BAP_FuncSampleAppend(&(lf->output), ret);

    *out = (ret*lf->a*lf->a*lf->T);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCInit(BAP_SMC_S* smc, int order, float dT, float sat_upper, float sat_lower, float k1, float K, float yd, float scale)
{
    if(smc == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if(order > BAP_MAX_DFUNC_MINUS_SAMPLES || dT == 0 || sat_upper <= sat_lower || k1 == 0 || K == 0)
    {
        return BAP_FAILED_WRONG_PAR;
    }

    memset(smc, 0, sizeof(BAP_SMC_S));

    smc->yd.num_of_minus_samples = order;
    smc->yd.dT = dT;

    smc->e.num_of_minus_samples = order - 1;
    smc->e.dT = dT;

    smc->sat.upper_limit = sat_upper;
    smc->sat.lower_limit = sat_lower;

    smc->k1 = k1;
    smc->K = K;
    smc->yd_in = yd;
    smc->scale = scale;

    smc->with_limit = 0;

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCSliddingSurfaceCal(BAP_SMC_S* smc, float* out) //Slidding surface: sigma = e_dot + k1*e
{
    float e_dot = 0;
    float e = 0;

    if(smc == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    BAP_FuncGetRecentSample(&(smc->e), &e);
    BAP_FuncDerivate(&(smc->e), 1, &e_dot);
    *out = e_dot + smc->k1*e;

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCOuputCal(BAP_SMC_S* smc, float* out)
{
    float e_dot = 0;
    float yd_dot_dot = 0;
    float sigma = 0;
    float sign_sat_value = 0;
    float result = 0;
    float output = 0;

    if(smc == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    BAP_FuncDerivate(&(smc->e), 1, &e_dot);
    BAP_FuncDerivate(&(smc->yd), 2, &yd_dot_dot);
    BAP_SMCSliddingSurfaceCal(smc, &sigma);
    BAP_FuncGetSatOutput(&(smc->sat), sigma, &sign_sat_value);

    result = BAP_SMC_FORMULA_CONST_D*(yd_dot_dot + smc->k1*e_dot + smc->K*sign_sat_value)*smc->scale;
    if(smc->with_limit == 1)
    {
        BAP_FuncGetSatOutput(&(smc->output_limit), result, &output);
    }
    *out = output;
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCChangeSetPoint(BAP_SMC_S* smc, float yd)
{
    if(smc == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    smc->yd_in = yd;
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCUpdateParam(BAP_SMC_S* smc, float y)
{
    if(smc == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    BAP_FuncSampleAppend(&(smc->yd), smc->yd_in);
    BAP_FuncSampleAppend(&(smc->e), smc->yd_in - y);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCOutputLimitEnable(BAP_SMC_S* smc, float lower, float upper)
{
    if(smc == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if (lower > upper)
    {
        return BAP_FAILED_WRONG_PAR;
    }
    
    smc->with_limit = 1;
    smc->output_limit.upper_limit = upper;
    smc->output_limit.lower_limit = lower;
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCOutputLimitDisable(BAP_SMC_S* smc)
{
    if(smc == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    smc->with_limit = 0;
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCUpdate(BAP_SMC_S* smc, float k1, float K)
{
    if(smc == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    smc->k1 = k1;
    smc->K = K;
    return BAP_SUCCESS;
}
