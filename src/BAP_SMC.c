#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "BAP_define.h"
#include "BAP_DiscreteFunction.h"
#include "BAP_SMC.h"

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

    memset(lf, 0, sizeof(lf));

    lf->input.num_of_plus_samples = 0;
    lf->input.num_of_minus_samples = 2;
    lf->input.dT = dT;

    lf->output.num_of_plus_samples = 0;
    lf->output.num_of_minus_samples = 2;    
    lf->output.dT = dT;

    lf->t_on_a2 = dT/(a*a);
    lf->exp_minus_t_on_a2 = (float)exp((double)(-lf->t_on_a2));

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCSecondOrderLFGetOutput(BAP_SecondOrderLF_S* lf, float* out)
{
    if(lf == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    *out = 2*lf->exp_minus_t_on_a2*lf->output.x_minus[1] 
        - lf->exp_minus_t_on_a2*lf->exp_minus_t_on_a2*lf->output.x_minus[2]
        + lf->t_on_a2*lf->exp_minus_t_on_a2*lf->input.x_minus[1];

    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_SMCInit(BAP_SMC_S* smc, int order, float dT, float sat_upper, float sat_lower, float k1, float K)
{
    if(smc == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    if(order > 3 || dT == 0 || sat_upper <= sat_lower || k1 == 0 || K == 0)
    {
        return BAP_FAILED_WRONG_PAR;
    }

    memset(smc, 0, sizeof(smc));

    smc->yd.num_of_plus_samples = order;
    smc->yd.num_of_minus_samples = 0;
    smc->yd.dT = dT;

    smc->e.num_of_plus_samples = order - 1;
    smc->e.num_of_minus_samples = 0;
    smc->e.dT = dT;

    smc->sat.upper_limit = sat_upper;
    smc->sat.lower_limit = sat_lower;

    smc->k1 = k1;
    smc->K = K;

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

    if(smc == NULL || out == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    BAP_FuncDerivate(&(smc->e), 1, &e_dot);
    BAP_FuncDerivate(&(smc->yd), 2, &yd_dot_dot);
    BAP_SMCSliddingSurfaceCal(smc, &sigma);
    BAP_FuncGetSatOutput(&(smc->sat), sigma, &sign_sat_value);

    *out = BAP_SMC_FORMULA_CONST_D*(yd_dot_dot + smc->k1*e_dot + smc->K*sign_sat_value);
    return BAP_SUCCESS;
}
