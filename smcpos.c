#include "general.h"
#include "smcpos.h"

MOTOR_ESTIM_PARM_T motorParm;
extern SMC smc1;
ESTIM_PARM_T estimator;

void SMCInit(SMC *s)
{
    motorParm.qLsDtBase = NORM_LSDTBASE;
    motorParm.qLsDt = motorParm.qLsDtBase;
    motorParm.qRs = NORM_RS;

	if (((int32_t)motorParm.qRs<<NORM_RS_SCALINGFACTOR) >= ((int32_t)motorParm.qLsDt<<NORM_LSDTBASE_SCALINGFACTOR))

		s->Fsmopos = Q15(0.0);
	else
		s->Fsmopos = (0x7FFF - __builtin_divsd(((int32_t)motorParm.qRs<<(15+NORM_RS_SCALINGFACTOR-NORM_LSDTBASE_SCALINGFACTOR)),motorParm.qLsDt));

    if (((int32_t)motorParm.qLsDt<<NORM_LSDTBASE_SCALINGFACTOR)<32767)
		s->Gsmopos = 0x7FFF;
	else
		s->Gsmopos = __builtin_divsd(((int32_t)0x7FFF<<(15-NORM_LSDTBASE_SCALINGFACTOR)),motorParm.qLsDt);

	s->Kslide = Q15(SMCGAIN);
	s->MaxSMCError = Q15(MAXLINEARSMC);
	s->FiltOmCoef = (int16_t)(__builtin_mulss(ENDSPEED_ELECTR,THETA_FILTER_CNST)>>15);
	return;
}

inline void SMC_Position_Estimation_Inline (SMC* s)
{
    int16_t Kslf_min;
    PUSHCORCON();
	CORCONbits.SATA = 1;
	CORCONbits.SATB = 1;
	CORCONbits.ACCSAT = 1;

    CalcEstI();
	CalcBEMF();

    s->Theta = atan2CORDIC(-s->EalphaFinal,s->EbetaFinal);

    AccumTheta += s->Theta - PrevTheta;
	PrevTheta = s->Theta;

	AccumThetaCnt++;
	if (AccumThetaCnt == IRP_PERCALC)
	{        
        s->Omega = (int16_t)(__builtin_mulss(AccumTheta,SMO_SPEED_EST_MULTIPLIER)>>15);
		AccumThetaCnt = 0;
		AccumTheta = 0;
	}
    trans_counter++;
    if ( trans_counter == TRANSITION_STEPS) trans_counter = 0;

    s->OmegaFltred = s->OmegaFltred +(__builtin_mulss(s->FiltOmCoef,(s->Omega-s->OmegaFltred))>>15);

    s->Kslf = s->KslfFinal = (int16_t)(__builtin_mulss(s->OmegaFltred,THETA_FILTER_CNST)>>15);

    Kslf_min = (int16_t)(__builtin_mulss(ENDSPEED_ELECTR,THETA_FILTER_CNST)>>15);
	if (s->Kslf < Kslf_min)
	{
		s->Kslf = Kslf_min;
		s->KslfFinal = s->Kslf;
	}
	s->ThetaOffset = CONSTANT_PHASE_SHIFT;
	s->Theta = s->Theta + s->ThetaOffset;
    s->Kslide = Q15(SMCGAIN);

    POPCORCON();
    return;
}

void CalcEstI()
{
    int16_t temp_int1,temp_int2,temp_int3;
    temp_int1 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Valpha)>>15);
    temp_int2 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Ealpha)>>15);
    temp_int3 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Zalpha)>>15);

    temp_int1 = temp_int1 - temp_int2;
    temp_int1 = temp_int1 - temp_int3;
    smc1.EstIalpha = temp_int1 + (int16_t)(__builtin_mulss(smc1.Fsmopos,smc1.EstIalpha)>>15);

    temp_int1 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Vbeta)>>15);
    temp_int2 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Ebeta)>>15);
    temp_int3 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Zbeta)>>15);

    temp_int1 = temp_int1 - temp_int2;
    temp_int1 = temp_int1 - temp_int3;
    smc1.EstIbeta = temp_int1 + (int16_t)(__builtin_mulss(smc1.Fsmopos,smc1.EstIbeta)>>15);

    smc1.IalphaError = smc1.EstIalpha - smc1.Ialpha;
    smc1.IbetaError  = smc1.EstIbeta - smc1.Ibeta;

    if( _Q15abs(smc1.IalphaError)<smc1.MaxSMCError)
    {
		CalcZalpha();
    }
    else if(smc1.IalphaError>0)
    {
        smc1.Zalpha = smc1.Kslide;
    }
    else
    {
        smc1.Zalpha = -smc1.Kslide;
    }
    if( _Q15abs(smc1.IbetaError)<smc1.MaxSMCError)
    {
		CalcZbeta();
    }
    else if(smc1.IbetaError>0)
    {
        smc1.Zbeta = smc1.Kslide;
    }
    else
    {
        smc1.Zbeta = -smc1.Kslide;
    }
}

void CalcZalpha()
{
    int32_t temp_int;
    temp_int = __builtin_mulss(smc1.Kslide,smc1.IalphaError);
    smc1.Zalpha = __builtin_divsd(temp_int,smc1.MaxSMCError);
}

void CalcZbeta()
{
    int32_t temp_int;
    temp_int = __builtin_mulss(smc1.Kslide,smc1.IbetaError);
    smc1.Zbeta = __builtin_divsd(temp_int,smc1.MaxSMCError);
}

void CalcBEMF()
{
    int16_t temp_int1,temp_int2;
    temp_int1 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Zalpha)>>15);
    temp_int2 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Ealpha)>>15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.Ealpha = smc1.Ealpha + temp_int1;
    temp_int1 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Ealpha)>>15);
    temp_int2 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.EalphaFinal)>>15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.EalphaFinal = smc1.EalphaFinal + temp_int1;

    temp_int1 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Zbeta)>>15);
    temp_int2 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Ebeta)>>15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.Ebeta = smc1.Ebeta + temp_int1;
    temp_int1 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Ebeta)>>15);
    temp_int2 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.EbetaFinal)>>15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.EbetaFinal = smc1.EbetaFinal + temp_int1;
}