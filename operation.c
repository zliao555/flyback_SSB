
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "operation.h"
#include "global_define.h"

void update_d(float32 duty)
{
	Uint16 d = 0;

	if (duty > (1 - duty_limit))
		d = 0.5*PERIOD*(1 - duty_limit);
	else if (duty < duty_limit)
		d = 0.5*duty_limit*PERIOD ;
	else
		d = 0.5*duty*PERIOD;

	//EPwm9Regs.CMPA.bit.CMPA = d;
	//EPwm5Regs.CMPA.bit.CMPA = d;
	//EPwm4Regs.CMPA.bit.CMPA = d;
	//EPwm3Regs.CMPA.bit.CMPA = d;
	EPwm2Regs.CMPA.bit.CMPA = d;
	//EPwm1Regs.CMPA.bit.CMPA = d;
	//DBRED = deadtime; //EPWM_MIN_DB;
	//DBFED = deadtime; //EPWM_MIN_DB;
	EPwm5Regs.DBRED = deadtime1; //EPWM_MIN_DB;
	EPwm4Regs.DBRED = deadtime1; //EPWM_MIN_DB;
	EPwm3Regs.DBRED = deadtime1; //EPWM_MIN_DB;
	EPwm2Regs.DBRED = deadtime1; //EPWM_MIN_DB;
	EPwm1Regs.DBRED = deadtime1; //EPWM_MIN_DB;

	EPwm5Regs.DBFED = deadtime1; //EPWM_MIN_DB;
	EPwm4Regs.DBFED = deadtime1; //EPWM_MIN_DB;
	EPwm3Regs.DBFED = deadtime1; //EPWM_MIN_DB;
	EPwm2Regs.DBFED = deadtime1; //EPWM_MIN_DB;
	EPwm1Regs.DBFED = deadtime1; //EPWM_MIN_DB;

}

void update_d_buffer(float32 duty)
{
	Uint16 d_test = 0;


    if (duty > (1 - duty_limit))
        d_test = PERIOD*(1 - duty_limit);
    else if (duty < duty_limit)
        d_test = duty_limit*PERIOD ;
    else
        d_test = duty*PERIOD;

       EPwm10Regs.CMPA.bit.CMPA = d_test;
	   EPwm9Regs.CMPA.bit.CMPA = d_test;
	    EPwm8Regs.CMPA.bit.CMPA = d_test;
	    EPwm6Regs.CMPA.bit.CMPA = d_test;
	    EPwm7Regs.CMPA.bit.CMPA = d_test;


		EPwm10Regs.DBRED = deadtime; //EPWM_MIN_DB;
		EPwm9Regs.DBRED = deadtime; //EPWM_MIN_DB;
		EPwm8Regs.DBRED = deadtime; //EPWM_MIN_DB;
		EPwm7Regs.DBRED = deadtime; //EPWM_MIN_DB;
		EPwm6Regs.DBRED = deadtime; //EPWM_MIN_DB;

		EPwm10Regs.DBFED = deadtime; //EPWM_MIN_DB;
		EPwm9Regs.DBFED = deadtime; //EPWM_MIN_DB;
		EPwm8Regs.DBFED = deadtime; //EPWM_MIN_DB;
		EPwm7Regs.DBFED = deadtime; //EPWM_MIN_DB;
		EPwm6Regs.DBFED = deadtime; //EPWM_MIN_DB;



}


//sin check for the buffer full-bridge




