//###########################################################################
// fcml full ripple port control, Zitao Liao
//


#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "global_variables.h"
#include "global_define.h"
#include "initialize.h"
#include "operation.h"
#include "buffer_operation.h"
#include "SFO_V8.h"
//#include "fpu_math.h"
#include "math.h"
//#include "examples_setup.h"


// some variables for SFO library
int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP

                     // register by SFO(0) function.

//
// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
volatile struct EPWM_REGS *ePWM[PWM_CH] =
{&EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs,
  &EPwm6Regs, &EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs};


void main(void)
{
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2837xD_SysCtrl.c file.





	InitSysCtrl();  //!!!! This function might be different from default.
    				//!!!! go this this function and double check the clock is
    				// set to 120MHz.s

    // disable peripheral clks that we do not use to save power
	disable_unused_clk();

    // Step 2. Initialize GPIO.
    InitGpio(); // set the GPIO to it's default state (i.e., high impedance input)

    // initialize the following pin for active rectifier
    // GPIO14: rec_neg_PWM
    // GPIO15: rec_pos_PWM
    // GPIO16: rec_shutdown
    //init_rec_GPIO();



	// Step 3. Clear all interrupts and initialize PIE vector table:
    clear_interrupts();

	// Step 4: peripheral setup
    init_ADCs(); // initialize all ADCs (a,b,c,d)

    init_DACs(); // initialize DACs for debugging

    init_pwms(); // initialize phase-shifted PWM

    SFO_status = SFO_INCOMPLETE;
    while(SFO_status == SFO_INCOMPLETE) // Call until complete
    {
        SFO_status = SFO();
        if (SFO_status == SFO_ERROR)// SFO function returns 2 if an error occurs & # of MEP steps/coarse step exceeds maximum of 255.
        {
        	ESTOP0;     // Stop here and handle error
        }

    }


	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; //enable synchronization of all ePWMs to the TBCLK
										// this will start ADC conversion since ADC are triggered by ePWM1
	EDIS;

	//ADC_bias();
	#ifdef ADC_calibration
	ADC_manual_calibration();
	#endif

	// measure amplifier IC bias value here before anything turn on
	// things to measure include bias for IL, Iout
	//bias_measurement();  // this function will take a few seconds

	//Init_SSB();
	// clearing array
	// this has to be done, otherwise the corresponding XX_sum variable might not have the correct number
	//clear the moving average array for Vout signal
	memset(Vout_sample, 0, MOV_AVE_SIZE);
	memset(ibuf_sample, 0, MOV_AVE_SIZE);
//	memset(pll_out_sample, 0, MOV_AVE_SIZE);



	// Step 5: interrup setup
	init_interrupts();

	//eanble global interrupts.
	//this will enable the flowing:
	//1, boost converter PFC function
	//2, unfolder start function
	EnableInterrupts();

	// Step 6: infinite loop, two ISR handle the rest
	while(1)
	{
	    while(SFO_status == SFO_INCOMPLETE) // Call until complete
	    {
	        SFO_status = SFO();
	    }
	}

}



// purpose of function ac_trigger():
// given PWM in count_up_down mode, this function should update 300kHz
// 1, based on Iout measurement, calculate the moving average value, figure out the AC part and generate reference for IL
interrupt void ac_trigger(void)
{
	Vc2 = (signed)AdcdResultRegs.ADCRESULT0;
    //high pass vc1, transfer function  = s/(s+wc)

    //detect amplitude to determine dc bias
    Vout_sum = Vout_sum + abs(vc1_ripple) - Vout_sample[sine_index]; // Vout_sum = Vout_sum + newest value - oldest value
    Vout_sample[sine_index] = abs(vc1_ripple); // replace the sample value
    Vout_ave = Vout_sum * MOV_AVE_SIZE_DIV*0.5;

    //abs then moving average
    //scale vc1 to vab
    //add dc-bias and scaled vc1 ripple

    Vab_ref =(k_shift*Vout_ave - k_cap*vc1_ripple)*k_on;
    //0.78 for kcap, 1.4 for k_shift
    //sense vc2 and modulate half bridge

    if (Vc2<50)
    main_duty =  Vab_ref/500;
    else
    main_duty = Vab_ref/Vc2;


    if (sine_index == num_points){
        sine_index = 0;
    }
    sine_index++;
    theta = step*sine_index;


	DacaRegs.DACVALS.bit.DACVALS = Vc1;
	DacbRegs.DACVALS.bit.DACVALS = vc1_ripple+400;
	DaccRegs.DACVALS.bit.DACVALS = (Vc1-vc1_out);
	// clear flag and get ready for next interrupt
	AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

}

// purpose of function bd_trigger():
interrupt void bd_trigger(void)
{

    //update_d_buffer (main_duty_debug);
    Vout = (signed)AdcbResultRegs.ADCRESULT1;
    Vab_count =  (signed)AdccResultRegs.ADCRESULT1;
    Vc1 = Vout-Vab_count;
    //Vc1 = 400 + 200*__sinpuf32(theta);
    HighPassFilter_vc1();
    update_d(main_duty);
  //  update_d_buffer(main_duty);


    //clear flag and get ready for next interrupt
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT2 flag
	//AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

}


