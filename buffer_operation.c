
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "buffer_operation.h"

//initialize all parameters related to the buffer
//call init_Filter_and_Diff and init_ADCtoVolts_Ratio


//process all necessary signal for VC2 compensation
//low pass vc2 to get vc2,dc
//band pass vc1 to get 120 Hz component
//differentiate vc1 to get ic1.



//initialize parameters for all digital filters and differentiators
//initialize adc count to volts ratio based on the bias measurement for vc2, vbus and vab
void Init_ADCtoVolts_Ratio(void){
	

	// Declare and define local variables for adc conversion from full voltage to counts (and vice versa)
	float Vbus_adc_range_count = (float) (4096 - 0); 						// Full adc range in counts (w/ bias)
	float Vbus_adc_range_count_div = 1/Vbus_adc_range_count; 			// Inverse of full adc range in counts (w/ bias)
	float Vbus_adc_range_fullvolt = (float) (VBUS_ADC_MAX_VOLT - VBUS_ADC_MIN_VOLT); 		// Full adc range in volts (full voltage)
	float Vbus_adc_range_fullvolt_div = 1/Vbus_adc_range_fullvolt; 	// Inverse of full adc range in volts (full voltage)
	// Define global adc conversion ratios for adc conversion from full voltage to counts (and vice versa)
	Vbus_adc_fullvolt_to_count_ratio = Vbus_adc_range_count*Vbus_adc_range_fullvolt_div; // Full volt to count adc conversion. Count = Volt*Ratio.
	Vbus_adc_count_to_fullvolt_ratio = Vbus_adc_range_count_div*Vbus_adc_range_fullvolt; // Full volt to count adc conversion. Volt = Count*Ratio.

	// Declare and define local variables for adc conversion from full voltage to counts (and vice versa)


}


//extract dc component for vc2
void HighPassFilter_vc1 (void){

    vc1_out2 = vc1_out1;
    vc1_out1 = vc1_out;
    vc1_in2 = vc1_in1;
    vc1_in1 = vc1_in;

    vc1_in = (Vc1);

	vc1_out = b2_hpf*vc1_in + b1_hpf*vc1_in1+b0_hpf*vc1_in2 - a1_hpf*vc1_out1-a0_hpf*vc1_out2;
	//notch_a1*notch_out1-notch_a0*notch_out2;
	


	vc1_ripple = vc1_out;

}
/*

//extract 120 Hz ac component for vc1
void BandPassFilter_VC1 (void){
	notch_in_SSB = Vc1_V;
	
	notch_out_SSB = b0_notch*notch_in_SSB + b1_notch*notch_in1_SSB + b2_notch*notch_in2_SSB - (a1_notch*notch_out1_SSB + a2_notch*notch_out2_SSB);
	notch_out2_SSB = notch_out1_SSB;
	notch_out1_SSB = notch_out_SSB;
	notch_in2_SSB = notch_in1_SSB;
	notch_in1_SSB = notch_in_SSB;
	
	Vc1_notch_V = (float) notch_out_SSB;
	Vc1_bpf_V = Vc1_V - Vc1_notch_V;
	
}
*/





