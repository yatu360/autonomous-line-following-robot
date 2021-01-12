#include "p18f8722.h"   
#include "timers.h"
#include "pwm.h"
#include "Delays.h"
#include "adc.h"

void configure_ADC(void);
unsigned int get_ADC_value(void);
void display_LED(unsigned char display_val);
void check_case (unsigned char chk);

	
unsigned char S [5];


void main(void){
unsigned char x;
TRISA = 0xFF;
TRISF = 0x00;
ADCON1 = 0x0F;

configure_ADC();
LATF = 0x00;

while (1){

	x = get_ADC_value();
	LATF = x;
	
}
}	


void configure_ADC(void)
{
	OpenADC(ADC_FOSC_16 & ADC_RIGHT_JUST & ADC_20_TAD, 
	    ADC_CH3 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,0 );	
}	

unsigned int get_ADC_value(void)
{
		int result;
		ConvertADC();
		while(BusyADC());
		result=ReadADC();
		result = (result >>2 );
        return result;
}

void check_case (unsigned char chk){

switch (chk)		//Switch-case function
		{
	

		}

}