#include <p18f8722.h>
#include <pwm.h>
#include <timers.h>`	
#include <delays.h>
#include <adc.h>




unsigned int speed = 535;                               // average speed, between 1023 and 0
unsigned char offset = 17;                              // offset for motor losses, makes right wheel slower
unsigned int maxZeroReadings = 14;                      // number of 0's read by sensors before stopping
int Kp = 65;                                            // proportional constant
int Ki = 2;                                             // integral constant
int Kd = 1100; 
unsigned char R = 100;
unsigned char stop_delay = 75;

unsigned char A, B, C, D, E, F, G;                                        // derivative constant

unsigned char sensorResults[8] = {0};                   // array to hold sensor readings
unsigned int timer = 0;                                 // used to read time between sensor readings

void configure_driveboard (void);
void configure_ADC(void);
unsigned int get_ADC_value(void);
signed char getsensor(void);
void controlled_stop (void);

void SetupMotors(void)
{
        //TRISGbits.TRISG0 = 0;   // PWM
        //TRISCbits.TRISC2 = 0;   // PWM
		


        OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);

        OpenPWM1 (250);
        OpenPWM3 (250);
}



// Used to apply limits to speed values

unsigned int VerifySpeed(int speed)
{
        unsigned int result;

        if (speed > 1023)
                result = 1023;
        else if (speed < 0)
                result = 0;
        else
                result = (unsigned int) speed;

        return result;
}



char ReadSensors(void)
{
        char noOfSensors;
		getsensor();
        sensorResults[0] = (A) & 0b00000001;
        sensorResults[1] = (B) & 0b00000001;
        sensorResults[2] = (C) & 0b00000001;
        sensorResults[3] = (D) & 0b00000001;
        sensorResults[4] = (E) & 0b00000001;
        sensorResults[5] = (F) & 0b00000001;
       sensorResults[6] = (G) & 0b00000001;
    

        noOfSensors = sensorResults[0] + sensorResults[1] + sensorResults[2] + 
                                        sensorResults[3] + sensorResults[4] + sensorResults[5]+ sensorResults[6];
                                        

        timer = ReadTimer0();
        WriteTimer0(0);

        return noOfSensors;
}






// Averaging of sensor values eg. Fuzzy Logic

char CalculateError(char noOfSensors)
{
        char error = 0;
        char i;
        char value = -6;

        for (i = 0; i < 7 ; i++)
        {
                if(sensorResults[i])
                        error = error + value;
                value = value + 2;
        }

        error = error / noOfSensors;

        return error;
}




int Proportional(char error)
{
        return Kp * error;
}




// Multiplied by 2/3 to dampen integral effect
// Divided by 1000 to put integral in proper range eg. 0.00x

int Integral(int i)
{
        return ((Ki * i) * 2)/3000;
}





int Derivative(char previousError, char error)
{
        return Kd * ((error - previousError)/timer);
}



void main(void)
{
        int leftSpeed;
        int rightSpeed;
        char sensorData;
        char error = 0;
        char previousError = 0;
        int errorSign;
        int previousErrorSign = 1;
        int p;
        int i;
        int d;
        int pid;
        int summingI = 0;
        unsigned int whiteLineFound = 0;
		
		TRISA = 0xFF;

		TRISC = 0x00;
		TRISG = 0x00;
		TRISJ = 0x00;
		ADCON1 = 0x0F;
		configure_driveboard();
		configure_ADC();
	

        
        SetupMotors();
       


        SetDCPWM1(250+190);         // initial right speed
        SetDCPWM3(250+190+offset);         // initial left speed


        OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);


		while(1){
        while(whiteLineFound < maxZeroReadings)
        {
				
                sensorData = ReadSensors();
                if (sensorData == 0)
                {
                        if((error > -3) && (error < 3)){                 // if zeros occuring on straight then increment counter
                                whiteLineFound++;
                }		
				}
                else
                {
                        whiteLineFound = 0;
                        error = CalculateError(sensorData);
        
                        if (error == 0)                                         // -- Section used to reset integral
                                summingI = 0;                                   //    if 0 error or error changes sign
                        else                                                    //
                        {                                                       //
                                if (error > 0)                                  //
                                        errorSign = 1;                          //
                                else                                            //
                                        errorSign = -1;                         //
                                if (errorSign != previousErrorSign)             //
                                        summingI = 0;                           //
                                else
                                        summingI = summingI + error*timer;
                                previousErrorSign = errorSign;
                        }

                        p = Proportional(error);
                        i = Integral(summingI);
                        d = Derivative(previousError, error);
                        pid = p + i + d;
                        previousError = error;
        
                        rightSpeed = speed - pid - offset;                       // for positive proportional, leftspeed is larger
                        leftSpeed = speed +  pid;              // and rightspeed is smaller
        
                        SetDCPWM1(VerifySpeed(leftSpeed));             // right
                        SetDCPWM3(VerifySpeed(rightSpeed));              // left
						
						
                }
        }


        // controlled stop
        
        //Delay10KTCYx(125);
		controlled_stop();
		sensorData = ReadSensors();
		if (sensorData > 0){
			whiteLineFound = 0;
			LATJbits.LATJ4 = 1;
			LATJbits.LATJ1= 0;
			LATJbits.LATJ3 = 0;
		}
			

  }     
	

}

signed char getsensor (void)
{
unsigned char S0, S1, S2, S3, S4, S5, S6;

SetChanADC(ADC_CH0);
		S0 = get_ADC_value();
			if (S0 > R){
				A = 1;
			}	else  {
				A = 0;		

			}
		SetChanADC(ADC_CH1);
		S1 = get_ADC_value();
			if (S1 > R ){ 
				B = 1;
			}	else {
				B = 0;
			}
		SetChanADC(ADC_CH2);
		S2 = get_ADC_value();
			if (S2 > R ){ 
				C = 1;
				
			}	else {
				C = 0;
					
		
			}

		
		SetChanADC(ADC_CH3);
		S3 = get_ADC_value();
			if (S3 > R ){ 
				D = 1;
			}	else {
				D = 0;
			}
		SetChanADC(ADC_CH4);
		S4 = get_ADC_value();
			if (S4 > R ){ 
				E = 1;
			}	else {
				E = 0;
			}
		
		
		SetChanADC(ADC_CH5);
		S5 = get_ADC_value();
			if (S5 > R ){ 
				F = 1;
			}	else {
				F = 0;
			}
		
		SetChanADC(ADC_CH6);
		S6 = get_ADC_value();
			if (S6 > R){
				G = 1;
			}	else {
				G = 0;
			}
		
	
}

	void configure_ADC(void)
	{
	OpenADC(ADC_FOSC_16 & ADC_RIGHT_JUST & ADC_20_TAD, 
	    ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,0 );	
	}

void configure_driveboard (void){
	
	LATJbits.LATJ0= 0;
	LATJbits.LATJ1= 0; //Direction 1
	LATJbits.LATJ2 =0;
	LATJbits.LATJ3 = 0;	//Direction 2	
	LATJbits.LATJ4 = 1;	//enable	 		
	LATJbits.LATJ5 = 1;	//One wire
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

void controlled_stop (void){
	

	
	
	LATJbits.LATJ1= 1;
	LATJbits.LATJ3 = 1;
	Delay10KTCYx(stop_delay);
	LATJbits.LATJ4 = 0;

	

}	