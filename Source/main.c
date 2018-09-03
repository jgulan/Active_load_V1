#include "LPC11xx.h"
#include "math.h"

#define LCD_PIO0_RS	6				//LCD Register select
#define LCD_PIO0_EN	1				//LCD Enable




														//LCD data pins
#define LCD_PIO0_DB4	2
#define LCD_PIO0_DB5	9
#define LCD_PIO1_DB6	6
#define LCD_PIO0_DB7	11

#define GPIO0_CONTROL_MASK ((1<<LCD_PIO0_RS)|(1<<LCD_PIO0EN))										//Mask for clearing control pins
#define GPIO0_DATA_MASK ((1<<LCD_PIO0_DB4)|(1<<LCD_PIO0_DB5)|(1<<LCD_PIO0_DB7))	//Mask for clearing data pins on PIO0
#define GPIO1_DATA_MASK (1<<LCD_PIO1_DB6)																				//Mask for clearing data pins on PIO1

#define ADCmaxValue 		255.0f
#define V_REF						3.3f
#define R_shunt					0.007f
#define Isens_R1				47000.0f
#define Isens_R2				1000.0f
#define Vsens_R1				47000.0f
#define Vsens_R2				1000.0f

#define PWMfreq					10000.0f		
#define CPUfreq					12000000.0f

#define Kp							20						//PI proportional constant
#define Ki							1							//PI integral constant

#define LCDrefreshRate	100						//number of executions before LCD is refreshed

void delay(int count);
void LCDsendData(char data);
void LCDsendNibble(char nibble);
void LCDsendCmd(char cmd);
void LCDsendString(char *string);

void initPWM(void);
void initGPIO(void);
void initADC(void);
void initLCD(void);

void reverse(char *str, int len);													//Reversing string
void floatToStr(float n, char *res, int afterpoint);			//Float to string function
int intToStr(int x, char str[], int d);										//Integer to string function

int counter = 0;		//Counter for LCD refrshing

int main ()
{
	initGPIO();
	initADC();
	initPWM();
	initLCD();
	delay(500);
	
	LCDsendString("**Active  load**");

	delay(1000000);
	
	char currentSens[8];			//strings for numbers to display
	char voltageSens[8];
	char currentSet[8];
	char powerString[8];
	
	int AD1, AD2, AD3;										//ADC values
	float AD1f, AD2f, AD3f;								//ADC float values
	float powerWatt;
	
	//PI regulator
	double error, integral = 0, PWM;
	
	
	
	while(1)
	{
		while((LPC_ADC->DR[1] < 0x7FFFFFFF));											//wait for flag "done" to be set
		AD1 = (LPC_ADC->DR[1] & 0xFFC0)>>8;												//voltag sensor data is stored in bits 15:6
		AD1f = AD1 / ADCmaxValue;																	//voltage sensor float vale
		AD1f = AD1f * V_REF *((Vsens_R1 + Vsens_R2) / Vsens_R2);
		floatToStr(AD1f, voltageSens, 2);
		
		while((LPC_ADC->DR[2] < 0x7FFFFFFF));											//wait for flag "done" to be set
		AD2 = (LPC_ADC->DR[2] & 0xFFC0)>>8;												//data is stored in bits 15:6
		AD2f = (ADCmaxValue - AD2) / ADCmaxValue;									//current setpoint
		AD2f = AD2f * 10;
		floatToStr(AD2f, currentSet, 2);
		
		while((LPC_ADC->DR[3] < 0x7FFFFFFF));																	//wait for flag "done" to be set
		AD3 = (LPC_ADC->DR[3] & 0xFFC0)>>8;																		//data is stored in bits 15:6
		AD3f = AD3 / ADCmaxValue;																							//currrent sensor
		AD3f = AD3f * V_REF * (1/R_shunt) * (Isens_R2/(Isens_R2+Isens_R1));	
		floatToStr(AD3f, currentSens, 2);
		
		powerWatt = AD1f * AD3f;														//P = U * I
		floatToStr(powerWatt, powerString, 2);
		
		//PI regulator
		
		error = AD2f - AD3f;										//error = setpoint curent - current current
		integral = integral + error;
		
		if (integral > ADCmaxValue)							//integral max limit
			integral = ADCmaxValue;
		
		PWM = (Kp * error) + (Ki * integral);
		
		if (PWM > ADCmaxValue)			//output max limit
			PWM = ADCmaxValue;
		else if (PWM < 0)						//output min limit
			PWM = 0;
		
		if(AD2f < 0.01)							//if setpiont is less than 0.01A, turn off MosFET
			PWM = 0;
		
		PWM = ADCmaxValue - PWM;														//invert PWM value because duty cycle is inverted
		
		LPC_TMR16B0->MR0 = (((float)PWM)/ADCmaxValue) * (CPUfreq / PWMfreq);	//duty cycle
		
		if (counter == LCDrefreshRate)
		{
			LCDsendCmd(0x01);							//clear display
			LCDsendCmd(0x80);							//move cursor to beginning of first line
			delay(1500);
			LCDsendString("Iset:");
			LCDsendString(currentSet);
			
			LCDsendCmd(0x14);							//shift cursor right by one character
			LCDsendString("I:");
			LCDsendString(currentSens);
			
			LCDsendCmd(0xC0);							//new line
			LCDsendString("U:");
			LCDsendString(voltageSens);
			
			LCDsendCmd(0x14);							//shift cursor right by one character
			LCDsendString("P:");
			LCDsendString(powerString);
			
			counter = 0;
		}
		counter ++;
	}
}

void initPWM(void)
{
	LPC_IOCON->PIO0_8					= (LPC_IOCON->PIO0_8 & ~(0x3FF)) |0x2;		//set pin for PWM use with counter
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);																//enables clock for 16 bit counter (CT16B0)
	LPC_TMR16B0->PR						= 0x0;																		//prescaler value
	LPC_TMR16B0->MCR					= 0x10;																		//set for reset when counter matches MR1
	LPC_TMR16B0->EMR					|= 0x20;																	//set pin on HIGH on match
	LPC_TMR16B0->CCR					= 0;																			//set to timer mode
	LPC_TMR16B0->PWMC					= 0x1;																		//set TR16B0 to PWM mode
	LPC_TMR16B0->MR1					= (CPUfreq / PWMfreq);										//set value for period
	LPC_TMR16B0->MR0					= 0x0;																		//set value for duty cycle
	LPC_TMR16B0->TCR					|= 0x3;																		//enable and reset counter
	LPC_TMR16B0->TCR					&= ~(0x2);																//clear reset bit
}

void initGPIO(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL	|= (1<<6);																//bit6 enables clock for GPIO
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<16);																//bit 16 enables the clock to the IOCON block (for modifying IOCON registers)
	LPC_GPIO0->DIR						|= (1<<1)|(1<<2)|(1<<6)|(1<<9)|(1<<11);		//set pin 1, 2, 6, 9, 11 to output
	LPC_IOCON->R_PIO0_11			|= (1<<0);																//bit 0 enabls PIO0_11
	LPC_GPIO1->DIR						|= (1<<6);																//set pin 11 to output
}

void initADC(void)
{
	LPC_SYSCON->PDRUNCFG			&= ~(0x1<<4);				//power up ADC
	LPC_SYSCON->SYSAHBCLKCTRL	|= (1<<13);					//bit6 enables clock for ADC
	
	LPC_IOCON->R_PIO1_0				=0x02;							//set pin to ADC mode
	LPC_IOCON->R_PIO1_1				=0x02;
	LPC_IOCON->R_PIO1_2				=0x02;
	
	LPC_ADC->CR								= 0x010B0E;					//BURST mode ON, CLKDIV = B so clock for ADC is 12Mhz/(CLKDIV+1), SEL = E to select ADC 1, 2, 3 1110
	LPC_ADC->INTEN						&=~(1<<8);					//must be set 0 in BURST mode
	LPC_ADC->CR								&=~(7<<24);					//when BURST bit is 1, START bits must be 000;
}
	
	/*
Command list:
0x01	Clear display screen
0x30	8 bit, 1 line
0x38	8 bit, 2 line
0x20	4 bit, 1 lines
0x28	4 bit, 2 line
0x06	Entry mode
0x08	Display off, cursor off
0x0E	Display on, Cursor on
0x0C	Display on, Cursor off
0x0F	Display on, Cursor blinking
0x18	Shift entire display lft
0x1C	Shift entire display right
0x10	Move cursor left by on character
0x14	Move cursor right by on character
0x80	Force cursor o beginning of 1st row
0xC0	Force cursor to beginning of 2nd row
*/

/* ----------LCD functions--------- */

void initLCD(void)
{
	LCDsendCmd(0x03);				//initialize LCD
	delay(100000);
	LCDsendCmd(0x02);
	delay(10000);
	LCDsendCmd(0x28);				//enable 5x7 mode for charactersž
	delay(10000);
	LCDsendCmd(0x0C);				//display ON, cursor ON
	delay(10000);
	LCDsendCmd(0x01);				//clear display
	delay(10000);
	LCDsendCmd(0x80);				//move cursor to beginning of first line
	delay(10000);
	LCDsendCmd(0x06);
}

void LCDsendNibble(char nibble)
{
	LPC_GPIO0->DATA 		&= ~(GPIO0_DATA_MASK);											//clear previous data on pins DB4, DB5, DB7
	LPC_GPIO1->DATA			&= ~(GPIO1_DATA_MASK);											//clear previous data on pin DB6
	LPC_GPIO0->DATA			|= (((nibble >> 0) & 1) << LCD_PIO0_DB4);		//set data pins
	LPC_GPIO0->DATA			|= (((nibble >> 1) & 1) << LCD_PIO0_DB5);
	LPC_GPIO1->DATA			|= (((nibble >> 2) & 1) << LCD_PIO1_DB6);
	LPC_GPIO0->DATA			|= (((nibble >> 3) & 1) << LCD_PIO0_DB7);
}

void LCDsendCmd(char cmd)
{
	LCDsendNibble((cmd >> 4) & 0x0F);
	LPC_GPIO0->DATA			&= ~(1 << LCD_PIO0_RS);			//send LOW puls to Rgister Select for command 
	LPC_GPIO0->DATA			|= (1 << LCD_PIO0_EN);			//high to low pulse on enable pin
	delay(100);
	LPC_GPIO0->DATA			&= ~(1 << LCD_PIO0_EN);
	delay(100);
	
	LCDsendNibble(cmd & 0x0F);
	LPC_GPIO0->DATA			&= ~(1 << LCD_PIO0_RS);			//send LOW puls to Rgister Select
	LPC_GPIO0->DATA			|= (1 << LCD_PIO0_EN);			//high to low pulse on enable pin
	delay(100);
	LPC_GPIO0->DATA			&= ~(1 << LCD_PIO0_EN);
	delay(100);
	
}

void LCDsendData(char data){
	LCDsendNibble((data >> 4) & 0x0F);
	LPC_GPIO0->DATA			|= (1 << LCD_PIO0_RS);			//send HIGH puls to Rgister Select for data 
	LPC_GPIO0->DATA			|= (1 << LCD_PIO0_EN);			//high to low pulse on enable pin
	delay(100);
	LPC_GPIO0->DATA			&= ~(1 << LCD_PIO0_EN);
	delay(100);
	
	LCDsendNibble(data & 0x0F);
	LPC_GPIO0->DATA			|= (1 << LCD_PIO0_RS);			//send HIGH puls to Rgister Select
	LPC_GPIO0->DATA			|= (1 << LCD_PIO0_EN);			//high to low pulse on enable pin
	delay(100);
	LPC_GPIO0->DATA			&= ~(1 << LCD_PIO0_EN);
	delay(100);
}

void LCDsendString(char *string)
{
	while(*string != '\0')
	{
		LCDsendData(*string);
		string++;
	}
}

void delay(int count)
{
	for(int i = 0; i < count; i++);
}


void reverse(char *str, int len)
{
	int i = 0, j = len-1, temp;
	while (i < j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

int intToStr(int x, char str[], int d)
{
	int i=0;
	while (x)
	{
		str[i++] = (x % 10) + '0';
		x = x/10;
	}
	while(i<d)
		str[i++] = '0';
	
	reverse(str, i);
	str[i] = '\0';
	return i;
}

void floatToStr(float n, char *res, int afterpoint)
{
	int ipart = (int)n;
	
	float fpart = n - (float)ipart;
	
	int i = intToStr(ipart, res, 1);
	
	if(afterpoint != 0)
	{
		res[i] = '.';
		fpart = fpart * pow(10, afterpoint);
		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}
