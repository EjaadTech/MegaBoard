#include "template.h"

#include "lcd.h"

#define CALIBRATION_ADDRESS 10
unsigned volatile int btnVal[6];
void btnCalibration(void);
void calibrate(void);

unsigned int getADC(unsigned char ch);

unsigned volatile static int	adcVal;

unsigned volatile static char	btnCounter=0,
								menuState=0;

int main(void)
{
	// initialize LCD
	lcd_init(LCD_DISP_ON);
	lcd_puts("Testing DevBoard");
	delay(500);

	btnCalibration();

	// initialize RTC
	// if(setRTC==1)
	// 		SET RTC TIME

	// initialize Timer0 with Interrupt
	// on every overflow, do ADC on ADC3
	// if VALID Btn value, perform button function

	TIMSK0 = BIT(TOIE0);
	TCCR0B = BIT(CS00) | BIT(CS02);
	// TimerClock = F_CPU/1024
	// ADC Freq = TimerClock / 256 = 61.03Hz

	sei();
	// Enable Interrupt

	while(1);

	return(0);
}

// Timer0 overflow Interrupt
ISR(TIMER0_OVF_vect){
	cli();
	adcVal = getADC(3);
	if((btnVal[1]-10)<adcVal && (btnVal[1]+10)>adcVal ){
		lcd_gotoxy(14,1);
		lcd_puts("UP");
		btnCounter++;
	}else if((btnVal[2]-10)<adcVal && (btnVal[2]+10)>adcVal ){
		lcd_gotoxy(12,1);
		lcd_puts("DOWN");
		btnCounter++;
	}else if((btnVal[3]-10)<adcVal && (btnVal[3]+10)>adcVal ){
		lcd_gotoxy(12,1);
		lcd_puts("LEFT");
		btnCounter++;
	}else if((btnVal[4]-10)<adcVal && (btnVal[4]+10)>adcVal ){
		lcd_gotoxy(11,1);
		lcd_puts("RIGHT");
		btnCounter++;
	}else if((btnVal[5]-10)<adcVal && (btnVal[5]+10)>adcVal ){
		lcd_gotoxy(10,1);
		lcd_puts("BUTTON");
		btnCounter++;
	}else
		lcd_clrscr();
	if(btnCounter>=61){
		
	}
	sei();
}

unsigned int getADC(unsigned char ch){
	ADMUX = ch;
	ADCSRA = BIT(ADEN) | BIT(ADSC);
		while(ADCSRA & 0b01000000);
	return(ADC);
}

void btnCalibration(void){
	// check if EEPROM has Btn Calibration values
	lcd_clrscr();
	adcVal = (getADC(3)+getADC(3)+getADC(3))/3;
	eeprom_busy_wait();
	unsigned int* addr=(unsigned int*)CALIBRATION_ADDRESS;
	unsigned int eepromVal=0;
	for(eepromVal=0; eepromVal<6; eepromVal++){
		if(eeprom_is_ready())
			btnVal[eepromVal] = eeprom_read_word(addr++);
	}
	eepromVal=btnVal[0];
	if( (eepromVal-10)<adcVal && (eepromVal+10)>adcVal ){
		lcd_puts("Calibrated...!\n");
		delay(500);		
		}
	else{
		lcd_puts("Needs\nCalibration");
		delay(500);
		calibrate();
	}
	lcd_clrscr();
}

void calibrate(void){
	unsigned int* addr=(unsigned int*)CALIBRATION_ADDRESS;
	char *calibrateMsgs[] = {	"Depress all\nButtons",
								"Up",
								"Down",
								"Left",
								"Right",
								"Button" };
	lcd_clrscr();
	lcd_puts("Starting\nCalibration");
	delay(250);
	for(int a=0; a<6; a++){
		lcd_clrscr();
		if(a>0){
			lcd_puts("Press ");
			lcd_puts(calibrateMsgs[a]);
		}else
			lcd_puts(calibrateMsgs[a]);
		delay(1000);
		btnVal[a] = (getADC(3)+getADC(3)+getADC(3))/3;
		while(!eeprom_is_ready())
			eeprom_busy_wait();
		eeprom_update_word(addr++, btnVal[a]);
	}
}
