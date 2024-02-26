/*
 * Labornetzteil ELV PPS 5330 hack
 * MCU ATmega88PA
 *
 * Created: 11.09.2020 Version 1.5a
 * Author : Rolli
 * Project: PowerSupply PPS5330 with temperature screen
 */ 


// includes --------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

// defines ---------------------------------------------------------------
#define F_CPU 8000000UL
#define ADC_AD0		PD4  
#define ADC_AD1		PD5	 
#define ADC_AD2		PD7	 
#define ADC_ADW		PB0	
#define FAN         PD6 
#define Standby		PC3	 
#define UI			PC1
#define Up          PC0
#define Down        PC2
#define Recall      PD1
#define Enter		PC4
#define Memory      PC5
#define Regler      PD0
#define Key_standby (PINC & (1<<PC4))	// A4
#define Enc_A       PD3  // (Encoder_Pin A)
#define Enc_B       PD2  // (Encoder_Pin B)
#define PHASE_A     (PIND & 1<<PD3)
#define PHASE_B     (PIND & 1<<PD2)
#define U_Mess		0 
#define I_Mess		1
#define T2_Mess		2
#define T1_Mess		3
#define TRUE 	1
#define FALSE 	0
#define Vref 2.50
#define SPI_MOSI    3     // PC5 = SPI MOSI
#define SPI_MISO    4     // PC6 = SPI MISO
#define SPI_SCK     5     // PC7 = SPI SCK
#define SPI_SS      2     // PC4 = SPI SS
#define NOP() asm volatile ("nop" ::) 



// prototyps -------------------------------------------------------------
void soft_delay (uint16_t time_value);
void SOFT_SPI_init(void);
void init_SPI(void);
void SPI_wr2(unsigned char dataout);
void wr_SPI_buffer3(uint8_t data0, uint8_t data1, uint8_t data2);
void wr_SPI_buffer4(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3);
void init_IO_Port(void);
void print_value(uint8_t Index, uint16_t value);
void init_Interrupt(void);
void init_Timer0(void);
void init_Timer1(void);
void init_Timer2(void);
void start_Measurement_timer(void);
void print_U_result(uint16_t mess_time);
void print_I_result(uint16_t mess_time);
void print_T1_result(uint16_t mess_time);
void init_Encoder(void);
void pull_encoder(void);
int8_t encode_read4(void);
void Measurement(void);
void set_UI_Limits(void);
void save_print_UIlimit (void);
void set_Usoll (uint16_t Usoll_value);
void set_Isoll (int Isoll);
void save_Ulimit (uint16_t Ulimit);
void save_Ilimit (uint16_t Ilimit);
void read_Ulimit (void);
void read_Ilimit (void);
void setDigitPos (void);
void buttonFunction (uint8_t Button_nr);
void pressButton (uint8_t Button_nr);
void lessButton (uint8_t Button_nr);
void readButton (void);
void UI_control(void);
void init_LCD (void);
void init_UIlimits (void);
void send_LCD_commands (const uint8_t Com_Adr[]);
void print_degree(uint16_t value);
void set_memory_nr (void);
void wr_UI_eeprom(void);
void rd_UI_eeprom (void);
void flash_PrgNo(void);


// global variables ------------------------------------------------------
uint8_t enc_value = 0;
int8_t enc_delta = 0;          // -128 ... 127
int8_t last;
volatile uint8_t timer_max = 0;
volatile uint8_t counter_h = 0;
volatile uint8_t ADW_flag = 0;
uint8_t Mess_phase = 0;
uint8_t Mess_type = 0;
volatile uint8_t Timer2_run_flag = 0;
uint8_t Standby_flag = 0;
uint8_t buttonState = 0;
uint8_t buttonDebounce = 0;
int32_t Usoll = 0;
int Isoll = 0;
int32_t Ulimit = 0;
int32_t Ilimit = 0;
uint8_t change_Enc = 0;
int Multiplier = 1000;
int Multiplier_2 = 100;
uint8_t Button_nr_old = 0;
uint8_t UI_flag = 0;
uint8_t Daten_Buffer[3];
uint8_t Hold_time = 0;
uint8_t Refresh_Ulimit = FALSE;
uint8_t Refresh_Ilimit = FALSE;
uint8_t Button5_flag = 0;
uint8_t Button6_flag = 0;
uint8_t Button7_flag = 0;
uint8_t Memory_flag = 0;
int8_t Memory_nr = 0;
uint8_t Recall_flag = 0;
uint8_t Enter_flag = 0;
uint8_t flash_time = 5;
uint8_t blinki_flag = 0;
int16_t Uist = 0;
int16_t Iist = 0;
uint8_t blinki_flag1 = 0;
uint8_t flash_time1 = 0;
uint8_t UI_switch = 0;
uint8_t digit[4];



//LCD commands -----------------------------------------------------------
const uint8_t clr_Digit_Lines[] PROGMEM = {24,0x24,0x07,0x1D,0x24,0x06,0x15,
	0x24,0x06,0x1D,0x24,0x05,0x15,0x27,0x01,0x1D,0x27,0x01,0x15,0x27,0x02,
	0x1D,0x27,0x02,0x15};
	
const uint8_t Standby_on[] PROGMEM = {10,0xA1,0x22,0x03,0x31,0x20,0x05,
	0x1D,0x27,0x03,0x1D};
	
const uint8_t Standby_off[] PROGMEM = {10,0xA0,0x22,0x03,0x1E,0x20,0x05,0x32,
	0x27,0x03,0x1D};
	
const uint8_t Degree_Symbol[] PROGMEM = {12,0x23,0x02,0x31,0x30,0x17,0x20,
	0x30,0x26,0x38,0x30,0x24,0x17};
	
const uint8_t Activ_V[] PROGMEM = {6,0x20,0x05,0x32,0x27,0x03,0x1D};	
	
const uint8_t Activ_A[] PROGMEM = {6,0x27,0x03,0x32,0x20,0x05,0x1D};
	
const uint8_t clr_Memory[] PROGMEM = {8,0x27,0x03,0x15,0x6B,0x5D,0x5D,0x5D,0x5D};
	
const uint8_t LCD_inits[] PROGMEM = {59,0x23,0x05,0x34,0x23,0x06,0x34,0x27,
	0x03,0x31,0x27,0x01,0x34,0x23,0x02,0x31,0x30,0x17,0x20,0x30,0x26,0x38,
	0x30,0x24,0x17,0x00,0x23,0x38,0x04,0x20,0x38,0x04,0x21,0x38,0x04,0x27,
0x34,0x23,0x05,0x31,0x27,0x05,0x31,0x24,0x06,0x38,0x43,0x5D,0x50,0x50,0x50,
0x47,0x50,0x50,0x50,0x50,0x4B,0x50,0x50,0x50,0x50};
	

//*************************************************************************
// soft delay
//*************************************************************************
void soft_delay (uint16_t time_value)
{
	for (uint16_t i = 0; i < time_value; i++)
	{
		for (uint16_t count = 0; count < 0x7FF; count++)
		{
			NOP();
		}
	}
}

//*************************************************************************
// SPI init
//*************************************************************************
void init_SPI(void)
{
	DDRB |= (1<<SPI_SS)|(1<<SPI_MOSI)|(1<<SPI_SCK);    // SCK, MOSI and SS as outputs
	DDRB &= ~(1<<SPI_MISO);								// MISO as input
	PORTB |= (1<< SPI_MISO);
	SPCR |= (1<<MSTR);								// Set as Master
	SPCR |= (1<<SPR0) | (1<<CPOL) | (1<<CPHA);		// set CLK 500KHz and SPI Mode
	SPCR |= (1<<SPE);								// Enable SPI
}

//*************************************************************************
// SPI write
//*************************************************************************
void SPI_wr2(unsigned char dataout)
{		
	SPDR = dataout;
	while(!(SPSR & (1<<SPIF)));
	while(PINB & (1<<SPI_MISO));
}

//*************************************************************************
// write SPI send data block
//*************************************************************************
void wr_SPI_buffer3(uint8_t data0, uint8_t data1, uint8_t data2)
{
	SPI_wr2(data0);
	SPI_wr2(data1);
	SPI_wr2(data2);
}

void wr_SPI_buffer4(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
	SPI_wr2(data0);
	SPI_wr2(data1);
	SPI_wr2(data2);
	SPI_wr2(data3);
}

//*************************************************************************
// init Timer1   14Bit/500Hz PWM mode (voltage and curent controll)
//*************************************************************************
void init_Timer1(void)
{
	ICR1 = 0x3FFF;
	// set TOP to 14bit 16383/500Hz
		
	OCR1A = 0;	// set Usoll
	OCR1B = 0;	// set Isoll

	TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
	// set none-inverting mode

	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12)|(1 << WGM13);
	// set Fast PWM mode using ICR1 as TOP
	
	TCCR1B |= (1 << CS10);
	// START the timer with no prescaler
	
}

//*************************************************************************
// init IO Ports
//*************************************************************************
void init_IO_Port(void)
{	
	DDRD |=(1<<ADC_AD0);		// Multiplexer IC6 
	DDRD |=(1<<ADC_AD1);
	DDRD |=(1<<ADC_AD2);
	DDRB &= ~(1<<ADC_ADW);		// extern PCINT0 for Meassurement
	DDRB |= (1<<PIN1);			// PB1 OC1A	PWM Voltage
	DDRB |= (1<<PIN2);			// PB2 OC1B	PWM Ampere
	DDRC |= (1<<PIN3);			// Key "Standby"
	DDRD &= ~(1<<PIN1);			// Key "Recall"
	DDRC &= ~(1<<PIN5);			// Key "Memory"
	DDRC &= ~(1<<PIN4);			// Key "Enter"
	DDRD &= ~(1<<PIN0);			// Regler
	DDRD &= ~(1<<PIN2);			// Encoder Pin A
	DDRD &= ~(1<<PIN3);			// Encoder Pin B
	DDRC &= ~(1<<UI);			// Key "U/I"
	DDRC &= ~(1<<Up);			// Key "<-"
	DDRC &= ~(1<<Down);			// Key "->"
	DDRD |= (1<<PIN6);			// FAN PWM out
	
	PORTB |= (1<<ADC_ADW);		// Pullup enabled
	PORTC |= (1<<UI);			// Pullup UI switch
	PORTD |= (1<<PIN0);			// Pullup Regler
	PORTD |= (1<<PIN1);			// Pullup Key "Regler"
	PORTC |= (1<<PIN3);			// Pullup Key Standby
	PORTC |= (1<<PIN5);			// Pullup Key "Memory"
	PORTC |= (1<<PIN4);			// Pullup Key "Enter"
	PORTD |= (1<<Enc_A);		// Pullup Encoder Pin A
	PORTD |= (1<<Enc_B);		// Pullup Encoder Pin B
	PORTC |= (1<<Up);			// Pullup Key "<--"
	PORTC |= (1<<Down);			// Pullup Key "-->"
	
}

//*************************************************************************
// print values on screen
//*************************************************************************
void print_value(uint8_t Index, uint16_t value)
{
	uint16_t zahl = value;
	
	// print Voltage -----------------------------------------------------
	if (Index == 0x43 || Index == 0x63){
			digit[0] = (zahl/10000)+0x50;
			if (digit[0] == 0x50)
			{
				digit[0] = 0x5D;	// if fist digit 0 than print "Space"
			}
			zahl=zahl % 10000;
			digit[1] = (zahl/1000)+0x50;
			zahl=zahl % 1000;
			digit[2] = (zahl/100)+0x50;
			zahl=zahl % 100;
			digit[3] = (zahl/10)+0x50;
		}
		
		// print Ampere --------------------------------------------------
		else if (Index == 0x47 || Index == 0x4B || Index == 0x67){
			digit[0] = (zahl/1000)+0x50;
			if (Index == 0x4B && digit[0] == 0x50)
			{
				digit[0] = 0x5D;	// if fist digit 0 than print "Space"
			}
			zahl=zahl % 1000;
			digit[1] = (zahl/100)+0x50;
			zahl=zahl % 100;
			digit[2] = (zahl/10)+0x50;
			zahl=zahl % 10;
			digit[3] = (zahl/1)+0x50;
		}
				
		SPI_wr2(Index);
		wr_SPI_buffer4(digit[3], digit[2], digit[1], digit[0]);
}

//*************************************************************************
// print Memory number
//*************************************************************************
void print_memory_nr (uint8_t number)
{
	uint8_t zahl = number;
	digit[0] = (zahl/10) + 0x50;
	zahl = zahl % 10;
	digit[1] = zahl + 0x50;
	SPI_wr2(0x6B);
	wr_SPI_buffer4(0x5D, 0x5D, digit[1], digit[0]);
}

//*************************************************************************
// print temperature
//*************************************************************************
void print_degree(uint16_t value)
{
	uint16_t zahl = value /10;
	digit[0] = (zahl/100)+0x50;
	zahl=zahl % 100;
	digit[1] = (zahl/10)+0x50;
	zahl=zahl % 10;
	digit[2] = (zahl+0x50);
	digit[3] = 0x52;				// "2" special for degree symbol
	SPI_wr2(0x4B);					// print temp "xx.xC"
	wr_SPI_buffer4(digit[3], digit[2], digit[1],digit[0]);
	send_LCD_commands(Degree_Symbol);
	wr_SPI_buffer3(0x23,0x03,0x1E);		// clr "W"
}

//-------------------------------------------------------------------------
// init extern INT0
//-------------------------------------------------------------------------
void init_Interrupt(void)
{
	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT0);
	sei();						// enable Global Interrupts 
}

//-------------------------------------------------------------------------
// init Timer2 (ADC measurement)
//-------------------------------------------------------------------------
void init_Timer2(void)
{	
	TCCR2B |= (1<<CS00) | (1<<CS01);	// Prescaler 64  (4us) 
}

//-------------------------------------------------------------------------
// init Timer0 (Fan_PWM)
//-------------------------------------------------------------------------
void init_Timer0(void)
{
	TCCR0B |= (1<<CS00)|(1<<CS01);						// prescaler 64
	TCCR0A |= (1<<COM0A1) | (1<<WGM00) | (1<<WGM01);	// Set OC2B at bottom, clear OC2B
	OCR0A = 127;										// 100% Fan-PWM 
}

//*************************************************************************
// start measurment Timer
//*************************************************************************
void start_Measurement_timer(void)
{
	// clear timer register
	OCR2B = 0;
	TCNT2 = 10;
		
	if (Mess_phase == 3) {
		ADW_flag = 1;
	}
	else ADW_flag = 0;
	
	// set overflow Interrupt
	TIMSK2 |= (1<<TOIE2);
	
	// clear Timer2 overflow_flag
	TIFR2 |= (1<<TOV2);
	
	// clear extern INT0 Flag
	PCIFR |= (1<<PCIF0);
	
	// start Timer 2
	TCCR2B |= (1<<CS00)| (1<<CS01);
}

//*************************************************************************
// Timer2 Overflow interrupt for measurement results
//*************************************************************************
ISR (TIMER2_OVF_vect)
{   
  OCR2B++;
  
  if (OCR2B == 48 && ADW_flag == 0){
	  TCCR2B &= ~((1<<CS00) | (1<<CS01));	// stopp timer and disabled ADC
	  PORTD |= ((1 << ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2));
	  Timer2_run_flag = 0;
	  }
}

//*************************************************************************
// Meassurement interrupt from ADW-Pin
//*************************************************************************
ISR(PCINT0_vect)
{
	//  fallende Flanke
	if (!(PINB & (1<<PB0))) {
		TCCR2B &= ~((1<<CS00) | (1<<CS01));	// stopp timer and disabled ADC
		PORTD |= ((1 << ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2));
		Timer2_run_flag = 0;
		}
}

//*************************************************************************
// print U_measurement result
//*************************************************************************
void print_U_result(uint16_t mess_time)
{
	#define Digital_offset_v 162
	static int8_t count_v;
	
	if (Standby_flag == 1 || mess_time <= Digital_offset_v)
	{
		print_value(0x43,0);
		return;
	}
	
	int16_t result = (((int32_t)mess_time - Digital_offset_v) * 198358) >> 16;
	
	if (Ulimit > 0)
	{
		// automatic voltage regulation --------------------------------------
		if (result != Ulimit){
			if (result > (Ulimit + 5)){
				count_v--;
				set_Usoll(Ulimit + count_v);
			}
			else if (result < (Ulimit+1)){
				count_v ++;
				set_Usoll(Ulimit + count_v);
			}
		}
	}
	
	print_value(0x43,result);
	Uist = result;
}

//*************************************************************************
// print I_measurement result
//*************************************************************************
void print_I_result(uint16_t mess_time)
{
	#define Digital_offset_a 155
	
	if (Standby_flag == 1 || mess_time <= Digital_offset_a) 
	{
		print_value(0x47,0);
		Iist = 0;
		return;
	}
	uint16_t result = (((uint32_t)mess_time - Digital_offset_a) * 15839) >> 16;	// 15805
	print_value(0x47,result);
	Iist = result;
}

//*************************************************************************
// print T1_measurement result (heat sink)
//*************************************************************************
void print_T1_result(uint16_t mess_time)
{	
	#define Digital_offset_b 5720  // 0.0'C
	
	//uint16_t result = (((uint32_t)mess_time - Digital_offset_b) * 3.035);
	uint16_t result = (((uint32_t)mess_time - Digital_offset_b) * 198902) >> 16;
	
	if (Iist > 0 && blinki_flag1 == 0 && flash_time1 > 0)
	{
		uint16_t watt = ((uint32_t)Uist * Iist) /10000;
		wr_SPI_buffer3(0x23,0x03,0x31);		// print "W"
		print_value(0x4B,watt);	
	}
	else if (Iist > 0 && blinki_flag1 == 1 && flash_time1 > 0){
		print_degree(result);	
	}
	else print_degree(result);
	
	if (flash_time1 == 0)
	{
		flash_time1 = 30;
			if (blinki_flag1 == 0)
			{
				blinki_flag1 = 1;
			}
			else blinki_flag1 = 0;
	}
	
	result = result / 100;
		
	// heat alert
	if (result >= 75)
	{
		wr_SPI_buffer4(0xA1, 0x22, 0x03, 0x31);
		Standby_flag = 1;
		return;
	}
	else if (result >= 70.0)
	{
		wr_SPI_buffer3(0x22, 0x01 , 0x34);
		OCR0A = 255;		// set max Fan PWM
		return;
	}
	else if (result < 69)
	{
		wr_SPI_buffer3(0x22, 0x01, 0x11);
		
		if (result <= 35)
		{
			OCR0A = 20;		// set min Fan PWM
		}
		else
		{
			uint8_t fan_speed = ((result-31) * 4);	// calc fan_speed
			OCR0A = fan_speed;
		}
	}
}


//*************************************************************************
// Encoder
//*************************************************************************
void init_Encoder(void)
{
	int8_t new;
	new = 0;
	if( PHASE_A ) new = 3;
	if( PHASE_B ) new ^= 1;				// convert gray to binary
	last = new;							// power on state
	enc_delta = 0;
}

// poll Encoder-Port  ----------------------------------------------------
void pull_encoder(void)
{
	int8_t new, diff;
	new = 0;
	if( PHASE_A ) new = 3;
	if( PHASE_B ) new ^= 1;				// convert gray to binary
	diff = last - new;					// difference last - new
	if( diff & 1 ) {					// bit 0 = value (1)
		last = new;						// store new as next last
		enc_delta += (diff & 2) - 1;	// bit 1 = direction (+/-)
	}
}

// read Encoder ----------------------------------------------------------
int8_t encode_read4(void)				// read four step encoders
{
	int8_t val;
	val = enc_delta;
	enc_delta = val & 3;
	return val >> 2;
}

//*************************************************************************
// U-I-P-T Measurement
//*************************************************************************
void Measurement(void)
{
		// Abort when a measuring phase is running -----------------------
		if (Timer2_run_flag == 1)		
		{
			return;
		}
		
		// clear integrator ----------------- ----------------------------
		if (Mess_phase == 0)
		{
			// clear ADC
			Mess_phase = 1;
			Timer2_run_flag = 1;
			PORTD |= (1 << ADC_AD0) | (1 << ADC_AD2);
			PORTD &= ~(1<<ADC_AD1);
			start_Measurement_timer();
			return;
		}
		
		// load integrator -----------------------------------------------
		if (Mess_phase == 1)
		{
			// clear flags
			Mess_phase = 2;
			Timer2_run_flag = 1;
			
			// set measurement typ
			if (Mess_type == 0)			// U_measurement
			{
				PORTD &= ~((1 << ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2));
			}
			if (Mess_type == 1)			// I_measurement
			{
				PORTD &= ~((1 << ADC_AD1) | (1<<ADC_AD2));
				PORTD |= (1 << ADC_AD0);
			}
			if (Mess_type == 2)			// T1_measurement
			{
				PORTD &= ~(1<<ADC_AD2);
				PORTD |= (1 << ADC_AD0) | (1<<ADC_AD1);
			}
			
			start_Measurement_timer();
			return;
		}
		
		// measurement integrator ----------------------------------------
		if (Mess_phase == 2)
		{
			Mess_phase = 3;
			Timer2_run_flag = 1;
			// start ADC measurment
			PORTD |= (1 << ADC_AD2);
			PORTD &= ~((1 << ADC_AD0) | (1<<ADC_AD1));
			start_Measurement_timer();
			return;			
		}
		
		// print measurment result ---------------------------------------
		if (Mess_phase == 3)
		{
			// read timer value for measurements
			uint16_t mess_time = ((OCR2B << 8) | TCNT2);
			
			ADW_flag = 0;
			
			if (Mess_type == 0)
			{
				if (change_Enc == FALSE && Hold_time == 0)
				{
					print_U_result(mess_time);
				}
			}
			
			if (Mess_type == 1)
			{
				if (change_Enc == FALSE && Hold_time == 0)
				{
					print_I_result(mess_time);
				}
			}
			
			if (Mess_type == 2)
			{
				print_T1_result(mess_time);
			}
			
			Mess_type++;
			if (Mess_type >= 3)
			{
				Mess_type = 0;
			}
			
			// clear flags
			Mess_phase = 0;
			Timer2_run_flag = 0;
			
			// set UI-Limits
			set_UI_Limits();
			
			if (Hold_time > 0)
			{
				Hold_time--;
			}
			
			if (flash_time > 0)
			{
				flash_time--;
			}
			
			if (flash_time1 > 0)
			{
				flash_time1--;
			}
		}
}

//*************************************************************************
// set UI-Limits
//*************************************************************************
void set_UI_Limits(void)
{
	if (Memory_flag != 0 || Recall_flag != 0)
	{
		set_memory_nr();
		return;
	}
	// read encoder ------------------------------------------------------
	int Enc_value = encode_read4();
	
	if (Enc_value == 0)
	{
		change_Enc = FALSE;
		return;
	}
	else change_Enc = TRUE;
	
	// set Ulimit --------------------------------------------------------	
	if (UI_flag == 0){
		Ulimit = Ulimit + (Multiplier * Enc_value);
		
		if (Ulimit < 0) {
			Ulimit = 0;
		}
		else if (Ulimit >= 30000) {
			Ulimit = 30000;
		}
		
		if (Ulimit >= 15000) {
			SPI_wr2(0xB1);			// Relais on  unregulated VDC 48.0V
		}
		else if (Ulimit <= 14500) {
			SPI_wr2(0xB0);			// Relais off  unregulated VDC 24.0V
		}
		
		// set Usoll
		set_Usoll(Ulimit);
		print_value(0x43,Ulimit);

		Hold_time = 30;				// time befor refresh UI-Limits
		Refresh_Ulimit = TRUE;
		return;
	}
	
	// set Ilimit --------------------------------------------------------
	else if (UI_flag == 1){
		Ilimit = Ilimit + (Multiplier_2 * Enc_value);
		
		if (Ilimit < 0) {
			Ilimit = 0;
		}
		else if (Ilimit >= 3000) {
			Ilimit = 3000;
		}
		
		// set Isoll
		set_Isoll(Ilimit);
		print_value(0x47,Ilimit);
		
		Hold_time = 30;
		Refresh_Ilimit = TRUE;
		return;
	}
}

//*************************************************************************
// set memory number
//*************************************************************************
void set_memory_nr (void)
{
	// read encoder ------------------------------------------------------
	int Enc_value = encode_read4();
	
	if (Enc_value == 0)
	{
		change_Enc = FALSE;
		return;
	}
	else change_Enc = TRUE;
	
	Memory_nr = Memory_nr + Enc_value;
	
	if (Memory_nr >= 15)
	{
		Memory_nr = 15;
	}
	else if (Memory_nr < 0)
	{
		Memory_nr = 0;
	}
	print_memory_nr(Memory_nr);
	
	blinki_flag = 0;
	flash_time = 3;
}

//*************************************************************************
// save and print Ulimit/ Ilimit
//*************************************************************************
void save_print_UIlimit (void)
{
	uint8_t Temp_flag = UI_flag;		// save UI_flag
	
	if (Hold_time == 0 && Refresh_Ulimit == TRUE)
	{
		UI_flag = 0;
		save_Ulimit(Ulimit);
		print_value(0x63,Ulimit);
		Refresh_Ulimit = FALSE;
	}
	
	if (Hold_time == 0 && Refresh_Ilimit == TRUE)
	{
		UI_flag = 1;
		save_Ilimit(Ilimit);
		print_value(0x67,Ilimit);
		Refresh_Ilimit = FALSE;
	}
	
	UI_flag = Temp_flag;			// restore UI_flag
}

//*************************************************************************
// set Usoll (0 - 30.000mV)
//*************************************************************************
void set_Usoll (uint16_t Usoll_value)
{
	uint8_t Digi_offset = 120;
	#define Umax 30000
	//#define counts_per_30v 15419
	
	if (Usoll_value == 0)
	{
		OCR1A = Digi_offset;
		return;
	}
	else Digi_offset = 127;
	
	uint16_t counts_per_30v = 15423;
	uint32_t u_factor = (counts_per_30v * 65536 / Umax);
	OCR1A = Digi_offset + ((Usoll_value * u_factor) >> 16) +
	(((Usoll_value * u_factor) >> 15) & 1);
}


//*************************************************************************
// set Isoll
//*************************************************************************
void set_Isoll (int Isoll)
{
	const uint8_t offset = 160;
	OCR1B = offset + (((int32_t)Isoll * 10000) / 2045);	// OCR1B 600=100mA 1200=230mA
}

//*************************************************************************
// save Ulimit (EEPROM Adr. 0x000)
//*************************************************************************
void save_Ulimit (uint16_t Ulimit)
{
	uint16_t * eeAddr = 0;
	eeprom_write_word(eeAddr, Ulimit);
}

//*************************************************************************
// read Ulimit (EEPROM Adr. 0x000)
//*************************************************************************
void read_Ulimit (void)
{
	uint16_t * eeAddr = 0;
	Ulimit = eeprom_read_word(eeAddr);
}

//*************************************************************************
// save Ilimit (eeprom Adr. 0x004)
//*************************************************************************
void save_Ilimit (uint16_t Ilimit)
{
	uint16_t * eeAddr = 2;
	eeprom_write_word(eeAddr, Ilimit);
}

//*************************************************************************
// read Ilimit (eeprom Adr. 0x004)
//*************************************************************************
void read_Ilimit (void)
{
	uint16_t * eeAddr = 2;
	Ilimit = eeprom_read_word(eeAddr);
}

//*************************************************************************
// set Voltage Digit Line 
//*************************************************************************
void setDigitPos (void)
{
	// Voltage Digit Line ------------------------------------------------
	if (UI_flag == FALSE) {
		if (Multiplier == 10)	
		{
			wr_SPI_buffer3(0x24,0x05,0x38);
		}
		else if (Multiplier == 100)
		{
			wr_SPI_buffer3(0x24,0x06,0x32);
		}
		else if (Multiplier == 1000)
		{
			wr_SPI_buffer3(0x24,0x06,0x38);
		}
		else if (Multiplier == 10000)
		{
			wr_SPI_buffer3(0x24,0x07,0x32);
		}
	}
	
	// Ampere Digit Line -------------------------------------------------
	else {		
		if (Multiplier_2 == 1)
		{
			wr_SPI_buffer3(0x27,0x02,0x38);
		}
		else if (Multiplier_2 == 10)
		{
			wr_SPI_buffer3(0x27,0x02,0x32);
		}
		else if (Multiplier_2 == 100)
		{
			wr_SPI_buffer3(0x27,0x01,0x38);
		}
		else if (Multiplier_2 == 1000)
		{
			wr_SPI_buffer3(0x27,0x01,0x32);
		}
	}
}

//*************************************************************************
// send LCD commands
//*************************************************************************
void send_LCD_commands (const uint8_t Com_Adr[])
{
	uint8_t counts = pgm_read_byte (Com_Adr);
	
	for (uint8_t i = 1; i <= counts; i++)
	{
		SPI_wr2(pgm_read_byte (Com_Adr + i));
	}
}

//*************************************************************************
// Button function
//*************************************************************************
void buttonFunction (uint8_t Button_nr)
{
	// Standby on -------------------------------------------------------- 
	if (Button_nr == 1 && Standby_flag == 0) {
		send_LCD_commands(Standby_on);
		set_Isoll(0);
		set_Usoll(0);
		print_value(0x43,0);
		print_value(0x47,0);
		Standby_flag = 1;
		return;
	}
	
	// Standby off -------------------------------------------------------
	else if (Button_nr == 1 && Standby_flag == 1) {
		read_Ulimit();		// read Ulimit from EEPROM
		read_Ilimit();
		set_Usoll(Ulimit);
		set_Isoll(Ilimit);
		//print_value(0x43,Ulimit);
		send_LCD_commands(Standby_off);
		Standby_flag = 0;
		if (Ulimit >= 15000) {
			SPI_wr2(0xB1);			// Relais on  unregulated VDC 48.0V
		}
		else if (Ulimit <= 14500) {
			SPI_wr2(0xB0);			// Relais off  unregulated VDC 24.0V
		}
		return;
	}
	// set Voltage Up "<--" ----------------------------------------------
	else if (Button_nr == 2 && UI_flag == 0) {
		if (Multiplier < 10000) {
			Multiplier = Multiplier * 10;
			send_LCD_commands(clr_Digit_Lines);
			setDigitPos();
		} return;
	}
	// set Ampere Up "<--" ----------------------------------------------
	else if (Button_nr == 2 && UI_flag == 1) {
		if (Multiplier_2 < 1000) {
			Multiplier_2 = Multiplier_2 * 10;
			send_LCD_commands(clr_Digit_Lines);
			setDigitPos();
		} return;
	}
	
	// set Voltage Down "-->" --------------------------------------------
	else if (Button_nr == 3 && UI_flag == 0) {
		if (Multiplier > 10) {
			Multiplier = Multiplier / 10;
			send_LCD_commands(clr_Digit_Lines);
			setDigitPos();
		} return;
	}
	
	// set Ampere Down "-->" ---------------------------------------------
	else if (Button_nr == 3 && UI_flag == 1) {
		if (Multiplier_2 > 1) {
			Multiplier_2 = Multiplier_2 / 10;
			send_LCD_commands(clr_Digit_Lines);
			setDigitPos();
		} return;
	}
	
	// UI switch ---------------------------------------------------------
	else if (Button_nr == 4) {
		send_LCD_commands(clr_Digit_Lines);
		if (UI_flag == FALSE) {
			UI_flag = TRUE;
			setDigitPos();
			return;
		}
		else {
			UI_flag = FALSE;
			setDigitPos();
		} return;		
	}
	
	//  "Recall" ---------------------------------------------------------
	else if (Button_nr == 5) {
		if (Button5_flag == 0){
		wr_SPI_buffer3(0x27, 0x03, 0x38);	// print "Memory"
		print_memory_nr(Memory_nr);
		Memory_flag = 0;
		Recall_flag = 1;
		Button5_flag = 1;
		flash_time = 5;			// flash-time for PrgNo
		return;
		}
		else{
			send_LCD_commands(clr_Memory);
			Button5_flag = 0;
			Button6_flag = 0;
			Memory_flag = 0;
			Recall_flag = 0;
			return;
		}
		
	}
	
	//  "Memory" ---------------------------------------------------------
	else if (Button_nr == 6) {
		if (Button6_flag == 0){
			wr_SPI_buffer3(0x27, 0x03, 0x38);	// print "Memory"
			print_memory_nr(Memory_nr);
			Button6_flag = 1;
			Memory_flag = 1;
			Recall_flag = 0;
			flash_time = 5;		// flash-time for PrgNo
			return;
		}
		else{
			send_LCD_commands(clr_Memory);
			Button5_flag = 0;
			Button6_flag = 0;
			Memory_flag = 0;
			Recall_flag = 0;
			return;
		}	
	}
	
	//  "Enter" ---------------------------------------------------------
	else if (Button_nr == 7 && Button7_flag == 0){
		if (Memory_flag == 0 && Recall_flag == 1){ 
			rd_UI_eeprom();
			uint8_t temp_flag = UI_flag;	// save flag
			UI_flag = 0;
			if (Ulimit >= 15000) {
				SPI_wr2(0xB1);			// Relais on  unregulated VDC 48.0V
			}
			else if (Ulimit <= 14500) {
				SPI_wr2(0xB0);			// Relais off  unregulated VDC 24.0V
			}
			set_Usoll(Ulimit);
			print_value(0x63, Ulimit);
			UI_flag = 1;
			set_Isoll(Ilimit);
			print_value(0x67, Ilimit);
			send_LCD_commands(clr_Memory);
			UI_flag = temp_flag;
			Memory_flag = 0;
			Recall_flag = 0;
			Button5_flag = 0;
			Button6_flag = 0;
			Button7_flag = 0;
			Hold_time = 0;				// time befor refresh UI-Limits
			Refresh_Ulimit = TRUE;
			Refresh_Ilimit = TRUE;
		}
		else if (Memory_flag == 1 && Recall_flag == 0){
			wr_UI_eeprom();
			send_LCD_commands(clr_Memory);
			Memory_flag = 0;
			Recall_flag = 0;
			Button5_flag = 0;
			Button6_flag = 0;
			Button7_flag = 0;
		}
	}
}

//*************************************************************************
// flaching Memory PrgNo
//*************************************************************************
void flash_PrgNo (void)
{
	if ((Memory_flag != 0 || Recall_flag != 0) && flash_time == 0){
		if (blinki_flag != 0){
			print_memory_nr(Memory_nr);		// print PrgNo
			blinki_flag = 0;
			flash_time = 3;
		}
		else{
			SPI_wr2(0x6B);					// clr PrgNo
			wr_SPI_buffer4(0x5D, 0x5D, 0x5D, 0x5D);
			blinki_flag = 1;
			flash_time = 3;
		}
	}
} 

//*************************************************************************
// save UI data to Memory
//*************************************************************************
void wr_UI_eeprom (void)
{
	uint16_t * eeAddr = (10 + (Memory_nr * 4));
	eeprom_write_word(eeAddr, Ulimit);
	eeprom_write_word(eeAddr+1, Ilimit);
}

//*************************************************************************
// read UI data from Memory
//*************************************************************************
void rd_UI_eeprom (void)
{
	uint16_t * eeAddr = (10 + (Memory_nr * 4));
	Ulimit = eeprom_read_word(eeAddr);
	Ilimit = eeprom_read_word(eeAddr+1);
}

//*************************************************************************
// press Button
//*************************************************************************
void pressButton (uint8_t Button_nr)
{
	if (buttonDebounce == 0) {
		if (Button_nr != Button_nr_old) {
			Button_nr_old = Button_nr;
			buttonDebounce = 25;
			buttonFunction(Button_nr);
		}
	}
}

//*************************************************************************
// less Button
//*************************************************************************
void lessButton (uint8_t Button_nr)
{
	if (Button_nr_old == Button_nr) {
		if (buttonDebounce == 0) {
			buttonDebounce = 25;
			Button_nr = 0;
			Button_nr_old = 0;
		}
	}
}

//*************************************************************************
// read Button Pin
//*************************************************************************
void readButton (void)
{
	uint8_t buttonState = 0;
	
	buttonState = PINC & (1<<Standby);
	if (buttonState == 0) {
		 pressButton(1);
		 return;
	} else lessButton(1);
	
	buttonState = PINC & (1<<Up);
	if (buttonState == 0) {
		pressButton(2);
		return;
	} else lessButton(2);
	
	buttonState = PINC & (1<<Down);
	if (buttonState == 0) {
		pressButton(3);
		return;
	} else lessButton(3);
	
	buttonState = PINC & (1<<UI);
	if (buttonState == 0) {
		pressButton(4);
		return;
	} else lessButton(4);
	
	buttonState = PIND & (1<<Recall);
	if (buttonState == 0) {
		pressButton(5);
		return;
	} else lessButton(5);
	
	buttonState = PINC & (1<<Memory);
	if (buttonState == 0) {
		pressButton(6);
		return;
	} else lessButton(6);
	
	buttonState = PINC & (1<<Enter);
	if (buttonState == 0) {
		pressButton(7);
		return;
	} else lessButton(7);
	
	// dec Debounce counter
	if (buttonDebounce >0)
	{
		buttonDebounce--;
	}
}

//*************************************************************************
// set UI "Aktive"
//*************************************************************************
void UI_control(void)
{
	uint8_t ReglerStatus = 0;
	static uint8_t ReglerStatus_old = 0;
	
	ReglerStatus = PIND & (1<<Regler);
	
	if (ReglerStatus == ReglerStatus_old || Standby_flag == 1)
	{
		return;
	}
	
	if (ReglerStatus != 0)
	{
		send_LCD_commands(Activ_A);
	}
	else
	{
		send_LCD_commands(Activ_V);
	}
	
	ReglerStatus_old = ReglerStatus;
}


//*************************************************************************
// init LCD
//*************************************************************************
void init_LCD (void)
{
	//_delay_ms(1000);
	soft_delay(1000);
	SPI_wr2(0xF0);						// clear LCD
	soft_delay(100);
	SPI_wr2(0xC1);						// LCD-Backlight on
	soft_delay(100);
	SPI_wr2(0xB0);						// Relais off  VCC 24.0V
	SPI_wr2(0xA1);						// Stanby on
	send_LCD_commands(Standby_on);
	Standby_flag = 1;
	send_LCD_commands(LCD_inits);
}

//*************************************************************************
// init UI_Limits
//*************************************************************************
void init_UIlimits (void)
{
	UI_flag = 0;						// print Ulimit
	read_Ulimit();
	print_value(0x63,Ulimit);
	UI_flag = 1;						// print Ilimit
	read_Ilimit();
	set_UI_Limits();
	set_Isoll(Ilimit);
	print_value(0x67,Ilimit);
	UI_flag = 0;
}

//*************************************************************************
// Main
//*************************************************************************
int main(void) {
	init_SPI();
	init_IO_Port();						// init Input and Output Port
	init_Timer0();						// FAN PWM
	init_Timer1();						// UI controll PWM
	init_Timer2();						// for ADC measurments
	init_Encoder();
	init_LCD();
	init_UIlimits();
	init_Interrupt();
	
			
	// Main loop ---------------------------------------------------------
	while(1) {	
		Measurement();
		save_print_UIlimit();
		UI_control();
		pull_encoder();
		readButton();
		flash_PrgNo();	
	}	
}