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

#define VERSION 199
#define BUILD   002

#define CALIB_COUNTS_PER_20V_DEFAULT 10022
#define CALIB_COUNTS_OFFSET_0V_DEFAULT 123
#define CALIB_COUNTS_PER_1200MA_DEFAULT 5758
#define CALIB_COUNTS_OFFSET_0MA_DEFAULT 118

#define CALIB_MAGIC_WORD 0xca1

#define F_CPU 8000000UL
#define ADC_AD0		PD4  
#define ADC_AD1		PD5	 
#define ADC_AD2		PD7	 
#define ADC_ADW		PB0	
#define FAN         PD6 
#define B_Standby	PC3	 
#define B_UI		PC1
#define B_Left      PC0
#define B_Right     PC2
#define B_Recall    PD1
#define B_Enter		PC4
#define B_Memory    PC5
#define Regulator   PD0
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

#define U_act_cmd       0x43
#define I_act_cmd       0x47
#define P_act_cmd       0x4B
#define U_limit_cmd     0x63
#define I_limit_cmd     0x67
#define Memory_nr_cmd   0x6B
#define Standby_Off_cmd 0xA0
#define Standby_On_cmd  0xA1
#define Relais_Off_24V  0xB0
#define Relais_On_48V   0xB1
#define Backlit_off_cmd 0xC0
#define Backlit_on_cmd  0xC1
#define LCD_Firmware_cmd  0xE0
#define Clear_Screen_cmd  0xF0
#define All_on_Screen_cmd 0xF1



// prototypes -------------------------------------------------------------
void soft_delay (uint16_t time_value);
void SOFT_SPI_init(void);
void init_SPI(void);
void SPI_wr2(unsigned char dataout);
void wr_SPI_buffer3(uint8_t data0, uint8_t data1, uint8_t data2);
void wr_SPI_buffer4(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3);
void wr_SPI_buffer5(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);
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

uint16_t calib_counts_per_20V    = CALIB_COUNTS_PER_20V_DEFAULT;
uint16_t calib_counts_offset_0V  = CALIB_COUNTS_OFFSET_0V_DEFAULT;
uint16_t calib_counts_per_1200mA = CALIB_COUNTS_PER_1200MA_DEFAULT;
uint16_t calib_counts_offset_0mA = CALIB_COUNTS_OFFSET_0MA_DEFAULT;

//LCD commands -----------------------------------------------------------
const uint8_t clr_Digit_Lines[] PROGMEM = {24, 0x24,0x07,0x1D, 0x24,0x06,0x15, 0x24,0x06,0x1D, 0x24,0x05,0x15, 
                                               0x27,0x01,0x1D, 0x27,0x01,0x15, 0x27,0x02,0x1D, 0x27,0x02,0x15};
    
const uint8_t Standby_on[]  PROGMEM = {10, Standby_On_cmd, 0x22,0x03,0x31, 0x20,0x05,0x1D, 0x27,0x03,0x1D};
const uint8_t Standby_off[] PROGMEM = {4, Standby_Off_cmd, 0x22,0x03,0x1E};
    
const uint8_t Degree_Symbol[] PROGMEM = {12, 0x23,0x02,0x31, 0x30,0x17,0x20, 0x30,0x26,0x38, 0x30,0x24,0x17};
    
const uint8_t Activ_V[] PROGMEM = {6, 0x20,0x05,0x32, 0x27,0x03,0x1D};	
const uint8_t Activ_A[] PROGMEM = {6, 0x27,0x03,0x32, 0x20,0x05,0x1D};
    
const uint8_t clr_Memory[] PROGMEM = {8, 0x27,0x03,0x15, Memory_nr_cmd,0x5D,0x5D,0x5D,0x5D};
    
const uint8_t LCD_inits[] PROGMEM = {60, 0x23,0x05,0x34, 0x23,0x06,0x34, 0x27,0x03,0x31, 0x27,0x01,0x34,
                                         0x23,0x02,0x31, 0x30,0x17,0x20, 0x30,0x26,0x38, 0x30,0x24,0x17,
                                         0x00,0x23,0x38, 0x04,0x20,0x38, 0x04,0x21,0x38, 0x04,0x27,0x34,
                                         0x23,0x05,0x31, 0x27,0x05,0x31, 0x24,0x06,0x38,
                                         U_act_cmd,0x5D,0x50,0x50,0x50,  I_act_cmd,0x50,0x50,0x50,0x50, P_act_cmd,0x50,0x50,0x50,0x50};

//*************************************************************************
// soft delay
//*************************************************************************
void soft_delay (uint16_t time_value)
{
    for (uint16_t i = 0; i < time_value; i++)
    {
        // 800*10 = 8000 Clock cycles ==> 1ms per inner loop
        for (uint16_t count = 0; count < 800; count++)
        {
            NOP(); // this inner loop takes about 10(!) clock cycles per iteration
        }
    }
}

//*************************************************************************
// SPI init
//*************************************************************************
void init_SPI(void)
{
    DDRB |= (1<<SPI_SS)|(1<<SPI_MOSI)|(1<<SPI_SCK); // SCK, MOSI and SS as outputs
    DDRB &= ~(1<<SPI_MISO);							// MISO as input
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

void wr_SPI_buffer5(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
    SPI_wr2(data0);
    SPI_wr2(data1);
    SPI_wr2(data2);
    SPI_wr2(data3);
    SPI_wr2(data4);
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
    DDRD |= ((1<<ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2));	// Multiplexer IC6 
    DDRB &= ~(1<<ADC_ADW);		// extern PCINT0 for Meassurement
    DDRB |= (1<<PIN1);			// PB1 OC1A	PWM Voltage
    DDRB |= (1<<PIN2);			// PB2 OC1B	PWM Ampere
    DDRC |= (1<<B_Standby);		// Key "Standby"
    DDRD &= ~(1<<B_Recall);		// Key "Recall"
    DDRC &= ~(1<<B_Memory);		// Key "Memory"
    DDRC &= ~(1<<B_Enter);		// Key "Enter"
    DDRD &= ~(1<<Regulator);	// Regler
    DDRD &= ~(1<<Enc_A);		// Encoder Pin A
    DDRD &= ~(1<<Enc_B);		// Encoder Pin B
    DDRC &= ~(1<<B_UI);			// Key "U/I"
    DDRC &= ~(1<<B_Left);		// Key "<-"
    DDRC &= ~(1<<B_Right);		// Key "->"
    DDRD |= (1<<PIN6);			// FAN PWM out
    
    PORTB |= (1<<ADC_ADW);		// Pullup enabled
    PORTC |= (1<<B_UI);			// Pullup UI switch
    PORTD |= (1<<Regulator);	// Pullup Regler
    PORTD |= (1<<B_Recall);		// Pullup Key Recall
    PORTC |= (1<<B_Standby);	// Pullup Key Standby
    PORTC |= (1<<B_Memory);		// Pullup Key "Memory"
    PORTC |= (1<<B_Enter);		// Pullup Key "Enter"
    PORTD |= (1<<Enc_A);		// Pullup Encoder Pin A
    PORTD |= (1<<Enc_B);		// Pullup Encoder Pin B
    PORTC |= (1<<B_Left);		// Pullup Key "<-"
    PORTC |= (1<<B_Right);		// Pullup Key "->"
    
}

//*************************************************************************
// print values on screen
//*************************************************************************
void print_value(uint8_t cmd_index, uint16_t value)
{
    uint8_t print_leading_space = 0;
    
    if (cmd_index == U_act_cmd || cmd_index == U_limit_cmd)
    {
        value /= 10;
        print_leading_space = 1;
    }

    digit[0] = (value/1000)+0x50;
    if ((print_leading_space || cmd_index == P_act_cmd) && digit[0] == 0x50)
    {
        digit[0] = 0x5D;	// if fist digit 0 than print "Space"
    }
    value %= 1000;
    digit[1] = (value/100)+0x50;
    value %= 100;
    digit[2] = (value/10)+0x50;
    value %= 10;
    digit[3] = (value/1)+0x50;
                
    wr_SPI_buffer5(cmd_index, digit[3], digit[2], digit[1], digit[0]);
}

void print_value_signed(uint8_t cmd_index, int16_t value)
{
    if (value < 0)
    {
        value = -value;
        digit[0] = 0x5F;
    }
    else
    {
        digit[0] = 0x5D;
    }

    value %= 1000;
    digit[1] = (value/100)+0x50;
    if (digit[1]==0x50) digit[1] = 0x5D;
    value %= 100;
    digit[2] = (value/10)+0x50;
    if (digit[2]==0x50) digit[2] = 0x5D;
    value %= 10;
    digit[3] = (value/1)+0x50;
    
    wr_SPI_buffer5(cmd_index, digit[3], digit[2], digit[1], digit[0]);
}

//*************************************************************************
// print Memory number
//*************************************************************************
void print_memory_nr (uint8_t number)
{
    digit[0] = (number/10) + 0x50;
    digit[1] = (number%10) + 0x50;
    wr_SPI_buffer5(Memory_nr_cmd, 0x5D, 0x5D, digit[1], digit[0]);
}

//*************************************************************************
// print temperature
//*************************************************************************
void print_degree(uint16_t value)
{
    value /= 10;
    digit[0] = (value/100)+0x50;
    value %= 100;
    digit[1] = (value/10)+0x50;
    value %= 10;
    digit[2] = (value+0x50);
    digit[3] = 0x52;					// "2" special for degree symbol
    wr_SPI_buffer5(P_act_cmd, digit[3], digit[2], digit[1],digit[0]);
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
      TCCR2B &= ~((1<<CS00) | (1<<CS01));	// stop timer and disable ADC
      PORTD |= ((1 << ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2));
      Timer2_run_flag = 0;
      }
}

//*************************************************************************
// Measurement interrupt from ADW-Pin
//*************************************************************************
ISR(PCINT0_vect)
{
    // falling edge
    if (!(PINB & (1<<PB0))) {
        TCCR2B &= ~((1<<CS00) | (1<<CS01));	// stop timer and disable ADC
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
    //static int8_t count_v;
    
    if (Standby_flag == 1 || mess_time <= Digital_offset_v)
    {
        print_value(U_act_cmd, 0);
        return;
    }
    
    int16_t result = (((int32_t)mess_time - Digital_offset_v) * 198358) >> 16;

#if 0    
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
#endif
    
    print_value(U_act_cmd, result);
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
        print_value(I_act_cmd, 0);
        Iist = 0;
        return;
    }
    uint16_t result = (((uint32_t)mess_time - Digital_offset_a) * 15839) >> 16;	// 15805
    print_value(I_act_cmd, result);
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
        print_value(P_act_cmd, watt);	
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
        // start ADC measurement
        PORTD |= (1 << ADC_AD2);
        PORTD &= ~((1 << ADC_AD0) | (1<<ADC_AD1));
        start_Measurement_timer();
        return;			
    }
        
    // print measurement result ---------------------------------------
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
            SPI_wr2(Relais_On_48V);
        }
        else if (Ulimit <= 14500) {
            SPI_wr2(Relais_Off_24V);
        }
        
        // set Usoll
        set_Usoll(Ulimit);
        print_value(U_act_cmd, Ulimit);

        Hold_time = 10;				// time before refresh UI-Limits
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
        print_value(I_act_cmd, Ilimit);
        
        Hold_time = 10;
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
        print_value(U_limit_cmd, Ulimit);
        Refresh_Ulimit = FALSE;
    }
    
    if (Hold_time == 0 && Refresh_Ilimit == TRUE)
    {
        UI_flag = 1;
        save_Ilimit(Ilimit);
        print_value(I_limit_cmd, Ilimit);
        Refresh_Ilimit = FALSE;
    }
    
    UI_flag = Temp_flag;			// restore UI_flag
}

//*************************************************************************
// set Usoll (0 - 30.000mV)
//*************************************************************************
void set_Usoll (uint16_t Usoll_value)
{
    uint32_t calib_factor = (((uint32_t) calib_counts_per_20V << 16) + 10000) / 20000;
    OCR1A = calib_counts_offset_0V + ((Usoll_value * calib_factor) >> 16);
}


//*************************************************************************
// set Isoll
//*************************************************************************
void set_Isoll (int Isoll_value)
{
    uint32_t calib_factor = (((uint32_t) calib_counts_per_1200mA << 16) + 600) / 1200;
    OCR1B = calib_counts_offset_0mA + ((Isoll_value * calib_factor) >> 16);
}

//*************************************************************************
// save Ulimit (EEPROM Adr. 0x000)
//*************************************************************************
void save_Ulimit (uint16_t Ulimit)
{
    uint16_t *eeAddr = (uint16_t*) 0;
    eeprom_write_word(eeAddr, Ulimit);
}

//*************************************************************************
// read Ulimit (EEPROM Adr. 0x000)
//*************************************************************************
void read_Ulimit (void)
{
    uint16_t *eeAddr =  (uint16_t*) 0;
    Ulimit = eeprom_read_word(eeAddr);
}

//*************************************************************************
// save Ilimit (eeprom Adr. 0x002)
//*************************************************************************
void save_Ilimit (uint16_t Ilimit)
{
    uint16_t *eeAddr = (uint16_t*) 2;
    eeprom_write_word(eeAddr, Ilimit);
}

//*************************************************************************
// read Ilimit (eeprom Adr. 0x002)
//*************************************************************************
void read_Ilimit (void)
{
    uint16_t *eeAddr = (uint16_t*) 2;
    Ilimit = eeprom_read_word(eeAddr);
}

//*************************************************************************
// read magic_calib (eeprom Adr. 0x004)
//*************************************************************************
uint8_t is_calibration_valid(void)
{
    uint16_t *eeAddr = (uint16_t*) 4;
    uint16_t magic = eeprom_read_word(eeAddr);
    return magic == CALIB_MAGIC_WORD;
}

//*************************************************************************
// set magic_calib (eeprom Adr. 0x004)
//*************************************************************************
void set_calibration_valid(void)
{
    uint16_t *eeAddr = (uint16_t*) 4;
    eeprom_write_word(eeAddr, CALIB_MAGIC_WORD);
}

//*************************************************************************
// set calibration data (eeprom Adr. 0x006, 0x008, 0x00A, 0x00C)
//*************************************************************************
void save_calibration_data(void)
{
    uint16_t *eeAddr = (uint16_t*) 6;
    eeprom_write_word(eeAddr+0, calib_counts_offset_0V);
    eeprom_write_word(eeAddr+1, calib_counts_per_20V);
    eeprom_write_word(eeAddr+2, calib_counts_offset_0mA);
    eeprom_write_word(eeAddr+3, calib_counts_per_1200mA);
    
    set_calibration_valid();
}

//*************************************************************************
// read calibration data (eeprom Adr. 0x006, 0x008, 0x00A, 0x00C)
//*************************************************************************
void read_calibration_data(void)
{
    uint16_t *eeAddr = (uint16_t*) 6;
    calib_counts_offset_0V  = eeprom_read_word(eeAddr);
    calib_counts_per_20V    = eeprom_read_word(eeAddr+1);
    calib_counts_offset_0mA = eeprom_read_word(eeAddr+2);
    calib_counts_per_1200mA = eeprom_read_word(eeAddr+3);
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
        print_value(U_act_cmd, 0);
        print_value(I_act_cmd, 0);
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
            SPI_wr2(Relais_On_48V);
        }
        else if (Ulimit <= 14500) {
            SPI_wr2(Relais_Off_24V);
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
                SPI_wr2(Relais_On_48V);
            }
            else if (Ulimit <= 14500) {
                SPI_wr2(Relais_Off_24V);
            }
            set_Usoll(Ulimit);
            print_value(U_limit_cmd, Ulimit);
            UI_flag = 1;
            set_Isoll(Ilimit);
            print_value(I_limit_cmd, Ilimit);
            send_LCD_commands(clr_Memory);
            UI_flag = temp_flag;
            Memory_flag = 0;
            Recall_flag = 0;
            Button5_flag = 0;
            Button6_flag = 0;
            Button7_flag = 0;
            Hold_time = 0;				// time before refresh UI-Limits
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
            wr_SPI_buffer5(Memory_nr_cmd, 0x5D, 0x5D, 0x5D, 0x5D);
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
    uint16_t *eeAddr = (uint16_t*) (0x10 + (Memory_nr * 4));
    eeprom_write_word(eeAddr, Ulimit);
    eeprom_write_word(eeAddr+1, Ilimit);
}

//*************************************************************************
// read UI data from Memory
//*************************************************************************
void rd_UI_eeprom (void)
{
    uint16_t *eeAddr = (uint16_t*) (0x10 + (Memory_nr * 4));
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
    
    buttonState = PINC & (1<<B_Standby);
    if (buttonState == 0) {
         pressButton(1);
         return;
    } else lessButton(1);
    
    buttonState = PINC & (1<<B_Left);
    if (buttonState == 0) {
        pressButton(2);
        return;
    } else lessButton(2);
    
    buttonState = PINC & (1<<B_Right);
    if (buttonState == 0) {
        pressButton(3);
        return;
    } else lessButton(3);
    
    buttonState = PINC & (1<<B_UI);
    if (buttonState == 0) {
        pressButton(4);
        return;
    } else lessButton(4);
    
    buttonState = PIND & (1<<B_Recall);
    if (buttonState == 0) {
        pressButton(5);
        return;
    } else lessButton(5);
    
    buttonState = PINC & (1<<B_Memory);
    if (buttonState == 0) {
        pressButton(6);
        return;
    } else lessButton(6);
    
    buttonState = PINC & (1<<B_Enter);
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
    uint8_t status = 0;
    static uint8_t status_old = 0;
    
    status = PIND & (1<<Regulator);
    
    if (status == status_old || Standby_flag == 1)
    {
        return;
    }
    
    if (status != 0)
    {
        send_LCD_commands(Activ_A);
    }
    else
    {
        send_LCD_commands(Activ_V);
    }
    
    status_old = status;
}

//*************************************************************************
// init LCD
//*************************************************************************
void display_fw_version(void)
{
    wr_SPI_buffer3(0x23, 0x05, 0x19);			// clr "V"
    wr_SPI_buffer3(0x27, 0x03, 0x1E);			// clr "A"
    wr_SPI_buffer3(0x23, 0x03, 0x1E);			// clr "W"
    wr_SPI_buffer3(0x23, 0x06, 0x1B);		    // clr "." in V-value
    wr_SPI_buffer3(0x27, 0x01, 0x19);		    // clr "." in A value
    wr_SPI_buffer3(0x24, 0x06, 0x15);		    // clr "_" in V value

    print_value(U_act_cmd, VERSION*10);
    print_value(I_act_cmd, BUILD);
    wr_SPI_buffer5(P_act_cmd, 0x5d, 0x5d, 0x5d, 0x5d); // No Power display
}

//*************************************************************************
// init LCD
//*************************************************************************
void init_LCD (void)
{
    soft_delay(1000);
    SPI_wr2(Clear_Screen_cmd);
    soft_delay(100);
    SPI_wr2(Backlit_on_cmd);
    soft_delay(100);
    SPI_wr2(Relais_Off_24V);
    SPI_wr2(Standby_On_cmd);
    send_LCD_commands(LCD_inits);
    
    display_fw_version();
    soft_delay(3000);

    send_LCD_commands(Standby_on);
    Standby_flag = 1;
}

//*************************************************************************
// init UI_Limits
//*************************************************************************
void init_UIlimits (void)
{
    UI_flag = 0;						// print Ulimit
    read_Ulimit();
    print_value(U_limit_cmd, Ulimit);
    UI_flag = 1;						// print Ilimit
    read_Ilimit();
    set_UI_Limits();
    set_Isoll(Ilimit);
    print_value(I_limit_cmd, Ilimit);
    UI_flag = 0;
}

int16_t calibration_user(uint16_t is_U)
{
    uint16_t ocrval_start = is_U ? OCR1A : OCR1B;
    int16_t ocrval_diff = 0;
    do
    {
        pull_encoder();
        int diff = encode_read4();
        ocrval_diff += diff;
        if (is_U)
            OCR1A = ocrval_start + ocrval_diff ;
        else
            OCR1B = ocrval_start + ocrval_diff ;
        
        print_value_signed(I_act_cmd, ocrval_diff);
    } while ((PINC & (1<<B_Enter)) != 0);
    
    return ocrval_start + ocrval_diff;
}        

void wait_for_standby_off(void)
{
    do 
    {
        wr_SPI_buffer3(0x22,0x03,0x31);
        soft_delay(200);
        wr_SPI_buffer3(0x22,0x03,0x1E);
        soft_delay(200);
    } while ((PINC & (1<<B_Standby)) != 0);
    send_LCD_commands(Standby_off);
}    

//*************************************************************************
// Check calibration
//*************************************************************************
void startup_calibration(void)
{
    if (is_calibration_valid())
    {
        read_calibration_data();
    }
    
    if ((!is_calibration_valid()) || ((PINC & ((1<<B_Left) | (1<<B_Right))) == 0))
    {
        const uint16_t u_ref_low = 5000;
        const uint16_t u_ref_high = 25000;
        const uint16_t i_ref_low = 300;     // use a 12V/5W car bulb as load
        const uint16_t i_ref_high = 1500;   // use a 12V/21W car bulb as load
        
        // Limit current to 100mA; calibration should be done in idle anyway
        set_Isoll(100);
        print_value(I_limit_cmd, 100);

        uint16_t u_refs[] = { u_ref_low, u_ref_high };
        uint16_t i_refs[] = { i_ref_low, i_ref_high };
        uint16_t ocrs[] = { 0, 0 };

        wr_SPI_buffer5(P_act_cmd, 0x5D, 0x5D, 0x5D, 0x5D); // Clr P display
            
        // Voltage calibration
        for (uint8_t i=0; i<2; i++)
        {
            send_LCD_commands(Standby_on);
            wr_SPI_buffer5(I_act_cmd, 0x5D, 0x5D, 0x5D, 0x5D); // Clr I display

            set_Usoll(u_refs[i]);
            wr_SPI_buffer5(U_act_cmd, 0x51+i, 0x5B, 0x5A ,0x5C);	// "CAL1/2"
            wr_SPI_buffer3(0x23, 0x05, 0x34);                       //add "V"
            print_value(U_limit_cmd, u_refs[i]);

            wait_for_standby_off();
            send_LCD_commands(Activ_V);
            wr_SPI_buffer3(0x27, 0x02, 0x38);                       // "_"

            if (u_refs[i] >= 15000)
                SPI_wr2(Relais_On_48V);
            else
                SPI_wr2(Relais_Off_24V);

            soft_delay(200);
            ocrs[i] = calibration_user(1);
        }

        calib_counts_per_20V = ocrs[1] - ocrs[0];
        calib_counts_offset_0V = ocrs[0] - calib_counts_per_20V / 4;   // subtract 5V to get offset @0V

        //wr_SPI_buffer5(U_act_cmd, 0x5D, 0x5B, 0x5A, 0x5C);  // "CAL"
        //print_value(I_act_cmd, calib_counts_per_20V/2);     // Print Counts per 10V (we have only 4 digits!) (~5005)
        //print_value(P_act_cmd, calib_counts_offset_0V);		// Print offset (~122)
        //soft_delay(5000);

        // Current calibration
        set_Usoll(12000);           // 12V
        SPI_wr2(Relais_Off_24V);
        print_value(U_limit_cmd, 12000);

        for (uint8_t i=0; i<2; i++)
        {
            send_LCD_commands(Standby_on);
            wr_SPI_buffer5(I_act_cmd, 0x5D, 0x5D, 0x5D, 0x5D);      // Clr I display
            wr_SPI_buffer3(0x23, 0x05, 0x19);                       // remove "V"
            wr_SPI_buffer3(0x27, 0x03, 0x31);                       // add "A"

            set_Isoll(i_refs[i]);
            wr_SPI_buffer5(U_act_cmd, 0x53+i, 0x5B, 0x5A ,0x5C);	// "CAL3/4"
            print_value(I_limit_cmd, i_refs[i]);

            wait_for_standby_off();
            send_LCD_commands(Activ_A);

            soft_delay(200);
            ocrs[i] = calibration_user(0);
        }

        calib_counts_per_1200mA = ocrs[1] - ocrs[0];
        calib_counts_offset_0mA = ocrs[0] - calib_counts_per_1200mA / 4;   // subtract 300mA get offset @0mA

        //wr_SPI_buffer5(U_act_cmd, 0x5D, 0x5B, 0x5A, 0x5C);  // "CAL"
        //print_value(I_act_cmd, calib_counts_per_1200mA);    // Print Counts per 1200mA) (~6000)
        //print_value(P_act_cmd, calib_counts_offset_0mA);    // Print offset (~120)
        //soft_delay(5000);

        save_calibration_data();

    }

    send_LCD_commands(Standby_on);
    
    wr_SPI_buffer3(0x23, 0x05, 0x34);			// set "V"
    wr_SPI_buffer3(0x27, 0x03, 0x31);			// set "A"
    wr_SPI_buffer3(0x23, 0x03, 0x31);			// set "W"
    wr_SPI_buffer3(0x23, 0x06, 0x34);	        // set "." in V-value
    wr_SPI_buffer3(0x27, 0x01, 0x34);		    // set "." in A value
    wr_SPI_buffer3(0x24, 0x06, 0x38);		    // set "_" in V value

    return;
}

//*************************************************************************
// Main
//*************************************************************************
int main(void) 
{
    init_SPI();
    init_IO_Port();     // init Input and Output Port
    init_Timer0();      // FAN PWM
    init_Timer1();      // UI control PWM
    init_Timer2();      // for ADC measurements
    init_Encoder();
    init_LCD();
    
    startup_calibration();

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