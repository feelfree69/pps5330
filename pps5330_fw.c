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
#define BUILD   005

#define CALIB_COUNTS_PER_20V_DEFAULT 10022
#define CALIB_COUNTS_OFFSET_0V_DEFAULT 123
#define CALIB_COUNTS_PER_1200MA_DEFAULT 5758
#define CALIB_COUNTS_OFFSET_0MA_DEFAULT 118

#define CALIB_MAGIC_WORD   0x0ca1
#define DISPLAY_MAGIC_WORD 0xcc00

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
#define Enc_A       PD3
#define Enc_B       PD2
#define PHASE_A     (PIND & 1<<Enc_A)
#define PHASE_B     (PIND & 1<<Enc_B)

#define TRUE 	1
#define FALSE 	0

#define Vref 2.50

#define SPI_MOSI    3     // PC5 = SPI MOSI
#define SPI_MISO    4     // PC6 = SPI MISO
#define SPI_SCK     5     // PC7 = SPI SCK
#define SPI_SS      2     // PC4 = SPI SS

#define NOP() asm volatile ("nop" ::) 

#define MEAS_TYPE_U 0
#define MEAS_TYPE_I 1
#define MEAS_TYPE_T_TRANSFORMER 2
#define MEAS_TYPE_T_HEATSINK 3

#define UI_FLAG_U 0
#define UI_FLAG_I 1

#define BUTTON_STANDBY  0
#define BUTTON_LEFT     1
#define BUTTON_RIGHT    2
#define BUTTON_UI       3
#define BUTTON_RECALL   4
#define BUTTON_MEMORY   5
#define BUTTON_ENTER    6
#define BUTTON_COUNT    7

#define MEAS_PHASE_CLEAR_INTEGRATOR    0
#define MEAS_PHASE_LOAD_INTEGRATOR     1
#define MEAS_PHASE_MEASURE_INTEGRATOR  2
#define MEAS_PHASE_READ_RESULT         3


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
void print_U_result(uint16_t meas_time);
void print_I_result(uint16_t meas_time);
void print_T_transformer_result(uint16_t meas_time);
void print_T_heatsink_result(uint16_t meas_time);
void init_Encoder(void);
void pull_encoder(void);
int8_t encode_read4(void);
void Measurement(void);
void set_UI_Limits(void);
void save_print_UIlimit (void);
void set_Usoll (int16_t value);
void set_Isoll (int16_t value);
void save_eeprom_ulimit(uint16_t ulim);
void save_eeprom_ilimit(uint16_t ilim);
uint16_t read_eeprom_ulimit(void);
uint16_t read_eeprom_ilimit(void);
void wr_eeprom_memory(uint8_t nr, int16_t ulim, int16_t ilim);
void rd_eeprom_memory(uint8_t nr, int16_t *ulim, int16_t *ilim);
void setDigitPos (void);
void buttonFunction (uint8_t Button_nr);
void pressButton (uint8_t Button_nr);
void releaseButton (uint8_t Button_nr);
void readButtons(void);
void UI_control(void);
void init_LCD (void);
void init_UIlimits (void);
void send_LCD_commands (const uint8_t Com_Adr[]);
void print_degree(uint16_t value);
void set_memory_nr (void);
void flash_PrgNo(void);
void displaySettings(void);
void printContrast(uint8_t contrast);


// global variables ------------------------------------------------------
int8_t enc_delta = 0;          // -128 ... 127
int8_t enc_last;
uint8_t enc_changed = FALSE;
volatile uint8_t timer_max = 0;
volatile uint8_t counter_h = 0;
volatile uint8_t ADW_flag = 0;
uint8_t Meas_phase = 0;
uint8_t Meas_type = MEAS_TYPE_U;
volatile uint8_t Timer2_run_flag = 0;
uint8_t Standby_flag = 0;
int16_t Ulimit = 0;
int16_t Ilimit = 0;
uint16_t Multiplier_U = 1000;
uint16_t Multiplier_I = 100;
uint8_t UI_flag = 0;
uint8_t Hold_time = 0;
uint8_t Refresh_Ulimit = FALSE;
uint8_t Refresh_Ilimit = FALSE;
uint8_t Memory_flag = FALSE;
uint8_t Memory_nr = 0;
uint8_t Recall_flag = FALSE;
uint8_t flash_time = 5;
uint8_t blinki_flag = 0;
int16_t U_meas = 0;
int16_t I_meas = 0;
uint32_t buttonPressDuration[BUTTON_COUNT];
uint32_t buttonReleaseDuration[BUTTON_COUNT];

uint16_t calib_counts_per_20V    = CALIB_COUNTS_PER_20V_DEFAULT;
uint16_t calib_counts_offset_0V  = CALIB_COUNTS_OFFSET_0V_DEFAULT;
uint16_t calib_counts_per_1200mA = CALIB_COUNTS_PER_1200MA_DEFAULT;
uint16_t calib_counts_offset_0mA = CALIB_COUNTS_OFFSET_0MA_DEFAULT;

// LCD commands

#define U_act_cmd         0x43
#define I_act_cmd         0x47
#define P_act_cmd         0x4B
#define U_limit_cmd       0x63
#define I_limit_cmd       0x67
#define Memory_nr_cmd     0x6B
#define Standby_Off_cmd   0xA0
#define Standby_On_cmd    0xA1
#define Relais_Off_24V    0xB0
#define Relais_On_48V     0xB1
#define Backlit_off_cmd   0xC0
#define Backlit_on_cmd    0xC1
#define LCD_Contrast_cmd  0xD8
#define LCD_Firmware_cmd  0xE0
#define Clear_Screen_cmd  0xF0
#define All_on_Screen_cmd 0xF1

#define set_U1_underline 0x24,0x07,0x32
#define set_U2_underline 0x24,0x06,0x38
#define set_U3_underline 0x24,0x06,0x32
#define set_U4_underline 0x24,0x05,0x38

#define clr_U_underlines 0x24,0x07,0x1D, 0x24,0x06,0x15, 0x24,0x06,0x1D, 0x24,0x05,0x15

#define set_I1_underline 0x27,0x01,0x32
#define set_I2_underline 0x27,0x01,0x38
#define set_I3_underline 0x27,0x02,0x32
#define set_I4_underline 0x27,0x02,0x38

#define clr_I_underlines 0x27,0x01,0x1D, 0x27,0x01,0x15, 0x27,0x02,0x1D, 0x27,0x02,0x15

#define set_V_symbol 0x23,0x05,0x34
#define clr_V_symbol 0x23,0x05,0x19
#define set_A_symbol 0x27,0x03,0x31
#define clr_A_symbol 0x27,0x03,0x1E
#define set_W_symbol 0x23,0x03,0x31
#define clr_W_symbol 0x23,0x03,0x1E

#define set_V_point 0x23,0x06,0x34
#define clr_V_point 0x23,0x06,0x1B
#define set_A_point 0x27,0x01,0x34
#define clr_A_point 0x27,0x01,0x19
#define set_W_point 0x23,0x02,0x31
#define clr_W_point 0x23,0x02,0x1E

#define set_U_limit 0x04,0x20,0x38
#define set_I_limit 0x23,0x05,0x31

#define set_V_Ulimit 0x04,0x21,0x38
#define set_A_Ilimit 0x27,0x05,0x31

#define set_point_ulimit 0x00,0x23,0x38
#define set_point_ilimit 0x04,0x27,0x34

#define set_memory 0x27,0x03,0x38
#define clr_memory 0x27,0x03,0x15

#define set_standby 0x22,0x03,0x31
#define clr_standby 0x22,0x03,0x1E

#define set_active_V 0x20,0x05,0x32
#define clr_active_V 0x20,0x05,0x1D

#define set_active_A 0x27,0x03,0x32
#define clr_active_A 0x27,0x03,0x1D

#define set_overtemp 0x22,0x01,0x34
#define clr_overtemp 0x22,0x01,0x11

//LCD commands -----------------------------------------------------------

const uint8_t clr_Digit_Lines[] PROGMEM = {24, clr_U_underlines, clr_I_underlines};
    
const uint8_t Standby_on[]  PROGMEM = {10, Standby_On_cmd, set_standby, 0x20,0x05,0x1D, 0x27,0x03,0x1D};
const uint8_t Standby_off[] PROGMEM = {4, Standby_Off_cmd, clr_standby};
    
//const uint8_t Degree_Symbol[] PROGMEM = {12, 0x23,0x02,0x31, 0x30,0x17,0x20, 0x30,0x26,0x38, 0x30,0x24,0x17};
 
const uint8_t Activ_V[] PROGMEM = {6, set_active_V, clr_active_A};
const uint8_t Activ_A[] PROGMEM = {6, set_active_A, clr_active_V};

const uint8_t clr_Memory[] PROGMEM = {8, clr_memory, Memory_nr_cmd,0x5D,0x5D,0x5D,0x5D};

//const uint8_t LCD_inits[] PROGMEM = {60, set_V_symbol,   set_V_point,    set_A_symbol,   set_A_point,
//                                         0x00,0x23,0x38, set_U_limit,    set_V_Ulimit,   0x04,0x27,0x34,
//                                         set_I_limit,    set_V_Ulimit,   set_U2_underline,


const uint8_t LCD_inits[] PROGMEM = {54, set_V_symbol, set_V_point,
                                         set_A_symbol, set_A_point,
                                         set_W_symbol, set_W_point,
                                         set_point_ulimit,
                                         set_U_limit, set_V_Ulimit, set_point_ilimit,
                                         set_I_limit, set_A_Ilimit, 
                                         set_U2_underline,    
                                         U_act_cmd,0x5D,0x50,0x50,0x50,  
                                         I_act_cmd,0x50,0x50,0x50,0x50, 
                                         P_act_cmd,0x50,0x50,0x50,0x50};

const uint8_t restore_unit_display[] PROGMEM = {15, set_V_symbol, set_A_symbol, set_W_symbol, set_V_point, set_A_point, set_W_point };
const uint8_t clr_unit_display[]     PROGMEM = {15, clr_V_symbol, clr_A_symbol, clr_W_symbol, clr_V_point, clr_A_point, clr_W_point };

const uint8_t clr_all_digits[] PROGMEM = { 15, U_act_cmd,0x5D, 0x5D, 0x5D, 0x5D,
                                               I_act_cmd,0x5D, 0x5D, 0x5D, 0x5D,
                                               P_act_cmd,0x5D, 0x5D, 0x5D, 0x5D };

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
    uint8_t digit[4];
    if (cmd_index == U_act_cmd || cmd_index == U_limit_cmd)
    {
        value /= 10;
        print_leading_space = 1;
    }

    digit[0] = (value/1000)+0x50;
    if ((print_leading_space || cmd_index == P_act_cmd) && digit[0] == 0x50)
    {
        digit[0] = 0x5D;	// if first digit 0 than print "Space"
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
    uint8_t digit[4];
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
    wr_SPI_buffer5(Memory_nr_cmd, 0x5D, 0x5D, (number%10)+0x50, (number/10)+0x50);
}

//*************************************************************************
// print temperature
//*************************************************************************
void print_degree(uint16_t value) // 9700
{
    value /= 100;                 // 97
    wr_SPI_buffer5(Memory_nr_cmd, 0x5D, 0x5D,(value%10)+0x50, (value/10)+0x50);
}

//-------------------------------------------------------------------------
// init extern INT0
//-------------------------------------------------------------------------
void init_Interrupt(void)
{
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);
    sei();                  // enable Global Interrupts 
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
    TCCR0A |= (1<<COM0A1) | (1<<WGM00) | (1<<WGM01);	// Set OC2B at bottom, clear OC2BFdegree
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
        
    if (Meas_phase == MEAS_PHASE_READ_RESULT) {
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
    
    Timer2_run_flag = TRUE;  
}

//*************************************************************************
// Timer2 Overflow interrupt for measurement results
//*************************************************************************
ISR (TIMER2_OVF_vect)
{   
    OCR2B++;
  
    if (OCR2B == 48 && ADW_flag == 0)
    {
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
    if (!(PINB & (1<<PB0)))
    {
        TCCR2B &= ~((1<<CS00) | (1<<CS01));	// stop timer and disable ADC
        PORTD |= ((1 << ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2));
        Timer2_run_flag = 0;
    }
}

void print_P_calculation(void)
{
    print_value(P_act_cmd, ((uint32_t) U_meas * I_meas)/10000);
}

//*************************************************************************
// print U_measurement result
//*************************************************************************
void print_U_result(uint16_t meas_time)
{
    #define Digital_offset_v 163
    
    if (Standby_flag == 1 || meas_time <= Digital_offset_v)
    {
        print_value(U_act_cmd, 0);
        print_value(P_act_cmd, 0);
        return;
    }
    
    U_meas = (((int32_t)meas_time - Digital_offset_v) * 189774) >> 16;

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
    
    print_value(U_act_cmd, U_meas);

    print_P_calculation();
}

//*************************************************************************
// print I_measurement result
//*************************************************************************
void print_I_result(uint16_t meas_time)
{
    #define Digital_offset_a 260
    
    if (Standby_flag == 1 || meas_time <= Digital_offset_a) 
    {
        print_value(I_act_cmd, 0);
        I_meas = 0;
        return;
    }
    I_meas = (((uint32_t)meas_time - Digital_offset_a) * 15725) >> 16;
    print_value(I_act_cmd, I_meas);

    print_P_calculation();
}

//*************************************************************************
// print T measurement result (heat sink) and set Fan PWM/Overtemp alarm
//*************************************************************************
void print_T_heatsink_result(uint16_t meas_time)
{	
    #define Digital_offset_b 6000  // Calibration to be done
    
    uint16_t result = (((uint32_t)meas_time - Digital_offset_b) * 198902) >> 16;

    if (! (Memory_flag || Recall_flag))
    {
        print_degree(result);
    }        

    result /= 100;
        
    // heat alert
    if (result >= 75)
    {
        wr_SPI_buffer4(Standby_On_cmd, set_standby);
        Standby_flag = 1;
        return;
    }
    else if (result >= 70)
    {
        wr_SPI_buffer3(set_overtemp);
        OCR0A = 255;		// set max Fan PWM
        return;
    }
    else if (result < 69)
    {
        wr_SPI_buffer3(clr_overtemp);
        
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
// print T measurement result (transformer)
//*************************************************************************
void print_T_transformer_result(uint16_t meas_time)
{	
    #define Digital_offset_b 6000  // Calibration to be done (shared with T1?)
    
    uint16_t result = (((uint32_t)meas_time - Digital_offset_b) * 198902) >> 16;
    
    if (! (Memory_flag || Recall_flag))
    {
        print_degree(result);
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
    if( PHASE_B ) new ^= 1;     // convert gray to binary
    enc_last = new;             // power on state
    enc_delta = 0;
}

// poll Encoder-Port  ----------------------------------------------------
void pull_encoder(void)
{
    int8_t new, diff;
    new = 0;
    if( PHASE_A ) new = 3;
    if( PHASE_B ) new ^= 1;             // convert gray to binary
    diff = enc_last - new;              // difference last - new
    if( diff & 1 ) {                    // bit 0 = value (1)
        enc_last = new;                 // store new as next last
        enc_delta += (diff & 2) - 1;    // bit 1 = direction (+/-)
    }
}

// read Encoder ----------------------------------------------------------
int8_t encode_read4(void)               // read fou- step-encoder
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
    static uint8_t counter = 0;
    
    // Abort when a measuring phase is running -----------------------
    if (Timer2_run_flag == 1)
    {
        return;
    }
    
    #define AD_Input_Mask          ((1<<ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2))
    #define AD_input_U               0
    #define AD_input_I              (1<<ADC_AD0)                   
    #define AD_input_T_Transformer  (1<<ADC_AD1)                   
    #define AD_input_T_Heatsink     ((1<<ADC_AD0) | (1<<ADC_AD1))   
    #define AD_input_2V5            (1<<ADC_AD2)                   
    #define AD_input_Clear         ((1<<ADC_AD0) | (1<<ADC_AD2))   

    // clear integrator ----------------- ----------------------------
    if (Meas_phase == MEAS_PHASE_CLEAR_INTEGRATOR)
    {
        Meas_phase = MEAS_PHASE_LOAD_INTEGRATOR;

        // clear ADC
        //PORTD = (PORTD & ~AD_Input_Mask) | AD_input_Clear;
        PORTD |= (1 << ADC_AD0) | (1 << ADC_AD2);
        PORTD &= ~(1<<ADC_AD1);
        start_Measurement_timer();
        return;
    }
        
    // load integrator -----------------------------------------------
    if (Meas_phase == MEAS_PHASE_LOAD_INTEGRATOR)
    {
         Meas_phase = MEAS_PHASE_MEASURE_INTEGRATOR;
            
        // set AD input
        if (Meas_type == MEAS_TYPE_U)
        {
            PORTD = (PORTD & ~AD_Input_Mask) | AD_input_U;
        }
        if (Meas_type == MEAS_TYPE_I)
        {
            PORTD = (PORTD & ~AD_Input_Mask) | AD_input_I;
        }   
        if (Meas_type == MEAS_TYPE_T_TRANSFORMER)
        {
            PORTD = (PORTD & ~AD_Input_Mask) | AD_input_T_Transformer;
        }
        if (Meas_type == MEAS_TYPE_T_HEATSINK)
        {
            PORTD = (PORTD & ~AD_Input_Mask) | AD_input_T_Heatsink;
        }
            
        start_Measurement_timer();
        return;
    }
        
    // measurement integrator ----------------------------------------
    if (Meas_phase == MEAS_PHASE_MEASURE_INTEGRATOR)
    {
        Meas_phase = MEAS_PHASE_READ_RESULT;
        // start ADC measurement
        PORTD = (PORTD & ~AD_Input_Mask) | AD_input_2V5;
        start_Measurement_timer();
        return;
    }
        
    // print measurement result ---------------------------------------
    if (Meas_phase == 3 /*MEAS_PHASE_READ_RESULT*/)
    {
        // read timer value for measurements
        uint16_t meas_time = ((OCR2B << 8) | TCNT2);

        ADW_flag = 0;
            
        if (Meas_type == MEAS_TYPE_U)
        {
            if (enc_changed == FALSE && Hold_time == 0)
            {
                print_U_result(meas_time);
            }
        }
            
        if (Meas_type == MEAS_TYPE_I)
        {
            if (enc_changed == FALSE && Hold_time == 0)
            {
                print_I_result(meas_time);
            }
        }
            
        if (Meas_type == MEAS_TYPE_T_TRANSFORMER)
        {
            print_T_transformer_result(meas_time);
        }

        if (Meas_type == MEAS_TYPE_T_HEATSINK)
        {
            print_T_heatsink_result(meas_time);
        }

        // Get next measurement type; it doesn't make sense to read temperatures that often
        counter++;
        if (counter%50 == 0)
        {
            Meas_type = MEAS_TYPE_T_TRANSFORMER;
        } 
        else if (counter%25 == 0)
        {
            Meas_type = MEAS_TYPE_T_HEATSINK;
        }
        else if (counter%2 == 0)            
        {
            Meas_type = MEAS_TYPE_U;
        }
        else
        {
            Meas_type = MEAS_TYPE_I;
        }                        
            
        // clear flags
        Meas_phase = MEAS_PHASE_CLEAR_INTEGRATOR;
        Timer2_run_flag = 0;
            
        // set UI-Limits
        set_UI_Limits();
            
        if (Hold_time > 0)  Hold_time--;
        if (flash_time > 0) flash_time--;
    }
}

//*************************************************************************
// set UI-Limits
//*************************************************************************
void set_UI_Limits(void)
{
    if (Memory_flag || Recall_flag)
    {
        set_memory_nr();
        return;
    }
    // read encoder ------------------------------------------------------
    int Enc_value = encode_read4();
    
    if (Enc_value == 0)
    {
        enc_changed = FALSE;
        return;
    }
    else enc_changed = TRUE;
    
    // set Ulimit --------------------------------------------------------	
    if (UI_flag == UI_FLAG_U)
    {
        Ulimit = Ulimit + (Multiplier_U * Enc_value);
        
        if      (Ulimit < 0)      Ulimit = 0;
        else if (Ulimit >= 30000) Ulimit = 30000;
        
        set_Usoll(Ulimit);

        Hold_time = 30;         // time before refresh UI-Limits
        Refresh_Ulimit = TRUE;
        return;
    }
    
    // set Ilimit --------------------------------------------------------
    else if (UI_flag == UI_FLAG_I)
    {
        Ilimit = Ilimit + (Multiplier_I * Enc_value);
        
        if      (Ilimit < 0)     Ilimit = 0;
        else if (Ilimit >= 3000) Ilimit = 3000;
        
        set_Isoll(Ilimit);
        
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
        enc_changed = FALSE;
        return;
    }
    else enc_changed = TRUE;
    
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
    if (Hold_time == 0 && Refresh_Ulimit)
    {
        save_eeprom_ulimit(Ulimit);
        print_value(U_limit_cmd, Ulimit);
        Refresh_Ulimit = FALSE;
    }
    
    if (Hold_time == 0 && Refresh_Ilimit)
    {
        save_eeprom_ilimit(Ilimit);
        print_value(I_limit_cmd, Ilimit);
        Refresh_Ilimit = FALSE;
    }
}

//*************************************************************************
// set Usoll (0 - 30.000mV)
//*************************************************************************
void set_Usoll (int16_t value)
{
    uint32_t calib_factor = (((uint32_t) calib_counts_per_20V << 16) + 10000) / 20000;
    OCR1A = calib_counts_offset_0V + ((value * calib_factor) >> 16);
    
    if (value >= 15000) {
        SPI_wr2(Relais_On_48V);
    }
    else if (value <= 14500) {
        SPI_wr2(Relais_Off_24V);
    }
    
    print_value(U_act_cmd, value);
}


//*************************************************************************
// set Isoll
//*************************************************************************
void set_Isoll (int16_t value)
{
    uint32_t calib_factor = (((uint32_t) calib_counts_per_1200mA << 16) + 600) / 1200;
    OCR1B = calib_counts_offset_0mA + ((value * calib_factor) >> 16);

    print_value(I_act_cmd, value);
}

//*************************************************************************
// save Ulimit (EEPROM addr. 0x000)
//*************************************************************************
void save_eeprom_ulimit(uint16_t ulim)
{
    uint16_t *eeAddr = (uint16_t*) 0;
    eeprom_write_word(eeAddr, ulim);
}

//*************************************************************************
// read Ulimit (EEPROM addr. 0x000)
//*************************************************************************
uint16_t read_eeprom_ulimit(void)
{
    uint16_t *eeAddr =  (uint16_t*) 0;
    return eeprom_read_word(eeAddr);
}

//*************************************************************************
// save Ilimit (eeprom addr. 0x002)
//*************************************************************************
void save_eeprom_ilimit(uint16_t ilim)
{
    uint16_t *eeAddr = (uint16_t*) 2;
    eeprom_write_word(eeAddr, ilim);
}

//*************************************************************************
// read Ilimit (eeprom addr. 0x002)
//*************************************************************************
uint16_t read_eeprom_ilimit (void)
{
    uint16_t *eeAddr = (uint16_t*) 2;
    return eeprom_read_word(eeAddr);
}

//*************************************************************************
// read magic_calib (eeprom addr. 0x004)
//*************************************************************************
uint8_t is_calibration_valid(void)
{
    uint16_t *eeAddr = (uint16_t*) 4;
    uint16_t magic = eeprom_read_word(eeAddr);
    return magic == CALIB_MAGIC_WORD;
}

//*************************************************************************
// set magic_calib (eeprom addr. 0x004)
//*************************************************************************
void set_calibration_valid(void)
{
    uint16_t *eeAddr = (uint16_t*) 4;
    eeprom_write_word(eeAddr, CALIB_MAGIC_WORD);
}

//*************************************************************************
// set calibration data (eeprom addr. 0x006, 0x008, 0x00A, 0x00C)
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
// read calibration data (eeprom addr. 0x006, 0x008, 0x00A, 0x00C)
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
// save display settings (eeprom addr. 0x00E)
//*************************************************************************
void save_eeprom_display_settings(uint8_t contrast, uint8_t backlit)
{
    uint16_t *eeAddr = (uint16_t*) 0x0e;
    uint16_t word = (contrast & 0x07) | (backlit & 0x01)<<7 | DISPLAY_MAGIC_WORD;
    eeprom_write_word(eeAddr, word);
}

//*************************************************************************
// save display settings (eeprom addr. 0x00E)
//*************************************************************************
void read_eeprom_display_settings(uint8_t *contrast, uint8_t *backlit)
{
    uint16_t *eeAddr = (uint16_t*) 0x0e;
    uint16_t word = eeprom_read_word(eeAddr);
    if ((word & 0xff00) == DISPLAY_MAGIC_WORD)
    {
        // Setting valid
        *contrast = word & 0x07;
        *backlit = (word & 0x80) >> 7;
    }
    else
    {
        // Setting invalid, use default
        *contrast = 7;
        *backlit = 1;
    }
}


//*************************************************************************
// set Voltage Digit Line 
//*************************************************************************
void setDigitPos (void)
{
    if (UI_flag == UI_FLAG_U) 
    {
        if      (Multiplier_U == 10)    wr_SPI_buffer3(set_U4_underline);
        else if (Multiplier_U == 100)   wr_SPI_buffer3(set_U3_underline);
        else if (Multiplier_U == 1000)  wr_SPI_buffer3(set_U2_underline);
        else if (Multiplier_U == 10000) wr_SPI_buffer3(set_U1_underline);
    }
    else 
    {
        if      (Multiplier_I == 1)     wr_SPI_buffer3(set_I4_underline);
        else if (Multiplier_I == 10)    wr_SPI_buffer3(set_I3_underline);
        else if (Multiplier_I == 100)   wr_SPI_buffer3(set_I2_underline);
        else if (Multiplier_I == 1000)  wr_SPI_buffer3(set_I1_underline);
    }
}

//*************************************************************************
// send LCD commands
//*************************************************************************
void send_LCD_commands (const uint8_t Com_Adr[])
{
    uint8_t counts = pgm_read_byte(Com_Adr);
    for (uint8_t i = 1; i <= counts; i++)
    {
        SPI_wr2(pgm_read_byte(Com_Adr + i));
    }
}

//*************************************************************************
// Button function
//*************************************************************************
void buttonFunction (uint8_t button)
{
    switch (button)
    {
        case BUTTON_STANDBY: 
        {
            // Standby on -------------------------------------------------------- 
            if (Standby_flag == 0) {
                send_LCD_commands(Standby_on);
                set_Usoll(0);
                set_Isoll(0);
                Standby_flag = 1;
            }
            else // Standby off -------------------------------------------------------
            {
                set_Usoll(Ulimit);
                set_Isoll(Ilimit);
                send_LCD_commands(Standby_off);
                send_LCD_commands(Activ_V);
                Standby_flag = 0;
             }                    
        }
        break;            

        case BUTTON_LEFT:
        {
            if (UI_flag == UI_FLAG_U) 
            {
                // Voltage digit Up "<--"
                if (Multiplier_U < 10000) Multiplier_U *= 10;
            }
            else
            {
                // Ampere digit Up "<--"
                if (Multiplier_I < 1000) Multiplier_I *=  10;
            }                
            send_LCD_commands(clr_Digit_Lines);
            setDigitPos();
        }
        break;
    
        case BUTTON_RIGHT:
        {
            if (UI_flag == UI_FLAG_U) 
            {
                // Voltage digit Down "-->"
                if (Multiplier_U > 10) Multiplier_U /= 10;
            }                                    
            else
            {
                // Ampere digit Down "-->"
                if (Multiplier_I > 1) Multiplier_I /= 10;
            }                                    
            send_LCD_commands(clr_Digit_Lines);
            setDigitPos();
        } 
        break;
    
        case BUTTON_UI:
        {
            send_LCD_commands(clr_Digit_Lines);
            if (UI_flag == UI_FLAG_U) 
                UI_flag = UI_FLAG_I;
            else 
                UI_flag = UI_FLAG_U;
            setDigitPos();
        }
        break;
    
        case BUTTON_RECALL:
        {
            if (Recall_flag == FALSE)
            {
                wr_SPI_buffer3(set_memory);
                print_memory_nr(Memory_nr);
                Recall_flag = TRUE;
                flash_time = 5;                     // flash-time for PrgNo
            }
            else
            {
                send_LCD_commands(clr_Memory);
                print_value(U_limit_cmd, Ulimit);
                print_value(I_limit_cmd, Ilimit);
                Recall_flag = FALSE;
            }
            Memory_flag = FALSE;
        }
        break;            
        
        case BUTTON_MEMORY:
        {
            if (Memory_flag == FALSE)
            {
                wr_SPI_buffer3(set_memory);
                print_memory_nr(Memory_nr);
                Memory_flag = TRUE;
                flash_time = 5;                     // flash-time for PrgNo
            }
            else
            {
                send_LCD_commands(clr_Memory);
                Memory_flag = FALSE;
            }
            Recall_flag = FALSE;
        }
        break;
    
        case BUTTON_ENTER:
        {
            if (Recall_flag)
            { 
                rd_eeprom_memory(Memory_nr, &Ulimit, &Ilimit);
                set_Usoll(Ulimit);
                set_Isoll(Ilimit);
                print_value(U_limit_cmd, Ulimit);
                print_value(I_limit_cmd, Ilimit);
                Hold_time = 0;
                Refresh_Ulimit = TRUE;
                Refresh_Ilimit = TRUE;
            }
            else if (Memory_flag)
            {
                wr_eeprom_memory(Memory_nr, Ulimit, Ilimit);
            }
            send_LCD_commands(clr_Memory);
            Memory_flag = FALSE;
            Recall_flag = FALSE;
        }
        break;
       
        default:
        break;
     }               
}

//*************************************************************************
// flashing Memory PrgNo
//*************************************************************************
void flash_PrgNo (void)
{
    if ((Memory_flag || Recall_flag ) && flash_time == 0)
    {
        if (blinki_flag != 0)
        {
            print_memory_nr(Memory_nr);		// print PrgNo
            if (Recall_flag != 0)
            {
                int16_t ilim;
                int16_t ulim;
                rd_eeprom_memory(Memory_nr, &ulim, &ilim);
                print_value(U_limit_cmd, ulim);
                print_value(I_limit_cmd, ilim);
            }
        }
        else
        {
            wr_SPI_buffer5(Memory_nr_cmd, 0x5D, 0x5D, 0x5D, 0x5D);
            if (Recall_flag != 0)
            {
                wr_SPI_buffer5(U_limit_cmd, 0x5D, 0x5D, 0x5D, 0x5D);
                wr_SPI_buffer5(I_limit_cmd, 0x5D, 0x5D, 0x5D, 0x5D);
            }
        }
        blinki_flag = !blinki_flag;
        flash_time = 3;
    }
} 

//*************************************************************************
// save UI data to Memory
//*************************************************************************
void wr_eeprom_memory (uint8_t nr, int16_t ulim, int16_t ilim)
{
    uint16_t *eeAddr = (uint16_t*) (0x10 + (nr * 4));
    eeprom_write_word(eeAddr, ulim);
    eeprom_write_word(eeAddr+1, ilim);
}

//*************************************************************************
// read UI data from Memory
//*************************************************************************
void rd_eeprom_memory (uint8_t nr, int16_t *ulim, int16_t *ilim)
{
    uint16_t *eeAddr = (uint16_t*) (0x10 + (nr * 4));
    *ulim = eeprom_read_word(eeAddr);
    *ilim = eeprom_read_word(eeAddr+1);
}

//*************************************************************************
// press Button
//*************************************************************************
void pressButton (uint8_t button)
{
    buttonPressDuration[button]++;
    if (buttonPressDuration[button] == 500) 
    {
        buttonFunction(button);
    }

    if ((button == BUTTON_UI) && buttonPressDuration[button] == 60000) 
    {
        displaySettings();
    }
}

//*************************************************************************
// release Button
//*************************************************************************
void releaseButton (uint8_t button)
{
    if (buttonPressDuration[button] > 0)
    {
        buttonReleaseDuration[button]++;
        if (buttonReleaseDuration[button] > 100)
        {
            buttonReleaseDuration[button] = 0;
            buttonPressDuration[button] = 0;
        }
    }
}

//*************************************************************************
// read Button Pins
//*************************************************************************
void readButtons(void)
{
    uint16_t pinc = PINC;
    
    if ((pinc & (1<<B_Standby)) == 0)
         pressButton(BUTTON_STANDBY);
    else 
        releaseButton(BUTTON_STANDBY);
    
    if ((pinc & (1<<B_Left)) == 0)
        pressButton(BUTTON_LEFT);
    else 
        releaseButton(BUTTON_LEFT);
    
    if ((pinc & (1<<B_Right)) == 0)
        pressButton(BUTTON_RIGHT);
    else 
        releaseButton(BUTTON_RIGHT);
    
    if ((pinc & (1<<B_UI)) == 0)
        pressButton(BUTTON_UI);
    else 
        releaseButton(BUTTON_UI);
    
    if ((PIND & (1<<B_Recall)) == 0)
        pressButton(BUTTON_RECALL);
    else
        releaseButton(BUTTON_RECALL);

    if ((pinc & (1<<B_Memory)) == 0)
        pressButton(BUTTON_MEMORY);
    else 
        releaseButton(BUTTON_MEMORY);
    
    if ((pinc & (1<<B_Enter)) == 0)
        pressButton(BUTTON_ENTER);
    else 
        releaseButton(BUTTON_ENTER);

    

}

//*************************************************************************
// set UI "Aktive"
//*************************************************************************
void UI_control(void)
{
    static uint8_t status_old = UI_FLAG_U;
    
    uint8_t status = PIND & (1<<Regulator);
    
    if (status == status_old || Standby_flag == 1)
    {
        return;
    }
    
    if (status == UI_FLAG_I)
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
    send_LCD_commands(clr_Digit_Lines);
    send_LCD_commands(clr_unit_display);
    send_LCD_commands(clr_all_digits);
    wr_SPI_buffer3(set_V_point);
    print_value(U_act_cmd, VERSION*10);
    print_value(I_act_cmd, BUILD);
}

//*************************************************************************
// init LCD
//*************************************************************************
void init_LCD (void)
{
    uint8_t contrast, backlit;
    soft_delay(1000);
    SPI_wr2(Clear_Screen_cmd);
    read_eeprom_display_settings(&contrast, &backlit);
    SPI_wr2(backlit ? Backlit_on_cmd : Backlit_off_cmd);
    SPI_wr2(LCD_Contrast_cmd+contrast);
    soft_delay(100);
    SPI_wr2(Relais_Off_24V);
    SPI_wr2(Standby_On_cmd);
    soft_delay(100);

    display_fw_version();
    soft_delay(3000);

    send_LCD_commands(LCD_inits);
    send_LCD_commands(Standby_on);
}

//*************************************************************************
// init UI_Limits
//*************************************************************************
void init_UIlimits (void)
{
    UI_flag = UI_FLAG_U;
    
    Ulimit = read_eeprom_ulimit();
    print_value(U_limit_cmd, Ulimit);
    
    Ilimit = read_eeprom_ilimit();
    print_value(I_limit_cmd, Ilimit);

    Standby_flag = 1;
    set_Usoll(0);
    set_Isoll(0);
    setDigitPos();
    
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
        wr_SPI_buffer3(set_standby);
        soft_delay(200);
        wr_SPI_buffer3(clr_standby);
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
        
        SPI_wr2(Clear_Screen_cmd);
        wr_SPI_buffer3(set_V_Ulimit);
        wr_SPI_buffer3(set_A_Ilimit);
        
        // Limit current to 100mA; calibration should be done in idle anyway
        set_Isoll(100);
        print_value(I_limit_cmd, 100);

        uint16_t u_refs[] = { u_ref_low, u_ref_high };
        uint16_t i_refs[] = { i_ref_low, i_ref_high };
        uint16_t ocrs[] = { 0, 0 };

        //wr_SPI_buffer5(P_act_cmd, 0x5D, 0x5D, 0x5D, 0x5D); // Clr P display
            
        // Voltage calibration
        for (uint8_t i=0; i<2; i++)
        {
            send_LCD_commands(Standby_on);
            wr_SPI_buffer3(set_V_symbol);

            set_Usoll(u_refs[i]);
            wr_SPI_buffer5(U_act_cmd, 0x51+i, 0x5B, 0x5A ,0x5C);    // "CAL1/2"
            print_value(U_limit_cmd, u_refs[i]);
            wr_SPI_buffer5(I_act_cmd, 0x5D, 0x5D, 0x5D, 0x5D);      // Clr I display

            wait_for_standby_off();
            send_LCD_commands(Activ_V);
            wr_SPI_buffer3(set_I4_underline);

            soft_delay(200);
            ocrs[i] = calibration_user(1);
        }

        calib_counts_per_20V = ocrs[1] - ocrs[0];
        calib_counts_offset_0V = ocrs[0] - calib_counts_per_20V / 4;   // subtract 5V to get offset @0V

        //wr_SPI_buffer5(U_act_cmd, 0x5D, 0x5B, 0x5A, 0x5C);  // "CAL"
        //print_value(I_act_cmd, calib_counts_per_20V/2);     // Print Counts per 10V (we have only 4 digits!) (~5005)
        //print_value(P_act_cmd, calib_counts_offset_0V);     // Print offset (~122)
        //soft_delay(5000);

        // Current calibration
        set_Usoll(12000);           // 12V
        SPI_wr2(Relais_Off_24V);
        print_value(U_limit_cmd, 12000);

        for (uint8_t i=0; i<2; i++)
        {
            send_LCD_commands(Standby_on);
            wr_SPI_buffer3(clr_V_symbol);
            wr_SPI_buffer3(set_A_symbol);

            set_Isoll(i_refs[i]);
            wr_SPI_buffer5(U_act_cmd, 0x53+i, 0x5B, 0x5A ,0x5C);    // "CAL3/4"
            print_value(I_limit_cmd, i_refs[i]);
            wr_SPI_buffer5(I_act_cmd, 0x5D, 0x5D, 0x5D, 0x5D);      // Clr I display

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
    
    send_LCD_commands(LCD_inits);
    send_LCD_commands(Standby_on);

    return;
}

void printContrast(uint8_t contrast)
{
    wr_SPI_buffer5(I_act_cmd, 0x50+contrast+1, 0x5D, 0x50, 0x5C);
}

void displaySettings(void)
{
    uint8_t contrast;
    uint8_t backlit;
     
    send_LCD_commands(clr_unit_display);
    send_LCD_commands(clr_Digit_Lines);
    send_LCD_commands(clr_all_digits);
    wr_SPI_buffer3(set_I4_underline);
    printContrast(contrast);

    read_eeprom_display_settings(&contrast, &backlit);
    printContrast(contrast);
    
    do
    {
        pull_encoder();
        int diff = encode_read4();
        
        if (diff> 0 && contrast < 7)
        {
            contrast++;
            SPI_wr2(LCD_Contrast_cmd + contrast);
            printContrast(contrast);
        }
        else if (diff<0 && contrast>0)
        {
            contrast--;
            SPI_wr2(LCD_Contrast_cmd + contrast);
            printContrast(contrast);
        }

        if ((PINC & (1<<B_Right)) == 0)
        {
            SPI_wr2(Backlit_on_cmd);
            backlit = 1;
        }        
        if ((PINC & (1<<B_Left)) == 0)
        {
            SPI_wr2(Backlit_off_cmd);
            backlit = 0;
        }
    } while ((PINC & (1<<B_Enter)) != 0);
    
    save_eeprom_display_settings(contrast, backlit);

    send_LCD_commands(restore_unit_display);
    send_LCD_commands(clr_Digit_Lines);
    setDigitPos();
    
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
        readButtons();
        flash_PrgNo();	
    }	
}