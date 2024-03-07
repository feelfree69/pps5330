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

// version info
#define VERSION 199
#define BUILD     9

#define TRUE    1
#define FALSE   0

// default calibration values
#define CALIB_COUNTS_PER_20V_DEFAULT           10022
#define CALIB_COUNTS_OFFSET_0V_DEFAULT           123
#define CALIB_COUNTS_PER_1200MA_DEFAULT         5758
#define CALIB_COUNTS_OFFSET_0MA_DEFAULT          118

#define CALIB_COUNTS_MEAS_PER_20V_DEFAULT       6905
#define CALIB_COUNTS_MEAS_0V_OFFSET_DEFAULT      163
#define CALIB_COUNTS_MEAS_PER_1200MA_DEFAULT    4992
#define CALIB_COUNTS_MEAS_0MA_OFFSET_DEFAULT     267

#define CALIB_MAGIC_WORD   0x0ca1
#define DISPLAY_MAGIC_WORD 0xcc00

// Pin definitions
#define ADC_ADW     PB0	
#define PWM_U       PB1
#define PWM_I       PB2
#define SPI_SS      PB2  /* why is this not conflicting with PWM_I??? */
#define SPI_MOSI    PB3
#define SPI_MISO    PB4
#define SPI_SCK     PB5

#define B_Left      PC0
#define B_UI        PC1
#define B_Right     PC2
#define B_Standby   PC3
#define B_Enter     PC4
#define B_Memory    PC5

#define Regulator   PD0
#define B_Recall    PD1
#define Enc_B       PD2
#define Enc_A       PD3
#define ADC_AD0     PD4
#define ADC_AD1     PD5
#define PWM_FAN     PD6
#define ADC_AD2     PD7

#define PHASE_A     (PIND & 1<<Enc_A)
#define PHASE_B     (PIND & 1<<Enc_B)

#define NOP() asm volatile ("nop" ::) 

#define AD_Input_Mask          ((1<<ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2))
#define AD_input_U               0                                          /* Input 0 */
#define AD_input_I              (1<<ADC_AD0)                                /* Input 1 */
#define AD_input_T_Transformer                 (1<<ADC_AD1)                 /* Input 2 */
#define AD_input_T_Heatsink    ((1<<ADC_AD0) | (1<<ADC_AD1))                /* Input 3 */
#define AD_input_Minus_2V5                                    (1<<ADC_AD2)  /* Input 4 */
#define AD_input_Clear         ((1<<ADC_AD0) |                (1<<ADC_AD2)) /* Input 5 */
#define AD_input_None          ((1<<ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2)) /* Input 7 */

#define MEAS_TYPE_U             0
#define MEAS_TYPE_I             1
#define MEAS_TYPE_T_TRANSFORMER 2
#define MEAS_TYPE_T_HEATSINK    3

#define MEAS_PHASE_CLEAR_INTEGRATOR    0
#define MEAS_PHASE_LOAD_INTEGRATOR     1
#define MEAS_PHASE_MEASURE_INTEGRATOR  2
#define MEAS_PHASE_READ_RESULT         3

#define UI_STATUS_U       0
#define UI_STATUS_I       1
#define UI_STATUS_STANDBY 2

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

#define BUTTON_PRESS_DEBOUNCE 10    /* 10ms */
#define BUTTON_LONGPRESS_FLAG -1    /* must be < 0 */
#define BUTTON_LONGPRESS_TIME 3000  /* 3 seconds */

// prototypes -------------------------------------------------------------

void soft_delay (uint16_t time_value);

// init functions
void init_SPI(void);
void init_IO_Port(void);
void init_Interrupt(void);
void init_Timer0(void);
void init_Timer1(void);
void init_Timer2(void);
void init_LCD(void);
void init_UIlimits(void);

// SPI related
void wr_SPI_buffer1(uint8_t data0);
void wr_SPI_buffer3(uint8_t data0, uint8_t data1, uint8_t data2);
void wr_SPI_buffer5(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);


void print_value(uint8_t Index, uint16_t value);
void print_U_result(uint16_t meas_time);
void print_I_result(uint16_t meas_time);
void print_T_transformer_result(uint16_t meas_time);
void print_T_heatsink_result(uint16_t meas_time);

void start_Measurement_timer(void);
uint16_t singleMeasurement(uint8_t uiflag);

void set_UI_Limits(void);
void set_Usoll (int16_t value);
void set_Isoll (int16_t value);

void clr_eeprom();
void save_eeprom_ulimit(uint16_t ulim);
void save_eeprom_ilimit(uint16_t ilim);
uint16_t read_eeprom_ulimit(void);
uint16_t read_eeprom_ilimit(void);
void wr_eeprom_memory(uint8_t nr, int16_t ulim, int16_t ilim);
void rd_eeprom_memory(uint8_t nr, int16_t *ulim, int16_t *ilim);

void setDigitPos (void);

void init_Encoder(void);
int8_t encode_read4(void);
void buttonFunction (uint8_t button);
void pressButton (uint8_t button);
void releaseButton (uint8_t button);
void longPressButton (uint8_t button);

void display_fw_version(void);
void send_LCD_commands (const uint8_t Com_Adr[]);
void print_degree(uint16_t value);
void set_memory_nr(void);

void startDisplaySettings(void);
void handleDisplaySettings(void);
void printContrast(uint8_t contrast);
void printIllu(uint8_t illum);

void triggerBacklitTimeout(void);
void checkBacklitTimeout(void);

void set_standby_mode(void);
void clr_standby_mode(void);

void lcd_clr_digit_lines(void);
void lcd_clr_unit_display(void);
void lcd_clr_digits(uint8_t display);
void lcd_clr_all_digits(void);
void lcd_clr_memory(void);

void startup_calibration(void);

// functions in main loop...
void save_print_UIlimit(void);
void UI_status_display(void);
void readButtons(void);
void flash_PrgNo(void);
void Measurement(void);



// global variables ------------------------------------------------------
int8_t enc_delta = 0;          // -128 ... 127
int8_t enc_last;
uint8_t enc_changed = FALSE;
volatile uint8_t timer_max = 0;
volatile uint8_t ADW_flag = 0;
uint8_t Meas_phase = 0;
uint8_t Meas_type = MEAS_TYPE_U;
volatile uint8_t Timer2_running = FALSE;
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
int32_t buttonPressEvent[BUTTON_COUNT] = { 0,0,0,0,0,0,0 };

uint16_t calib_counts_per_20V    = CALIB_COUNTS_PER_20V_DEFAULT;
uint16_t calib_counts_offset_0V  = CALIB_COUNTS_OFFSET_0V_DEFAULT;
uint16_t calib_counts_per_1200mA = CALIB_COUNTS_PER_1200MA_DEFAULT;
uint16_t calib_counts_offset_0mA = CALIB_COUNTS_OFFSET_0MA_DEFAULT;

uint16_t meas_counts_per_20V     = CALIB_COUNTS_MEAS_PER_20V_DEFAULT;
uint16_t meas_counts_offset_0V   = CALIB_COUNTS_MEAS_0V_OFFSET_DEFAULT;
uint16_t meas_counts_per_1200mA  = CALIB_COUNTS_MEAS_PER_1200MA_DEFAULT;
uint16_t meas_counts_offset_0mA  = CALIB_COUNTS_MEAS_0MA_OFFSET_DEFAULT;

#define MENU_CONTRAST 0
#define MENU_BACKLIT 1
uint8_t displaySettingsActive = FALSE;
uint8_t contrast;
uint8_t backlit;
uint8_t menu_contrast = MENU_CONTRAST;

uint32_t lastButtonActivity = 0;
uint32_t backlitState = 1;

uint8_t panel_locked = FALSE;

#define TICKS_PER_SECOND 862 
uint32_t runtime = 0;

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

#define set_locked 0x22,0x02,0x31
#define clr_locked 0x22,0x02,0x1E

#define set_overtemp 0x22,0x01,0x34
#define clr_overtemp 0x22,0x01,0x11

//LCD commands -----------------------------------------------------------

//const uint8_t Degree_Symbol[] PROGMEM = {12, 0x23,0x02,0x31, 0x30,0x17,0x20, 0x30,0x26,0x38, 0x30,0x24,0x17};
 
const uint8_t UI_Activ_V[] PROGMEM = {9, set_active_V, clr_active_A, clr_standby};
const uint8_t UI_Activ_A[] PROGMEM = {9, set_active_A, clr_active_V, clr_standby};
const uint8_t UI_Standby[] PROGMEM = {9, set_standby,  clr_active_A, clr_active_V};

const uint8_t LCD_inits[] PROGMEM = {54, set_V_symbol, set_V_point,
                                         set_A_symbol, set_A_point,
                                         set_W_symbol, set_W_point,
                                         set_point_ulimit, set_point_ilimit,
                                         set_U_limit, set_V_Ulimit, 
                                         set_I_limit, set_A_Ilimit, 
                                         set_U2_underline,
                                         U_act_cmd,0x5D,0x50,0x50,0x50,  
                                         I_act_cmd,0x50,0x50,0x50,0x50, 
                                         P_act_cmd,0x50,0x50,0x50,0x50};

const uint8_t clr_underlines[] PROGMEM = {24, clr_U_underlines, clr_I_underlines};

const uint8_t restore_unit_display[] PROGMEM = {15, set_V_symbol, set_A_symbol, set_W_symbol, set_V_point, set_A_point, set_W_point };
const uint8_t clr_unit_display[]     PROGMEM = {15, clr_V_symbol, clr_A_symbol, clr_W_symbol, clr_V_point, clr_A_point, clr_W_point };

const uint8_t illu_values[] = { 0, 1, 5, 10, 30, 60, 120, 240 };

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
    DDRB &= ~(1<<SPI_MISO);                         // MISO as input
    PORTB |= (1<< SPI_MISO);
    SPCR |= (1<<MSTR);                              // Set as Master
    SPCR |= (1<<SPR0) | (1<<CPOL) | (1<<CPHA);      // set CLK 500KHz and SPI Mode
    SPCR |= (1<<SPE);                               // Enable SPI
}

//*************************************************************************
// SPI write one byte
//*************************************************************************
void wr_SPI_buffer1(uint8_t dataout)
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
    wr_SPI_buffer1(data0);
    wr_SPI_buffer1(data1);
    wr_SPI_buffer1(data2);
}

void wr_SPI_buffer5(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
    wr_SPI_buffer1(data0);
    wr_SPI_buffer1(data1);
    wr_SPI_buffer1(data2);
    wr_SPI_buffer1(data3);
    wr_SPI_buffer1(data4);
}

//*************************************************************************
// init Timer1   14Bit/500Hz PWM mode (voltage and cufrent control)
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
    // Outputs
    DDRD |= ((1<<ADC_AD0) | (1<<ADC_AD1) | (1<<ADC_AD2));	// Multiplexer IC6 
    DDRB |= (1<<PWM_U);         // PB1 OC1A	PWM Voltage
    DDRB |= (1<<PWM_I);         // PB2 OC1B	PWM Ampere
    DDRD |= (1<<PWM_FAN);       // FAN PWM out

    // Inputs
    DDRB &= ~(1<<ADC_ADW);      // extern PCINT0 for Measurement
    DDRC &= ~(1<<B_Memory);     // Key "Memory"
    DDRC &= ~(1<<B_Enter);      // Key "Enter"
    DDRC &= ~(1<<B_UI);         // Key "U/I"
    DDRC &= ~(1<<B_Left);       // Key "<-"
    DDRC &= ~(1<<B_Right);      // Key "->"
    DDRC &= ~(1<<B_Standby);    // Key "Standby"
    DDRD &= ~(1<<B_Recall);     // Key "Recall"
    DDRD &= ~(1<<Regulator);    // Indicates which Regulator (U/I) is active
    DDRD &= ~(1<<Enc_A);        // Encoder Pin A
    DDRD &= ~(1<<Enc_B);        // Encoder Pin B

    // Pullups for all Inputs    
    PORTB |= (1<<ADC_ADW);
    PORTC |= (1<<B_Memory);
    PORTC |= (1<<B_Enter);
    PORTC |= (1<<B_UI);
    PORTC |= (1<<B_Left);
    PORTC |= (1<<B_Right);
    PORTC |= (1<<B_Standby);
    PORTD |= (1<<B_Recall);
    PORTD |= (1<<Regulator);
    PORTD |= (1<<Enc_A);
    PORTD |= (1<<Enc_B);
    
}

//*************************************************************************
// print values on screen
//*************************************************************************
void get_digits(uint16_t value, uint8_t digits[])
{
    for (uint8_t i=0; i<4; i++)
    {
        digits[i] = value % 10 + 0x50;
        value /= 10;
    }
}

void print_digits(uint8_t cmd, uint8_t digits[])
{
    wr_SPI_buffer1(cmd);
    for (uint8_t i=0; i<4; i++)
        wr_SPI_buffer1(digits[i]);
}

void print_value(uint8_t cmd_index, uint16_t value)
{
    uint8_t digits[4];
    if (cmd_index == U_act_cmd || cmd_index == U_limit_cmd)
    {
        value /= 10;
    }

    get_digits(value, digits);

    if (((cmd_index == U_act_cmd || cmd_index == U_limit_cmd) || cmd_index == P_act_cmd) && digits[3] == 0x50)
    {
        digits[3] = 0x5D;    // if first digit 0 than print "Space"
    }
    
    print_digits(cmd_index, digits);
}

void print_value_signed(uint8_t cmd_index, int16_t value)
{
    uint8_t digits[4];

    get_digits(abs(value), digits);
    
    // replace leading zeros
    if (digits[3]==0x50) 
    {
        digits[3] = 0x5D;
        if (digits[2]==0x50)
        {
            digits[2] = 0x5D;
            if (digits[1]==0x50)
            {
                digits[1] = 0x5D;
            }
        }            
    }        

    // print "-"
    if (value<0)
    {
        if      (value >-10)  digits[1] = 0x5f;
        else if (value >-100) digits[2] = 0x5f;
        else                  digits[3] = 0x5f;
    }
    
    print_digits(cmd_index, digits);
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
void print_degree(uint16_t value)
{
    value /= 100;                // print only 2-digit integer
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
    TCCR2B |= (1<<CS20) | (1<<CS21);                    // Prescaler 32 (4us)
}

//-------------------------------------------------------------------------
// init Timer0 (Fan_PWM)
//-------------------------------------------------------------------------
void init_Timer0(void)
{
    TCCR0B |= (1<<CS00) | (1<<CS01);                    // prescaler 64
    TCCR0A |= (1<<COM0A1) | (1<<WGM00) | (1<<WGM01);    // Set OC2B at bottom, clear OC2BF
    OCR0A = 127;                                        // 100% Fan-PWM 
}

//*************************************************************************
// start measurment Timer
//*************************************************************************
void start_Measurement_timer(void)
{
    // clear timer register
    OCR2B = 0;
    TCNT2 = 10;
        
    ADW_flag = Meas_phase == MEAS_PHASE_READ_RESULT ? TRUE : FALSE;
    
    TIMSK2 |= (1<<TOIE2);               // set overflow Interrupt
    TIFR2  |= (1<<TOV2);                // clear Timer2 overflow_flag
    PCIFR  |= (1<<PCIF0);               // clear extern INT0 Flag
    TCCR2B |= (1<<CS20)| (1<<CS21);     // start Timer 2
    Timer2_running = TRUE;  
}

//*************************************************************************
// Timer2 Overflow interrupt for measurement results
//*************************************************************************
ISR(TIMER2_OVF_vect)
{   
    OCR2B++;
  
    if (OCR2B >= 48 && ADW_flag == 0)       // was "OCR2B == 48"; purpose???
    {
        TCCR2B &= ~((1<<CS00) | (1<<CS01)); // stop timer
        Timer2_running = FALSE;

        PORTD |= AD_input_None;
    }
   
    runtime++;  // 256*4us = 1.024ms plus the time the timer is not running...
                // measured: 1.16ms; use 1000/1.16 = 862 as a divider to get seconds (pi*thumb)
    if (runtime%TICKS_PER_SECOND == 0)
    {
        checkBacklitTimeout();
    }
}

//*************************************************************************
// Measurement interrupt from ADW-Pin
//*************************************************************************
ISR(PCINT0_vect)
{
    // falling edge
    if (!(PINB & (1<<ADC_ADW)))
    {
        TCCR2B &= ~((1<<CS00) | (1<<CS01)); // stop timer
        Timer2_running = FALSE;

        PORTD |= AD_input_None;
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
    if (Standby_flag || meas_time <= meas_counts_offset_0V)
    {
        print_value(U_act_cmd, 0);
        print_value(P_act_cmd, 0);
        U_meas = 0;
        return;
    }

    U_meas = ( (int32_t) ( (int32_t)meas_time - meas_counts_offset_0V) * 20000 + meas_counts_per_20V/2)  / meas_counts_per_20V;
    print_value(U_act_cmd, U_meas);

    print_P_calculation();
}

//*************************************************************************
// print I_measurement result
//*************************************************************************
void print_I_result(uint16_t meas_time)
{
    if (Standby_flag || meas_time <= meas_counts_offset_0mA) 
    {
        print_value(I_act_cmd, 0);
        print_value(P_act_cmd, 0);
        I_meas = 0;
        return;
    }

    I_meas = ( (int32_t) ( (int32_t)meas_time - meas_counts_offset_0mA) * 1200 + meas_counts_per_1200mA/2)  / meas_counts_per_1200mA;
    print_value(I_act_cmd, I_meas);

    print_P_calculation();
}

//*************************************************************************
// print T measurement result (heat sink) and set Fan PWM/Overtemp alarm
//*************************************************************************
void print_T_heatsink_result(uint16_t meas_time)
{	
    #define calib_offset_temp 6000      // Calibration to be done
    #define calib_factor_temp 196608    // Calibration to be done (use 196608/65536 = 3 as a starting point)
    
    uint16_t result = (((uint32_t)meas_time - calib_offset_temp) * calib_factor_temp) >> 16;

    if (! (Memory_flag || Recall_flag))
    {
        print_degree(result);
    }        

    result /= 100;
        
    // heat alert
    if (result >= 75)
    {
        set_standby_mode();
    }
    else if (result >= 70)
    {
        wr_SPI_buffer3(set_overtemp);
        OCR0A = 255;    // set max Fan PWM
    }
    else if (result < 69)
    {
        wr_SPI_buffer3(clr_overtemp);
        
        if (result <= 35)
        {
            OCR0A = 20; // set min Fan PWM
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
    uint16_t result = (((uint32_t)meas_time - calib_offset_temp) * calib_factor_temp) >> 16;
    
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
    int8_t new = 0;
    if( PHASE_A ) new = 3;
    if( PHASE_B ) new ^= 1;     // convert gray to binary
    enc_last = new;             // power on state
    enc_delta = 0;
}

// poll Encoder-Port  ----------------------------------------------------
void pull_encoder(void)
{
    if (!panel_locked)
    {
        int8_t new = 0, diff;
        if( PHASE_A ) new = 3;
        if( PHASE_B ) new ^= 1;             // convert gray to binary
        diff = enc_last - new;              // difference last - new
        if( diff & 1 ) {                    // bit 0 = value (1)
            enc_last = new;                 // store new as next last
            enc_delta += (diff & 2) - 1;    // bit 1 = direction (+/-)
        
        }
        if (diff) triggerBacklitTimeout();
    }        
}

// read Encoder ----------------------------------------------------------
int8_t encode_read4(void)               // read four-step-encoder
{
    int8_t val = enc_delta;
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
    if (Timer2_running)
    {
        return;
    }
    
    // clear integrator ----------------- ----------------------------
    if (Meas_phase == MEAS_PHASE_CLEAR_INTEGRATOR)
    {
        Meas_phase = MEAS_PHASE_LOAD_INTEGRATOR;

        // clear ADC
        PORTD = (PORTD & ~AD_Input_Mask) | AD_input_Clear;
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
        PORTD = (PORTD & ~AD_Input_Mask) | AD_input_Minus_2V5;
        start_Measurement_timer();
        return;
    }
        
    // print measurement result ---------------------------------------
    if (Meas_phase == MEAS_PHASE_READ_RESULT)
    {
        // read timer value for measurements
        uint16_t meas_time = ((OCR2B << 8) | TCNT2);

        ADW_flag = 0;
            
        if (!displaySettingsActive)
        {
            if (enc_changed == FALSE && Hold_time == 0)
            {
                if (Meas_type == MEAS_TYPE_U)
                {
                    print_U_result(meas_time);
                }
                            
                if (Meas_type == MEAS_TYPE_I)
                {
                    print_I_result(meas_time);
                }
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
        Timer2_running = FALSE;
            
        // set UI-Limits
        set_UI_Limits();
            
        if (Hold_time > 0)  Hold_time--;
        if (flash_time > 0) flash_time--;
    }
}


//*************************************************************************
// Voltage/Current measurement for calibration
//*************************************************************************
uint16_t singleMeasurement(uint8_t uiflag)
{
    uint16_t result = 0;
    uint16_t old_result = 0;
    uint8_t  stable_result_count = 0;
    uint8_t i=0;
    
    for (i=0; i<50; i++)
    {
        old_result = result;
        
        // Phase 0  
        PORTD = (PORTD & ~AD_Input_Mask) | AD_input_Clear;
        start_Measurement_timer();

        while (Timer2_running) {};   // busy waiting

        // Phase 1
        PORTD = (PORTD & ~AD_Input_Mask) | (uiflag == UI_FLAG_U ? AD_input_U : AD_input_I);
        start_Measurement_timer();

        while (Timer2_running) {};
        
        // Phase 2
        PORTD = (PORTD & ~AD_Input_Mask) | AD_input_Minus_2V5;
        start_Measurement_timer();

        while (Timer2_running) {};
        
        // Phase 3
        result = (OCR2B << 8) | TCNT2;

        if (old_result == result)        
        {
            if (++stable_result_count == 5)
            {
                break;
            }
        }
        else
        {
            stable_result_count = 0;
        }
    };

    return result;
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
void set_memory_nr(void)
{
    // read encoder ------------------------------------------------------
    int diff = encode_read4();
    
    if (diff == 0)
    {
        enc_changed = FALSE;
        return;
    }
    else enc_changed = TRUE;
    
    if (diff > 0 && Memory_nr < 15)
    {
        Memory_nr++;
    }
    else if (diff < 0 && Memory_nr > 0)
    {
        Memory_nr--;
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
        wr_SPI_buffer1(Relais_On_48V);
    }
    else if (value <= 14500) {
        wr_SPI_buffer1(Relais_Off_24V);
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
// clear complete EEPROM (EEPROM addr. 0x00-0x5f)
//*************************************************************************
void clr_eeprom(void)
{
    uint16_t *eeAddr = (uint16_t*) 0;
    for (uint8_t i=0; i<0x30; i++)
        eeprom_write_word(eeAddr+i, 0);
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
// read Ulimit (EEPROM addr. 0x00)
//*************************************************************************
uint16_t read_eeprom_ulimit(void)
{
    uint16_t *eeAddr =  (uint16_t*) 0x00;
    return eeprom_read_word(eeAddr);
}

//*************************************************************************
// save Ilimit (eeprom addr. 0x02)
//*************************************************************************
void save_eeprom_ilimit(uint16_t ilim)
{
    uint16_t *eeAddr = (uint16_t*) 0x02;
    eeprom_write_word(eeAddr, ilim);
}

//*************************************************************************
// read Ilimit (eeprom addr. 0x02)
//*************************************************************************
uint16_t read_eeprom_ilimit (void)
{
    uint16_t *eeAddr = (uint16_t*) 0x02;
    return eeprom_read_word(eeAddr);
}

//*************************************************************************
// save display settings (eeprom addr. 0x04)
//*************************************************************************
void save_eeprom_display_settings(uint8_t contrast, uint8_t backlit)
{
    uint16_t *eeAddr = (uint16_t*) 0x04;
    uint16_t word = (contrast & 0x07) | (backlit & 0x07)<<4 | DISPLAY_MAGIC_WORD;
    eeprom_write_word(eeAddr, word);
}

//*************************************************************************
// save display settings (eeprom addr. 0x04)
//*************************************************************************
void read_eeprom_display_settings(uint8_t *contrast, uint8_t *backlit)
{
    uint16_t *eeAddr = (uint16_t*) 0x04;
    uint16_t word = eeprom_read_word(eeAddr);
    if ((word & 0xff00) == DISPLAY_MAGIC_WORD)
    {
        // Setting valid
        *contrast = word & 0x07;
        *backlit = (word & 0x70) >> 4;
    }
    else
    {
        // Setting invalid, use default
        *contrast = 7;
        *backlit = 1;
    }
}

//*************************************************************************
// read magic_calib (eeprom addr. 0x0E)
//*************************************************************************
uint8_t is_calibration_valid(void)
{
    uint16_t *eeAddr = (uint16_t*) 0x0E;
    uint16_t magic = eeprom_read_word(eeAddr);
    return magic == CALIB_MAGIC_WORD;
}

//*************************************************************************
// set magic_calib (eeprom addr. 0x00E)
//*************************************************************************
void set_calibration_valid(void)
{
    uint16_t *eeAddr = (uint16_t*) 0x0E;
    eeprom_write_word(eeAddr, CALIB_MAGIC_WORD);
}

//*************************************************************************
// set calibration data (eeprom addr. 0x10..0x1F)
//*************************************************************************
void save_calibration_data(void)
{
    uint16_t *eeAddr = (uint16_t*) 0x10;
    eeprom_write_word(eeAddr+0, calib_counts_offset_0V);
    eeprom_write_word(eeAddr+1, calib_counts_per_20V);
    eeprom_write_word(eeAddr+2, calib_counts_offset_0mA);
    eeprom_write_word(eeAddr+3, calib_counts_per_1200mA);

    eeprom_write_word(eeAddr+4, meas_counts_offset_0V);
    eeprom_write_word(eeAddr+5, meas_counts_per_20V);
    eeprom_write_word(eeAddr+6, meas_counts_offset_0mA);
    eeprom_write_word(eeAddr+7, meas_counts_per_1200mA);
    
    set_calibration_valid();
}

//*************************************************************************
// read calibration data (eeprom addr. 0x10..0x1F
//*************************************************************************
void read_calibration_data(void)
{
    uint16_t *eeAddr = (uint16_t*) 0x10;
    calib_counts_offset_0V  = eeprom_read_word(eeAddr+0);
    calib_counts_per_20V    = eeprom_read_word(eeAddr+1);
    calib_counts_offset_0mA = eeprom_read_word(eeAddr+2);
    calib_counts_per_1200mA = eeprom_read_word(eeAddr+3);

    meas_counts_offset_0V   = eeprom_read_word(eeAddr+4);
    meas_counts_per_20V     = eeprom_read_word(eeAddr+5);
    meas_counts_offset_0mA  = eeprom_read_word(eeAddr+6);
    meas_counts_per_1200mA  = eeprom_read_word(eeAddr+7);
}

//*************************************************************************
// save UI data to Memory (eeprom addr. 0x20..0x5F)
//*************************************************************************
void wr_eeprom_memory (uint8_t nr, int16_t ulim, int16_t ilim)
{
    uint16_t *eeAddr = (uint16_t*) (0x20 + (nr * 4));
    eeprom_write_word(eeAddr, ulim);
    eeprom_write_word(eeAddr+1, ilim);
}

//*************************************************************************
// read UI data from Memory (eeprom addr. 0x20..0x5F)
//*************************************************************************
void rd_eeprom_memory (uint8_t nr, int16_t *ulim, int16_t *ilim)
{
    uint16_t *eeAddr = (uint16_t*) (0x20 + (nr * 4));
    *ulim = eeprom_read_word(eeAddr);
    *ilim = eeprom_read_word(eeAddr+1);
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
// LCD functions
//*************************************************************************
void send_LCD_commands(const uint8_t Com_Adr[])
{
    uint8_t counts = pgm_read_byte(Com_Adr);
    for (uint8_t i = 1; i <= counts; i++)
    {
        wr_SPI_buffer1(pgm_read_byte(Com_Adr + i));
    }
}

void set_standby_mode(void)
{
    wr_SPI_buffer1(Standby_On_cmd);
    Standby_flag = TRUE;
}    

void clr_standby_mode(void)
{
    wr_SPI_buffer1(Standby_Off_cmd);
    Standby_flag = FALSE;
}

void lcd_clr_digit_lines(void)
{
    send_LCD_commands(clr_underlines);
}

void lcd_clr_unit_display(void)
{
    send_LCD_commands(clr_unit_display);
}

void lcd_clr_digits(uint8_t display)
{
    wr_SPI_buffer5(display, 0x5D, 0x5D, 0x5D, 0x5D);
}

void lcd_clr_all_digits(void)
{
    lcd_clr_digits(U_act_cmd);
    lcd_clr_digits(I_act_cmd);
    lcd_clr_digits(P_act_cmd);
}

void lcd_clr_memory(void)
{
    wr_SPI_buffer3(clr_memory);
    lcd_clr_digits(Memory_nr_cmd);
}

//*************************************************************************
// Button function
//*************************************************************************
void buttonFunction (uint8_t button)
{
    // if panel is locked, the only key we care for is setting to standby
    if (panel_locked && (button == BUTTON_STANDBY) && !Standby_flag)
    {
         set_Usoll(0);
         set_Isoll(0);
         set_standby_mode();
         return;
    }

    if (displaySettingsActive)
    {
        switch (button)
        {
            case BUTTON_ENTER:
                save_eeprom_display_settings(contrast, backlit);

                send_LCD_commands(restore_unit_display);
                lcd_clr_digit_lines();
                setDigitPos();
                displaySettingsActive = FALSE;
                break;

            case BUTTON_UI:
                menu_contrast = menu_contrast ? 0 : 1;
                
                if (menu_contrast == MENU_CONTRAST) 
                    printContrast(contrast);
                else
                    printIllu(backlit);
                break;
        }
        
        return;
    }

    if (!panel_locked)
    {
      switch (button)
      {
        case BUTTON_STANDBY:
        {
            // Standby on -------------------------------------------------------- 
            if (Standby_flag == 0) {
                set_standby_mode();
                set_Usoll(0);
                set_Isoll(0);
            }
            else // Standby off -------------------------------------------------------
            {
                set_Usoll(Ulimit);
                set_Isoll(Ilimit);
                clr_standby_mode();
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
            lcd_clr_digit_lines();
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
            lcd_clr_digit_lines();
            setDigitPos();
        } 
        break;
    
        case BUTTON_UI:
        {
            lcd_clr_digit_lines();
            UI_flag = UI_flag == UI_FLAG_U ? UI_FLAG_I : UI_FLAG_U;
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
                lcd_clr_memory();
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
                lcd_clr_memory();
                Memory_flag = FALSE;
            }
            Recall_flag = FALSE;
        }
        break;
    
        case BUTTON_ENTER:
        {
            if ((Recall_flag) ||  (Memory_flag))
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
                else
                {
                    wr_eeprom_memory(Memory_nr, Ulimit, Ilimit);
                }
                lcd_clr_memory();
                Memory_flag = FALSE;
                Recall_flag = FALSE;
            }
            else
            {
                // any idea for a function here?
            }                
        }
        break;
       
        default:
        break;
     }
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
            print_memory_nr(Memory_nr);
            if (Recall_flag)
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
            lcd_clr_digits(Memory_nr_cmd);
            if (Recall_flag)
            {
                lcd_clr_digits(U_limit_cmd);
                lcd_clr_digits(I_limit_cmd);
            }
        }
        blinki_flag = !blinki_flag;
        flash_time = 3;
    }
} 

void longPressButton (uint8_t b)
{
    if ((b == BUTTON_UI) && !displaySettingsActive)
    {
        startDisplaySettings();
    }
    
    // LOCK PANEL function
    if ((b == BUTTON_LEFT) || (b == BUTTON_RIGHT))
    {
        // Ensure both buttons are long pressed
        if ((buttonPressEvent[BUTTON_LEFT]  == BUTTON_LONGPRESS_FLAG ) && 
            (buttonPressEvent[BUTTON_RIGHT] == BUTTON_LONGPRESS_FLAG))
        {
            if (buttonPressEvent[BUTTON_ENTER] != 0)
            {
                // any useful function?
            }
            else if (buttonPressEvent[BUTTON_MEMORY] != 0)
            {
                // user feedback on display
                for (uint8_t i=0; i<10; i++)
                {
                    wr_SPI_buffer3(set_memory);
                    soft_delay(200);
                    wr_SPI_buffer3(clr_memory);
                    soft_delay(200);
                }
                clr_eeprom();
            }
            else
            {
                // Panel lock function
                if (panel_locked)
                {
                    wr_SPI_buffer3(clr_locked);
                    panel_locked = FALSE;
                }
                else
                {
                    wr_SPI_buffer3(set_locked);
                    panel_locked = TRUE;
                }
            }                
        }
    }
}

void triggerBacklitTimeout(void)
{
    if (backlitState == 0)
    {
        wr_SPI_buffer1(Backlit_on_cmd);
        backlitState = 1;
    }
    lastButtonActivity = runtime;
}

void checkBacklitTimeout(void)
{
    uint16_t backlitTimeoutInSeconds = illu_values[backlit] * 60;
    
    if ( backlitTimeoutInSeconds &&
         backlitState &&
         (runtime-lastButtonActivity)/TICKS_PER_SECOND > backlitTimeoutInSeconds )
    {
        wr_SPI_buffer1(Backlit_off_cmd);
        backlitState = 0;
    }
}

//*************************************************************************
// press Button
//*************************************************************************
void pressButton (uint8_t b)
{
    if (buttonPressEvent[b] == 0)
    {
        buttonPressEvent[b] = runtime;               // store time when button first pressed
    }
    
    if (buttonPressEvent[b] != BUTTON_LONGPRESS_FLAG)
    {
        if (runtime - buttonPressEvent[b] > BUTTON_LONGPRESS_TIME)
        {
            buttonPressEvent[b] = BUTTON_LONGPRESS_FLAG; 
            longPressButton(b);
        }
    }
    // activate shortpress actions at button release

    triggerBacklitTimeout();
}

//*************************************************************************
// release Button
//*************************************************************************
void releaseButton (uint8_t b)
{
    if (buttonPressEvent[b] > 0)
    {
        if (runtime - buttonPressEvent[b] > BUTTON_PRESS_DEBOUNCE)
        {
            buttonFunction(b);
        }            
    }
    buttonPressEvent[b] = 0;
}

//*************************************************************************
// read Buttons and Encoder
//*************************************************************************
void readButtons(void)
{
    uint8_t pinc = PINC;
    
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

    pull_encoder();
}

//*************************************************************************
// set UI "Active"
//*************************************************************************
void UI_status_display(void)
{
    static uint8_t status_old = 0xff;       // force update on first iteration
    uint8_t status;
    
    if (Standby_flag)
    {
        status = UI_STATUS_STANDBY;
    }
    else
    {
        status = PIND & (1<<Regulator);
    }
        
    if (status == status_old)
    {
        return;
    }
    
    switch (status)
    {
        case UI_STATUS_U:
            send_LCD_commands(UI_Activ_V);
            break;

        case UI_STATUS_I:
            send_LCD_commands(UI_Activ_A);
            break;

        case UI_STATUS_STANDBY:
            send_LCD_commands(UI_Standby);
            break;
    }
    
    status_old = status;
}


//*************************************************************************
// init LCD
//*************************************************************************
void display_fw_version(void)
{
    lcd_clr_digit_lines();
    lcd_clr_unit_display();
    lcd_clr_all_digits();
    wr_SPI_buffer3(set_V_point);
    print_value(U_act_cmd, VERSION*10);
    print_value_signed(I_act_cmd, BUILD);
}

//*************************************************************************
// init LCD
//*************************************************************************
void init_LCD (void)
{
    uint8_t contrast, backlit;
    
    soft_delay(1000);
    wr_SPI_buffer1(Clear_Screen_cmd);
    
    read_eeprom_display_settings(&contrast, &backlit);
    wr_SPI_buffer1(backlit ? Backlit_on_cmd : Backlit_off_cmd);
    wr_SPI_buffer1(LCD_Contrast_cmd+contrast);
    
    display_fw_version();
    soft_delay(3000);
    
    send_LCD_commands(LCD_inits);
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
}

//*************************************************************************
// calibration functions
//*************************************************************************
int16_t calibration_user(uint8_t ui_flag)
{
    uint16_t ocrval_start = ui_flag == UI_FLAG_U ? OCR1A : OCR1B;
    int16_t ocrval_diff = 0;
    do
    {
        pull_encoder();
        int diff = encode_read4();
        ocrval_diff += diff;
        if (ui_flag == UI_FLAG_U)
            OCR1A = ocrval_start + ocrval_diff ;
        else
            OCR1B = ocrval_start + ocrval_diff ;
        
        print_value_signed(I_act_cmd, ocrval_diff);
    } while ((PINC & (1<<B_Enter)) != 0);
    
    lcd_clr_digits(I_act_cmd);
    soft_delay(300);
    print_value_signed(I_act_cmd, ocrval_diff);
    soft_delay(1000);
    
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
    clr_standby_mode();
}    

void calibrate_U(void)
{
        const uint16_t u_ref_low = 5000;
        const uint16_t u_ref_high = 25000;
        
        uint16_t u_refs[] = { u_ref_low, u_ref_high };
        uint16_t ocrs[] = { 0, 0 };
        uint16_t u_meas[2];
        
        // Limit current to 100mA; calibration should be done in idle anyway
        set_Isoll(100);
        print_value(I_limit_cmd, 100);

        for (uint8_t i=0; i<2; i++)
        {
            set_standby_mode();
            wr_SPI_buffer3(set_V_symbol);

            set_Usoll(u_refs[i]);
            wr_SPI_buffer5(U_act_cmd, 0x51+i, 0x5B, 0x5A ,0x5C);    // "CAL1/2"
            print_value(U_limit_cmd, u_refs[i]);
            lcd_clr_digits(I_act_cmd);

            wait_for_standby_off();
            send_LCD_commands(UI_Activ_V);
            wr_SPI_buffer3(set_I4_underline);

            soft_delay(200);
            ocrs[i] = calibration_user(UI_FLAG_U);
            
            u_meas[i] = singleMeasurement(UI_FLAG_U);
        }

        calib_counts_per_20V = ocrs[1] - ocrs[0];
        calib_counts_offset_0V = ocrs[0] - calib_counts_per_20V / 4;   // subtract 5V to get offset @0V

        meas_counts_per_20V = u_meas[1] - u_meas[0];
        meas_counts_offset_0V = u_meas[0] - meas_counts_per_20V / 4;
}

void calibrate_I(void)
{
        const uint16_t i_ref_low = 300;     // use a 12V/5W car bulb as load
        const uint16_t i_ref_high = 1500;   // use a 12V/21W car bulb as load
        
        uint16_t i_refs[] = { i_ref_low, i_ref_high };
        uint16_t ocrs[] = { 0, 0 };
        uint16_t i_meas[2];
        
        // Use 12V for usage with bulbs
        set_Usoll(12000);
        print_value(U_limit_cmd, 12000);

        for (uint8_t i=0; i<2; i++)
        {
            set_standby_mode();
            wr_SPI_buffer3(clr_V_symbol);
            wr_SPI_buffer3(set_A_symbol);

            set_Isoll(i_refs[i]);
            wr_SPI_buffer5(U_act_cmd, 0x53+i, 0x5B, 0x5A ,0x5C);    // "CAL3/4"
            print_value(I_limit_cmd, i_refs[i]);
            lcd_clr_digits(I_act_cmd);

            wait_for_standby_off();
            send_LCD_commands(UI_Activ_A);

            soft_delay(200);
            ocrs[i] = calibration_user(UI_FLAG_I);
            
            i_meas[i] = singleMeasurement(UI_FLAG_I);
        }

        calib_counts_per_1200mA = ocrs[1] - ocrs[0];
        calib_counts_offset_0mA = ocrs[0] - calib_counts_per_1200mA / 4;   // subtract 300mA get offset @0mA

        meas_counts_per_1200mA = i_meas[1] - i_meas[0];
        meas_counts_offset_0mA = i_meas[0] - meas_counts_per_1200mA / 4;
}

void startup_calibration(void)
{
    if (is_calibration_valid())
    {
        read_calibration_data();
    }
    
    if ((!is_calibration_valid()) || ((PINC & ((1<<B_Left) | (1<<B_Right))) == 0))
    {
        wr_SPI_buffer1(Clear_Screen_cmd);
        wr_SPI_buffer3(set_V_Ulimit);
        wr_SPI_buffer3(set_A_Ilimit);
        
        calibrate_U();
        calibrate_I();
        save_calibration_data();
    }
    
    send_LCD_commands(LCD_inits);
    set_standby_mode();

    return;
}

void printContrast(uint8_t contrast)
{
    wr_SPI_buffer5(U_act_cmd, 0x5d, 0x5D, 0x50, 0x5C);
    print_value_signed(I_act_cmd, contrast+1);
}

void printIllu(uint8_t illum)
{
    wr_SPI_buffer5(U_act_cmd, 0x5d, 0x5b, 0x5b, 0x51);
    print_value_signed(I_act_cmd,  illu_values[illum]);
}

    
void startDisplaySettings(void)
{
    lcd_clr_unit_display();
    lcd_clr_digit_lines();
    lcd_clr_all_digits();
    wr_SPI_buffer3(set_I4_underline);

    read_eeprom_display_settings(&contrast, &backlit);
    printContrast(contrast);
    menu_contrast = MENU_CONTRAST;
    
    displaySettingsActive = TRUE;
    
}

void handleDisplaySettings(void)
{
    int diff = encode_read4();
        
    if (menu_contrast == MENU_CONTRAST)
    {
        if (diff > 0 && contrast < 7)
        {
            contrast++;
            wr_SPI_buffer1(LCD_Contrast_cmd + contrast);
            printContrast(contrast);
        }
        else if (diff < 0 && contrast > 0)
        {
            contrast--;
            wr_SPI_buffer1(LCD_Contrast_cmd + contrast);
            printContrast(contrast);
        }
    }
    else
    {
        if ((diff > 0) && (backlit < 7))
        {
            backlit++;
            wr_SPI_buffer1(Backlit_on_cmd);
            printIllu(backlit);
            
        }
        else if ((diff < 0) && (backlit > 0))
        {
            backlit--;
            if (backlit == 0)
            {
                wr_SPI_buffer1(Backlit_off_cmd);
            }
            printIllu(backlit);
        }
    }
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
    init_Interrupt();
    
    startup_calibration();

    init_UIlimits();
    set_standby_mode();

    // Main loop ---------------------------------------------------------
    while(TRUE) 
    {
        save_print_UIlimit();
        UI_status_display();
        readButtons();
        flash_PrgNo();
        Measurement();
        if (displaySettingsActive)
            handleDisplaySettings();
    }
}
