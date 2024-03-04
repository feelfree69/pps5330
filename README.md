# Reverse Engineered Firmware for the ELV PPS5330 Power Supply

The PPS5330 Power supply is controlled by an ATMEGA88PA microcontroller.
The firmware has been developed from scratch by Rolf D., see this thread in the german forum: https://www.mikrocontroller.net/topic/499101

## WARNING!
```
If you decide to flash this firmware to your PPS5330, you'll lose the original firmware, 
and you won't get it back, unless you pay ELV for reprogramming the ATMEGA88.
(ELV's firmware is closed source). 
```
YOU HAVE BEEN WARNED!


## STATUS

WORKING:
- All Basic functionality is working, the Power Supply is in a usable state 
- Added Temperature measuring of the internal heatsink with display on LCD
- PWM controlled FAN dependend on Heatsink's Temperature 
- Voltage Calibration 
- Current Calibration
- Measure transformer temperature
- Menu with LCD contrast/backlit settings
- Calibration of internal ADC measurements

NICE TO HAVE:
- Calibration of temperature sensor
- Temperature compensation (if needed)

IDEAS with hardware modifications:
- serial remote control (needs re-wiring of button input pins)

## HISTORY

 - 199b7
    - partly revised button handling (using timer interrupt for debouncing)
    - added seperate Contrast / Backlit settings like in original firmware, with backlit timeout in minutes

 - 199b6
    - added calibration of internal ADC measurements for U and I

 - 199b5
    - code cleanup, cosmetics

 - 199b4
    - add menu for display-settings (long press U/I), encoder/arrows for changing, enter for exit
    - recall-button: blinky display of stored values before applying
    - remove more magic numbers in code
    - rework button handling (still room for improvement...)

 - 199b3  
    - added T2 (transformer) measurement
    - toggle T1/T2 at 2-digit memory section
    - decrease interval for temperature measurements

 - 199b2  
    - Added Current calibration

 - 199b1  
    - Minor code cleanups (or mess-up's, dependend on your view..)
    - Display firmware version at startup
    - Added Voltage calibration

- V1.50a Original version, see link above
 