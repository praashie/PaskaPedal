/*
Firmware source of PraashPedal.

PraashPedal is a DIY project built on two unused KORG guitar volume pedals,
turning them into a class-compliant(-enough-for-Linux) USB MIDI controller,
used for controlling synthesizers and guitar effects.

Runs on an Atmel ATTiny84, with 16MHz clock.
Requires V-USB.

Sorry for writing a big lump of ugly code.

Elmo von Weissenberg/praash
*/

#include <stdlib.h>


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "usbdrv.h"
#include <util/delay.h>

//==========PIN DEFINITIONS==============
//Pins A0, A1 reserved for USB
#define PEDAL_0_PIN 4
#define PEDAL_1_PIN 5
#define DEBUG_LED 2

#define FOOT_0_PIN 6
#define FOOT_1_PIN 7

#define CAL_PIN 3
//=======================================

#define NUM_PEDALS 2
#define NUM_FEET 2

#define CAL_MIN 1
#define CAL_MAX 2

//Let's pretend we're developing with Arduino
#define setInputPin(P,BIT)  DDR##P &= ~(1 << BIT) //e.g. (A,2) -> DDRA &= ~(1 << BIT)
#define setOutputPin(P,BIT) DDR##P |=   1 << BIT

#define pinHigh(P,BIT)   PORT##P |=    1 << BIT
#define pinLow(P,BIT)    PORT##P &=  ~(1 << BIT)
#define pinToggle(P,BIT) PORT##P ^=    1 << BIT

#define digitalRead(P,BIT) (PIN##P & (1 << BIT))

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/* http://maxembedded.com/2011/06/the-adc-of-the-avr/ */
uint16_t adc_read(uint8_t ch) {
  // select the corresponding channel 0~7
  // ANDing with ’7′ will always keep the value
  // of ‘ch’ between 0 and 7
  ch &= 0b00000111;  // AND operation with 7
  ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing

  // start single convertion
  // write ’1′ to ADSC
  ADCSRA |= (1<<ADSC);

  // wait for conversion to complete
  // ADSC becomes ’0′ again
  // till then, run loop continuously
  while(ADCSRA & (1<<ADSC));

  return (ADC);
}

/*The USB configuration descriptor that tells the host computer
  what we want to do. In this case, we mostly copycat the descriptor
  example from the USB MIDI specification sheet.
  This half-assed class-compliance must be the reason Macs reject us.
  Writing this is a bebugging nightmare.
*/
PROGMEM const char usbDescriptorConfiguration[DESCRIPTOR_SIZE] = {
            9,      //bLength
            0x02,   //bDescriptorType (Configuration)
            DESCRIPTOR_SIZE, 0,   //wTotalLength
            2,      //bNumInterfaces
            0,      //bConfigurationValue
            0,      //Index of string, 0
            0b10000000,
            50,     //50*2mA = 100mA

                //AC Interface descriptor
                9, //size
                0x04, //descriptor type = interface
                0, //interface number
                0, //alternate interface
                0, //numEndPoints
                0x01,//Interface class, AUDIO
                0x01,//Interface subclass, AUDIO_CONTROL
                0x00,//unused
                0x00,

                //CS. AC interface descriptor
                9,
                0x24, //CS_INTERFACE
                0x01, //HEADER
                1, 0, //Class spec. 1.0
                43, 0,//total length of class specific descriptors
                1, //number of streaming interfaces
                1,//MS interface 1 belongs to this

                    //MidiStreaming descriptor
                    9,
                    0x04, //interface descriptor
                    1, //interface index
                    0, //alternateS
                    1, //endpoints
                    0x01, //int. class, AUDIO
                    0x03, //subclass, midistreaming
                    0,
                    0,

                    //class specific ms interface descriptor
                    7,
                    0x24, //descriptor type, class specific
                    0x01, //MS_HEADER subtype
                    1, 0, //spec 1.0
                    25, 0,//total length of class-specific descr.

                        //Midi IN jack descriptor
                        6,
                        0x24,
                        0x02, //descr. type, MIDI_IN_JACK
                        0x01, //subtype, EMBEDDED
                        1, //ID 1
                        0,

                        //Endpoint descr.
                        7,
                        0x05, //ENDPOINT
                        0b10000001, //Endpoint 1, data source (0 is control)
                        0b00000010, //bmAttributes: interrupt data
                        0, 8, //8 bytes per packet
                        0,
                            //CS BULK OUT
                            5,
                            0x25, //CS_ENDPOINT
                            1, //MS_GENERAL
                            1, //Number of embedded MIDI IN jacks
                            1 //Jack ID
/*
                        //Element descriptor
                        12,
                        0x24,
                        0x04, //descr. subtype element
                        1, //nr. input pins
                        1, //baSourceID
                        1, //baSourcePin
                        0, //nr. output pins
                        0, //bInTerminalLink
                        0, //bOutTerminalLink
                        1, //bElCapsSize
                        0b00110000, //bmElementCaps, General Midi support
                        0*/
};

USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {
        return 0; // do nothing for now
}


float EEMEM eeprom_calibration_min[NUM_PEDALS] = {0};
float EEMEM eeprom_calibration_max[NUM_PEDALS] = {0};

//Using port A: Pins PA2 and PA3
//const char pedal_pin[2] = {2, 3};
typedef struct {
    const char pin;
    float mapMin;
    float mapMax;
    const unsigned char midiCC;
    float smoothedValue;
    uchar currentMidiValue;
    uchar lastSentMidiValue;
} MidiPedal_t;

MidiPedal_t pedals[NUM_PEDALS] = {{PEDAL_0_PIN, 99999, 0, 12},
                         {PEDAL_1_PIN, 99999, 0, 13}};

//Fancy copy-paste-based button debouncing code
#define DEBOUNCE_MAXIMUM 10
#define BUTTON_DOWN 1
#define BUTTON_PRESSED 2
#define BUTTON_RELEASED 4

typedef struct {
    uchar state;
    uchar integrator;
} Button_t;

void update_button(Button_t *b, uchar input) {
    b->state &= BUTTON_DOWN;
    uchar last = b->state;

    if (input) {
        if (b->integrator > 0) {
          b->integrator--;
        }
    }
    else if (b->integrator < DEBOUNCE_MAXIMUM) {
        b->integrator++;
    }

    if (b->integrator == 0) {
        b->state = 0;
        if (last) {
            b->state |= BUTTON_RELEASED;
        }
    } else if (b->integrator >= DEBOUNCE_MAXIMUM) {
        b->state = 1;
        if (!last) {
            b->state |= BUTTON_PRESSED;
        }
        b->integrator = DEBOUNCE_MAXIMUM;  /* defensive code if integrator got corrupted */
    }
}

Button_t cal_button = {0};

//"Nice" structs for the switches as well
typedef struct {
    uchar pin;
    uchar midiCC;
    Button_t button;
} FootSwitch_t;
FootSwitch_t footswitch[NUM_FEET] = {{FOOT_0_PIN, 16, {0}}, {FOOT_1_PIN, 17, {0}}};

//Flashing parameters
unsigned int debugLedCounter = 0;
unsigned int debugLedInterval = 0;

unsigned int calibState = 0;
unsigned int calibPedal = 0;

//unsigned long nextMessageMillis = millis();
#define SMOOTH_WEIGHT 0.08f //Pedal smoothing weight; tuned using the original Arduino prototype


#define midiCC(p, cc, value) midiMessage(p, 0x0B, 0xB0, cc, value)
#define midiNoteOn(p, noteN) midiMessage(p, 0x08, 0x80, noteN, 127)
#define midiNoteOff(p, noteN) midiMessage(p, 0x09, 0x90, noteN, 127)

unsigned char midiMessage(uchar **p, uchar a, uchar b, uchar c, uchar d){
    *p = malloc(sizeof(uchar)*4);
    uchar *m = *p;
    m[0] = a; //Cable index 0, message Control Change
    m[1] = b; //Control change, channel 0
    m[2] = c;
    m[3] = d; //padded

    return 4;
}

int main() {

    //============================
    //Init
    //============================
    uchar i;
    for (i=0; i<NUM_PEDALS; i++) {
        setInputPin(A, pedals[i].pin);
        //Read calibration from EEPROM
        pedals[i].mapMin = eeprom_read_float(&eeprom_calibration_min[i]);
        eeprom_busy_wait();
        pedals[i].mapMax = eeprom_read_float(&eeprom_calibration_max[i]);
        eeprom_busy_wait();
    }
    for (i=0; i<NUM_FEET; i++) {
        setInputPin(A, footswitch[i].pin);
        pinHigh(A, footswitch[i].pin);
    }

    setInputPin(A, CAL_PIN);
    pinHigh(A, CAL_PIN); //pull-up

    setOutputPin(A, DEBUG_LED);

    //ADC params
    ADMUX = 0; //AREF = AVcc
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);

    //V-USB example code, with original comments
    wdt_enable(WDTO_1S); // enable 1s watchdog timer

    usbInit();

    usbDeviceDisconnect(); // enforce re-enumeration
    for(i = 0; i<250; i++) { // wait 500 ms
        wdt_reset(); // keep the watchdog happy
        _delay_ms(2);
    }
    usbDeviceConnect();

    for(i = 0; i<250; i++) { // wait 500 ms
        wdt_reset(); // keep the watchdog happy
        _delay_ms(2);
    }

    //pinMode(2, OUTPUT);

    sei(); // Enable interrupts after re-enumeration


    //============================
    //Loop
    //============================
    while(1) {

        //Flash the LED if calibrating
        if (debugLedInterval > 0) {
            if (debugLedCounter == 0) {
                pinToggle(A,DEBUG_LED);
            }
            debugLedCounter = (debugLedCounter + 1) % debugLedInterval;
        } else {
            if (usbInterruptIsReady()) {
                pinHigh(A,DEBUG_LED);
            } else {
                pinLow(A,DEBUG_LED);
            }
        }

        update_button(&cal_button, digitalRead(A,CAL_PIN));

        //Handle pedals
        for (i = 0; i < NUM_PEDALS; i++) {
            float r = adc_read(pedals[i].pin)/1023.0 + 0.01;
             //r = (10.0/(adc_read(pedals[i].pin) / 1023.0 + 0.001)) - 10.0;
             r = (10.0*r)/(1.0 - r);

            //Exponential smoothing
            pedals[i].smoothedValue = SMOOTH_WEIGHT * r + (1 - SMOOTH_WEIGHT) * pedals[i].smoothedValue;

            int v = map(pedals[i].smoothedValue, pedals[i].mapMin, pedals[i].mapMax, 127, 0);
            if (v < 0) {v = 0;}
            if (v > 127) {v = 127;}
            pedals[i].currentMidiValue = (unsigned char) (v & 0x7F);

            //Pedal moved (or noise), send a CC message
            if (pedals[i].currentMidiValue != pedals[i].lastSentMidiValue) {
                if (usbInterruptIsReady()) {
                    uchar *data;
                    size_t len = midiCC(&data, pedals[i].midiCC, pedals[i].currentMidiValue);
                    usbSetInterrupt(data, len);
                    free(data);
                    pedals[i].lastSentMidiValue = pedals[i].currentMidiValue;
                }
            }
        }

        for (i = 0; i < NUM_FEET; i++) {
            if (!(footswitch[i].button.state & (BUTTON_PRESSED | BUTTON_RELEASED))) {
                update_button(&footswitch[i].button, digitalRead(A,footswitch[i].pin));
            } else if (usbInterruptIsReady()) {
                uchar *data;
                size_t len = midiCC(&data, footswitch[i].midiCC, 127*(footswitch[i].button.state & BUTTON_DOWN));
                usbSetInterrupt(data, len);
                free(data);
                update_button(&footswitch[i].button, digitalRead(A,footswitch[i].pin));
            }
        }

        //manual calibration: easy, fun, and reliable
        if (cal_button.state & BUTTON_PRESSED) {
            if (calibState == 0) {
                debugLedInterval = 100;
            }else if (calibState == CAL_MIN) {
                for (i=0; i < NUM_PEDALS; i++) {
                    pedals[i].mapMin = pedals[i].smoothedValue;
                    eeprom_update_float(&eeprom_calibration_min[i], pedals[i].mapMin); //Write to EEPROM
                    eeprom_busy_wait();
                }
                debugLedInterval = 50;
            } else if (calibState == CAL_MAX) {
                for (i=0; i < NUM_PEDALS; i++) {
                    pedals[i].mapMax = pedals[i].smoothedValue;
                    eeprom_update_float(&eeprom_calibration_max[i], pedals[i].mapMax);
                    eeprom_busy_wait();
                }
                debugLedInterval = 0;
            }
            debugLedCounter = 0;
            calibState = (calibState + 1) % 3;
        }
        _delay_ms(1);
        wdt_reset(); // keep the watchdog happy
        usbPoll();

    }
    return 0;
}
