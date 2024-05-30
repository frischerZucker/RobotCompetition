/* 
 * File:   main.c
 * Author: moritz
 *
 * Created on May 27, 2024, 9:50 AM
 */

#include <avr/io.h>
#include <avr/delay.h>
#include <definitions.h>

// if defined, debug info is send via UART
#define DEBUG

#define F_CPU 8000000

// ADC-channels
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3

#define ADC_TRANSISTOR_LEFT ADC2
#define ADC_TRANSISTOR_RIGHT ADC3
// how bright the lights AC-component has to be, to be considered as HIGH
#define LIGHT_HIGH_THRESHOLD 50 // its just a random number , needs to be tested irl

void UART_init() {
    // 8000000L/(16*2400)-1
    unsigned int ubrr = 207;
    // baud-rate
    UBRRH = (unsigned char) (ubrr >> 8);
    UBRRL = (unsigned char) ubrr;
    //enable reciever and transmitter
    UCSRB = (1 << RXEN) | (1 << TXEN);
    // 8 bit data, 1 stop-bit, no parity
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

/*
 * sends one byte via UART
 */
void UART_send(char msg) {
    // waiting untill data-buffer ist empty
    while (!(UCSRA & (1 << UDRE)));
    // writing the data
    UDR = msg;
}

/* 
 * recieves one byte via UART
 * untested
 */
char UART_recieve() {
    // waiting for data
    while (!(UCSRA & (1 << RXC)));

    char msg = UDR;

    return msg;
}

/* 
 * sends a string via UART
 * msg -> pointer to the string
 * len -> length of the string
 */
void UART_send_string(char *msg, char len) {
    for (char i = 0; i < len; i++) {
        if (msg[i] == '\0') break;
        UART_send(msg[i]);
    }
}

// untested
void ADC_init() {
    // AVcc with external C at AREF-Pin, left adjusted result 
    ADMUX = (1 << REFS0) | (1 << ADLAR);
    // enable ADC, prescaler 64
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

/*
 * starts a conversion of the given ADC-channel and returns the result
 * untested
 */
char ADC_get_value(char adc_channel) {
    char result;
    // select the requested ADC-channel
    ADMUX &= ~(0x0F);
    ADMUX |= (adc_channel << MUX0);
    // start conversion
    ADCSRA |= (1 << ADSC);
    // waiting for conversion to finish
    while (ADCSRA & (1 << ADSC));

    result = ADCH;
    return result;
}

/*
 * rotates the robot, until its facing the blinking lights
 * untested
 */
void rotate_to_lightsource() {
    unsigned char mean = 0;
    unsigned char transistor_left = 0, transistor_right = 0;

    // reading the first values to get a reference value to compare new values to
    transistor_left= ADC_get_value(ADC_TRANSISTOR_LEFT);
    transistor_right = ADC_get_value(ADC_TRANSISTOR_RIGHT);
    mean = (transistor_left + transistor_right) / 2;
    
    // start turning left
    gangschaltung(BACKWARD, FORWARD);
    gaspedal(50, 50);
    
    while (1) {
        // delay to match the sampling rate roughly to the flashing LEDs frequency (5 Hz =^ 200 ms)
        _delay_ms(100);
        
        transistor_left= ADC_get_value(ADC_TRANSISTOR_LEFT);
        transistor_right = ADC_get_value(ADC_TRANSISTOR_RIGHT);

        // update reference (DC-component)
        mean = (mean + transistor_left + transistor_right) / 3;
        
#ifdef DEBUG
        UART_send(';');
        UART_send(transistor_left);
        UART_send(transistor_right);
#endif
        
        /* 
         * removing the DC-component from the values -> filter that lets only the blinking light pass
         * if sampling rate matches the frequency of the LEDs, there should be a square signal with low and high levels
         */
        transistor_left -= mean;
        transistor_right -= mean;
        
#ifdef DEBUG
        UART_send(mean);
        UART_send(transistor_left);
        UART_send(transistor_right);
#endif
        
        if(transistor_left < LIGHT_HIGH_THRESHOLD && transistor_right < LIGHT_HIGH_THRESHOLD){
            // both transistors dont notice a flashing light -> turn left
            gangschaltung(BACKWARD, FORWARD);
#ifdef DEBUG
                UART_send('l');
#endif
        }
        else if(transistor_left > LIGHT_HIGH_THRESHOLD && transistor_left < LIGHT_HIGH_THRESHOLD){
            // only the left transistor notices a flashing light -> turn left
            gangschaltung(BACKWARD, FORWARD);
#ifdef DEBUG
                UART_send('l');
#endif
        }
        else if(transistor_left < LIGHT_HIGH_THRESHOLD && transistor_right > LIGHT_HIGH_THRESHOLD){
            // only the right transistor notices a flashing light -> turn right
            gangschaltung(FORWARD, BACKWARD);
#ifdef DEBUG
                UART_send('r');
#endif
        }
        else{
            /*
             * both transistors notice a flashing light -> stop rotating and return from the function
             * 
             * maybe this will be enough, but i think we will have to look for the point where they both get the max amount of light
             */
            gaspedal(0, 0);
            gangschaltung(FORWARD, FORWARD);
#ifdef DEBUG
                UART_send('x');
#endif
            return;
        } 
    }
}

/*
 * 
 */
int main(int argc, char** argv) {
    // define the pins of D2 and D3 as outputs
    DDRD |= (1 << PD5) | (1 << PD4);
    // turn on D2
    PORTD |= (1<<PD4);

    UART_init();
    ADC_init();

    char temp;

    while (1) {
        UART_send_string("test\0", 5);
    }
}

