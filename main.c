/* 
 * File:   main.c
 * Author: moritz
 *
 * Created on May 27, 2024, 9:50 AM
 */

#include <avr/io.h>

#define F_CPU 8000000L
#include <avr/delay.h>
#include <avr/interrupt.h>

// if defined, debug info is send via UART
#define DEBUG

#define rote_beete 0

// ADC-channels
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3

#define ADC_TRANSISTOR_LEFT ADC2
#define ADC_TRANSISTOR_RIGHT ADC0
// how bright the lights AC-component has to be, to be considered as HIGH
#define LIGHT_HIGH_THRESHOLD 10 // its just a random number , needs to be tested irl

#define FORWARD 0
#define BACKWARD 1

#define true 1
#define false 0

uint16_t global_time;

void UART_init() {
    DDRD |= (1<<PD1);
    
    // 8000000L/(16*2400)-1
    unsigned int ubrr = 207;
    // baud-rate
    UBRRH = (unsigned char) (ubrr >> 8);
    UBRRL = (unsigned char) (ubrr);
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
    for (unsigned char i = 0; i < len; i++) {
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

void timer1_init() {
    // Set Timer1 to Fast PWM mode
    TCCR1A |= (1 << WGM10) | (1<< COM1A1) | (1<< COM1B1); // Fast PWM
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Prescaler 64
 
    // Enable Timer1 overflow interrupt
    TIMSK |= (1 << TOIE1);
   
    sei(); // Enable global interrupts
}



ISR(TIMER1_OVF_vect) {
    global_time += 1;
}

/*
 * rotates the robot, until its facing the blinking lights
 * untested
 */
void rotate_to_lightsource() {
    unsigned char mean_l = 0, mean_r = 0;
    // signed 16 bit, so that the substraction doesnt return a high result, if mean>transistor
    int16_t transistor_l = 0, transistor_r = 0;
    
    float mean_weight = .9;
    unsigned char timer_started_l = false;
    unsigned char l_h = 0, l_l = 0;
    unsigned char high_l = 0, low_l = 0;
    
    unsigned char edge_height = 0;
    
    uint16_t time = 0;
    
    // reading the first values to get a reference value to compare new values to
    transistor_l = ADC_get_value(ADC_TRANSISTOR_LEFT);
    transistor_r = ADC_get_value(ADC_TRANSISTOR_RIGHT);
    
    mean_l = transistor_l;
    mean_r = transistor_r;

    // start turning left
    //gangschaltung(BACKWARD, FORWARD);
    //gaspedal(50, 50);

    while (1) {
        transistor_l = ADC_get_value(ADC_TRANSISTOR_LEFT);
        transistor_r = ADC_get_value(ADC_TRANSISTOR_RIGHT);
    
        // gewicchteter mittelwert, alter mittelwert geht mehr ein als neuer wert
        mean_l = mean_weight*mean_l + (1-mean_weight)*transistor_l;
        mean_r = mean_weight*mean_r + (1-mean_weight)*transistor_r;
        
        if(transistor_l > mean_l){
            l_h += 1;
            l_l = 0;    // setzt rauschabblockung für low zurück, weil ein high kam
            // wird nur als high erkannt, wenn mehrmals am stück high gewesen -> blockt rauschen ab
            if(l_h == 5){
                // wenn davor low gewesen (steigende flanke) -> zeitmessung starten
                if (!timer_started_l) {
                    global_time = 0;
                    high_l = transistor_l;
                    
                    timer_started_l = true;
                }

                l_h = 0;
            }
        }
        else {
            l_l += 1;
            l_h = 0;
            // wird nur als low erkannt, wenn mehrmals am stück low gewesen -> blockt rauschen ab
            if(l_l == 5){
                // wenn vorher high gewesen (fallende flanke) -> zeit auslesen
                if(timer_started_l) {
                    time = global_time * 2;
                    low_l = transistor_l;
                }    
                
                timer_started_l = false;
                l_l = 0;
            }
        }
        
        // wenn gemessene zeit ~100ms -> blinken erkannt
        if (time >= 50 && time <= 150){
            PORTD &= ~(1<<PD5);
        }
        else{
             PORTD |= (1<<PD5);
        }
        
        
        
        //height = high_l - low_l;
    }
}

/*
 * 
 */
int main(int argc, char** argv) {
    // define the pins of D2 and D3 as outputs
    DDRD |= (1 << PD5) | (1 << PD4);
    // turn on D2
    PORTD |= (1 << PD4);

    UART_init();
    ADC_init();
    timer1_init();
    
    while (1) {
             
        rotate_to_lightsource();
 
    }
}

