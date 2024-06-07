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

#define Q1 ADC2
#define Q2 ADC3

#define ADC_TRANSISTOR_LEFT Q1
#define ADC_TRANSISTOR_RIGHT Q2
// how high a edge has to be to be considered as not noise
#define EDGE_HEIGHT_THRESHOLD 10 // its just a random number , needs to be tested irl

#define true 1
#define false 0

#define left 0
#define right 1

uint16_t global_time_blinking_led[2];

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

void ADC_init() {
    // AVcc with external C at AREF-Pin, left adjusted result 
    ADMUX = (1 << REFS0) | (1 << ADLAR);
    // enable ADC, prescaler 64
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

/*
 * starts a conversion of the given ADC-channel and returns the result
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

/*
 * interrupt when timer1 is overflowing
 */
ISR(TIMER1_OVF_vect) {
    global_time_blinking_led[left] += 1;
    global_time_blinking_led[right] += 1;
}

/*
 * rotates the robot until its facing the blinking leds
 */
void turn_towards_blinking_led() {
    unsigned char mean[2] = {0, 0};
    // signed 16 bit, so that the substraction doesnt return a high result, if mean>transistor
    int16_t transistor[2] = {0, 0};
    // used for calculating the mean, so that the old mean has more weight as the new value
    float mean_weight = .9;

    unsigned char timer_started[2] = {false, false};
    // some variables for noisecanceling the edge-detection
    unsigned char l_h = 0, l_l = 0, r_h = 0, r_l = 0;
  
    unsigned char high_value[2] = {0, 0}, low_value[2] = {0, 0};
    
    unsigned char edge_height[2] = {0, 0};
    // time between a rising and a falling edge
    uint16_t time[2] = {0, 0};
    
    // reading the first values to get a reference value to compare new values to
    transistor[left] = ADC_get_value(ADC_TRANSISTOR_LEFT);
    transistor[right] = ADC_get_value(ADC_TRANSISTOR_RIGHT);
    
    mean[left] = transistor[left];
    mean[right] = transistor[right];

    // start turning left

    while (1) {
        transistor[left] = ADC_get_value(ADC_TRANSISTOR_LEFT);
        transistor[right] = ADC_get_value(ADC_TRANSISTOR_RIGHT);
    
        // calculates the mean, old mean has more weight then the new value of the transistor
        mean[left] = mean_weight*mean[left] + (1-mean_weight)*transistor[left];
        mean[right] = mean_weight*mean[right] + (1-mean_weight)*transistor[right];
        // detecting rising / falling edges on the left transistor
        if(transistor[left] > mean[left]){
            l_h += 1;
            l_l = 0;    // setzt rauschabblockung für low zurück, weil ein high kam
            // wird nur als high erkannt, wenn mehrmals am stück high gewesen -> blockt rauschen ab
            if(l_h == 5){
                // wenn davor low gewesen (steigende flanke) -> zeitmessung starten
                if (!timer_started[left]) {
                    global_time_blinking_led[left] = 0;
                    high_value[left] = transistor[left];
                    
                    timer_started[left] = true;
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
                if(timer_started[left]) {
                    time[left] = global_time_blinking_led[left] * 2;
                    low_value[left] = transistor[left];
                }    
                
                timer_started[left] = false;
                l_l = 0;
            }
        }
        
        // detecting rising / falling edges on the right transistor
        if(transistor[right] > mean[right]){
            r_h += 1;
            r_l = 0;    // setzt rauschabblockung für low zurück, weil ein high kam
            // wird nur als high erkannt, wenn mehrmals am stück high gewesen -> blockt rauschen ab
            if(r_h == 5){
                // wenn davor low gewesen (steigende flanke) -> zeitmessung starten
                if (!timer_started[right]) {
                    global_time_blinking_led[right] = 0;
                    high_value[right] = transistor[right];
                    
                    timer_started[right] = true;
                }

                r_h = 0;
            }
        }
        else {
            r_l += 1;
            r_h = 0;
            // wird nur als low erkannt, wenn mehrmals am stück low gewesen -> blockt rauschen ab
            if(r_l == 5){
                // wenn vorher high gewesen (fallende flanke) -> zeit auslesen
                if(timer_started[right]) {
                    time[right] = global_time_blinking_led[right] * 2;
                    low_value[right] = transistor[right];
                }    
                
                timer_started[right] = false;
                r_l = 0;
            }
        }
        
        // wenn gemessene zeit ~100ms -> blinken erkannt
        if (time[left] >= 50 && time[left] <= 150){
#ifdef DEBUG
            PORTD &= ~(1<<PD5);
#endif
            edge_height[left] = high_value[left] - low_value[left];
        }
#ifdef DEBUG
        else{
             PORTD |= (1<<PD5);
        }
#endif
        if (time[right] >= 50 && time[right] <= 150){
#ifdef DEBUG
            PORTD &= ~(1<<PD4);
#endif
            edge_height[right] = high_value[right] - low_value[right];
            //UART_send(edge_height[right]);
        }
#ifdef DEBUG
        else{
             PORTD |= (1<<PD4);
        }
#endif
        
        /* 
         * if both are almost equal, the robot should be facing the light
         * doesnt trigger if the left one is below a specified value, so that it doesnt trigger at 0
         */
        if((edge_height[left] >= (edge_height[right] - 10)) && (edge_height[left] <= (edge_height[right] + 10)) && (edge_height[left] > EDGE_HEIGHT_THRESHOLD)){
            // stop rotating
            //return;
        }
        // turn in the direction of the brighter light
        if(edge_height[left] > edge_height[right]){
            // turn left
        }
        else{
            // turn right
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
    PORTD |= (1 << PD4);

    UART_init();
    ADC_init();
    timer1_init();
    
    while (1) {
             
        turn_towards_blinking_led();
    }
}

