/* 
 * File:   main.c
 * Author: moritz
 *
 * Created on May 27, 2024, 9:50 AM
 */

#include <avr/io.h>

// ADC-channels
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3

void UART_init(){
    // 8000000L/(16*2400)-1
    unsigned int ubrr = 207;
    // baud-rate
    UBRRH = (unsigned char)(ubrr>>8);
    UBRRL = (unsigned char)ubrr;
    //enable reciever and transmitter
    UCSRB = (1<<RXEN) | (1<<TXEN);
    // 8 bit data, 1 stop-bit, no parity
    UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
}
// sends one byte via UART
void UART_send(char msg){
    // waiting untill data-buffer ist empty
    while(!(UCSRA & (1<<UDRE)));
    // writing the data
    UDR = msg;
}
// recieves one byte via UART
char UART_recieve(){
    // waiting for data
    while(!(UCSRA & (1<<RXC)));
    
    char msg = UDR;
    
    return msg;
}
/* 
 * sends a string via UART
 * msg -> pointer to the string
 * len -> length of the string
 */
void UART_send_string(char *msg, char len){
    for(char i = 0; i < len; i++){
        UART_send(msg[i]);
        if(msg[i] == '\0') break;
    }
}

void ADC_init(){
    // AVcc with external C at AREF-Pin, left adjusted result 
    ADMUX = (1<<REFS0) | (1<<ADLAR);
    // enable ADC, prescaler 64
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
}
/*
 * starts a conversion of the given ADC-channel and returns thre result
 */
char ADC_get_value(char adc_channel){
    char result;
    // select the requested ADC-channel
    ADMUX |= (adc_channel<<MUX0);
    // start conversion
    ADCSRA |= (1<<ADSC);
    // waiting for conversion to finish
    while(ADCSRA & (1<<ADSC));
    
    result = ADCH;
    return result;
}

void rotate_to_lightsource(){
    unsigned char mean = 0;
    unsigned char values = 0;
    
    values = ADC_get_value(ADC0);
    
    
}

/*
 * 
 */
int main(int argc, char** argv) {

    UART_init();
    //ADC_init();
    
    while(1){
        UART_send('0');
    }
}

