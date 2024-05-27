/* 
 * File:   main.c
 * Author: moritz
 *
 * Created on May 27, 2024, 9:50 AM
 */

#include <avr/io.h>
#include <definitions.h>

void UART_init(){
    // 8000000L/(16*2400)-1
    unsigned int ubrr = 208;
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
    // waiting until sending is completed
    while(!(UCSRA & (1<<TXC)));
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
    
}
/*
 * starts a conversion of the given ADC-channel and returns thre result
 * c: ADC-channel
 */
char ADC_get_value(char c){
    
}

/*
 * 
 */
int main(int argc, char** argv) {

    UART_init();
    
    while(1){
        
    }
}

