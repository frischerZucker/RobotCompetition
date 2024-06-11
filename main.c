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
#define EDGE_HEIGHT_THRESHOLD 5
// for calculating the mean, so that the old mean has more weight as the new value
#define MEAN_WEIGHT .9f

#define true 1
#define false 0

#define LEFT 0
#define RIGHT 1

#define FORWARD 1
#define BACKWARD 0

#define TRIG_PIN PD0
#define ECHO_PIN PD3

uint16_t global_time_target_led_frequency[2];
uint16_t time_status_led;

volatile uint16_t overflowCount;
volatile uint16_t pulseWidth;
volatile uint8_t distanceMeasured;
volatile uint8_t risingEdge;

void ext_int1_init() {
    // Set INT1 to trigger on rising edge
    MCUCR |= (1 << ISC11) | (1 << ISC10); // Rising edge
    GICR |= (1 << INT1); // Enable INT1 interrupt
}

float calculateDistance(uint16_t pulseWidth) {
    // Calculate distance in cm
    float distance = pulseWidth * 0.0343 * 8 / 2; // distance in cm
    return distance;
}

void SensorInit() {
    DDRD |= (1 << TRIG_PIN); // Set trigger pin as output
    DDRD &= ~(1 << ECHO_PIN); // Set echo pin as input
}

void trigSensor() {
    // Trigger the ultrasonic sensor
    PORTD |= (1 << TRIG_PIN); // Send a 10us high pulse
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);
    distanceMeasured = 0;

    risingEdge = 1; // Prepare to detect rising edge again
    //UART_send(pulseWidth);
    MCUCR |= (1 << ISC10); // Rising edge

    overflowCount = 0; // Reset overflow count
}

void stopRobot() {
    // without the first iterations before the distance is measured for the first time will trigger as distance < 8
    static char c = 0;

    cli();
    if (((distanceMeasured == 1) && (overflowCount > 30)) || (overflowCount > 500)) {
        trigSensor();
    }


    float distance = calculateDistance(pulseWidth);
    //if(distance > 20) distance = 20;
    sei();

    if (distance) c = 1;

    if ((distance < 8) && c) {

        PORTD &= ~(1 << PD5);
    } else {
        PORTD |= (1 << PD5);
    }

    //UART_send(distance); 
}

void init_M(void) {
    DDRB |= (1 << PB1) | (1 << PB2);
    DDRD |= (1 << PD6) | (1 << PD7);

    TCCR1A |= (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);

    OCR1A = 0;
    OCR1B = 0;
}

void Gangschaltung(char richtung_L, char richtung_R) {

    // 1 = vorw�rts    0 = r�ckw�rts
    if (richtung_L == 0) {
        PORTD &= ~(1 << PD6);
    } else {
        PORTD |= (1 << PD6);
    }

    if (richtung_R == 0) {
        PORTD &= ~(1 << PD7);
    } else {
        PORTD |= (1 << PD7);
    }
}

void Gaspedal(unsigned char geschw_L, unsigned char geschw_R) {
    //0 = motor aus     255= vollgas wen Gangschaltung = 1
    //255 = motor aus     0= vollgas wen Gangschaltung = 0
    // Geschwindigkeit Motor Links (0-255)
    if ((PORTD & (1 << PD6)) == 0) {
        OCR1B = geschw_L;
    } else {
        OCR1B = 255 - geschw_L;
    }

    // Geschwindigkeit Motor Rechts (0-255)
    if ((PORTD & (1 << PD7)) == 0) {
        OCR1A = geschw_R;
    } else {
        OCR1A = 255 - geschw_R;
    }
}

void UART_init() {
    DDRD |= (1 << PD1);

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
    TCCR1A |= (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Prescaler 64

    // Enable Timer1 overflow interrupt
    TIMSK |= (1 << TOIE1);

    sei(); // Enable global interrupts
}

ISR(TIMER1_OVF_vect) {
    global_time_target_led_frequency[LEFT] += 1;
    global_time_target_led_frequency[RIGHT] += 1;

    overflowCount++;

    time_status_led += 1;
}

ISR(INT1_vect) {
    static uint8_t a;
    if (risingEdge) {
        // Rising edge detected
        a = TCNT1; // read Timer1 value
        //UART_send(a);
        overflowCount = 0; // Reset overflow count
        risingEdge = 0; // Prepare to detect falling edge
        MCUCR &= ~(1 << ISC10); // Falling edge
        //UART_send('r');

    } else {
        // Falling edge detected
        pulseWidth = ((overflowCount * 256UL) + TCNT1) - a; // Calculate pulse width
        distanceMeasured = 1; // Indicate that measurement is complete
        //UART_send('f');

    }
}

/*
 * calculates how bright the flashing light is from each transistors POV
 */
void get_flashing_led_brightness(unsigned char *edge_height) {
    static unsigned char mean[2] = {0, 0};
    // signed 16 bit, so that the substraction doesnt return a high result, if mean>transistor
    static int16_t transistor[2] = {0, 0};

    static unsigned char timer_started[2] = {false, false};
    // some variables for noisecanceling the edge-detection
    static unsigned char l_h = 0, l_l = 0, r_h = 0, r_l = 0;

    static unsigned char high_value[2] = {0, 0}, low_value[2] = {0, 0};

    // time between a rising and a falling edge
    static uint16_t time[2] = {0, 0};

    transistor[LEFT] = ADC_get_value(ADC_TRANSISTOR_LEFT);
    transistor[RIGHT] = ADC_get_value(ADC_TRANSISTOR_RIGHT);

    // calculates the mean, old mean has more weight then the new value of the transistor
    if (mean[LEFT] == 0) mean[LEFT] = transistor[LEFT]; // sets the mean to the value at the transistor in the first iteration
    else mean[LEFT] = MEAN_WEIGHT * mean[LEFT] + (1 - MEAN_WEIGHT) * transistor[LEFT];
    if (mean[LEFT] == 0) mean[LEFT] = transistor[LEFT]; // sets the mean to the value at the transistor in the first iteration
    else mean[RIGHT] = MEAN_WEIGHT * mean[RIGHT] + (1 - MEAN_WEIGHT) * transistor[RIGHT];

    // detecting rising / falling edges on the left transistor
    if (transistor[LEFT] > mean[LEFT]) {
        l_h += 1;
        l_l = 0; // setzt rauschabblockung f�r low zur�ck, weil ein high kam
        // wird nur als high erkannt, wenn mehrmals am st�ck high gewesen -> blockt rauschen ab
        if (l_h == 5) {
            // wenn davor low gewesen (steigende flanke) -> zeitmessung starten
            if (!timer_started[LEFT]) {
                global_time_target_led_frequency[LEFT] = 0;
                high_value[LEFT] = transistor[LEFT];

                timer_started[LEFT] = true;
            }

            l_h = 0;
        }
    } else {
        l_l += 1;
        l_h = 0;
        // wird nur als low erkannt, wenn mehrmals am st�ck low gewesen -> blockt rauschen ab
        if (l_l == 5) {
            // wenn vorher high gewesen (fallende flanke) -> zeit auslesen
            if (timer_started[LEFT]) {
                time[LEFT] = global_time_target_led_frequency[LEFT] * 2;
                low_value[LEFT] = transistor[LEFT];
            }

            timer_started[LEFT] = false;
            l_l = 0;
        }
    }

    // detecting rising / falling edges on the right transistor
    if (transistor[RIGHT] > mean[RIGHT]) {
        r_h += 1;
        r_l = 0; // setzt rauschabblockung f�r low zur�ck, weil ein high kam
        // wird nur als high erkannt, wenn mehrmals am st�ck high gewesen -> blockt rauschen ab
        if (r_h == 5) {
            // wenn davor low gewesen (steigende flanke) -> zeitmessung starten
            if (!timer_started[RIGHT]) {
                global_time_target_led_frequency[RIGHT] = 0;
                high_value[RIGHT] = transistor[RIGHT];

                timer_started[RIGHT] = true;
            }

            r_h = 0;
        }
    } else {
        r_l += 1;
        r_h = 0;
        // wird nur als low erkannt, wenn mehrmals am st�ck low gewesen -> blockt rauschen ab
        if (r_l == 5) {
            // wenn vorher high gewesen (fallende flanke) -> zeit auslesen
            if (timer_started[RIGHT]) {
                time[RIGHT] = global_time_target_led_frequency[RIGHT] * 2;
                low_value[RIGHT] = transistor[RIGHT];
            }

            timer_started[RIGHT] = false;
            r_l = 0;
        }
    }

    // wenn gemessene zeit ~100ms -> blinken erkannt
    if (time[LEFT] >= 50 && time[LEFT] <= 150) {
#ifdef DEBUG
        PORTD &= ~(1 << PD5);
#endif
        edge_height[LEFT] = high_value[LEFT] - low_value[LEFT];
    }
#ifdef DEBUG
    else {
        PORTD |= (1 << PD5);
    }
#endif
    if (time[RIGHT] >= 50 && time[RIGHT] <= 150) {
#ifdef DEBUG
        PORTD &= ~(1 << PD4);
#endif
        edge_height[RIGHT] = high_value[RIGHT] - low_value[RIGHT];
        //UART_send(edge_height[right]);
    }
#ifdef DEBUG
    else {
        PORTD |= (1 << PD4);
    }
#endif
}

/*
 * toggles D2 every 500ms -> D2 is blinking with 1Hz if status_led_should_blink is set to true
 */
void status_led_blink(char status_led_should_blink) {
    // turn off D2 if it shouldnt blink
    if (!status_led_should_blink) {
        PORTD |= (1 << PD5);
        return;
    }
    // toggles D2
    if (time_status_led >= 500) {
        PORTD ^= (1 << PD5);
        time_status_led = 0;
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
    init_M();
    ext_int1_init();
    SensorInit();

    overflowCount = 0;
    risingEdge = 1;
    pulseWidth = 0;
    distanceMeasured = 1;

    // regulator
    float p = 1.5; //small value lets it turn a little and big values make it turn quicker
    float i = .0; //it helps to stop the robot from oscilating, but if the value is to big it makes the opposite
    float d = .0; //the quicker the curve comes, the higher the steering effect
    int error = 0; //Difference
    int P = error;
    float I = error;
    int lastError = error;
    float D = error - lastError;
    int steer = P * p + I * i + D*d;

    unsigned char edge_height[2] = {0, 0};
    
    unsigned char status_led_should_blink = false;

    while (1) {

        status_led_blink(status_led_should_blink);
        
        get_flashing_led_brightness(edge_height);

        // no flashing led detected -> rotate left
        if ((edge_height[LEFT] < EDGE_HEIGHT_THRESHOLD) && (edge_height[RIGHT] < EDGE_HEIGHT_THRESHOLD)) {
            Gangschaltung(BACKWARD, FORWARD);
            Gaspedal(60, 70);
        } else {
            // drive towards the flashing led, steered by a pid-regulator
            error = edge_height[LEFT] - edge_height[RIGHT];
            P = error;
            I = I + error;
            lastError = error;
            D = error - lastError;
            steer = P * p + I * i + D*d;

            // turn in the direction of the brighter light
            if (edge_height[LEFT] > edge_height[RIGHT]) {
                if (edge_height[LEFT] > edge_height[RIGHT]) {
                    // turn left
                } else {
                    Gangschaltung(FORWARD, FORWARD);
                    if (steer > 127) steer = 100;
                    else if (steer < 0) steer = 0;
                    Gaspedal(127 - steer, 127 + steer);
                }
            } else if (edge_height[LEFT] < edge_height[RIGHT]) {
                // turn right
                Gangschaltung(FORWARD, FORWARD);
                if (steer > 127) steer = 100;
                else if (steer < 0) steer = 0;
                Gaspedal(127 + steer, 127 - steer);
            }
        }
    }
}

