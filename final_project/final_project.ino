// Wiem Boubaker
// Josue Santa Cruz Ramirez
// CPE 301 - Final Project

#define RDA 0x80
#define TBE 0x20

#include <avr/interrupt.h>
#include <stdbool.h>
#include <DHT.h>  // Requires Arduino DHT sensor library

#define DHTPIN 22     // DHT11 connected to digital pin 22
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

volatile unsigned char* myUCSR0A = (unsigned char*) 0xC0;
volatile unsigned char* myUCSR0B = (unsigned char*) 0xC1;
volatile unsigned char* myUCSR0C = (unsigned char*) 0xC2;
volatile unsigned int*  myUBRR0  = (unsigned int*)  0xC4;
volatile unsigned char* myUDR0   = (unsigned char*) 0xC6;

volatile unsigned char* ddr_k  = (unsigned char*) 0x107;
volatile unsigned char* pin_k  = (unsigned char*) 0x106;
volatile unsigned char* port_k = (unsigned char*) 0x108;

volatile unsigned char* ddr_l  = (unsigned char*) 0x10A;
volatile unsigned char* port_l = (unsigned char*) 0x10B;

volatile unsigned char* my_ADMUX   = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB  = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA  = (unsigned char*) 0x7A;
volatile unsigned int*  my_ADC_DATA = (unsigned int*) 0x78;

enum SystemState { DISABLED, IDLE, RUNNING, ERROR };
volatile SystemState currentState = DISABLED;

ISR(PCINT2_vect) {
    bool buttonState = (*pin_k & (1 << 2)) == 0;
    if (buttonState) {
        if (currentState == DISABLED) {
            currentState = IDLE;
            putString("System STARTED -> IDLE\\r\\n");
        } else {
            currentState = DISABLED;
            putString("System STOPPED -> DISABLED\\r\\n");
        }
    }
}

void setup() {
    cli();
    U0init(9600);
    adc_init();
    dht.begin();

    *ddr_k &= ~(1 << 2);  // PK2 as input
    *port_k |= (1 << 2);  // pull-up

    *ddr_l |= 0x0F;  // PL0-PL3 output
    *port_l &= ~0x0F; // All LEDs OFF

    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18);
    sei();
}

void loop() {
    switch (currentState) {
        case DISABLED:
            setLEDState(DISABLED);
            break;

        case IDLE:
            setLEDState(IDLE);
            putString("State: IDLE\\r\\n");

            if (adc_read(0) < 100) {
                currentState = ERROR;
                putString("-> ERROR: Water level LOW\\r\\n");
            }

            float temp = dht.readTemperature();
            if (!isnan(temp)) {
                if (temp > 25.0) {
                    currentState = RUNNING;
                    putString("Temp > 25C -> RUNNING\\r\\n");
                }
            }

            delay_ms(1000);
            break;

        case RUNNING:
            setLEDState(RUNNING);
            putString("State: RUNNING\\r\\n");

            if (adc_read(0) < 100) {
                currentState = ERROR;
                putString("-> ERROR: Water level LOW\\r\\n");
            }

            float currTemp = dht.readTemperature();
            if (!isnan(currTemp) && currTemp <= 25.0) {
                currentState = IDLE;
                putString("Temp <= 25C -> IDLE\\r\\n");
            }

            delay_ms(1000);
            break;

        case ERROR:
            setLEDState(ERROR);
            putString("ERROR: Low water level\\r\\n");

            delay_ms(1000);
            break;
    }
}

void setLEDState(SystemState state) {
    *port_l &= ~0x0F;
    switch (state) {
        case DISABLED: *port_l |= (1 << 0); break;
        case IDLE:     *port_l |= (1 << 1); break;
        case RUNNING:  *port_l |= (1 << 3); break;
        case ERROR:    *port_l |= (1 << 2); break;
    }
}

void putString(const char* str) {
    while (*str) putChar(*str++);
}

void putChar(unsigned char c) {
    while (!(*myUCSR0A & TBE));
    *myUDR0 = c;
}

void U0init(int baud) {
    unsigned long FCPU = 16000000;
    unsigned int tbaud = (FCPU / 16 / baud - 1);
    *myUCSR0A = 0x20;
    *myUCSR0B = 0x18;
    *myUCSR0C = 0x06;
    *myUBRR0  = tbaud;
}

void adc_init() {
    *my_ADCSRA |= (1 << 7);
    *my_ADCSRA &= ~(1 << 5);
    *my_ADCSRA &= ~(1 << 3);
    *my_ADCSRA |= (1 << 2) | (1 << 1) | (1 << 0);
    *my_ADCSRB &= ~(1 << 3);
    *my_ADCSRB &= ~((1 << 2) | (1 << 1) | (1 << 0));
    *my_ADMUX |= (1 << 6);
    *my_ADMUX &= ~(1 << 5);
    *my_ADMUX &= 0xE0;
}

unsigned int adc_read(unsigned char ch) {
    *my_ADMUX = (*my_ADMUX & 0xE0) | (ch & 0x1F);
    *my_ADCSRA |= (1 << 6);
    while ((*my_ADCSRA & (1 << 6)));
    return *my_ADC_DATA;
}

void delay_ms(unsigned int ms) {
    for (unsigned int i = 0; i < ms; i++)
        for (unsigned int j = 0; j < 1600; j++)
            asm volatile("nop");
}