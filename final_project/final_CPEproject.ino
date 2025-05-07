// Wiem Boubaker
// Josue Santa Cruz
// CPE 301

#include <avr/interrupt.h>
#include <stdbool.h>

// ---------- UART Definitions ----------
#define RDA 0x80
#define TBE 0x20

volatile unsigned char* myUCSR0A = (unsigned char*) 0xC0;
volatile unsigned char* myUCSR0B = (unsigned char*) 0xC1;
volatile unsigned char* myUCSR0C = (unsigned char*) 0xC2;
volatile unsigned int* myUBRR0 = (unsigned int*) 0xC4;
volatile unsigned char* myUDR0 = (unsigned char*) 0xC6;

// Used for the water sensor
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA= (unsigned int*) 0x78;

// ---------- UART Functions ----------
void U0Init(int U0baud)
{
unsigned long FCPU = 16000000;
unsigned int tbaud = (FCPU / 16 / U0baud - 1);
*myUCSR0A = 0x20; // Clear status flags
*myUCSR0B = 0x18; // Enable TX and RX
*myUCSR0C = 0x06; // 8N1
*myUBRR0 = tbaud;
}

unsigned char kbhit()
{
return (*myUCSR0A & RDA) ? 1 : 0;
}

unsigned char getChar()
{
return *myUDR0;
}

void putChar(unsigned char U0pdata)
{
while (!(*myUCSR0A & TBE)) {}
*myUDR0 = U0pdata;
}

void putString(const char* str)
{
while (*str)
{
putChar(*str++);
}
}

// ---------- ADC Functions ----------
void adc_init()
{
*my_ADCSRA |= (1 << 7); // Enable ADC
*my_ADCSRA &= ~(1 << 5); // Disable auto trigger
*my_ADCSRA &= ~(1 << 3); // Disable interrupt
*my_ADCSRA |= (1 << 2) | (1 << 1) | (1 << 0); // Prescaler 128

*my_ADCSRB &= ~(1 << 3); // ACME off
*my_ADCSRB &= ~((1 << 2) | (1 << 1) | (1 << 0)); // Free running

*my_ADMUX |= (1 << 6); // AVCC ref
*my_ADMUX &= ~(1 << 5); // Right adjust
*my_ADMUX &= ~((1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0)); // MUX
}

unsigned int adc_read(unsigned char adc_channel_num)
{
*my_ADMUX &= ~((1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
*my_ADCSRB &= ~(1 << 3); // ACME
*my_ADMUX |= (adc_channel_num & 0x07); // MUX bits
*my_ADCSRA |= (1 << 6); // Start conversion

while((*my_ADCSRA & (1 << 6)) != 0); // Wait

return *my_ADC_DATA;
}

// ---------- GPIO Setup ----------
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* pin_k = (unsigned char*) 0x106;
volatile unsigned char* port_k = (unsigned char*) 0x108;

volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* port_l = (unsigned char*) 0x10B;

// Timer
volatile unsigned char* myTCCR1A = (unsigned char*) 0x80;
volatile unsigned char* myTCCR1B = (unsigned char*) 0x81;
volatile unsigned char* myTCCR1C = (unsigned char*) 0x82;
volatile unsigned char* myTIFR1 = (unsigned char*) 0x36;
volatile unsigned char* myTIMSK1 = (unsigned char*) 0x6F;
volatile unsigned int* myTCNT1 = (unsigned int*) 0x84;

// Port B for timer ISR (PB6)
volatile unsigned char* portB = (unsigned char*) 0x25;

// ---------- Global Variables ----------
volatile bool isOn = false;
volatile bool buttonPressed = false;
unsigned int currentTicks = 0;

// ---------- ISR for Button Press ----------
ISR(PCINT2_vect)
{
// Simple debounce delay
for (volatile int i = 0; i < 10000; i++);

// Check if button is actually pressed (active low)
if (!(*pin_k & (1 << 2))) // PK2 is low
{
isOn = !isOn;
buttonPressed = true;
}
}

ISR(TIMER1_OVF_vect)
{
*myTCCR1B &= 0xF8;
*myTCNT1 = (unsigned int)(65535 - (unsigned long)(currentTicks));
*myTCCR1B |= 0x01;
if (currentTicks != 65535)
{
*portB ^= (1 << 6);
}
}

// ---------- Timer Setup ----------
void setup_timer_regs()
{
*myTCCR1A = 0x00;
*myTCCR1B = 0x00;
*myTCCR1C = 0x00;
*myTIFR1 |= 0x01;
*myTIMSK1 |= (1 << TOIE1);
}

// ---------- Setup ----------
void setup()
{
cli(); // Disable interrupts

U0Init(9600);

*ddr_k &= ~(1 << 2); // PK2 input
*port_k |= (1 << 2); // Enable pull-up

*ddr_l |= (1 << 5) | (1 << 3) | (1 << 1); // PL5, PL3 output

*port_l |= (1 << 5); // LED1 ON
*port_l &= ~(1 << 3); // LED2 OFF

PCICR |= (1 << PCIE2); // Enable PCINT[23:16]
PCMSK2 |= (1 << PCINT18); // Enable PCINT18 (PK2)

setup_timer_regs();
adc_init();

sei(); // Enable interrupts
}

// ---------- Main Loop ----------
void loop()
{
if (isOn)
{
*port_l &= ~(1 << 5); // LED1 OFF
*port_l |= (1 << 3); // LED2 ON

unsigned int sensorValue = adc_read(0); // Read A0
int threshold = 100;

if (sensorValue <= threshold) {
*port_l |= (1 << 1); // Turn ON LED on pin 48 (PL1)
*port_l &= ~(1 << 3); // Turn OFF LED on pin 46 (PL3)
} else {
*port_l &= ~(1 << 1); // Turn OFF LED on pin 48 (PL1)
*port_l |= (1 << 3); // Turn ON LED on pin 46 (PL3)
}

// Small delay
for (volatile long i = 0; i < 500000; i++); //MAKE SURE TO TAKE THIS OUT AT THE END, THIS CAN COUNT AS A DELAY
//Which is not allowed on the LAB
}
else
{
*port_l |= (1 << 5); // LED1 ON
*port_l &= ~(1 << 3); // LED2 OFF
*port_l &= ~(1 << 1); //LED3 OFF
}
}