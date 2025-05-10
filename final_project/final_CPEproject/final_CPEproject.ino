// Wiem Boubaker
// Josue Santa Cruz
// CPE 301

#include <avr/interrupt.h>
#include <stdbool.h>
#include <DHT.h>  // Requires Arduino DHT sensor library
#include <LiquidCrystal.h> //Requires Liquid Crystal
#include <Stepper.h>
#include <Servo.h>

// Used for the temperature sensor (DHT11)
#define DHTPIN 22     // DHT11 connected to digital pin 22
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); 

//Used for LCD <->
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//Used for stepper motor
int motorSpeed = 10;  // change this to fit the number of steps per revolution
Stepper myStepper(2048, 32, 30, 28, 26);

//ServoMotor
Servo myServo;

//Used for millis() function
unsigned long previousMillis = 0;
const unsigned long interval = 600000;

// ---------- UART Definitions ----------
#define RDA 0x80
#define TBE 0x20

volatile unsigned char* myUCSR0A = (unsigned char*) 0xC0;
volatile unsigned char* myUCSR0B = (unsigned char*) 0xC1;
volatile unsigned char* myUCSR0C = (unsigned char*) 0xC2;
volatile unsigned int*  myUBRR0  = (unsigned int*)  0xC4;
volatile unsigned char* myUDR0   = (unsigned char*) 0xC6;

// Used for the water sensor
volatile unsigned char* my_ADMUX   = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB  = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA  = (unsigned char*) 0x7A;
volatile unsigned int*  my_ADC_DATA= (unsigned int*)  0x78;

//

// ---------- UART Functions ----------
void U0Init(int U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20; // Clear status flags
  *myUCSR0B = 0x18; // Enable TX and RX
  *myUCSR0C = 0x06; // 8N1
  *myUBRR0  = tbaud;
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
volatile unsigned char* ddr_k  = (unsigned char*) 0x107;
volatile unsigned char* pin_k  = (unsigned char*) 0x106;
volatile unsigned char* port_k = (unsigned char*) 0x108;

volatile unsigned char* ddr_l  = (unsigned char*) 0x10A;
volatile unsigned char* port_l = (unsigned char*) 0x10B;

volatile unsigned char* ddr_c  = (unsigned char*) 0x27;
volatile unsigned char* pin_c  = (unsigned char*) 0x26;
volatile unsigned char* port_c = (unsigned char*) 0x28;


// Timer
volatile unsigned char* myTCCR1A = (unsigned char*) 0x80;
volatile unsigned char* myTCCR1B = (unsigned char*) 0x81;
volatile unsigned char* myTCCR1C = (unsigned char*) 0x82;
volatile unsigned char* myTIFR1  = (unsigned char*) 0x36;
volatile unsigned char* myTIMSK1 = (unsigned char*) 0x6F;
volatile unsigned int*  myTCNT1  = (unsigned int*)  0x84;

// Port B for timer ISR (PB6)
volatile unsigned char* portB = (unsigned char*) 0x25;

// ---------- Global Variables ----------
volatile bool isOn = false;
volatile bool buttonPressed = false;
unsigned int currentTicks = 0;
volatile bool buttonA4Pressed = false;
volatile bool buttonA5Pressed = false;

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

   // Check PC4 (A4)
  if (!(*pin_c & (1 << 4)))
  {
    buttonA4Pressed = true;
  }

  // Check PC5 (A5)
  if (!(*pin_c & (1 << 5)))
  {
    buttonA5Pressed = true;
  }
}

void waitTicks(unsigned int waitAmount)
{
  unsigned int start = *myTCNT1;
  while ((unsigned int)(*myTCNT1 - start) < waitAmount);
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

  *ddr_k &= ~(1 << 2);    // PK2 input
  *port_k |= (1 << 2);    // Enable pull-up

  *ddr_c &= ~((1 << 4) | (1 << 5));   // PC4 and PC5 (A4 and A5) as input
  *port_c |= (1 << 4) | (1 << 5);     // Enable pull-ups on A4 and A5

  *ddr_l |= (1 << 5) | (1 << 3) | (1 << 1); // PL5, PL3 output

  *port_l |= (1 << 5);  // LED1 ON
  *port_l &= ~(1 << 3); // LED2 OFF

  PCICR |= (1 << PCIE2);     // Enable PCINT[23:16]
  PCMSK2 |= (1 << PCINT18);  // Enable PCINT18 (PK2)

  PCICR |= (1 << PCIE1);              // Enable PCINT[14:8] (Port C)
  PCMSK1 |= (1 << PCINT12) | (1 << PCINT13);  // Enable PCINT12 (PC4) and PCINT13 (PC5)

  setup_timer_regs();
  adc_init();

  sei(); // Enable interrupts

  dht.begin(); // Start up the DHT
  lcd.begin(16, 2); // 
  myStepper.setSpeed(motorSpeed);
  myServo.attach(9);
  myServo.write(90);
}

// ---------- Main Loop ----------
void loop()
{
  //lcd.write();
  if (isOn) //IDLE STATE
  {
    //lcd.clear();
    *port_l &= ~(1 << 5); // LED1 OFF
    *port_l |= (1 << 3);  // LED2 ON
    *portB &= ~(1 << 3); //LED4 OFF

    unsigned int sensorValue = adc_read(0); // Read A0
    int threshold = 100;

    if (sensorValue <= threshold) { // ERROR STATE
      *port_l |= (1 << 1);  // Turn ON LED on pin 48 (PL1)
      *port_l &= ~(1 << 3); // Turn OFF LED on pin 46 (PL3)

      lcd.clear();
    } else //RUNNING STATE
    {
      *portB &= ~(1 << 3);
      *port_l &= ~(1 << 1); // Turn OFF LED on pin 48 (PL1)
      *port_l |= (1 << 3);  // Turn ON LED on pin 46 (PL3)
      

      float temp = dht.readTemperature();
      float hum = dht.readHumidity();


      lcd.setCursor(0, 0);
lcd.print("Temp: ");
lcd.print(temp);
lcd.print(" C   ");

lcd.setCursor(0, 1);
lcd.print("Humid: ");
lcd.print(hum);
lcd.print(" %   ");
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis; // Reset the timer

        float temp = dht.readTemperature();
        float hum = dht.readHumidity();

      lcd.setCursor(0, 0); // Top line
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.print(" C   "); // Extra spaces to clear old digits

      lcd.setCursor(0, 1); // Bottom line
      lcd.print("Humid: ");
      lcd.print(hum);
      lcd.print(" %   "); // Extra spaces to clear old digits
      }

      if (!isnan(temp)) 
      {
        //buttonA4Pressed = false;
        //buttonA5Pressed = false;
        if ((temp > 24) && (hum > 20)) 
        {
          *port_l &= ~(1 << 3); // LED2 OFF
          *port_l &= ~(1 << 1); //LED3 OFF
          *portB |= (1 << 3); //LED4 ON
          myServo.write(90);       // Move to 90 degrees
          delay(100);
          for (volatile long i = 0; i < 800000; i++);
          //lcd.clear();

          if (buttonA4Pressed)
          {
            myStepper.step(-400);
            buttonA4Pressed = false;
          }
          
          if (buttonA5Pressed)
          {
            myStepper.step(400);
            buttonA5Pressed = false;
          }
        }
        else
        {
          *portB &= ~(1 << 3);
          *port_l &= ~(1 << 1);
          putString("HOLA\\r\\n");
        }
      }
    }

    // Small delay
    for (volatile long i = 0; i < 500000; i++); //MAKE SURE TO TAKE THIS OUT AT THE END, THIS CAN COUNT AS A DELAY
                                                //Which is not allowed on the LAB
  }
  else
  {
    *port_l |= (1 << 5);  // LED1 ON
    *port_l &= ~(1 << 3); // LED2 OFF
    *port_l &= ~(1 << 1); //LED3 OFF
    *portB &= ~(1 << 3); //LED4 OFF
  }
}
