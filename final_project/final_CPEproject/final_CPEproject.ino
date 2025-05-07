#define RDA 0x80
 #define TBE 0x20
 
 volatile unsigned char* myUCSR0A = (unsigned char*) 0xC0;
 volatile unsigned char* myUCSR0B = (unsigned char*) 0xC1;
 volatile unsigned char* myUCSR0C = (unsigned char*) 0xC2;
 volatile unsigned int*  myUBRR0  = (unsigned int*)  0xC4;
 volatile unsigned char* myUDR0   = (unsigned char*) 0xC6;
 
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
   while (!kbhit()) {}
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
 
 // ---------- GPIO Setup ----------
 volatile unsigned char* ddr_k  = (unsigned char*) 0x107;
 volatile unsigned char* pin_k  = (unsigned char*) 0x106;
 volatile unsigned char* port_k = (unsigned char*) 0x108;
 
 volatile unsigned char* ddr_l  = (unsigned char*) 0x10A;
 volatile unsigned char* port_l = (unsigned char*) 0x10B;
 
 // ---------- Global Variables ----------
 volatile bool isOn = false;
 
 // ---------- ISR for Button Press ----------
 ISR(PCINT2_vect)
 {
   bool buttonState = (*pin_k & (1 << 2)) == 0; // Active LOW
 
   if (buttonState)
   {
     isOn = !isOn;
 
     if (isOn)
       putString("System ON\r\n");
     else
       putString("System OFF\r\n");
   }
 }
 
 // ---------- Setup ----------
 void setup()
 {
   cli(); // Disable interrupts
 
   // UART
   U0Init(9600);
 
   // Set PK2 as input and enable pull-up
   *ddr_k &= ~(1 << 2);
   *port_k |= (1 << 2);
 
   // Set digital 44 (PL5) and 46 (PL3) as outputs
   *ddr_l |= (1 << 5) | (1 << 3);
 
   // Initially ON
   *port_l |= (1 << 5); // LED1 ON
   *port_l &= ~(1 << 3); // LED2 OFF
 
   // Enable Pin Change Interrupt on PK2
   PCICR |= (1 << PCIE2);     // Enable PCINT[23:16]
   PCMSK2 |= (1 << PCINT18);  // Enable PCINT18 (PK2)
 
   sei(); // Enable global interrupts
 }
 
 // ---------- Main Loop ----------
 void loop()
 {
   if (isOn)
   {
     *port_l &= ~(1 << 5); // LED1 OFF
     *port_l |= (1 << 3);  // LED2 ON
   }
   else
   {
     *port_l |= (1 << 5);  // LED1 ON
     *port_l &= ~(1 << 3); // LED2 OFF
   }
 }