/*
PIC16F628 Pinout
                  --------------
        led1Red -|1           18|- led2Red
      led1Green -|2           17|- led2Green
                -|3           16|- led2Blue
           MCLR -|4           15|- led4Red
         Ground -|5           14|- +5V
       led1Blue -|6           13|- led4Green
       RS232 RX -|7           12|- led3Red
       RS232 TX -|8           11|- led3Green
       led4Blue -|9           10|- led3Blue
                  --------------

PIC16F628 PCBDR1r1 Pinout
                  --------------
      led1Green -|1           18|- led2Green
        led1Red -|2           17|- led2Red
                -|3           16|- led2Blue
           MCLR -|4           15|- led4Green
         Ground -|5           14|- +5V
       led1Blue -|6           13|- led4Red
       RS232 RX -|7           12|- led3Green
       RS232 TX -|8           11|- led3Red
       led4Blue -|9           10|- led3Blue
                  --------------
*/
#include <16F628.h>
#fuses INTRC_IO, NOWDT, NOPROTECT, NOCPD, NOLVP, NOBROWNOUT, MCLR
#use delay(clock=4000000)
#rom 0x2100={0,1,1,1,1,1,1,1,1,1,1,1,1} 

#use rs232(UART1, stream=PC, baud=19200, xmit=PIN_B2, rcv=PIN_B1, ERRORS)

#define PCBDR1r1     // Red and Green lines are switched on the DR1r1 PCB

#ifdef PCBDR1r1
   #define led1Red PIN_A3
   #define led1Green PIN_A2
   #define led1Blue PIN_B0
   
   #define led2Red PIN_A0
   #define led2Green PIN_A1
   #define led2Blue PIN_A7
   
   #define led3Red PIN_B5
   #define led3Green PIN_B6
   #define led3Blue PIN_B4
   
   #define led4Red PIN_B7
   #define led4Green PIN_A6
   #define led4Blue PIN_B3
#else
   #define led1Red PIN_A2
   #define led1Green PIN_A3
   #define led1Blue PIN_B0
   
   #define led2Red PIN_A1
   #define led2Green PIN_A0
   #define led2Blue PIN_A7
   
   #define led3Red PIN_B6
   #define led3Green PIN_B5
   #define led3Blue PIN_B4
   
   #define led4Red PIN_A6
   #define led4Green PIN_B7
   #define led4Blue PIN_B3
#endif

volatile int1 runPWM = FALSE;
volatile int8 PWMcount = 0;
volatile int8 theAddress = 0;
volatile int8 PWMduty[12];
int8 const eTable[16] = {0,1,2,3,5,7,10,13,17,22,27,33,39,47,55,64};

void doPWM() {
   #USE FAST_IO (ALL) 
   output_bit(led1Red,     PWMduty[0] > PWMcount);
   output_bit(led1Green,   PWMduty[1] > PWMcount);
   output_bit(led1Blue,    PWMduty[2] > PWMcount);
   
   output_bit(led2Red,     PWMduty[3] > PWMcount);
   output_bit(led2Green,   PWMduty[4] > PWMcount);
   output_bit(led2Blue,    PWMduty[5] > PWMcount);
   
   output_bit(led3Red,     PWMduty[6] > PWMcount);
   output_bit(led3Green,   PWMduty[7] > PWMcount);
   output_bit(led3Blue,    PWMduty[8] > PWMcount);
   
   output_bit(led4Red,     PWMduty[9] > PWMcount);
   output_bit(led4Green,   PWMduty[10] > PWMcount);
   output_bit(led4Blue,    PWMduty[11] > PWMcount);
   #USE STANDARD_IO (ALL)
   if (++PWMcount == 64) {
      PWMcount = 0;
   }
}

#define BUFFER_SIZE 64
volatile int8 buffer[BUFFER_SIZE];
volatile int8 next_in = 0;
volatile int8 next_out = 0;
#priority RDA,RTCC

#INT_RDA
void serial_isr() {
   int8 t;

   buffer[next_in]=getc();
   fputc(buffer[next_in],PC);
   t=next_in;
   next_in=(next_in+1) % BUFFER_SIZE;
   if(next_in==next_out)
     next_in=t;           // Buffer full !!
}

#INT_RTCC
void clock_isr() {
   if (runPWM == TRUE) doPWM();
}

#define bkbhit (next_in!=next_out)

int8 bgetc() {
   int8 c;

   while(!bkbhit) ;
   c=buffer[next_out];
   next_out=(next_out+1) % BUFFER_SIZE;
   return(c);
}

void processData()
{
   int8 syncByte = 0;
   int8 data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
   int8 i;

   syncByte = bgetc();
   if (syncByte == 0xAA) {
      data[0] = bgetc();
      switch (data[0] >> 4) {
      case 1:
         // Update individual LEDs using 4-bit exponential update
         // Byte 1, Nibble 1 = led1Red;   Byte 1, Nibble 2 = led1Green
         // Byte 2, Nibble 1 = led1Blue;  Byte 2, Nibble 2 = led2Red
         // Byte 3, Nibble 1 = led2Green; Byte 3, Nibble 2 = led2Blue
         // Byte 4, Nibble 1 = led3Red;   Byte 4, Nibble 2 = led3Green
         // Byte 5, Nibble 1 = led3Blue;  Byte 5, Nibble 2 = led4Red
         // Byte 6, Nibble 1 = led4Green; Byte 6, Nibble 2 = led4Blue
         // Byte 7 = theAddress
            for(i=1; i<8; i++) {
               data[i] = bgetc();
            }
            if ((data[7] == 255) || (data[7] == theAddress)) {
               for(i=0; i<7; i++) {
                  PWMduty[2*i] = eTable[data[i+1] >> 4];
                  PWMduty[2*i+1] = eTable[(data[i+1] << 4) >> 4];
               }
            }
            break;
      case 2:
         // Update all LEDs using 4-bit exponential update
         // Byte 0, Nibble 2 = led1Red = led2Red = led3Red = led4Red
         // Byte 1, Nibble 1 = led1Green = led2Green = led3Green = led4Green
         // Byte 1, Nibble 2 = led1Blue = led2Blue = led3Blue = led4Blue
         // Byte 2 = theAddress
            data[1] = bgetc();
            data[2] = bgetc();
            if ((data[2] == 255) || (data[2] == theAddress)) {
               PWMduty[0] = eTable[(data[0] << 4) >> 4];
               PWMduty[1] = eTable[data[1] >> 4];
               PWMduty[2] = eTable[(data[1] << 4) >> 4];
               PWMduty[3] = PWMduty[0];
               PWMduty[4] = PWMduty[1];
               PWMduty[5] = PWMduty[2];
               PWMduty[6] = PWMduty[0];
               PWMduty[7] = PWMduty[1];
               PWMduty[8] = PWMduty[2];
               PWMduty[9] = PWMduty[0];
               PWMduty[10] = PWMduty[1];
               PWMduty[11] = PWMduty[2];
            }
            break;
      case 3:
         // Save settings
         // Byte 1 = theAddress
            data[1] = bgetc();
            if ((data[1] == 255) || (data[1] == theAddress)) {
               for(i=0; i<12; i++) {
                  write_eeprom(i+1,PWMduty[i]);
               }
            }
            break;
      case 4:
         // Load settings
         // Byte 1 = theAddress
            data[1] = bgetc();
            if ((data[1] == 255) || (data[1] == theAddress)) {
               for(i=0; i<12; i++) {
                  PWMduty[i] = READ_EEPROM(i+1);
               }
            }
            break;
      case 5:
         // Get address
         // Byte 1 = theAddress
            data[1] = bgetc();
            if ((data[1] == 255) || (data[1] == theAddress)) {
               fputc(theAddress,PC);
            }
            break;
      case 6:
         // Set address (one-time set)
         // Byte 1 = theAddress to save
         // Byte 2 = theAddress
            data[1] = bgetc();
            data[2] = bgetc();
            if (((data[2] == 255) || (data[2] == theAddress)) && (theAddress == 0)) {
               write_eeprom(0,data[1]);
               theAddress = READ_EEPROM(0);
            }
            break;
      case 7:
         // Self test
         // Byte 1 = theAddress
            data[1] = bgetc();
            if ((data[1] == 255) || (data[1] == theAddress)) {
               // Removed for space
               // selfTest();
            }
            break;
      case 8:
         // Update individual LEDs using 6-bit update
         // Byte 1 = led1Red;    Byte 2 = led1Green
         // Byte 3 = led1Blue;   Byte 4 = led2Red
         // Byte 5 = led2Green;  Byte 6 = led2Blue
         // Byte 7 = led3Red;    Byte 8 = led3Green
         // Byte 9 = led3Blue;   Byte 10 = led4Red
         // Byte 11 = led4Green; Byte 12 = led4Blue
         // Byte 13 = theAddress
            for(i=1; i<14; i++) {
               data[i] = bgetc();
            }
            if ((data[13] == 255) || (data[13] == theAddress)) {
               for(i=0; i<12; i++) {
                  PWMduty[i] = data[i+1];
               }
            }
            break;
      case 9:
         // Update all LEDs using 6-bit update
         // Byte 1 = led1Red = led2Red = led3Red = led4Red
         // Byte 2 = led1Green = led2Green = led3Green = led4Green
         // Byte 3 = led1Blue = led2Blue = led3Blue = led4Blue
         // Byte 4 = theAddress
            for(i=1; i<5; i++) {
               data[i] = bgetc();
            }
            if ((data[4] == 255) || (data[4] == theAddress)) {
               PWMduty[0] = data[1];
               PWMduty[1] = data[2];
               PWMduty[2] = data[3];
               PWMduty[3] = PWMduty[0];
               PWMduty[4] = PWMduty[1];
               PWMduty[5] = PWMduty[2];
               PWMduty[6] = PWMduty[0];
               PWMduty[7] = PWMduty[1];
               PWMduty[8] = PWMduty[2];
               PWMduty[9] = PWMduty[0];
               PWMduty[10] = PWMduty[1];
               PWMduty[11] = PWMduty[2];
            }
            break;
      }
   }
}

void main() {
   int8 i;
   
   delay_ms(200);
   
   runPWM = FALSE;
   setup_timer_0 (RTCC_8_BIT|RTCC_INTERNAL|RTCC_DIV_1);
   set_timer0(0);
   enable_interrupts(INT_RDA);
   enable_interrupts(INT_RTCC);
   enable_interrupts(GLOBAL);
   output_drive(led1Red); output_drive(led1Green); output_drive(led1Blue);
   output_drive(led2Red); output_drive(led2Green); output_drive(led2Blue);
   output_drive(led3Red); output_drive(led3Green); output_drive(led3Blue);
   output_drive(led4Red); output_drive(led4Green); output_drive(led4Blue);

   for(i=0; i<12; i++) {
      PWMduty[i] = READ_EEPROM(i+1);
   }
   theAddress = READ_EEPROM(0);

   delay_ms(200);
   runPWM = TRUE;
   
   while (TRUE) {
      while(bkbhit)
        processData();
   }
}
