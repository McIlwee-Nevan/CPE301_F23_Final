#include <LiquidCrystal.h>

#define RDA 0x80
#define TBE 0x20

/*LEDs on PORT B
BLUE_LED = 7; //PB4 ~D10
GREEN_LED = 6, //PB7 ~D13
YELLOW_LED = 5, //PB6 ~D12
RED_LED = 4, //PB5 ~D11
*/

unsigned int WATER_SENSOR = 7;

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

char states[][10] = {"DISABLED", "IDLE", "RUNNING", "ERROR"};
int curState = 0;
bool monitoring = false;

int temp = 0;
int tempThreshold = 1000;
int waterLevel = 0;
int waterLevelThreshold = 50;

void setup() 
{
  // setup the UART
  //U0init(9600);
  Serial.begin(9600);
  // setup the ADC
  adc_init();

  DDRE &= 0xEF; //Start Button, Input
  PORTE |= 0x10; //Start Button, Pullup
  DDRE &= 0xDF; //Reset Button, Input
  PORTE |= 0x20; //Reset Button, Pullup
  DDRB |= 0xF0; //LEDs, output
  PORTB = 0b00100000; //Enable yellow
  attachInterrupt(digitalPinToInterrupt(2), toggleSystem, FALLING);

  //Send Startup to serial

}
void loop() 
{
  //Stepper goes here

  //Monitoring
  if(monitoring){
    waterLevel = adc_read(WATER_SENSOR);
  }

  //State Machine Switch
  switch(curState){
    case 0:  //DISABLED
      break;
    
    case 1: //IDLE
      if(waterLevel <= waterLevelThreshold){
        PORTB = 0b00010000;
        //clear lcd, display error
        Serial.println("Idle to Error");
        curState = 3;
      }
      else if(temp > tempThreshold){
        //turn off green led
        //turn on blue led
        //turn on fan motor
        //report transition
        curState = 2;
      }
      break;
    
    case 2: //RUNNING
      if(waterLevel <= waterLevelThreshold){
        //turn off fan motor
        //turn off blue led
        //turn on red led
        //clear lcd, display error
        //report transition
        curState = 3;
      }
      else if(temp <= tempThreshold){
        //turn off fan motor
        //turn off blue led
        //turn on green led
        //report transition
        curState = 1;
      }
      break;

    case 3: //ERROR
      while(curState == 3){
        waterLevel = adc_read(WATER_SENSOR);
        if((waterLevel > waterLevelThreshold) && !(PINE & 0x01))
        {
          PORTB = 0b01000000;
          //clear lcd, print data
          Serial.println("Error to Idle");
          curState = 1;
        }
      }
  }


}


void toggleSystem(){
  if(curState == 0){ //DISABLED
    PORTB = 0b01000000;
    Serial.println("Disabled to Idle");
    monitoring = true;
    curState = 1;
  }
  else{
    monitoring = false;
    //turn off motor
    PORTB = 0b00100000;
    Serial.print(states[curState]);
    Serial.println(" to Disabled");
    curState = 0;
  }
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA |= 0b00000111; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
