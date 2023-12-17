#include <Stepper.h>

#include <LiquidCrystal.h>
#include <dht.h> //install the DHTLib library

#define RDA 0x80
#define TBE 0x20

/*Setup:
LEDs on PB7:4
BLUE - Pin 13
GREEN - Pin 12
YELLOW - Pin 11
RED - Pin 10

DHT11 on Pin 9, PH6
Water Sensor on Pin A7

Display
RS = 11, EN = 12, D4 = 3, D5 = 4, D6 = 5, D7 = 6

Stepper Motor Driver
IN1 - 49
IN2 - 46
IN3 - 47
IN4 - 48

*/

#define WATER_SENSOR 7

#define DHT11_PIN 9
dht DHT;
unsigned long previousMillis = 0;  // will store last time temp was checked
const long interval = 2000;  // interval at which to check temp (milliseconds)

const int RS = 19, EN = 18, D4 = 17, D5 = 16, D6 = 15, D7 = 14;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

const int stepsPerRevolution = 2038;
Stepper stepper = Stepper(stepsPerRevolution, 49, 46, 47, 48);


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
int tempThreshold = 25;
int waterLevel = 101;
int waterLevelThreshold = 100;

void setup() 
{
  // setup the UART
  //U0init(9600);
  Serial.begin(9600);
  // setup the ADC
  adc_init();

  //Start LCD
  lcd.begin(16, 2);

  //Configure Stepper
  stepper.setSpeed(5);

  DDRE &= 0xEF; //Start Button, Input
  PORTE |= 0x10; //Start Button, Pullup
  DDRE &= 0xDF; //Reset Button, Input
  PORTE |= 0x20; //Reset Button, Pullup

  DDRH &= 0xE7; //Vent Controls, Input
  PORTH |= 0x18; //Pullup

  DDRA |= 0x01; //Fan Motor Control
  PORTA &= 0xFE; //Fan Motor Start: Off

  DDRB |= 0xF0; //LEDs, output
  PORTB = 0b00100000; //Enable yellow
  attachInterrupt(digitalPinToInterrupt(2), toggleSystem, FALLING);


}
void loop() 
{
  //Stepper
  if(!(PINH & 0x08)){
    stepper.step(-25);
  }
  else if(!(PINH & 0x10)){
    stepper.step(25);
  }

  //Monitoring
  if(monitoring){
    waterLevel = adc_read(WATER_SENSOR);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
    
      int chk = DHT.read11(DHT11_PIN);
      temp = DHT.temperature;
      lcd.clear();
      lcd.home();
      lcd.print("Temperature = ");
      lcd.print(temp);
      lcd.setCursor(0, 1);
      lcd.print("Humidity = ");
      lcd.print(DHT.humidity);
    }
  }
  else{
    lcd.clear();
  }

  //State Machine Switch
  switch(curState){
    case 0:  //DISABLED
      break;
    
    case 1: //IDLE
      if(waterLevel <= waterLevelThreshold){
        Serial.println("Idle to Error");
        curState = 3;
      }
      else if(temp > tempThreshold){
        PORTB = 0b10000000;
        PORTA |= 0x01;
        Serial.println("Idle to Running");
        curState = 2;
      }
      break;
    
    case 2: //RUNNING
      if(waterLevel <= waterLevelThreshold){
        PORTA &= 0xFE;
        Serial.println("Running to Error");
        curState = 3;
      }
      else if(temp <= tempThreshold){
        PORTA &= 0xFE;
        PORTB = 0b01000000;
        Serial.println("Running to Idle");
        curState = 1;
      }
      break;

    case 3: //ERROR
      PORTB = 0b00010000;
      previousMillis = 0;
      lcd.clear();
      lcd.home();
      lcd.print("Water level");
      lcd.setCursor(0, 1);
      lcd.print("is too low");
      while(curState == 3){
        waterLevel = adc_read(WATER_SENSOR);
        if((waterLevel > waterLevelThreshold) && !(PINE & 0x20))
        {
          PORTB = 0b01000000;
          lcd.clear();
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
    previousMillis = 0;
    PORTA &= 0xFE;
    lcd.clear();
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
