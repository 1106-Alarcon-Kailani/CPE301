// Kailani Alarcon
// CPE 301
// Dec 15 2023
// Final Project

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Time.h>
#include <RTClib.h>
#include "DHT.h"

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define DHTPIN 13
#define DHTTYPE DHT11

#define POWER_PIN  12 // water sensor power
#define SIGNAL_PIN A0 // water sensor signal

#define RDA 0x80
#define TBE 0x20  

int SPEED_PIN = 56; // speed of fan
int LDIR_PIN = 55; // directional pin of fan
int RDIR_PIN = 54; // direectional pin of fan
 
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);
void logTime(bool b);

bool standby = false; // Status booleans
bool levelWater = false;
bool levelTemp = false;

// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// GPIO Pointers
volatile unsigned char *port_b = (unsigned char *) 0x25; 
volatile unsigned char *ddr_b =  (unsigned char *) 0x24;

// Timer Pointers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;


unsigned int currentTicks;
unsigned int timer_running;
unsigned int historyValue;
 
int valueWater = 0; // variable to store the water sensor value

const int stepsPerRevolution = 2038; // number of steps per a rotation

Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11); // pins for motors

const int RS = 48, EN = 49, D4 = 50, D5 = 51, D6 = 52, D7 = 53;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

DateTime t; //assigning variavle to date / time

DHT dht(DHTPIN, DHTTYPE);
const float TEMP_THRESHOLD = 23; //  threshold of temperature
const int WATER_THRESHOLD = 300; // thredshold of water
float tempC;
float humi;

RTC_DS1307 rtc;

ISR(TIMER3_COMPA_vect){
  int currentValue = adc_read(valueWater); // Check water level
  historyValue = currentValue;

  if((currentValue < WATER_THRESHOLD) && (levelWater == true)) 
  if(!(currentValue < WATER_THRESHOLD) && (levelWater == false))
  if(currentValue < WATER_THRESHOLD) { levelWater = false;}
  else { levelWater = true; }
  
  if(tempC > TEMP_THRESHOLD) { levelTemp = true; }
  else { levelTemp = false; }
}

void setup() {
  U0Init(9600);
  Serial.begin(9600);
  sei();                  // enable interrupts

  *ddr_b |= 0b00001111;
  
  pinMode(POWER_PIN, OUTPUT); 
  digitalWrite(POWER_PIN, LOW); // turn the sensor OFF

  dht.begin(); // initialize the sensor

  lcd.begin(16, 2); // set up number of columns and rows

  pinMode(SPEED_PIN, OUTPUT); // speed of fan pin
  pinMode(LDIR_PIN, OUTPUT); // left direction fan pin
  pinMode(RDIR_PIN, OUTPUT); // right directional fan pin
  pinMode(SIGNAL_PIN, OUTPUT); // water sensor output pin
}

void loop() {

  int analogValue = analogRead(A1);  // reads the value of the potentiometer (value between 0 and 1023)
  int angle = map(analogValue, -180, 0, 180, 360); // scales it to use it with the stepper (value between 0 and 180)
 
  humi  = dht.readHumidity(); // read humidity
  tempC = dht.readTemperature(); // read temperature as Celsius
  valueWater = analogRead(SIGNAL_PIN); // read the analog value from water sensor

  pinMode(POWER_PIN, HIGH);  // turn the water sensor ON
  digitalWrite(LDIR_PIN,LOW); // turn fan left direction OFF
  digitalWrite(RDIR_PIN,HIGH); // turn fan right direction ON
  
  myStepper.setSpeed(10); // motor
  myStepper.step((stepsPerRevolution/180)*angle); // sets the stepper position according to the scaled value

  Serial.print("Angle: ");
  Serial.println(angle); // print out the position of motor

  delay(6000);
  if (isnan(valueWater)){
    Serial.println("Failed to read from Water sensor!");
  } else {
    if(valueWater < WATER_THRESHOLD){
      Serial.println("Water resevoir is too low!");
    }else{
      Serial.print("Sensor value: "); // read the converted digital value from sensor
      Serial.println(valueWater);
    }
  }
  // check if any reads failed
  if (isnan(humi) || isnan(tempC)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.setCursor(0, 0);
    lcd.print("Failed");
    lcd.clear();
  } else {
    if(tempC > TEMP_THRESHOLD || tempC == TEMP_THRESHOLD){
    digitalWrite(DHTPIN, HIGH); // turn on
    analogWrite(SPEED_PIN, 255); // turn fan HIGH
    lcd.setCursor(0, 0);  // start to print at the first row

    lcd.clear();
    lcd.print("Humi: ");
    lcd.print(humi);      // print the humidity
    lcd.print("%");

    lcd.setCursor(0, 1);  // start to print at the second row

    lcd.print("Temp: ");
    lcd.print(tempC);     // print the temperature
    lcd.print((char)223); // print ° character
    lcd.print("C");
    Serial.println("The fan is turned on");
    *port_b |= 0b01000000; // GREEN led on

    } else if(tempC < TEMP_THRESHOLD){
    digitalWrite(DHTPIN, LOW); // turn off
    analogWrite(SPEED_PIN, LOW); // turn fan LOW

    lcd.clear();
    lcd.print("Humi: ");
    lcd.print(humi);      // print the humidity
    lcd.print("%");

    lcd.setCursor(0, 1);  // start to print at the second row

    lcd.print("Temp: ");
    lcd.print(tempC);     // print the temperature
    lcd.print((char)223); // print ° character
    lcd.print("C");
    Serial.println("The fan is turned off");
      *port_b |= 0b00100000;
      *port_b &= 0b00111111; 
    }
  }
}

void U0Init(int U0baud)
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

unsigned int adc_read(unsigned char adc_channel_num){
  uint8_t low, high;

  if (adc_channel_num >= 54) adc_channel_num -= 54; // allow for channel or pin numbers

  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adc_channel_num >> 3) & 0x01) << MUX5);  // where data is being read from

  ADMUX = (1 << 6) | (adc_channel_num & 0x07);

 #if defined(ADCSRA) && defined(ADCL)

  sbi(ADCSRA, ADSC); //intializes convo

  while (bit_is_set(ADCSRA, ADSC)); // clear bit
  
  low  = ADCL;
  high = ADCH;
#else
  // no data, return 0
  low  = 0;
  high = 0;
#endif

  return (high << 8) | low;
}

