#include <LiquidCrystal.h>
#include <AutoPID.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
/*
Automatic Plant Watering Control System -Credit to DIYhacking.com Arvind Sanjeev
Modified by Joshua C.Vismale for watering plants Autonomously control system 
using Moisture sensor and Flowrate sensor

Measure the liquid/water flow rate using this code using flowrate sensor and 
measure Moisture through moisture (resistivity) sensor. 
Connect Vcc and Gnd of sensor to arduino, and the 
signal line to arduino digital pin 2.
 
 */
#define PUMP 5
byte statusLed    = 13;

byte sensorInterrupt = 0;  
byte sensorPin       = 2;

// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 4.5;

volatile byte pulseCount;  


char PWM_var;

double flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
double moisture_percentage,setpointM,setpointF,outputValM,outputValF;
int sensor_analog;
unsigned long oldTime;
const int sensor_pin = A0;

//pid settings and gains moisture sensor
#define OUTPUT_MINM 30.0
#define OUTPUT_MAXM 70.0
#define KPM 1.7635
#define KIM 1.1035
#define KDM 0.00
AutoPID Moisture_sensor(&moisture_percentage,&setpointM,&outputValM,OUTPUT_MINM,OUTPUT_MAXM,KPM,KIM,KDM);

//pid settings and gains Flow sensor(DC-motor)
#define OUTPUT_MINF 0.10
#define OUTPUT_MAXF 0.50
#define KP_F 6.64
#define KI_F 38.87
#define KD_F 0.00
AutoPID Flow_sensor(&flowRate,&setpointF,&outputValF,OUTPUT_MINF,OUTPUT_MAXF,KP_F,KI_F,KD_F);

void setup() {
  pinMode(sensor_pin,INPUT);
  pinMode(PUMP,OUTPUT);
  Serial.begin(9600);
  while (! Serial); // allow serial to initialize and connect to the computer
   
 pinMode(statusLed, OUTPUT);      // Set up the status LED line as an output
 digitalWrite(statusLed, HIGH);  // We have an active-low LED attached
  
 pinMode(sensorPin, INPUT);
 digitalWrite(sensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;

  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
 attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
 Moisture_sensor.setBangBang(4);
  //set PID update interval to 0.1s 
 Moisture_sensor.setTimeStep(100);
 Flow_sensor.setBangBang(4);
 Flow_sensor.setTimeStep(100);
}

void loop() {
    sensor_analog = analogRead(sensor_pin);
    moisture_percentage = ( 100 - ( (sensor_analog/1023.00) * 100 ) );
    setpointM = 60;
    setpointF = 0.29;

   if((millis() - oldTime) > 1000)    // Only process counters once per second
  { 
    // Disable the interrupt while calculating flow rate and sending the value to
    // the host
    detachInterrupt(sensorInterrupt);
        
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    
    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;
    
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
      
    unsigned int frac;
    
    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;
    
    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING); 
    }

    Moisture_sensor.run();
    Flow_sensor.run();
     PWM_var = 255;
      //Drive dc-motor when we're at setpoint +-10% moisture
      if(Moisture_sensor.atSetPoint(10)){
          analogWrite(PUMP,PWM_var);
          delay(100);
          
          
            
               /*if(Flow_sensor.atSetPoint(10)){
               PWM_var=150;
               analogWrite(PUMP,PWM_var);
               Serial.println("The motor is on");
               Serial.print(PWM_var);
                         
               */
               
      }         
      //Turn off Motor when at setpoint +-1 moisture percentage
      else if(Moisture_sensor.atSetPoint(1)) {
          digitalWrite(PUMP,LOW);
          Serial.println("The motor is off");
          }

// Serial Monitor View
     Serial.print("Moisture Percentage:");
     Serial.print(moisture_percentage);
     Serial.print("Flowrate:");
     Serial.print(flowRate);
     Serial.print("Quantity:");
     Serial.print(totalMilliLitres);
     
//LCD View
    lcd.begin(16,2);
    lcd.setCursor(0,0);
    lcd.print("Moisture:");
    lcd.print(moisture_percentage);
    lcd.print("%");
    lcd.setCursor(0,1);
    lcd.print("FR:");
    lcd.print(flowRate);
    lcd.print("|");
    lcd.print("Qty:");
    lcd.print(totalMilliLitres);

    
/*
Insterrupt Service Routine
 */

}

void pulseCounter()
  {
  // Increment the pulse counter
  pulseCount++;
  }
