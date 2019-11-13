#include <Arduino.h>

volatile uint64_t flowMeterPulseCount = 0; 
volatile bool isFilling = false;
#define NUM_PULSES_FOR_15_LITRES 209785 // example


/*

Pin functions

Pin2 - Flow Meter  (INT)
Pin3 - Stop Pushbutton (INT)
Pin4 - Start Pushbutton 
Pin5 - Manual Pushbutton 
Pin6 - Solenoid Output
Pin7 - Buzzer Output 


*/ 

void digitalToggle(int pin);

static uint64_t microLitresPerPulseLookup[5][2] = 
{
  {62500, 2083},
  {30769, 2051},
  {20283, 2028},
  {15267, 2035},
  {12195, 2031}
};

void FlowMeterPulseDetectedISR()
{
 
uint64_t now = micros();
 cli();
static uint64_t last = 0;

uint64_t timeSinceLastPulse = now - last;
uint64_t microLitresPerPulse = 0;

if (timeSinceLastPulse <= 1000) // debouce/noise
{
  last = now;
  return;
}

// need frequency/period to determine flow rate per pulse. 

// for (int i = 0; i < 5; i++)
// {
//   if (timeSinceLastPulse >= microLitresPerPulseLookup[i][0])
//   {
//     microLitresPerPulse
//   }
//}
  char buf[50];
  sprintf(buf, "PulseCount=%d", flowMeterPulseCount);
  Serial.println(buf);
  digitalToggle(LED_BUILTIN);

  if (flowMeterPulseCount++ >= NUM_PULSES_FOR_15_LITRES )
  {
    isFilling = false;
    flowMeterPulseCount = 0;
    


    //todo: set output to LOW
  }

  last = now;
  sei();
}

void digitalToggle(int pin)
{
  if (digitalRead(pin) == HIGH)
    digitalWrite(pin, LOW);
  else
    digitalWrite(pin, HIGH);
}

void StartPushbuttonDetectedISR()
{
  uint32_t now = millis();
  cli();
  static uint32_t last = 0;
  
  if (now - last > 500)
  {
    Serial.println("StartPB Pressed");
    if (isFilling == false)
    {
      flowMeterPulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(PIN2), FlowMeterPulseDetectedISR, RISING);
      //todo: set output to HIGH

      //digitalToggle(LED_BUILTIN);
    }
    
  }
  last = now;
  sei();
}

void StopPushbuttonDetectedISR()
{
  uint32_t now = millis();
  cli();
  static uint32_t last = 0;
  
   Serial.println("StopPB Pressed");
  if (now - last > 50)
  {
   
    if (isFilling == true)
    {
       detachInterrupt(digitalPinToInterrupt(PIN2));
       flowMeterPulseCount = 0;
       //todo: set output to LOW
    }
    
  }
  last = now;
  sei();
  
}

#define FLOW_METER_PIN PIN2
#define STOP_PB_PIN PIN3
#define START_PB_PIN PIN4
#define MANUAL_PB_PIN PIN5
#define SOLENOID_PIN PIN6
#define BUZZER_PIN PIN7
#define TRIM_PIN PINA0


void setup() {
  flowMeterPulseCount = 0;
  pinMode(FLOW_METER_PIN, INPUT_PULLUP); //flow meter
  pinMode(STOP_PB_PIN, INPUT_PULLUP);
  pinMode(START_PB_PIN, INPUT_PULLUP);
  pinMode(MANUAL_PB_PIN, INPUT_PULLUP);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(STOP_PB_PIN), StopPushbuttonDetectedISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_A3), StartPushbuttonDetectedISR, RISING);
  // TODO: cant use interrupts on any other that 2 and 3

//test code
  // attachInterrupt(digitalPinToInterrupt(FLOW_METER_PIN), FlowMeterPulseDetectedISR, RISING);
  // pinMode(LED_BUILTIN, OUTPUT);
  


  
  Serial.begin(9600);
  Serial.println("START");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);  


}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  //Serial.println(map(analogRead(PINA0), 0,1024, 100, -100 ));
  

  delay(1000);                       // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  // delay(1000);       
}



