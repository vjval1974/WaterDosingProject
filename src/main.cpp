#include <Arduino.h>

volatile uint64_t flowMeterPulseCount = 0; 
volatile bool isFilling = false;
#define NUM_PULSES_FOR_15_LITRES 209785 // example

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

      digitalToggle(LED_BUILTIN);
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


void setup() {
  flowMeterPulseCount = 0;
  pinMode(PIN2, INPUT_PULLUP); //flow meter
  pinMode(PIN3, INPUT_PULLUP);
  
  
  attachInterrupt(digitalPinToInterrupt(PIN3), StopPushbuttonDetectedISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_A3), StartPushbuttonDetectedISR, RISING);
  // TODO: cant use interrupts on any other that 2 and 3

//test code
  attachInterrupt(digitalPinToInterrupt(PIN2), FlowMeterPulseDetectedISR, RISING);
  pinMode(LED_BUILTIN, OUTPUT);
  //sei();
  
  Serial.begin(9600);
  Serial.println("START");
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println(map(analogRead(PIN_A0), 0,1024, 100, -100 ));
  

  delay(1000);                       // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  // delay(1000);       
}



