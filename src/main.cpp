#include <Arduino.h>

volatile uint64_t flowMeterPulseCount = 0; 
volatile bool isFilling = false;
#define NUM_PULSES_FOR_15_LITRES 209785 // example

void FlowMeterPulseDetectedISR()
{
  if (flowMeterPulseCount++ >= NUM_PULSES_FOR_15_LITRES )
  {
    isFilling = false;
    flowMeterPulseCount = 0;
    //todo: set output to LOW
  }
}

void StartPushbuttonDetectedISR()
{
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last > 200)
  {
    if (isFilling == false)
    {
      flowMeterPulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(PIN2), FlowMeterPulseDetectedISR, RISING);
      //todo: set output to HIGH
    }
    
  }
  last = now;
}

void StopPushbuttonDetectedISR()
{
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last > 200)
  {
    if (isFilling == true)
    {
       detachInterrupt(digitalPinToInterrupt(PIN2));
       flowMeterPulseCount = 0;
       //todo: set output to LOW
    }
    
  }
  last = now;
}


void setup() {
  flowMeterPulseCount = 0;
  
  attachInterrupt(digitalPinToInterrupt(PIN3), StartPushbuttonDetectedISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN4), StopPushbuttonDetectedISR, RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
}



