#include <Arduino.h>

/*

Pin functions

Pin2 - Flow Meter  (INT)
Pin3 - Stop Pushbutton (INT)
Pin4 - Start Pushbutton 
Pin5 - Manual Pushbutton 
Pin6 - Solenoid Output
Pin7 - Buzzer Output 


*/


volatile uint64_t flowMeterPulseCount = 0;
volatile bool isFilling = false;
volatile long unsigned timeSinceLastPulse = 0;
volatile long unsigned microLitresPerPulse = 0;
volatile long unsigned now = 0;
volatile long offsetPct;

char buf[50];
char buf1[50];

#define NUM_PULSES_FOR_15_LITRES 2000 // example
#define NUM_PULSES_PER_LITRE 270
#define THIRTY_LITRES_IN_PULSES (45 * NUM_PULSES_PER_LITRE)
#define FLOW_METER_PIN PIN2
#define STOP_PB_PIN PIN3
#define START_PB_PIN PIN4
#define MANUAL_PB_PIN PIN5
#define SOLENOID_PIN PIN6
#define BUZZER_PIN PIN7
#define TRIM_PIN A0


// Prototypes
void digitalToggle(int pin);
void Beep(); // uses delay - bad

// State
typedef enum
{
  STOPPED,
  RUNNING,
  MANUAL,
  UNKNOWN
} OutputState;
OutputState state = UNKNOWN;

// static uint64_t microLitresPerPulseLookup[5][2] =
//     {
//         {62500, 2083},
//         {30769, 2051},
//         {20283, 2028},
//         {15267, 2035},
//         {12195, 2031}};

static uint64_t last = 0;

void FlowMeterPulseDetectedISR()
{
  //now = micros(); 
  //timeSinceLastPulse = now - last;

  digitalToggle(LED_BUILTIN);
  long offset = (long)(THIRTY_LITRES_IN_PULSES * offsetPct) / 100;
  
  if (flowMeterPulseCount++ >= (THIRTY_LITRES_IN_PULSES + offset))
  {
    isFilling = false;
    flowMeterPulseCount = 0;
    
    state = STOPPED;
    digitalWrite(SOLENOID_PIN, LOW);
  }
  last = now;
 
}


void OnStartPushbuttonPressed()
{
  //Serial.println("StartPB Pressed");
  Beep();
  if (isFilling == false)
  {
    flowMeterPulseCount = 0;
    offsetPct = map(analogRead(TRIM_PIN), 1023, 0, 30, -30);
    //Serial.println(offsetPct);
    attachInterrupt(digitalPinToInterrupt(PIN2), FlowMeterPulseDetectedISR, RISING);
    digitalWrite(SOLENOID_PIN, HIGH);
    isFilling = true;
  }
}

void OnStopPushbuttonPressed()
{
  //Serial.println("StopPB Pressed");
  Beep();
  if (isFilling == true)
  {
    digitalWrite(SOLENOID_PIN, LOW);
    detachInterrupt(digitalPinToInterrupt(PIN2));
    isFilling = false;
  }
}

void OnManualPushbuttonPressed()
{
  //Serial.println("ManualPB Pressed");
  flowMeterPulseCount = 0;
}

void setup()
{
  flowMeterPulseCount = 0;
  pinMode(FLOW_METER_PIN, INPUT_PULLUP); //flow meter
  pinMode(STOP_PB_PIN, INPUT_PULLUP);
  pinMode(START_PB_PIN, INPUT_PULLUP);
  pinMode(MANUAL_PB_PIN, INPUT_PULLUP);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);


  //Serial.begin(9600);
  //Serial.println("START");
  Beep();
  state = UNKNOWN;
}



void loop()
{
static uint8_t lastState = 0;
static uint16_t manualCount = 0;
//Serial.println((long)flowMeterPulseCount);

// if (lastState != state)
//   Serial.println(state);
lastState = state;
  switch (state)
  {
    case UNKNOWN:
    case STOPPED:
    {
      if (digitalRead(START_PB_PIN) == HIGH)
      {
        OnStartPushbuttonPressed();
        state = RUNNING;
      }
      else if (digitalRead(MANUAL_PB_PIN) == HIGH)
      {
        
        state = MANUAL;
      }
      break;
    }
    case RUNNING:
    {
      if (digitalRead(STOP_PB_PIN) == LOW)
      {
        OnStopPushbuttonPressed();
        state = STOPPED;
      }
      break;
    }
    case MANUAL:
    {
      OnManualPushbuttonPressed();
        if (digitalRead(MANUAL_PB_PIN) == HIGH)
        {
          if(manualCount >= 10)
          {
            digitalWrite(SOLENOID_PIN, HIGH);
            //Serial.println("Manual ON");
            //OnManualPushbuttonPressed();
          }
          else 
          {
            digitalWrite(SOLENOID_PIN, LOW);
            
          }
          state = MANUAL;
          manualCount++;
        }
        else 
        {
          state = STOPPED;
          digitalWrite(SOLENOID_PIN, LOW);
          manualCount = 0;
          
        }
      break;
    }
  }

  delay(100); // wait for a second
}

void Beep() // uses delay - bad
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(20);
  digitalWrite(BUZZER_PIN, LOW);
}

void digitalToggle(int pin)
{
  if (digitalRead(pin) == HIGH)
    digitalWrite(pin, LOW);
  else
    digitalWrite(pin, HIGH);
}

