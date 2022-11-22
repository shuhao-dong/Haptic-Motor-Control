#include <Arduino.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Stepper.h>

// define objects
Adafruit_MCP4725 dac0;

// PWM properties
const int pwmPin = 13;
const int freq = 732;
const int ledChannel = 0;
const int resolution= 8;

// boolean variables
bool autoMode = false;
bool loop2Break = false;

// experimental variables
int delayTime = 19;
int downStep = 255;
int cutoffVol = 0;

// stepper motor
const int stepsPerRevolution = 2048;

// ULN2003 motor driver pins
#define IN1 D10
#define IN2 D11
#define IN3 D12
#define IN4 D13

// Joystick Pins
#define inputX A3
#define inputY A4
#define inputC D2

// Joystick readings
int outputX = 0;
int outputY = 0;
// Switch readings
int outputC = 0;

// Initialise stepper motor
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

TaskHandle_t Task1;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(230400);

  while(!Serial);
  delay(1000);
  Serial.println("Serial communication set up");

  // set up pin mode
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(inputC, INPUT);

  // set up DAC converter
  //dac0.begin(0x62);
  if(!dac0.begin(0x62)){
    Serial.println("Failed to connect to the converter.");
    while(1);
  }

  // set up pwm properties
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(pwmPin, ledChannel);

  // stepper motor speed
  myStepper.setSpeed(5);

  // Initialise your task (2nd loop)
  xTaskCreatePinnedToCore(
    loop2,            // name of the task function
    "printLettera",   // name of the task
    1024,             // memory assigned for the task
    NULL,             // parameter to pass if any
    1,                // priority of task, starting from 0(Highestpriority) 
                      // *IMPORTANT*( if set to 1 and there is no activity in your 2nd loop, it will reset the esp32)
    &Task1,           // Reference name of taskHandle variable
    1);               // choose core (0 or 1)

   // LED for final check
   digitalWrite(LED_BUILTIN, HIGH);
   delay(500);
   digitalWrite(LED_BUILTIN, LOW);
}

// loop 2: For direction control
void loop2(void*parameter){
  for (;;){

    // Read touch sensor (channel C) and joystick (channel X);
    outputX = analogRead(inputX);

    // Mannually rotate the step motor for dynamic test
    if(outputX < 100){
      myStepper.step(-stepsPerRevolution/1024);
    }
    else if(outputX > 4000){
      myStepper.step(stepsPerRevolution/1024);
    }
  }
}

// Loop 1: For haptic motor control 
void loop() {
  // put your main code here, to run repeatedly:
  uint32_t dac0Vol = 4095, dac1Vol = 2047;
  const int dutyCycle = 127;
  int inComing = Serial.read();

  // write specified input signal wave
  ledcWrite(ledChannel, dutyCycle);

  // read joystick data and rotate the motor mannually
  outputY = analogRead(inputY);

  // Rotate stepper motor manually 
  if(outputY < 100){
    myStepper.step(-stepsPerRevolution/1024);
  }
  else if(outputY > 4000){
    myStepper.step(stepsPerRevolution/1024);
  }
  
  /*  Serial monitor input responses: 
   *  For 1~9: 9 configurations test mode
   *  For 0: initial display of 9 configurations
   */
  
  if(Serial.available()>0){
    if (inComing == 48){
      autoMode = true;
      Serial.println("Initial display is ready to go");
      if (autoMode){
        Serial.println("Display 9 configurations at once");
        delay(500);
        Serial.print("Starting in: \t");
        Serial.print("3 \t");
        delay(1000);
        Serial.print("2 \t");
        delay(1000);
        Serial.println("1 \t");
        delay(1000);

        // delay time configurations
        hapticConfig(19, 255,0,19);
        delay(3000);
        hapticConfig(0,255,0,39);
        delay(3000);
        hapticConfig(39,255,0,15);
        delay(3000);

        // ramp down step length configurations
        hapticConfig(19,1024,0,29);
        delay(3000);
        hapticConfig(19,64,0,15);
        delay(3000);
        hapticConfig(19,255,0,19);
        delay(3000);

        // cut-off voltage configurations
        hapticConfig(19,255,255,19);
        delay(3000);
        hapticConfig(19,255,1364,19);
        delay(3000);
        hapticConfig(19,255,511,19);
        delay(3000);

        Serial.println("Initial display ends");
      }
      autoMode = false;
    }
    
    // type "a" to start dynamic test haptic configuration
    else if (inComing == 97){
      autoMode = true;
      while (autoMode){
        outputC = digitalRead(inputC);
        if (outputC == 0){
          for (int counter = 0; counter <= delayTime; counter ++){
            dac0.setVoltage(4095, false);
            delay(1);
          } 
          for (int value = 4095; value >= cutoffVol; value = value - downStep){
            dac0.setVoltage(value, false);
            delay(1);
          }
        }
        else if (outputC == 1){
          Serial.println("Touched detected! Stop");
          autoMode = false;
        }
      }
    }

    // type "b" to trigger normal symmetrical vibrations during the dynamic test
    else if (inComing == 98){
      autoMode = true;
      while (autoMode){
        outputC = digitalRead(inputC);
        if (outputC == 0){
          for (int value = 0; value <= 4095; value += 255){
            dac0.setVoltage(value, false);
            Serial.println(value);
            delay(1);
          }
          for (int value = 4095; value >= 0; value -= 255){
            dac0.setVoltage(value, false);
            Serial.println(value,false);
            delay(1);
          }
        }
        else if (outputC == 1){
          autoMode = false;
        }
      }
    }

    // Static test: left or right or not sure
    else if (inComing == 49){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 1");
        delay(1500);
        hapticConfig(19,255,0,19);
        Serial.println("Test 1 finished");
      }
      autoMode = false;
    }

    else if (inComing == 50){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 2");
        delay(1500);
        hapticConfig(0,255,0,19);
        Serial.println("Test 2 finished");
      }
      autoMode = false;
    }

    else if (inComing == 51){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 3");
        delay(1500);
        hapticConfig(39,255,0,19);
        Serial.println("Test 3 finished");
      }
      autoMode = false;
    }

    else if (inComing == 52){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 4");
        delay(1500);
        hapticConfig(19,255,0,29);
        Serial.println("Test 4 finished");
      }
      autoMode = false;
    }

    else if (inComing == 53){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 5");
        delay(1500);
        hapticConfig(19,64,0,19);
        Serial.println("Test 5 finished");
      }
      autoMode = false;
    }

    else if (inComing == 54){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 6");
        delay(1500);
        hapticConfig(19,511,0,29);
        Serial.println("Test 6 finished");
      }
      autoMode = false;
    }

    else if (inComing == 55){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 7");
        delay(1500);
        hapticConfig(19,255,255,19);
        Serial.println("Test 7 finished");
      }
      autoMode = false;
    }

    else if (inComing == 56){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 8");
        delay(1500);
        hapticConfig(19,255,127,19);
        Serial.println("Test 8 finished");
      }
      autoMode = false;
    }

    else if (inComing == 57){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Test 9");
        delay(1500);
        hapticConfig(19,255,511,19);
        Serial.println("Test number 9 ends");
      }
      autoMode = false;
    }

    else if (inComing == 65){
      autoMode = true;
      if (autoMode){
        Serial.println("Starting Square Trajectory");
        delay(1500);
        /*Read in enoder value:
         * if value is 90 degree, then start the haptic sequence; 
         * if not, rotate the motor to 90 first automatically, then start the sequence.
         */    
         
      }
    }
  }
}

void hapticConfig(int delayTime, int downStep, int cutoffVol, int maxRepeat){
  uint32_t dac0Vol = 4095;
  for (int repeatTime = 0; repeatTime <= maxRepeat; repeatTime ++){
      for (int counter = 0; counter <= delayTime; counter ++){
        dac0.setVoltage(dac0Vol, false);
        delay(1);
        Serial.println(dac0Vol);
      } 
      for (int value = 4095; value >= cutoffVol; value = value - downStep){
        dac0.setVoltage(value, false);
        delay(1);
        Serial.println(value);
      }
    }
}

void noHapticConfig(int maxRepeat, int volStep){
  for (int repeatTime = 0; repeatTime <= maxRepeat; repeatTime ++){
    for (int value = 0; value <= 4095; value += volStep){
      dac0.setVoltage(value, false);
      Serial.println(value);
      delay(1);
    }
    for (int value = 4095; value >= 0; value -= volStep){
      dac0.setVoltage(value, false);
      Serial.println(value,false);
      delay(1);
    }
  }
}
