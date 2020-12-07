#include <OneWire.h>

#define TEMP_SENSOR_PIN 2

#define FIRST_RELAY_PIN 3
#define FIRST_FAN_PIN 0

#define SECOND_RELAY_PIN 4
#define SECOND_FAN_PIN 1

#define TEMP_READINGS 20
#define READ_TEMP_EVERY_MS 300
#define UPDATE_FAN_EVERY 1000

#define CRITICAL_TEMPERATURE 55
#define NO_DATA_DEFAULT_TEMP -127

#define MIN_TEMP 20
#define MAX_TEMP 50

#define MIN_PWM 40
#define MAX_PWM 255

#define UPDATE_DEBUG_EVERY 1000

OneWire _tempSensor(TEMP_SENSOR_PIN);

unsigned long _lastDebugUpdate;
unsigned long _lastTempRead;
unsigned long _lastFanUpdate;

float _tempReadings1[TEMP_READINGS] = {};
float _tempReadings2[TEMP_READINGS] = {};

void setup(void)
{
  //Digispark set max frequency to avoid pwm noise
  TCCR0B = TCCR0B & 0b11111000 | 0b001;
  
  pinMode(TEMP_SENSOR_PIN, INPUT);
  pinMode(FIRST_FAN_PIN, OUTPUT);
  pinMode(FIRST_RELAY_PIN, OUTPUT);
  pinMode(SECOND_FAN_PIN, OUTPUT);
  pinMode(SECOND_RELAY_PIN, OUTPUT);

  _lastTempRead = millis();
  _lastDebugUpdate = millis();
  _lastFanUpdate = millis();

  digitalWrite(FIRST_RELAY_PIN, LOW);
  digitalWrite(SECOND_RELAY_PIN, LOW);
}

void loop(void)
{
  if (millis() - _lastTempRead > READ_TEMP_EVERY_MS) {

    float tempC1 = readTempByIndex(0);
    insertToEnd(_tempReadings1, TEMP_READINGS, tempC1);

    float tempC2 = readTempByIndex(1);
    insertToEnd(_tempReadings2, TEMP_READINGS, tempC2);

    _lastTempRead = millis();
  }
  
  if (millis() - _lastDebugUpdate > UPDATE_DEBUG_EVERY) {

    //printArray(_tempReadings1, TEMP_READINGS);
    
    //float average = calculateAverage(_tempReadings1, TEMP_READINGS);
    //Serial.print("Average = ");
    //Serial.println(average);
    
    //int pwmValue = mapPWMValue(average);
    //Serial.print("PWM = ");
    //Serial.println(pwmValue);

    _lastDebugUpdate = millis();
  }

  if (millis() - _lastFanUpdate > UPDATE_FAN_EVERY) {

    updateFan(FIRST_FAN_PIN, FIRST_RELAY_PIN, _tempReadings1);
    updateFan(SECOND_FAN_PIN, SECOND_RELAY_PIN, _tempReadings2);
    
    _lastFanUpdate = millis();
  }
}

void updateFan(int fanPin, int relayPin, float values[]) {
  
    float average = calculateAverage(values, TEMP_READINGS);
    
    if (isnan(average) || average >= CRITICAL_TEMPERATURE || average < 0) {
      digitalWrite(relayPin, HIGH);
    } else {
      digitalWrite(relayPin, LOW);
    }

    int pwmValue = mapPWMValue(average);
    if (fanPin == SECOND_FAN_PIN) {
      pwmValue = 255;
    }
    analogWrite(fanPin, pwmValue);
}

int mapPWMValue(float value) {
  return map(value, MIN_TEMP, MAX_TEMP, MIN_PWM, MAX_PWM);
}

void insertToEnd(float values[], int arrLength, float newValue) {
    
  for(int i = arrLength-1; i > 0; i--){
      values[i] = values[i-1];
  }
  
  values[0] = newValue;
}

float calculateAverage(float values[], int arrLength) {

  float totalSum = 0;
  int totalCount = 0;
  
  for( int i=0; i <  arrLength; i++) {
    if (values[i] > 0) {
      totalSum += values[i];
      totalCount++;
    }
  }
  
  return totalSum / totalCount;
}

float readTempByIndex(int index) {
  
    _tempSensor.reset_search();
    //delay(250);
  
    byte addr[8];
    int currentSensorIndex = -1;

    do {
      if (_tempSensor.search(addr)) {
        currentSensorIndex++;
      }
      else {
        return (float)NO_DATA_DEFAULT_TEMP;
      }
    } while (currentSensorIndex != index);

    byte type_s;
    
    switch (addr[0]) {
      case 0x10:
        type_s = 1;
        break;
      case 0x28:
        type_s = 0;
        break;
      case 0x22:
        type_s = 0;
        break;
      default:
        return (float)NO_DATA_DEFAULT_TEMP;
    } 

    _tempSensor.reset();
    _tempSensor.select(addr);
    _tempSensor.write(0x44, 1);
    
    //delay(1000);
    delay(30);
    
    _tempSensor.reset();
    _tempSensor.select(addr);    
    _tempSensor.write(0xBE);

    byte i;
    byte data[12];
    
    for (i = 0; i < 9; i++) {
      data[i] = _tempSensor.read();
    }
  
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    return (float)raw / 16.0;
}
