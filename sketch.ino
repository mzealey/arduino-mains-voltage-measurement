/* Arduino sketch to measure a number of mains line power usage via split-core
 * transformer circuit (based on SCT013-060 which has 1800 windings) and
 * MySensors to integrate into Domoticz
 *
 * Use a circuit based on
 * http://www.homautomation.org/2013/09/17/current-monitoring-with-non-invasive-sensor-and-arduino/
 * with a single voltage divider linking into multiple parallel 120 Ohm burden
 * resistor/transformer circuits (which should be good for up to ~30A of
 * measurement per circuit) each of which go in to a nano analog input.
 *
 * Mark Zealey 2018. Released under GPL
 */
#define USE_MY_SENSORS

#define SENSOR_COUNT 3          // number of current sensors (A0-Ax)

#define INPUT_VOLTAGE 5.0       // for most arduinos
#define AC_VOLTAGE 240.0        // For calculating power
#define BURDEN_RESISTOR 120.0
#define TRANSFORMER_WINDINGS 1800.0

// Amount of time to wait between samples
#define WAIT_PERIOD 30
//#define WAIT_PERIOD 2   // for debugging

#ifdef USE_MY_SENSORS
#include <MyConfig.h>

#define MY_GATEWAY_SERIAL
#define MY_GATEWAY_FEATURE
//#define MY_DEBUG

#include <MySensors.h>
#else
#define wait delay
#endif

#ifdef USE_MY_SENSORS
void presentation()
{
  for ( uint8_t i = 0; i < SENSOR_COUNT; i++ )
    present( i, S_POWER );
}
#endif

void setup()
{
#ifndef USE_MY_SENSORS
  Serial.begin(115200);
#endif

  // Not strictly required
  for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ )
    pinMode(A0 + pin, INPUT);
}

void loop()
{
#ifdef USE_MY_SENSORS
  MyMessage msg(0, V_WATT);
#endif

  int reading_max[SENSOR_COUNT], reading_min[SENSOR_COUNT];
  for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ ) {
    reading_max[pin] = 0;
    reading_min[pin] = 1024;
  }

  // Spend 3 50hz cycles taking readings
  unsigned long start_time = millis();
  while( millis() - start_time < 3* (1000/50) ) {
    for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ ) {
        int cur = analogRead(A0 + pin);
        if( reading_max[pin] < cur )
            reading_max[pin] = cur;
        if( reading_min[pin] > cur )
            reading_min[pin] = cur;
        /*
      Serial.print(pin);
      Serial.print(" ");
      Serial.println(analogRead(pin));
      delay(1);
      */
    }
  }

  // Compute & print values for the pins
  for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ ) {
    int diff = reading_max[pin] - reading_min[pin];

    // All values +- 2LSB according to data sheet so max fluctuation due to
    // analogRead of 8, plus some interference from cable or input voltage
    // fluctuations. Ignore these.
    if( diff < 15 )
        diff = 0;

    float ac_power =
        // RMS value of input
        (float)diff * sqrt(2.0)
        // Convert to volts in (range of analogRead)
        * (INPUT_VOLTAGE / 1023.0)
        // Convert voltage read to current
        / BURDEN_RESISTOR
        // The amount the input is stepped down
        * TRANSFORMER_WINDINGS
        // AC voltage
        * AC_VOLTAGE;

#ifdef USE_MY_SENSORS
    msg.setSensor(pin);
    send(msg.set((int)ac_power));
#else
      Serial.print(pin);
      Serial.print(" ");
      Serial.print(reading_max[pin]);
      Serial.print(" ");
      Serial.print(reading_min[pin]);
      Serial.print(" ");
      Serial.print(ac_power);
      Serial.print(" ");
      Serial.println();
#endif
  }
  wait(WAIT_PERIOD * 1000);
}