/* Arduino sketch to measure a number of mains line power usage via split-core
 * transformer circuit (based on SCT013-060 which has 1800 windings
 * http://www.yhdc.us/ENpdf/SCT013-060-0-60A-0-1V_en.pdf) and MySensors to
 * integrate into Domoticz
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

#define SENSOR_COUNT 6          // number of current sensors (A0-Ax)

#define INPUT_VOLTAGE 5.0       // for most arduinos
#define AC_VOLTAGE 240.0        // For calculating power
#define BURDEN_RESISTOR 120.0
#define TRANSFORMER_WINDINGS 1800.0

// Amount of time to wait between samples
#define WAIT_PERIOD 30
//#define WAIT_PERIOD 2   // for debugging

// Spend 3 50hz cycles taking readings
#define POLL_SIGNAL_TIME (3 * (1000 / 50 ))

#ifdef USE_MY_SENSORS
#include <MyConfig.h>

#define MY_GATEWAY_SERIAL
#define MY_GATEWAY_FEATURE
//#define MY_DEBUG

#include <MySensors.h>
#else
#define wait delay
#endif

float total_usage[SENSOR_COUNT];

#ifdef USE_MY_SENSORS
void presentation()
{
  for ( uint8_t i = 0; i < SENSOR_COUNT; i++ ) {
    present( i, S_POWER );
    // try to get previous value from domoticz
    request( i, V_KWH );
  }
}

void receive(const MyMessage &message)
{
    if( message.sensor >= SENSOR_COUNT )
        return;

    if( message.type == V_KWH )
        total_usage[message.sensor] = message.getFloat() * 1000.0 * 3600.0;
}
#endif

void setup()
{
#ifndef USE_MY_SENSORS
  Serial.begin(115200);
#endif

  // Not strictly required
  for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ ) {
    total_usage[pin] = 0;
    pinMode(A0 + pin, INPUT);
  }
}

void loop()
{
  wait(WAIT_PERIOD * 1000);

#ifdef USE_MY_SENSORS
  MyMessage msg(0, V_KWH);
#endif

  int reading_max[SENSOR_COUNT], reading_min[SENSOR_COUNT];
  for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ ) {
    reading_max[pin] = 0;
    reading_min[pin] = 1024;
  }

  unsigned long start_time = millis();
  while( millis() - start_time < POLL_SIGNAL_TIME ) {
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

    total_usage[pin] += ac_power * ( (float)WAIT_PERIOD + (float)POLL_SIGNAL_TIME / 1000.0 );

#ifdef USE_MY_SENSORS
    msg.setSensor(pin);
    msg.setType(V_KWH);
    send(msg.set( total_usage[pin] / 1000.0 / 3600.0, 3 ));
    msg.setType(V_WATT);
    send(msg.set((uint32_t)ac_power));
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
}
