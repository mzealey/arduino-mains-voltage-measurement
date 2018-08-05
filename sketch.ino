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

#define SENSOR_COUNT 3          // number of current sensors (A0-Ax)

#define INPUT_VOLTAGE 5.0       // for most arduinos
#define AC_VOLTAGE 240.0        // For calculating power
#define BURDEN_RESISTOR 120.0
#define TRANSFORMER_WINDINGS 1800.0

// Amount of time to wait between samples
//#define WAIT_PERIOD 30
#define WAIT_PERIOD 0   // for debugging

// Length of time (ms) to sample for)
#define POLL_SIGNAL_TIME 60000

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

  unsigned long start_time = millis();

  // Algorithm per https://dsp.stackexchange.com/a/1187. But start mean at 512
  // which is half of the 0-1023 range of analogue readings. We keep track of
  // the mean across all pin inputs as we assume that the base input voltage is
  // the same between them all.
  float prev_mean = 511.0, mean = 511.0;
  float S[SENSOR_COUNT];
  unsigned long samples = 0;
  unsigned int mean_count = 1;
  for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ )
    S[pin] = 0.0;

  while( millis() - start_time < POLL_SIGNAL_TIME ) {
    samples++;
    for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ ) {
        unsigned int cur = analogRead(A0 + pin);

        // Only track the last X readings to avoid the mean going stale
        if( mean_count < 3000 )
            mean_count++;
        prev_mean = mean;
        mean += ((float)cur - mean) / mean_count;
        S[pin] += ((float)cur - mean) * ((float)cur - prev_mean);

        /*
        if( pin == 0 ) {
          Serial.print(pin);
          Serial.print("; cur = ");
          Serial.print(cur);
          Serial.print("; mean = ");
          Serial.print(mean);
          Serial.print("; Std dev = ");
          Serial.print(S[pin] / samples);
          Serial.println();
          //delay(1);
        }
        */
    }
  }

  // Compute & print values for the pins
  for ( uint8_t pin = 0; pin < SENSOR_COUNT; pin++ ) {
    float rms = sqrt(S[pin] / samples);

    // All values +- 2LSB according to data sheet so max fluctuation due to
    // analogRead of 8, plus some interference from cable or input voltage
    // fluctuations. Ignore this low-level chatter by setting a threshold
    // (sqrt( +-4 * 2 ) = 4 so say 3 as a reasonable cut-off)
    if( rms < 3 )
        rms = 0;

    float ac_power =
        // RMS value of input. TODO: Why * 4 needed here??
        rms * 4
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
      Serial.print(S[pin] / samples);
      Serial.print("; rms = ");
      Serial.print(rms);
      Serial.print("; mean = ");
      Serial.print(mean);
      Serial.print("; power = ");
      Serial.print(ac_power);
      Serial.print("; total (raw) = ");
      Serial.print(total_usage[pin]);
      Serial.print("; total (wh) = ");
      Serial.print(total_usage[pin] / 3600.0 );
      Serial.println();
#endif
  }
}
