Arduino sketch to measure a number of mains line power usage via split-core
transformer circuit (based on SCT013-060 which has 1800 windings -
http://www.yhdc.us/ENpdf/SCT013-060-0-60A-0-1V_en.pdf) and
MySensors to integrate into Domoticz

Use a circuit based on
http://www.homautomation.org/2013/09/17/current-monitoring-with-non-invasive-sensor-and-arduino/
with a single voltage divider linking into multiple parallel 120 Ohm burden
resistor/transformer circuits (which should be good for up to ~30A of
measurement per circuit) each of which go in to a nano analog input.

Mark Zealey 2018
