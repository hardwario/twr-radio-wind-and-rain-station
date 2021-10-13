# HARDWARIO TOWER - WindAndRainSensor

HARDWARIO node with wind vane (speed,gust and angle) and rain gauge.
--------

Connection (to newer TOWER Sensor Module with three channels):

- Channel A: Anemometer to ground (with optional external pull-up connected to VDD, see `EXTERNAL_PULLUP_VCC_A` below)
- Channel B: Wind vane to ground
- Channel C: Rain gauge to ground

# External 1 MΩ pull-up for 37 uA

MCU's Internal 60 kOhm pull-up on the wind speed reed contact can increase consumption if the contact is closed and the wind stops blowing.
This conspumption is around 50 uA when contact is opened and 150 uA when is closed. You can use external bigger pull-up resistor and enable it by uncommenting `EXTERNAL_PULLUP_VCC_A`. Then connect resistor between VDD and A screw terminal on the Sensor Module. Default binary is built with internal MCU pull-up.

# Scheduler interval period increase

With `TWR_SCHEDULER_INTERVAL_MS=50` set in the `Makefile` the sheduler is now called less often. So instead of checking every 10 ms for task to run, it checks every 50 ms which is a compromise between low-power and the radio communication which needs lower number than 100 ms to keep ACKing and retransmitting working ok.

# Climate Module support

by uncommenting `//#define CLIMATE_ENABLE` you can add Climate Module to the sensor


# MQTT2Influx configuration

To log wind data from MQTT to InfluxDB you need to add this configuration to /etc/bigclown/mqqt2influxdb.yml

```
  - measurement: rainfall
    topic: node/+/rainfall/+/mm
    fields:
      value: $.payload
    tags:
      id: $.topic[1]
      channel: $.topic[3]

  - measurement: wind
    topic: node/+/wind/+/+
    fields:
      value: $.payload
    tags:
      id: $.topic[1]
      channel: $.topic[4]

```

# Grafana dashboard

import file `grafana-dasboard-import.json` from the repository to display graphs for wind and rain.

![alt text](https://raw.githubusercontent.com/owarek/BC-WindAndRainSensor/master/img/IMG_20181106_155253.jpg)

--------
This is modified version of https://github.com/hubmartin/bcf-sigfox-wind-station. Thank you Martin :)
