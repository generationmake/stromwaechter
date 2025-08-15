# stromwaechter
![stromwaechter](http://stromwaechter.generationmake.de/images/stromwaechter_intro.jpg)

This is the software for the stromwaechter pcb. A board based on the ESP8266 which measures voltages and current on each channel and sends these values via MQTT.

The complete documentation can be found on http://stromwaechter.generationmake.de/

The board send the following MQTT messages:

- \<mac\>/vbus                 - bus voltage in volt
- \<mac\>/temperature          - temperature of board
- \<mac\>/\<numsensor\>/voltage  - voltage in volt of channel
- \<mac\>/\<numsensor\>/current  - current in ampere of channel
- \<mac\>/\<numsensor\>/state    - state of channel (1=on; 0=off)
- \<mac\>/version              - firmware version
- \<mac\>/ip                   - ip address of board
- \<mac\>/mac                  - mac address of board
- \<mac\>/wlan                 - wifi quality

\<mac\> is like b4-e6-2d-3f-62-05

\<numsensor\> is 1 for the first channel, 2 for the second and so on
