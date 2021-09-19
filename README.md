
# Passive Wifi Scanner and BLE, for home automation


This repository contains a passive indoor location sensor using an ESP32 and the serial line. 
Once uploaded this firmware periodically send WIFI and BLE advertizing information on the serial line.

Unlike other projects, the Wifi detection is done using the promiscious mode. This permit to observe every Wifi packet, 
even if the Wifi are NOT connected to a given ACCESS POINT.


Applications of this repository are :
- Indoor location
- Cyber pheripherial detection, and observation
- Presence detection (better using BLE)


It use hi serial speed (921600 bauds), to preserve the ASCII format (more human readable).

BLE advertizing are published once received.
WIFI communication, for each channels and sender are summarized every seconds




## Protocol

Datas are sent to serial using a simple correction protocol, with the given format.
This permit to log some usefull information along the datas transmitted.
	
	S[PAYLOAD_LENGTH_USING_3_DECIMAL_DIGIT][PAYLOAD]\n


## Packet format 


__Wifi Packet observation payload are formatted as below:__

	PACKET,CHAN=[CHANNEL_2_DIGIT],RS=[SUM_OF_SIGNAL_DB],C=[RSSI_MEASURE_COUNT],SNDR=[SENDER_MAC_ADDRESS]

They are sent every seconds, 
as there might have multiple values, the sum of signal received and measure count are given
to have the average of the signal, the information user must divide the sum of signal by the measure count

using the measure count, permit to have a cadinality of signal, and precision informations.


Wifi packet example :

	PACKET,CHAN=06,RS=-88,C=1,SNDR=30:f3:83:df:12:6f



__BLE packet :__

Bluetooth low energy advertizing packets are formatted as below :

    BLE,ADDR=[PERIPHERIAL_ADDR],RSSI=[RSSI_SIGNAL],ADVDATA=[PUBLISHED ADVERTISING MANUFACTURER DATAS]

example :

    BLE,ADDR=4ddd6a235c9c,RSSI=-86,ADVDATA=1eff060001092006f14fc1612305d34d1d37e32eca559bb082c90bbce91f94




## Compiling

Install IDF 3.x framework, then source the environement

    source /home/use/projets/2021_Arm_Robot/esp32_control/esp/esp-idf/export.sh

and Flash

    idf.py flash




