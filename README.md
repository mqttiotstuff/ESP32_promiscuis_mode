
# Passive Wifi Scanner and BLE, for home automation


This repository contains a passive indoor location sensor using an ESP32 and the serial line. 
Once uploaded this firmware periodically send WIFI and BLE advertizing information on the serial line.


![](doc/esp32_dev.jpg)


Unlike other projects, the Wifi detection is done using the promiscious mode. This permit to observe every Wifi packet around, on the several channels.


Applications of this repository are :
- Indoor location
- Device detection, and logging
- Presence detection (both mobile, ble devices) 


Metrics and observations are sent using serial hispeed (921600 bauds), in order to preserve the ASCII format (more human readable).

BLE advertizing are published once received.
WIFI communication, for each channels and sender are summarized every seconds


## Output sample published by the device on the serial :


	S101BLE,ADDR=feed5df56200,RSSI=-59,ADVDATA=0201041bff57010034d61e2774aeb31319e9a550871633ce02feed5df56200
	S048PACKET,CHAN=12,RS=-72,C=1,SNDR=d3:c3:19:a7:64:f0
	S048PACKET,CHAN=13,RS=-96,C=1,SNDR=a1:c7:56:a1:67:07
	S048PACKET,CHAN=13,RS=-97,C=1,SNDR=d4:89:33:89:21:7f
	S048PACKET,CHAN=13,RS=-87,C=1,SNDR=95:79:3c:77:39:53
	S048PACKET,CHAN=13,RS=-96,C=1,SNDR=1a:a2:37:ce:1f:ca
	S048PACKET,CHAN=13,RS=-96,C=1,SNDR=25:94:20:f9:ad:30
	S048PACKET,CHAN=02,RS=-52,C=1,SNDR=87:7d:85:81:8a:04
	S048PACKET,CHAN=12,RS=-72,C=1,SNDR=d3:c3:19:a7:64:f0
	S048PACKET,CHAN=13,RS=-96,C=1,SNDR=a1:c7:56:a1:67:07
	S048PACKET,CHAN=13,RS=-97,C=1,SNDR=d4:89:33:89:21:7f
	S048PACKET,CHAN=13,RS=-87,C=1,SNDR=95:79:3c:77:39:53
	S048PACKET,CHAN=13,RS=-96,C=1,SNDR=1a:a2:37:ce:1f:ca
	S048PACKET,CHAN=13,RS=-96,C=1,SNDR=25:94:20:f9:ad:30
	S048PACKET,CHAN=02,RS=-52,C=1,SNDR=87:7d:85:81:8a:04
	S101BLE,ADDR=feed5df56200,RSSI=-60,ADVDATA=0201041bff57010034d61e2774aeb31319e9a550871633ce02feed5df56200
	S048PACKET,CHAN=04,RS=-33,C=1,SNDR=7e:37:15:09:cf:a8
	S048PACKET,CHAN=04,RS=-35,C=1,SNDR=50:cc:e2:6f:b0:32
	S048PACKET,CHAN=04,RS=-33,C=1,SNDR=72:e7:0e:4a:44:66
	S048PACKET,CHAN=04,RS=-31,C=1,SNDR=ed:e2:82:ef:18:b4
	S048PACKET,CHAN=04,RS=-33,C=1,SNDR=e1:82:83:79:98:be
	S048PACKET,CHAN=04,RS=-32,C=1,SNDR=cc:6e:4a:04:33:63
	S048PACKET,CHAN=04,RS=-35,C=1,SNDR=b1:13:06:36:22:b0
	S048PACKET,CHAN=04,RS=-33,C=1,SNDR=62:db:48:a0:85:b6
	S048PACKET,CHAN=04,RS=-34,C=1,SNDR=79:3b:dc:50:34:7e
	S048PACKET,CHAN=04,RS=-32,C=1,SNDR=39:78:eb:23:56:ee
	S048PACKET,CHAN=04,RS=-36,C=1,SNDR=0a:af:dc:14:b2:a3
	S048PACKET,CHAN=04,RS=-33,C=1,SNDR=4f:4b:c7:17:ef:b8
	S048PACKET,CHAN=04,RS=-33,C=1,SNDR=d9:70:39:41:a0:29
	S048PACKET,CHAN=04,RS=-34,C=1,SNDR=95:f1:a4:4b:e1:e8



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




