
# Passive Wifi Scanner and BLE, for home automation

## Compile

    source /home/use/projets/2021_Arm_Robot/esp32_control/esp/esp-idf/export.sh


## packet receive format 


Wifi Packet :

    PACKET,TYPE=DATA,CHAN=07,RSSI=-90,RCVR=f8:ba:6e:26:a9:0d,SNDR=03:84:79:6b:69:56,FILT=85:a8:3f:eb:5c:4d

RCVR is the receiver mac address
SNDR is the sender mac address
FILT is the filtering mac address

BLE packet :

    BLE,ADDR=4ddd6a235c9c,RSSI=-86,ADVDATA=1eff060001092006f14fc1612305d34d1d37e32eca559bb082c90bbce91f94


using max baud speed to stdout : 921600

