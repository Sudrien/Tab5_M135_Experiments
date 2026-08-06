SCAD is from aoshimak https://www.thingiverse.com/thing:4764611 - copyright https://creativecommons.org/licenses/by-sa/4.0/

STL removes manual supports, minimum dimensions


Spacer/shim is 2.5mm off the top of https://github.com/m5stack/M5_Hardware/blob/master/Products/M001_Module_Proto/Structures/Module_Proto_Board.stl

It could probably be 2mm. 



This code is Claude Opus 5's attempt to get new sensors working. My compile flags for the tab5 are

```
arduino-cli compile --fqbn "esp32:esp32:m5stack_tab5:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc" . && arduino-cli upload --fqbn "esp32:esp32:m5stack_tab5:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc" --port /dev/ttyACM0 . && sleep 2 && arduino-cli monitor --port /dev/ttyACM0 --config baudrate=115200

```

- The GPS is fine NEO-M9N-00B is the expensive one which I'm testing here, AT6668 is the cheaper one. 
- the magnetometer is messed up by the Tab5's speaker. It's pretty useless as a compass.
- The barometer is fine, but needs secret binary stuff. Or maybe it was the magnetometer that needed the binary stuff?
- the gyro is redundant to the tab5. Use the one on the tab5.
- The three banks of DIP switches: RX 1 is on, TX 1 is on, PSS 3 is on. 




future plans

- a extended back that will accommodate a fatter sma antenna, using the second SMA adapter that was included. I got one of those hinged mushroom style ones on order.
- https://github.com/protomaps/PMTiles handling somehow. It will be interactive in the game map style, I think
- maybe a navball

- https://www.printables.com/model/1791811-m5stack-tab5-open-back-case variant opened up further for M5BUS modules
