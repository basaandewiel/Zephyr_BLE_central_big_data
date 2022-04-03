# Zephyr_BLE_central_big_data

Bluetooth peripheral (server side) - C version

This program is the counterpart of the accompanion program ```Zephyr_BLE_periph_big_data```. The latter can send high volume data via Bluetooth BLE, by splitting the data into small chunks and inserting extra information so that the receiving side (the central program) is able to reassemble the received data chunks, into the high volume data as sent by the peripheral program. This central program does this reassembling of received data.

The central part is both available as a Python script as as a C program (this repo).

This application can be used together with

    Zephyr_BLE_periph_big_data
    
    
## Building and running
For how to build and run, see https://basaandewiel.github.io/Zephyr_bluetooth_programming/
