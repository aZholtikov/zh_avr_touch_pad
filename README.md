# AVR firmware for wireless touch pad (nRF24L01+)

## Features

1. Wireless touch pad sensor for ATMega168/328 microcontrollers with nRF24L01+ transceiver.
2. Power consumption in the sleep mode approximately 2 uA.
3. Transmits device identifier and battery voltage level on each touch event.
4. Battery measurement through internal ADC with 1.1 V bandgap reference.
5. Wake-up from power-down mode by external interrupt on rising edge.
6. Automatic pairing with the gateway to obtain a device identifier stored in EEPROM.
7. Supports ATMega168, ATMega168P, ATMega168PB, ATMega328, ATMega328P, ATMega328PB.

## Attention

1. A compatible RF Gateway is required to receive and process transmitted packets.
2. After power-on reset the first touch event may not trigger a transmission.
3. Multi-touch is not supported, only a single touch event per transmission cycle.
4. Changing gateway address or channel requires reflashing the microcontroller.
5. If the gateway does not respond during pairing, the device enters permanent sleep until reset.

## Using

In an existing PlatformIO project, run the following command to install the firmware:

```bash
cd your_project
git clone https://github.com/aZholtikov/zh_avr_touch_pad
```
