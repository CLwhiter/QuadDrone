# QuadDrone ANO Remote Controller Board

This board definition provides support for the QuadDrone ANO Remote Controller hardware based on STM32F103C8T6 microcontroller.

## Hardware Specifications

- MCU: STM32F103C8T6 (ARM Cortex-M3, 72MHz, 20KB RAM, 64KB Flash)
- Clock: External 8MHz crystal with PLL multiplication (72MHz)
- LED: PC13 (active low)
- UART1: PA9 (TX) / PA10 (RX) for console
- I2C1: PB6 (SCL) / PB7 (SDA) for OLED display
- SPI2: PB13 (SCK), PB14 (MISO), PB15 (MOSI), PB12 (CSN) for NRF24L01
- GPIO keys: PA7, PA8 (pull-up), PB10 (pull-up)
- ADC1: 9 channels for joystick inputs and battery monitoring

## Build and Usage

```
west build -b quaddrone_remote QuadDrone/app
west build -b quaddrone_remote QuadDrone/app -t run
```

## Pin Mapping

| Function | Pin | Peripheral |
|----------|-----|------------|
| LED | PC13 | GPIO |
| UART1 TX | PA9 | USART1 |
| UART1 RX | PA10 | USART1 |
| I2C1 SCL | PB6 | I2C1 |
| I2C1 SDA | PB7 | I2C1 |
| SPI2 SCK | PB13 | SPI2 |
| SPI2 MISO | PB14 | SPI2 |
| SPI2 MOSI | PB15 | SPI2 |
| SPI2 CSN | PB12 | GPIO |
| SPI2 CE | PB5 | GPIO |
| ADC CH0 | PA0 | ADC1 |
| ADC CH1 | PA1 | ADC1 |
| ADC CH2 | PA2 | ADC1 |
| ADC CH3 | PA3 | ADC1 |
| ADC CH4 | PA4 | ADC1 |
| Key Left | PB0 | GPIO (pull-up) |
| Key Right | PB1 | GPIO (pull-up) |
| Key 1 | PA7 | GPIO (pull-up) |
| Key 2 | PA8 | GPIO (pull-up) |

## Dependencies

This board requires the Zephyr STM32 HAL support (included in west.yml).