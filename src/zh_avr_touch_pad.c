/**
 * @file zh_avr_touch_pad.c
 *
 * @brief AVR firmware for a touch pad sensor device with NRF24 wireless
 *        transmission and battery voltage monitoring.
 *
 * This module implements a low-power touch pad sensor that communicates via
 * NRF24L01+ radio module. The device wakes up on touch pad interrupt, reads
 * battery voltage, transmits sensor data, and returns to power-down sleep
 * mode. It supports ATmega168PB and ATmega328PB microcontrollers.
 *
 * Key features:
 * - Touch pad interrupt-driven wake-up with PCINT2
 * - NRF24L01+ radio communication with blocking send-and-wait
 * - Battery voltage measurement via internal ADC
 * - Watchdog timer for transmit timeout protection
 * - Ultra-low power sleep mode between events
 *
 * @note Device ID and sensor type are compiled into the transmitted data structure.
 * @note All SPI peripheral registers are mapped to SPCR0/SPSR0/SPDR0 for PB compatibility.
 */
#include "avr/io.h"
#include "avr/sleep.h"
#include "avr/fuse.h"
#include "avr/interrupt.h"
#include "avr/wdt.h"
#include "util/delay.h"
#include "math.h"
#include "stdbool.h"
#include "zh_rf_config.h"

#define GATEWAY_CHANNEL 120
#define GATEWAY_ADDRESS 0xDDEEFF
#define DEVICE_ID 6
#define BATTERY_VOLTAGE_CALIBRATION 1.0f

#define NRF_GPIO_CE PB1
#define NRF_GPIO_CSN PB2
#define NRF_GPIO_MOSI PB3
#define NRF_GPIO_MISO PB4
#define NRF_GPIO_SCK PB5
#define NRF_GPIO_IRQ PD2
#define PAD_GPIO_INT PD3

#if defined(__AVR_ATmega168PB__) || defined(__AVR_ATmega328PB__)
#ifndef SPCR
#define SPCR SPCR0
#endif
#ifndef SPDR
#define SPDR SPDR0
#endif
#ifndef SPSR
#define SPSR SPSR0
#endif
#ifndef SPE
#define SPE SPE0
#endif
#ifndef MSTR
#define MSTR MSTR0
#endif
#ifndef SPR0
#define SPR0 SPR00
#endif
#ifndef SPR1
#define SPR1 SPR10
#endif
#ifndef SPI2X
#define SPI2X SPI2X0
#endif
#ifndef SPIF
#define SPIF SPIF0
#endif
#ifndef PRR
#define PRR PRR0
#endif
#ifndef MCUSR
#define MCUSR MCUSR0
#endif
#endif

#define SPI_PRESCALER_IDX() ((((SPSR & (1 << SPI2X)) ? 4U : 0U) | ((SPCR & (1 << SPR1)) ? 2U : 0U) | ((SPCR & (1 << SPR0)) ? 1U : 0U)))
#define SPI_GET_PRESCALER() ((uint16_t)(1UL << ((0x65317642UL >> (SPI_PRESCALER_IDX() * 4U)) & 0x0FU)))
#define SPI_TIMEOUT_TICKS ((uint16_t)(((8UL * SPI_GET_PRESCALER()) / 6) * 4 + 10))

#define NRF_CMD_W_REGISTER 0x20
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_CMD_FLUSH_TX 0xE1
#define NRF_CMD_NOP 0xFF

#define NRF_REG_CONFIG 0x00
#define NRF_REG_EN_AA 0x01
#define NRF_REG_EN_RXADDR 0x02
#define NRF_REG_SETUP_AW 0x03
#define NRF_REG_SETUP_RETR 0x04
#define NRF_REG_RF_CH 0x05
#define NRF_REG_RF_SETUP 0x06
#define NRF_REG_STATUS 0x07
#define NRF_REG_RX_ADDR_P0 0x0A
#define NRF_REG_TX_ADDR 0x10

#define NRF_CONFIG_MASK_RX_DR 0x40
#define NRF_CONFIG_EN_CRC 0x08
#define NRF_CONFIG_PWR_UP 0x02

#define NRF_STATUS_RX_DR 0x40
#define NRF_STATUS_TX_DS 0x20
#define NRF_STATUS_MAX_RT 0x10

typedef char zh_nrf_payload_size_check[(sizeof(zh_rf_sensor_transmitted_data_t) <= 32) ? 1 : -1];

/**
 * @brief Initialize all GPIO ports to input with pull-ups enabled.
 *
 * Configures PORTB, PORTC, PORTD as inputs with internal pull-ups active.
 * On PB microcontrollers, PORTE is also configured identically.
 */
static void _zh_gpio_default_init(void);

/**
 * @brief Configure touch pad pin as interrupt source via PCINT2.
 *
 * Sets PD3 (PAD_GPIO_INT) as input with no pull-up, clears pending
 * interrupt flags, enables PCINT19 in the PCMSK2 mask, and enables
 * the PCIE2 interrupt group.
 */
static void _zh_pad_interrupt_init(void);

/**
 * @brief Initialize SPI peripheral in master mode.
 *
 * Sets CSN high, configures MISO as input, MOSI/SCK/CSN as outputs,
 * and configures SPI with SPE enabled, master mode, and Fosc/16 clock.
 * Reads and discards SPSR and SPDR to clear any pending state.
 */
static void _zh_spi_init(void);

/**
 * @brief Transfer one byte via SPI with timeout and fault handling.
 *
 * Writes data to SPDR, waits for SPIF flag, and returns the received byte.
 * On timeout, disables SPI, reinitializes the peripheral, sets _spi_fault,
 * and returns 0xFF to indicate a communication error.
 *
 * @param data Byte to transmit
 *
 * @return Received byte, or 0xFF on timeout/fault
 */
static uint8_t _zh_spi_transfer(uint8_t data);

/**
 * @brief Initialize NRF24L01+ radio module and configure registers.
 *
 * Waits 100ms for hardware stabilization, configures CE as output and IRQ
 * as input with pull-up, sets INT0 to trigger on falling edge. Writes TX
 * and RX addresses, sets channel to GATEWAY_CHANNEL, enables auto-ack on
 * pipe 0, configures CRC and retransmission settings.
 */
static void _zh_nrf24_init(void);

/**
 * @brief Send a payload via NRF24L01+ and wait for TX completion.
 *
 * Configures PWR_UP mode, flushes TX FIFO, writes the payload, and waits
 * for either TX_DS or MAX_RT interrupt via INT0. Uses watchdog timer as
 * a timeout mechanism. Sleeps in IDLE mode while waiting for the IRQ.
 * On completion, disables CE, stops watchdog, and returns to PWR_DOWN mode.
 * SPI faults during the send sequence abort the transmission.
 *
 * @param buf Pointer to the payload data to send
 * @param size Number of bytes in the payload
 */
static void _zh_nrf24_send(uint8_t *buf, uint8_t size);

/**
 * @brief Write multiple bytes to NRF24L01+ via SPI with a given command.
 *
 * Clears the SPI fault flag, asserts CSN, transfers the command byte,
 * then transfers all data bytes while checking for faults. Deasserts CSN
 * and returns the status byte from the command transfer.
 *
 * @param cmd SPI command byte
 * @param buf Pointer to data buffer to write
 * @param count Number of bytes to write
 *
 * @return Status byte from the command transfer
 */
static uint8_t _zh_nrf24_write_buffer(uint8_t cmd, uint8_t *buf, uint8_t count);

/**
 * @brief Write a single byte to an NRF24L01+ register.
 *
 * Clears the SPI fault flag, asserts CSN, transfers the register address
 * ORed with NRF_CMD_W_REGISTER, then transfers the value if no fault
 * occurred. Deasserts CSN and returns the status byte.
 *
 * @param reg Register address
 * @param val Value to write
 *
 * @return Status byte from the command transfer
 */
static uint8_t _zh_nrf24_write_register(uint8_t reg, uint8_t val);

/**
 * @brief Write multiple bytes to an NRF24L01+ register.
 *
 * Wraps _zh_nrf24_write_buffer with the register address ORed with
 * NRF_CMD_W_REGISTER as the command byte.
 *
 * @param reg Register address
 * @param buf Pointer to data buffer to write
 * @param count Number of bytes to write
 *
 * @return Status byte from the command transfer
 */
static uint8_t _zh_nrf24_write_buff_register(uint8_t reg, uint8_t *buf, uint8_t count);

/**
 * @brief Send a command-only byte to the NRF24L01+ (no data payload).
 *
 * Clears the SPI fault flag, asserts CSN, transfers the command byte,
 * and deasserts CSN. Returns the status byte from the transfer.
 *
 * @param cmd SPI command byte to send
 *
 * @return Status byte from the command transfer
 */
static uint8_t _zh_nrf24_send_command(uint8_t cmd);

/**
 * @brief Start the watchdog timer as a transmit timeout mechanism.
 *
 * Clears the WDRF flag, configures WDTCSR for interrupt-only mode (WDIE)
 * with a ~85ms timeout (WDP1|WDP0 = 256ms prescaler on 128kHz clock).
 * Preserves and restores SREG for atomicity.
 */
static void _zh_wdt_timeout_start(void);

/**
 * @brief Stop the watchdog timer.
 *
 * Clears the WDRF flag and disables WDTCSR by writing zero.
 * Preserves and restores SREG for atomicity.
 */
static void _zh_wdt_timeout_stop(void);

/**
 * @brief Measure battery voltage via internal ADC with bandgap reference.
 *
 * Enables ADC, sets REFS0 for internal 1.1V bandgap reference and selects
 * the 1.1V bandgap input (MUX3|MUX2|MUX1). Waits for settling, performs
 * two dummy conversions, then reads the final result. Disables ADC and
 * powers down the ADC module afterward. Returns voltage in volts, or 0.0f
 * if the ADC reading is zero (indicating a disconnected battery).
 *
 * @return Battery voltage in volts, or 0.0f on zero ADC reading
 */
static float _zh_get_battery_voltage(void);

static volatile bool _send_request = false;
static volatile bool _nrf_irq_fired = false;
static volatile bool _spi_fault = false;

zh_rf_sensor_transmitted_data_t _transmitted_data = {DEVICE_ID, ZH_RF_SENSOR_TOUCH_SWITCH, 0, 0, 0, 0, 0};

int main(void)
{
	cli();
	_zh_wdt_timeout_stop();
	ACSR |= (1 << ACD);
	PRR |= (1 << PRADC);
	_zh_gpio_default_init();
	_zh_pad_interrupt_init();
	_zh_spi_init();
	_zh_nrf24_init();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();
	for (;;)
	{
		if (_send_request == true)
		{
			cli();
			_send_request = false;
			sei();
			_transmitted_data.battery_voltage = (int16_t)lroundf(_zh_get_battery_voltage() * 100.0f);
			SPCR |= (1 << SPE);
			_zh_nrf24_send((uint8_t *)&_transmitted_data, sizeof(_transmitted_data));
			SPCR &= ~(1 << SPE);
		}
		else
		{
			cli();
			if (_send_request == false)
			{
				SPCR &= ~(1 << SPE);
				sleep_enable();
#if defined(sleep_bod_disable)
				sleep_bod_disable();
#endif
				sei();
				sleep_cpu();
			}
			sei();
			sleep_disable();
		}
	}
	return 0;
}

ISR(PCINT2_vect)
{
	if ((PIND & (1 << PAD_GPIO_INT)) != 0)
	{
		_send_request = true;
		sleep_disable();
	}
}

ISR(INT0_vect)
{
	EIMSK &= ~(1 << INT0);
	_nrf_irq_fired = true;
	sleep_disable();
}

ISR(WDT_vect)
{
	wdt_disable();
	_nrf_irq_fired = true;
	sleep_disable();
}

static void _zh_gpio_default_init(void)
{
	DDRB = 0x00;
	PORTB = 0xFF;
	DDRC = 0x00;
	PORTC = 0xFF;
	DDRD = 0x00;
	PORTD = 0xFF;
#if defined(__AVR_ATmega168PB__) || defined(__AVR_ATmega328PB__)
	DDRE = 0x00;
	PORTE = 0xFF;
#endif
}

static void _zh_pad_interrupt_init(void)
{
	DDRD &= ~(1 << PAD_GPIO_INT);
	PORTD &= ~(1 << PAD_GPIO_INT);
	PCIFR = (1 << PCIF2);
	PCMSK2 |= (1 << PCINT19);
	PCICR |= (1 << PCIE2);
}

static void _zh_spi_init(void)
{
	PORTB |= (1 << NRF_GPIO_CSN);
	PORTB &= ~(1 << NRF_GPIO_MISO);
	DDRB |= (1 << NRF_GPIO_CSN) | (1 << NRF_GPIO_MOSI) | (1 << NRF_GPIO_SCK);
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	(void)SPSR;
	(void)SPDR;
}

static uint8_t _zh_spi_transfer(uint8_t data)
{
	uint16_t timeout = SPI_TIMEOUT_TICKS;
	SPDR = data;
	while ((SPSR & (1 << SPIF)) == 0)
	{
		if (--timeout == 0)
		{
			SPCR &= ~(1 << SPE);
			_zh_spi_init();
			_spi_fault = true;
			return 0xFF;
		}
	}
	return SPDR;
}

static void _zh_nrf24_init(void)
{
	_delay_ms(100);
	PORTB &= ~(1 << NRF_GPIO_CE);
	DDRB |= (1 << NRF_GPIO_CE);
	DDRD &= ~(1 << NRF_GPIO_IRQ);
	PORTD |= (1 << NRF_GPIO_IRQ);
	EICRA = (EICRA & ~((1 << ISC01) | (1 << ISC00))) | (1 << ISC01);
	EIMSK &= ~(1 << INT0);
	uint64_t gateway_address = GATEWAY_ADDRESS;
	_zh_nrf24_write_register(NRF_REG_CONFIG, 0x00);
	_zh_nrf24_write_register(NRF_REG_EN_AA, 0x01);
	_zh_nrf24_write_register(NRF_REG_EN_RXADDR, 0x01);
	_zh_nrf24_write_register(NRF_REG_SETUP_AW, 0x01);
	_zh_nrf24_write_register(NRF_REG_SETUP_RETR, 0xFF);
	_zh_nrf24_write_register(NRF_REG_RF_CH, GATEWAY_CHANNEL);
	_zh_nrf24_write_register(NRF_REG_RF_SETUP, 0x26);
	_zh_nrf24_write_buff_register(NRF_REG_TX_ADDR, (uint8_t *)&gateway_address, 3);
	_zh_nrf24_write_buff_register(NRF_REG_RX_ADDR_P0, (uint8_t *)&gateway_address, 3);
	_zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
	_zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static void _zh_nrf24_send(uint8_t *buf, uint8_t size)
{
	bool tx_fault = false;
	_zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP);
	tx_fault |= _spi_fault;
	_delay_ms(5);
	_zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
	tx_fault |= _spi_fault;
	_zh_nrf24_write_buffer(NRF_CMD_W_TX_PAYLOAD, buf, size);
	tx_fault |= _spi_fault;
	if (tx_fault == false)
	{
		_zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
		if (_spi_fault)
		{
			tx_fault = true;
		}
	}
	if (tx_fault == false)
	{
		_nrf_irq_fired = false;
		EIFR = (1 << INTF0);
		EIMSK |= (1 << INT0);
		_zh_wdt_timeout_start();
		set_sleep_mode(SLEEP_MODE_IDLE);
		PORTB |= (1 << NRF_GPIO_CE);
		if ((PIND & (1 << NRF_GPIO_IRQ)) == 0)
		{
			_nrf_irq_fired = true;
		}
		while (_nrf_irq_fired == false)
		{
			cli();
			if (_nrf_irq_fired == false)
			{
				sleep_enable();
				sei();
				sleep_cpu();
			}
			sei();
			sleep_disable();
		}
		PORTB &= ~(1 << NRF_GPIO_CE);
		EIMSK &= ~(1 << INT0);
		_zh_wdt_timeout_stop();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	}
	_zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
	_zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
	_zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static uint8_t _zh_nrf24_write_buffer(uint8_t cmd, uint8_t *buf, uint8_t count)
{
	_spi_fault = false;
	PORTB &= ~(1 << NRF_GPIO_CSN);
	uint8_t status = _zh_spi_transfer(cmd);
	while (count-- != 0)
	{
		if (_spi_fault == true)
		{
			break;
		}
		_zh_spi_transfer(*(buf++));
	}
	PORTB |= (1 << NRF_GPIO_CSN);
	return status;
}

static uint8_t _zh_nrf24_write_register(uint8_t reg, uint8_t val)
{
	_spi_fault = false;
	PORTB &= ~(1 << NRF_GPIO_CSN);
	uint8_t status = _zh_spi_transfer(reg | NRF_CMD_W_REGISTER);
	if (_spi_fault == false)
	{
		_zh_spi_transfer(val);
	}
	PORTB |= (1 << NRF_GPIO_CSN);
	return status;
}

static uint8_t _zh_nrf24_write_buff_register(uint8_t reg, uint8_t *buf, uint8_t count)
{
	return _zh_nrf24_write_buffer((reg | NRF_CMD_W_REGISTER), buf, count);
}

static uint8_t _zh_nrf24_send_command(uint8_t cmd)
{
	_spi_fault = false;
	PORTB &= ~(1 << NRF_GPIO_CSN);
	uint8_t status = _zh_spi_transfer(cmd);
	PORTB |= (1 << NRF_GPIO_CSN);
	return status;
}

static void _zh_wdt_timeout_start(void)
{
	uint8_t sreg = SREG;
	cli();
	wdt_reset();
	MCUSR &= ~(1 << WDRF);
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDIE) | (1 << WDP1) | (1 << WDP0);
	SREG = sreg;
}

static void _zh_wdt_timeout_stop(void)
{
	uint8_t sreg = SREG;
	cli();
	wdt_reset();
	MCUSR &= ~(1 << WDRF);
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = 0x00;
	SREG = sreg;
}

static float _zh_get_battery_voltage(void)
{
	PRR &= ~(1 << PRADC);
	ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	_delay_ms(10);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
	{
	}
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
	{
	}
	uint16_t adc = ADC;
	ADCSRA &= ~(1 << ADEN);
	PRR |= (1 << PRADC);
	if (adc == 0)
	{
		return 0.0f;
	}
	return ((1024.0f * 1.1f) / (float)adc) * BATTERY_VOLTAGE_CALIBRATION;
}