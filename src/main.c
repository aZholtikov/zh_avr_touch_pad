#include "avr/io.h"
#include "avr/sleep.h"
#include "avr/fuse.h"
#include "avr/interrupt.h"
#include "util/delay.h"

#if defined(__AVR_ATmega328PB__)
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
#ifndef SPIF
#define SPIF SPIF0
#endif
#endif

#define GATEWAY_CHANNEL 120
#define GATEWAY_ADDRESS 0xDDEEFF
#define DEVICE_ID 6
#define DEVICE_TYPE 4

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

/**
 * @brief Initialize INT1 external interrupt for the touch-pad wake-up signal.
 *
 * Configures PD3 as a rising-edge triggered external interrupt source and
 * enables the INT1 interrupt mask bit. The ISR sets the send_request flag
 * to trigger a radio transmission from the main loop.
 */
static void zh_pad_interrupt_init(void);

/**
 * @brief Initialize the hardware SPI peripheral in master mode.
 *
 * Configures MOSI, SCK and SS pins as outputs, drives SS high, and enables
 * SPI with Fosc/16 clock rate.
 */
static void zh_spi_init(void);

/**
 * @brief Perform a single full-duplex SPI byte transfer.
 *
 * @param data Byte to transmit
 *
 * @return Byte received from the slave during the transfer
 */
static uint8_t zh_spi_transfer(uint8_t data);

/**
 * @brief Initialize the nRF24L01+ transceiver for TX-only operation.
 *
 * Drives CE low, configures auto-acknowledgment, address width, retransmit
 * settings, channel, RF power and data rate, and loads the gateway address
 * into both TX and RX pipe 0 registers.
 */
static void zh_nrf24_init(void);

/**
 * @brief Transmit a payload buffer through the nRF24L01+ module.
 *
 * Powers up the radio, flushes the TX FIFO, loads the payload, pulses CE to
 * start transmission, waits for the TX_DS or MAX_RT flag, then clears status
 * flags and returns the radio to power-down mode.
 *
 * @param buf Pointer to the payload data to transmit (must not be NULL)
 * @param size Number of bytes in the payload buffer (must be > 0)
 */
static void zh_nrf24_send(uint8_t *buf, uint8_t size);

/**
 * @brief Write a data buffer to the nRF24L01+ via SPI with a given command.
 *
 * Asserts CSN low, sends the command byte followed by the buffer contents,
 * then deasserts CSN.
 *
 * @param cmd SPI command byte to send before the data
 * @param buf Pointer to the data buffer to write (must not be NULL)
 * @param count Number of bytes to transfer from the buffer
 *
 * @return Status register value returned by the nRF24L01+
 */
static uint8_t zh_nrf24_write_buffer(uint8_t cmd, uint8_t *buf, uint8_t count);

/**
 * @brief Write a single byte value to an nRF24L01+ register.
 *
 * @param reg Register address to write to
 * @param val Byte value to store in the register
 *
 * @return Status register value returned by the nRF24L01+
 */
static uint8_t zh_nrf24_write_register(uint8_t reg, uint8_t val);

/**
 * @brief Write a multi-byte buffer into an nRF24L01+ register.
 *
 * Convenience wrapper around zh_nrf24_write_buffer that prepends the
 * W_REGISTER command to the register address.
 *
 * @param reg Register address to write to
 * @param buf Pointer to the data buffer to write (must not be NULL)
 * @param count Number of bytes to transfer from the buffer
 *
 * @return Status register value returned by the nRF24L01+
 */
static uint8_t zh_nrf24_write_buff_register(uint8_t reg, uint8_t *buf, uint8_t count);

/**
 * @brief Send a single-byte command to the nRF24L01+ and read the status.
 *
 * @param cmd Command byte to send
 *
 * @return Status register value returned by the nRF24L01+
 */
static uint8_t zh_nrf24_send_command(uint8_t cmd);

/**
 * @brief Measure the battery voltage using the internal AVcc reference.
 *
 * Enables the ADC with the 1.1 V internal reference on the 1.1 V supply
 * channel, performs two conversions and returns the calculated battery
 * voltage based on the AVcc-to-internal-reference ratio.
 *
 * @return Battery voltage in volts, or 0.0 if the ADC reading is zero
 */
static float zh_get_battery_voltage(void);

static volatile uint8_t send_request = 0;

int main(void)
{
	cli();
	zh_pad_interrupt_init();
	zh_spi_init();
	zh_nrf24_init();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();
	for (;;)
	{
		if (send_request == 1)
		{
			cli();
			send_request = 0;
			sei();
			int16_t data[] = {DEVICE_ID, DEVICE_TYPE, (int16_t)(zh_get_battery_voltage() * 100.0), 0, 0, 0, 0};
			zh_nrf24_send((uint8_t *)data, sizeof(data));
		}
		else
		{
			cli();
			if (send_request == 0)
			{
				sleep_enable();
				sei();
				sleep_cpu();
			}
			sei();
			sleep_disable();
		}
	}
	return 0;
}

ISR(INT1_vect)
{
	send_request = 1;
}

static void zh_pad_interrupt_init(void)
{
	DDRD &= ~(1 << PD3);
	PORTD &= ~(1 << PD3);
	EICRA &= ~((1 << ISC11) | (1 << ISC10));
	EICRA |= (1 << ISC11) | (1 << ISC10);
	EIFR |= (1 << INTF1);
	EIMSK |= (1 << INT1);
}

static void zh_spi_init(void)
{
	DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
	PORTB |= (1 << PB2);
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

static uint8_t zh_spi_transfer(uint8_t data)
{
	SPDR = data;
	while ((SPSR & (1 << SPIF)) == 0)
	{
	}
	return SPDR;
}

static void zh_nrf24_init(void)
{
	_delay_ms(100);
	DDRB |= (1 << PB1);
	PORTB &= ~(1 << PB1);
	DDRD &= ~(1 << PD2);
	PORTD |= (1 << PD2);
	uint64_t gateway_address = GATEWAY_ADDRESS;
	zh_nrf24_write_register(NRF_REG_CONFIG, 0x00);
	zh_nrf24_write_register(NRF_REG_EN_AA, 0x01);
	zh_nrf24_write_register(NRF_REG_EN_RXADDR, 0x01);
	zh_nrf24_write_register(NRF_REG_SETUP_AW, 0x01);
	zh_nrf24_write_register(NRF_REG_SETUP_RETR, 0xFF);
	zh_nrf24_write_register(NRF_REG_RF_CH, GATEWAY_CHANNEL);
	zh_nrf24_write_register(NRF_REG_RF_SETUP, 0x26);
	zh_nrf24_write_buff_register(NRF_REG_TX_ADDR, (uint8_t *)&gateway_address, 3);
	zh_nrf24_write_buff_register(NRF_REG_RX_ADDR_P0, (uint8_t *)&gateway_address, 3);
	zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
	zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static void zh_nrf24_send(uint8_t *buf, uint8_t size)
{
	zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP);
	_delay_ms(5);
	zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
	zh_nrf24_write_buffer(NRF_CMD_W_TX_PAYLOAD, buf, size);
	PORTB |= (1 << PB1);
	uint8_t count = 0;
	while ((PIND & (1 << PD2)) != 0)
	{
		if (++count > 100)
		{
			break;
		}
		_delay_ms(1);
	}
	PORTB &= ~(1 << PB1);
	uint8_t status = zh_nrf24_send_command(NRF_CMD_NOP);
	zh_nrf24_write_register(NRF_REG_STATUS, status);
	zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
	zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static uint8_t zh_nrf24_write_buffer(uint8_t cmd, uint8_t *buf, uint8_t count)
{
	PORTB &= ~(1 << PB2);
	uint8_t status = zh_spi_transfer(cmd);
	while (count-- != 0)
	{
		zh_spi_transfer(*(buf++));
	}
	PORTB |= (1 << PB2);
	return status;
}

static uint8_t zh_nrf24_write_register(uint8_t reg, uint8_t val)
{
	PORTB &= ~(1 << PB2);
	uint8_t status = zh_spi_transfer(reg | NRF_CMD_W_REGISTER);
	zh_spi_transfer(val);
	PORTB |= (1 << PB2);
	return status;
}

static uint8_t zh_nrf24_write_buff_register(uint8_t reg, uint8_t *buf, uint8_t count)
{
	return zh_nrf24_write_buffer((reg | NRF_CMD_W_REGISTER), buf, count);
}

static uint8_t zh_nrf24_send_command(uint8_t cmd)
{
	PORTB &= ~(1 << PB2);
	uint8_t status = zh_spi_transfer(cmd);
	PORTB |= (1 << PB2);
	return status;
}

static float zh_get_battery_voltage(void)
{
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
	uint16_t adc = ADCL;
	adc |= (uint16_t)ADCH << 8;
	ADCSRA &= ~(1 << ADEN);
	if (adc == 0)
	{
		return 0.0;
	}
	return (1024.0 * 1.1) / (float)adc;
}