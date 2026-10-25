#include "avr/io.h"
#include "avr/sleep.h"
#include "avr/fuse.h"
#include "avr/interrupt.h"
#include "avr/eeprom.h"
#include "avr/wdt.h"
#include "util/delay.h"
#include "math.h"
#include "stdbool.h"
#include "zh_nrf24_config.h"

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
#ifndef PRR
#define PRR PRR0
#endif
#ifndef PRTWI
#define PRTWI PRTWI0
#endif
#ifndef PRSPI
#define PRSPI PRSPI0
#endif
#endif

#define GATEWAY_CHANNEL 120
#define GATEWAY_ADDRESS 0xDDEEFF

#define EEPROM_DEVICE_ID_ADDR ((uint8_t *)0x00)
#define DEVICE_ID_UNASSIGNED 0xFF

#define PAIRING_TIMEOUT_ATTEMPTS 7

#define WDT_T_250MS 0x04
#define WDT_T_4S 0x08

#define NRF_CMD_W_REGISTER 0x20
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_CMD_R_RX_PAYLOAD 0x61
#define NRF_CMD_FLUSH_TX 0xE1
#define NRF_CMD_FLUSH_RX 0xE2
#define NRF_CMD_NOP 0xFF
#define NRF_CMD_ACTIVATE 0x50
#define NRF_CMD_R_RX_PL_WID 0x60

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
#define NRF_REG_RX_PW_P0 0x11
#define NRF_REG_DYNPD 0x1C
#define NRF_REG_FEATURE 0x1D

#define NRF_CONFIG_MASK_RX_DR 0x40
#define NRF_CONFIG_MASK_TX_DS 0x20
#define NRF_CONFIG_MASK_MAX_RT 0x10
#define NRF_CONFIG_EN_CRC 0x08
#define NRF_CONFIG_PWR_UP 0x02
#define NRF_CONFIG_PRIM_RX 0x01

#define NRF_STATUS_RX_DR 0x40
#define NRF_STATUS_TX_DS 0x20
#define NRF_STATUS_MAX_RT 0x10

#define NRF_FEATURE_EN_DPL 0x04
#define NRF_DYNPD_DPL_P0 0x01

/**
 * @brief Load the device identifier from EEPROM.
 *
 * @return Stored device id, or DEVICE_ID_UNASSIGNED if none is stored
 */
static uint8_t zh_load_device_id(void);

/**
 * @brief Save the device identifier to EEPROM.
 *
 * @param id Device identifier to store
 */
static void zh_save_device_id(uint8_t id);

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
 * @brief Initialize INT0 external interrupt for the nRF24 IRQ signal.
 *
 * Configures PD2 as a falling-edge triggered external interrupt source and
 * enables the INT0 interrupt mask bit. The ISR sets the nrf_event flag used
 * by the pairing state machine.
 */
static void zh_nrf24_interrupt_init(void);

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
 * @brief Start a transmission without blocking on completion.
 *
 * Loads the payload, pulses CE for 15 us and returns immediately. The
 * transmission completion is signalled by the nRF24 IRQ pin.
 *
 * @param buf Pointer to the payload data to transmit (must not be NULL)
 * @param size Number of bytes in the payload buffer (must be > 0)
 */
static void zh_nrf24_send_no_wait(uint8_t *buf, uint8_t size);

/**
 * @brief Send a single-byte command to the nRF24L01+ and read the status.
 *
 * @param cmd Command byte to send
 *
 * @return Status register value returned by the nRF24L01+
 */
static uint8_t zh_nrf24_send_command(uint8_t cmd);

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
 * @brief Read the received payload from the nRF24L01+ into a buffer.
 *
 * @param buf Pointer to the destination buffer (must not be NULL)
 * @param count Number of bytes to read from the RX FIFO
 */
static void zh_nrf24_read_payload(uint8_t *buf, uint8_t count);

/**
 * @brief Read the width of the payload in the nRF24L01+ RX FIFO.
 *
 * @return Payload width in bytes
 */
static uint8_t zh_nrf24_read_payload_width(void);

/**
 * @brief Switch the nRF24L01+ to RX mode on pipe 0.
 */
static void zh_nrf24_enter_rx(void);

/**
 * @brief Put the nRF24L01+ into power-down mode.
 */
static void zh_nrf24_enter_power_down(void);

/**
 * @brief Start the watchdog timer in interrupt mode with the given timeout.
 *
 * @param timeout_bits Watchdog timeout prescaler bits (WDP3:0)
 */
static void zh_wdt_start(uint8_t timeout_bits);

/**
 * @brief Stop the watchdog timer.
 */
static void zh_wdt_stop(void);

/**
 * @brief Generate a pseudo-random token from MCU state and ADC noise.
 *
 * @return 16-bit random token (never zero)
 */
static uint16_t zh_get_random_token(void);

/**
 * @brief Run the non-blocking pairing procedure to obtain a device id.
 *
 * Sends pairing requests, waits for the gateway acknowledgment in RX mode
 * and retries with watchdog-timed backoff. Sleeps forever if the gateway
 * does not answer within the allowed number of attempts.
 *
 * @return Device identifier assigned by the gateway
 */
static uint8_t zh_do_pairing(void);

/**
 * @brief Enter permanent power-down sleep until the next reset.
 *
 * Disables all interrupts, the watchdog, the radio, the ADC, SPI and all
 * unused peripheral clocks, then loops in power-down sleep mode.
 */
static void zh_enter_final_sleep(void);

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

static volatile uint8_t send_request = false;
static volatile uint8_t nrf_event = false;

zh_nrf24_sensor_transmitted_data_t transmitted_data = {0};

int main(void)
{
	transmitted_data.type = ZH_NRF24_SENSOR_TOUCH_SWITCH;
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	cli();
	zh_pad_interrupt_init();
	zh_spi_init();
	zh_nrf24_init();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sei();
	uint8_t device_id = zh_load_device_id();
	if (device_id == DEVICE_ID_UNASSIGNED)
	{
		device_id = zh_do_pairing();
	}
	transmitted_data.id = (int16_t)device_id;
	{
		uint8_t probe[ZH_NRF24_MAX_PAYLOAD_SIZE];
		if (zh_nrf24_sensor_build_payload(&transmitted_data, probe, sizeof(probe)) == 0)
		{
			zh_enter_final_sleep();
		}
	}
	for (;;)
	{
		if (send_request == true)
		{
			cli();
			send_request = false;
			sei();
			transmitted_data.battery = (int16_t)lroundf(zh_get_battery_voltage() * 100.0);
			uint8_t payload[ZH_NRF24_MAX_PAYLOAD_SIZE];
			uint8_t size = zh_nrf24_sensor_build_payload(&transmitted_data, payload, sizeof(payload));
			if (size > 0)
			{
				zh_nrf24_send(payload, size);
			}
		}
		else
		{
			cli();
			if (send_request == false)
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
	send_request = true;
}

ISR(INT0_vect)
{
	nrf_event = true;
}

ISR(WDT_vect)
{
}

static uint8_t zh_load_device_id(void)
{
	uint8_t id = eeprom_read_byte(EEPROM_DEVICE_ID_ADDR);
	if (id == 0xFF || id == 0x00)
	{
		return DEVICE_ID_UNASSIGNED;
	}
	return id;
}

static void zh_save_device_id(uint8_t id)
{
	eeprom_update_byte(EEPROM_DEVICE_ID_ADDR, id);
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
	uint8_t activation_key = 0x73;
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
	zh_nrf24_write_buffer(NRF_CMD_ACTIVATE, &activation_key, sizeof(activation_key));
	zh_nrf24_write_register(NRF_REG_FEATURE, NRF_FEATURE_EN_DPL);
	zh_nrf24_write_register(NRF_REG_DYNPD, NRF_DYNPD_DPL_P0);
	zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC);
}

static void zh_nrf24_interrupt_init(void)
{
	DDRD &= ~(1 << PD2);
	PORTD |= (1 << PD2);
	EICRA &= ~((1 << ISC01) | (1 << ISC00));
	EICRA |= (1 << ISC01);
	EIFR |= (1 << INTF0);
	EIMSK |= (1 << INT0);
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

static void zh_nrf24_send_no_wait(uint8_t *buf, uint8_t size)
{
	PORTB &= ~(1 << PB1);
	zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_MASK_RX_DR | NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP);
	_delay_ms(2);
	zh_nrf24_send_command(NRF_CMD_FLUSH_TX);
	zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT | NRF_STATUS_RX_DR);
	zh_nrf24_write_buffer(NRF_CMD_W_TX_PAYLOAD, buf, size);
	PORTB |= (1 << PB1);
	_delay_us(15);
	PORTB &= ~(1 << PB1);
}

static uint8_t zh_nrf24_send_command(uint8_t cmd)
{
	PORTB &= ~(1 << PB2);
	uint8_t status = zh_spi_transfer(cmd);
	PORTB |= (1 << PB2);
	return status;
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

static void zh_nrf24_read_payload(uint8_t *buf, uint8_t count)
{
	PORTB &= ~(1 << PB2);
	zh_spi_transfer(NRF_CMD_R_RX_PAYLOAD);
	while (count-- != 0)
	{
		*buf++ = zh_spi_transfer(0xFF);
	}
	PORTB |= (1 << PB2);
}

static uint8_t zh_nrf24_read_payload_width(void)
{
	PORTB &= ~(1 << PB2);
	zh_spi_transfer(NRF_CMD_R_RX_PL_WID);
	uint8_t width = zh_spi_transfer(0xFF);
	PORTB |= (1 << PB2);
	return width;
}

static void zh_nrf24_enter_rx(void)
{
	PORTB &= ~(1 << PB1);
	zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT | NRF_STATUS_RX_DR);
	zh_nrf24_send_command(NRF_CMD_FLUSH_RX);
	zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX);
	_delay_ms(2);
	PORTB |= (1 << PB1);
	_delay_us(130);
}

static void zh_nrf24_enter_power_down(void)
{
	PORTB &= ~(1 << PB1);
	zh_nrf24_write_register(NRF_REG_CONFIG, NRF_CONFIG_EN_CRC);
}

static void zh_wdt_start(uint8_t timeout_bits)
{
	uint8_t wdp = (timeout_bits & 0x07) | ((timeout_bits & 0x08) ? (1 << WDP3) : 0);
	cli();
	wdt_reset();
	WDTCSR = (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDIE) | wdp;
	sei();
}

static void zh_wdt_stop(void)
{
	cli();
	wdt_reset();
	WDTCSR = (1 << WDCE) | (1 << WDE);
	WDTCSR = 0;
	sei();
}

static uint16_t zh_get_random_token(void)
{
	uint16_t token = MCUSR;
	MCUSR = 0;
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	_delay_ms(5);
	for (uint8_t i = 0; i < 16; ++i)
	{
		ADCSRA |= (1 << ADSC);
		while ((ADCSRA & (1 << ADSC)) != 0)
		{
		}
		token = (token << 1) ^ (token >> 15) ^ ADC;
	}
	ADCSRA &= ~(1 << ADEN);
	if (token == 0)
	{
		token = 0xACE1;
	}
	return token;
}

static uint8_t zh_do_pairing(void)
{
	uint16_t token = zh_get_random_token();
	zh_nrf24_sensor_pairing_data_t pairing_request;
	pairing_request.type = (uint8_t)transmitted_data.type;
	pairing_request.magic = ZH_NRF24_PAIRING_REQ_MAGIC;
	pairing_request.token_hi = (uint8_t)(token >> 8);
	pairing_request.token_lo = (uint8_t)(token & 0xFF);
	pairing_request.id = 0;
	zh_nrf24_interrupt_init();
	zh_nrf24_sensor_pairing_state_t state = ZH_NRF24_PAIR_SEND;
	uint8_t attempts = 0;
	for (;;)
	{
		switch (state)
		{
		case ZH_NRF24_PAIR_SEND:
			nrf_event = false;
			zh_nrf24_send_no_wait((uint8_t *)&pairing_request, ZH_NRF24_PAIRING_PACKET_SIZE);
			state = ZH_NRF24_PAIR_TX_WAIT;
			break;
		case ZH_NRF24_PAIR_TX_WAIT:
			if ((PIND & (1 << PD2)) == 0)
			{
				nrf_event = true;
			}
			else
			{
				nrf_event = false;
				zh_wdt_start(WDT_T_250MS);
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
				zh_wdt_stop();
				sei();
			}
			if (nrf_event == true)
			{
				nrf_event = false;
				uint8_t status = zh_nrf24_send_command(NRF_CMD_NOP);
				zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT | NRF_STATUS_RX_DR);
				if ((status & NRF_STATUS_TX_DS) != 0)
				{
					zh_nrf24_enter_rx();
					state = ZH_NRF24_PAIR_RX_WAIT;
				}
				else
				{
					state = ZH_NRF24_PAIR_BACKOFF;
				}
			}
			else
			{
				state = ZH_NRF24_PAIR_BACKOFF;
			}
			break;
		case ZH_NRF24_PAIR_RX_WAIT:
			if ((PIND & (1 << PD2)) == 0)
			{
				nrf_event = true;
			}
			else
			{
				nrf_event = false;
				zh_wdt_start(WDT_T_250MS);
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
				zh_wdt_stop();
				sei();
			}
			if (nrf_event == true)
			{
				nrf_event = false;
				uint8_t status = zh_nrf24_send_command(NRF_CMD_NOP);
				if ((status & NRF_STATUS_RX_DR) != 0)
				{
					uint8_t width = zh_nrf24_read_payload_width();
					if (width > ZH_NRF24_MAX_PAYLOAD_SIZE)
					{
						zh_nrf24_send_command(NRF_CMD_FLUSH_RX);
						zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR);
						state = ZH_NRF24_PAIR_BACKOFF;
						break;
					}
					if (width != ZH_NRF24_PAIRING_PACKET_SIZE)
					{
						uint8_t dummy[ZH_NRF24_MAX_PAYLOAD_SIZE];
						zh_nrf24_read_payload(dummy, width);
						zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR);
						zh_nrf24_send_command(NRF_CMD_FLUSH_RX);
						state = ZH_NRF24_PAIR_BACKOFF;
						break;
					}
					zh_nrf24_sensor_pairing_data_t pairing_response;
					zh_nrf24_read_payload((uint8_t *)&pairing_response, width);
					zh_nrf24_write_register(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
					zh_nrf24_send_command(NRF_CMD_FLUSH_RX);
					if (pairing_response.magic == ZH_NRF24_PAIRING_ACK_MAGIC && pairing_response.type == pairing_request.type &&
						pairing_response.token_hi == pairing_request.token_hi && pairing_response.token_lo == pairing_request.token_lo &&
						pairing_response.id != 0 && pairing_response.id != DEVICE_ID_UNASSIGNED)
					{
						zh_nrf24_enter_power_down();
						EIMSK &= ~(1 << INT0);
						zh_save_device_id(pairing_response.id);
						return pairing_response.id;
					}
				}
				state = ZH_NRF24_PAIR_BACKOFF;
			}
			else
			{
				state = ZH_NRF24_PAIR_BACKOFF;
			}
			break;
		case ZH_NRF24_PAIR_BACKOFF:
			zh_nrf24_enter_power_down();
			nrf_event = false;
			zh_wdt_start(WDT_T_4S);
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
			zh_wdt_stop();
			sei();
			if (++attempts >= PAIRING_TIMEOUT_ATTEMPTS)
			{
				zh_enter_final_sleep();
			}
			state = ZH_NRF24_PAIR_SEND;
			break;
		}
	}
}

static void zh_enter_final_sleep(void)
{
	cli();
	EIMSK = 0;
	EIFR = 0xFF;
	zh_wdt_stop();
	PORTB &= ~(1 << PB1);
	zh_nrf24_write_register(NRF_REG_CONFIG, 0x00);
	ADCSRA = 0;
	SPCR = 0;
	PRR = (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRSPI) | (1 << PRUSART0);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	for (;;)
	{
		sleep_cpu();
	}
}

static float zh_get_battery_voltage(void)
{
	ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	_delay_ms(10);
	ADCSRA |= (1 << ADSC);
	while ((ADCSRA & (1 << ADSC)) != 0)
	{
	}
	ADCSRA |= (1 << ADSC);
	while ((ADCSRA & (1 << ADSC)) != 0)
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