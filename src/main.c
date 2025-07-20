#include "avr/io.h"
#include "avr/sleep.h"
#include "avr/fuse.h"
#include "avr/interrupt.h"
#include "util/delay.h"

FUSES = {0xE2, 0xDF, 0x07};

#define GATEWAY_CHANNEL 120
#define GATEWAY_ADDRESS 0xDDEEFF
#define DEVICE_ID 6
#define DEVICE_TYPE 4

static void zh_pad_interrupt_init(void);
static void zh_spi_init(void);
static uint8_t zh_spi_transfer(uint8_t data);
static void zh_nrf24_init(void);
static void zh_nrf24_send(uint8_t *buf, uint8_t size);
static uint8_t zh_nrf24_write_buffer(uint8_t cmd, uint8_t *buf, uint8_t count);
static uint8_t zh_nrf24_write_register(uint8_t reg, uint8_t val);
static uint8_t zh_nrf24_write_buff_register(uint8_t reg, uint8_t *buf, uint8_t count);
static uint8_t zh_nrf24_send_command(uint8_t cmd);
static float zh_get_battery_level_charge(void);

int main(void)
{
	cli();
	zh_pad_interrupt_init();
	zh_spi_init();
	zh_nrf24_init();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sei();
	for (;;)
	{
		sleep_cpu();
	}
	return 0;
}

ISR(INT1_vect)
{
	cli();
	int16_t data[] = {DEVICE_ID, DEVICE_TYPE, (zh_get_battery_level_charge() * 100), 0, 0, 0, 0};
	zh_nrf24_send((uint8_t *)&data, sizeof(data));
	sei();
}

static void zh_pad_interrupt_init(void)
{
	EICRA |= ((1 << ISC11) | (1 << ISC10));
	EIMSK |= (1 << INT1);
}

static void zh_spi_init(void)
{
	DDRB |= ((1 << PORTB2) | (1 << PORTB3) | (1 << PORTB5));
	PORTB |= (1 << PORTB2);
	SPCR |= ((1 << SPE) | (1 << MSTR));
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
	uint64_t gateway_address = GATEWAY_ADDRESS;
	DDRB |= (1 << PORTB1);
	PORTB &= ~(1 << PORTB1);
	zh_nrf24_write_register(0x00, 0x48);
	zh_nrf24_write_register(0x01, 0x01);
	zh_nrf24_write_register(0x02, 0x01);
	zh_nrf24_write_register(0x03, 0x01);
	zh_nrf24_write_register(0x04, 0xFF);
	zh_nrf24_write_register(0x05, GATEWAY_CHANNEL);
	zh_nrf24_write_register(0x06, 0x26);
	zh_nrf24_write_buff_register(0x0A, (uint8_t *)&gateway_address, 3);
	zh_nrf24_write_buff_register(0x10, (uint8_t *)&gateway_address, 3);
	PORTB &= ~(1 << PORTB2);
}

static void zh_nrf24_send(uint8_t *buf, uint8_t size)
{
	zh_nrf24_write_register(0x00, 0x4A);
	_delay_ms(5);
	zh_nrf24_send_command(0xE1);
	zh_nrf24_write_buffer(0xA0, buf, size);
	PORTB |= (1 << PORTB1);
	uint8_t count = 0;
	while ((PIND & (1 << PIND2)) != 0)
	{
		if (++count > 100)
		{
			break;
		}
		_delay_ms(1);
	}
	zh_nrf24_write_register(0x07, zh_nrf24_send_command(0xFF));
	zh_nrf24_send_command(0xE1);
	zh_nrf24_write_register(0x00, 0x48);
	PORTB &= ~((1 << PORTB1) | (1 << PORTB2));
}

static uint8_t zh_nrf24_write_buffer(uint8_t cmd, uint8_t *buf, uint8_t count)
{
	PORTB &= ~(1 << PORTB2);
	uint8_t status = zh_spi_transfer(cmd);
	while (count-- != 0)
	{
		zh_spi_transfer(*(buf++));
	}
	PORTB |= (1 << PORTB2);
	return status;
}

static uint8_t zh_nrf24_write_register(uint8_t reg, uint8_t val)
{
	PORTB &= ~(1 << PORTB2);
	uint8_t status = zh_spi_transfer(reg | 0x20);
	zh_spi_transfer(val);
	PORTB |= (1 << PORTB2);
	return status;
}

static uint8_t zh_nrf24_write_buff_register(uint8_t reg, uint8_t *buf, uint8_t count)
{
	return zh_nrf24_write_buffer((reg | 0x20), buf, count);
}

static uint8_t zh_nrf24_send_command(uint8_t cmd)
{
	PORTB &= ~(1 << PORTB2);
	uint8_t status = zh_spi_transfer(cmd);
	PORTB |= (1 << PORTB2);
	return status;
}

static float zh_get_battery_level_charge(void)
{
	ADMUX = ((1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1));
	ADCSRA |= (1 << ADEN);
	_delay_ms(10);
	ADCSRA |= (1 << ADSC);
	while ((ADCSRA & (1 << ADSC)) != 0)
	{
	}
	ADCSRA &= ~(1 << ADEN);
	return ((1024 * 1.1) / (ADCL + (ADCH * 256)));
}