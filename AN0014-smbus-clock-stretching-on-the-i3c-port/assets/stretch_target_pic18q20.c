/*
 * Reference I2C target for AN0014: a PIC18F16Q20 that stretches SCL in hardware, the way a busy
 * SMBus device does. The I2C1 peripheral holds SCL after the address ACK (CSD = 0) until firmware
 * clears CSTR.
 *
 * Protocol at 0x2A:
 *   write [0xF0][lo][hi]  -> stretch (hi<<8|lo) us after every later address match; 0 = never
 *   read                  -> 0xA5 per byte
 *   any other write       -> accepted and discarded
 *
 * Pins: I2C1 on RB6 = SCL, RB5 = SDA unless built with -DPINS_RC for RC0 = SCL, RC1 = SDA.
 * Pull-ups come from the adapter. Debug output: UART1 TX on RC4, 115200 8N1.
 *
 * Built with MPLAB XC8 for a PIC18F16Q20 Curiosity Nano. SPDX-License-Identifier: MIT
 */
#include <xc.h>
#include <stdint.h>

#pragma config FEXTOSC = OFF, RSTOSC = HFINTOSC_64MHZ, WDTE = OFF, MCLRE = EXTMCLR, LVP = ON
#pragma config XINST = OFF, PPS1WAY = OFF

#define _XTAL_FREQ 64000000UL
#define TARGET_ADDR 0x2Au		/* off 0x50-0x57, where 24LCxx EEPROM pull-up boards sit */
#define CFG_MARKER  0xF0u
#define READ_FILL   0xA5u

static volatile uint16_t stretch_us;
static volatile uint16_t addr_matches;

/* Debug console: UART1 TX on RC4 -> the Curiosity Nano's nEDBG virtual COM port, 115200 8N1. */
static void uart_init(void)
{
	TRISCbits.TRISC4 = 0;
	RC4PPS = 0x13;			/* TX1 */
	U1BRG = 138;			/* 64 MHz / (4 * 115200) - 1, BRGS = 1 */
	U1CON0 = 0x00;
	U1CON0bits.BRGS = 1;
	U1CON0bits.TXEN = 1;
	U1CON1bits.ON = 1;
}
static void uart_putc(char c) { while (U1FIFObits.TXBF) { } U1TXB = (uint8_t)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }
static void uart_hex(uint8_t v) { const char *h = "0123456789ABCDEF"; uart_putc(h[v >> 4]); uart_putc(h[v & 15]); }

static void pins_init(void)
{
#ifdef PINS_RC
	TRISCbits.TRISC0 = 0;   TRISCbits.TRISC1 = 0;	/* outputs: the I2C1 PPS output drives them, open-drain */
	ODCONCbits.ODCC0 = 1;   ODCONCbits.ODCC1 = 1;
	I2C1SCLPPS = 0x10;      I2C1SDAPPS = 0x11;	/* RC0, RC1 */
	RC0PPS = 0x1C;          RC1PPS = 0x1D;		/* SCL1, SDA1 */
	RC0FEAT = 0x23;         RC1FEAT = 0x23;		/* SYSBUF = SMBus 3.0 buffer for I2C1; I3CBUF at reset */
#else
	TRISBbits.TRISB6 = 0;   TRISBbits.TRISB5 = 0;	/* outputs: the I2C1 PPS output drives them, open-drain */
	ODCONBbits.ODCB6 = 1;   ODCONBbits.ODCB5 = 1;
	I2C1SCLPPS = 0x0E;      I2C1SDAPPS = 0x0D;	/* RB6, RB5 */
	RB6PPS = 0x1C;          RB5PPS = 0x1D;		/* SCL1, SDA1 */
	RB6FEAT = 0x23;         RB5FEAT = 0x23;		/* SYSBUF = SMBus 3.0 buffer for I2C1; I3CBUF at reset */
#endif
}

static void i2c_target_init(void)
{
	I2C1CON0 = 0x00;		/* MODE = 000: 7-bit target, one address */
	I2C1CON1 = 0x00;
	I2C1CON1bits.CSD = 1;		/* no clock stretching until F0 sets a stretch; ACKDT = ACKCNT = 0 (ACK) */
	I2C1CON2 = 0x00;
	I2C1ADR0 = (uint8_t)(TARGET_ADDR << 1);
	I2C1ADR1 = (uint8_t)(TARGET_ADDR << 1);
	I2C1ADR2 = (uint8_t)(TARGET_ADDR << 1);
	I2C1ADR3 = (uint8_t)(TARGET_ADDR << 1);
	I2C1PIR = 0x00;
	I2C1CNT = 0xFF;
	I2C1CON0bits.EN = 1;
	I2C1TXB = READ_FILL;
}

/* stretch_us == 0: no stretching at all (the control). Otherwise hold SCL after every address
 * match (ADRIE with CSD = 0) for stretch_us, the way a busy SMBus part does. */
static void apply_stretch_mode(void)
{
	I2C1CON0bits.EN = 0;
	I2C1CON1bits.CSD = (stretch_us == 0u) ? 1u : 0u;
	I2C1PIEbits.ADRIE = (stretch_us == 0u) ? 0u : 1u;
	I2C1PIR = 0x00;
	I2C1CON0bits.EN = 1;
	I2C1TXB = READ_FILL;
}

static void stretch(void)
{
	for (uint16_t n = stretch_us / 10u; n != 0u; n--) {
		__delay_us(10);
	}
}

void main(void)
{
	uint8_t idx = 0, cfg = 0, lo = 0;

	uart_init();
	uart_puts("\r\nboot i2c-stretch-target\r\n");
	pins_init();
	i2c_target_init();

	for (;;) {
		if (I2C1STAT0bits.SMA && idx == 0u && I2C1STAT0bits.R) {
			/* nothing: read data is loaded below */
		}
		if (I2C1PIRbits.ADRIF) {
			I2C1PIRbits.ADRIF = 0;
			I2C1CNT = 0xFF;
			idx = 0;
			cfg = 0;
			addr_matches++;
			stretch();
			I2C1CON0bits.CSTR = 0;		/* release SCL */
		}
		if (I2C1STAT1bits.RXBF) {
			uint8_t b = I2C1RXB;
			if (idx == 0u && b == CFG_MARKER) {
				cfg = 1;
			} else if (cfg && idx == 1u) {
				lo = b;
			} else if (cfg && idx == 2u) {
				stretch_us = (uint16_t)lo | ((uint16_t)b << 8);
			}
			idx++;
			I2C1CON0bits.CSTR = 0;
		}
		if (I2C1STAT0bits.R && I2C1STAT1bits.TXBE && I2C1STAT0bits.SMA) {
			I2C1TXB = READ_FILL;
		}
		if (I2C1PIRbits.PCIF) {
			I2C1PIRbits.PCIF = 0;
			uart_putc('P'); uart_hex(I2C1ERR); uart_hex(I2C1STAT0); uart_hex(I2C1STAT1);
			uart_hex(I2C1CON0); uart_hex(I2C1CON1); uart_hex(I2C1PIR); uart_hex(I2C1CNT); uart_putc('\n');
			I2C1ERR = 0x00;			/* clear NACK/collision flags left by the transaction */
			I2C1CON1bits.TXU = 0;
			I2C1CON1bits.RXO = 0;
			I2C1STAT1bits.CLRBF = 1;
			I2C1STAT1bits.TXWE = 0;
			I2C1STAT1bits.RXRE = 0;
			I2C1TXB = READ_FILL;		/* preload: without a stretch the first read byte leaves right after the address ACK */
			I2C1CNT = 0xFF;
			idx = 0;
			if (cfg) {
				cfg = 0;
				apply_stretch_mode();
			}
		}
	}
}
