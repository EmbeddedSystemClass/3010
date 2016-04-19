/**
  ******************************************************************************
  * @file    ex12_hamming.c
  * @author  MDS & KB
  * @date    04022015
  * @brief   Hamming encoder example.
  *			 Bytes received from the VCP are Hamming encoded and displayed.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_hamming.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
extern uint16_t hamming_byte_encoder(uint8_t input) {

	uint16_t out;

	/* first encode D0..D3 (first 4 bits),
	 * then D4..D7 (second 4 bits).
	 */
	out = hamming_hbyte_encoder(input & 0xF) |
		(hamming_hbyte_encoder(input >> 4) << 8);

	return(out);

}
uint8_t hamming_hbyte_encoder(uint8_t in) {

	uint8_t d0, d1, d2, d3;
	uint8_t p0 = 0, h0, h1, h2;
	uint8_t z;
	uint8_t out;

	/* extract bits */
	d0 = !!(in & 0x1);
	d1 = !!(in & 0x2);
	d2 = !!(in & 0x4);
	d3 = !!(in & 0x8);

	/* calculate hamming parity bits */
	h0 = d0 ^ d1 ^ d2;
	h1 = d0 ^ d1 ^ d3;
	h2 = d0 ^ d2 ^ d3;

	/* generate out byte without parity bit P0 */
	out = (h0 << 1) | (h1 << 2) | (h2 << 3) |
		(d0 << 4) | (d1 << 5) | (d2 << 6) | (d3 << 7);

	/* calculate even parity bit */
	for (z = 1; z<8; z++)
		p0 = p0 ^ !!(out & (1 << z));

	out |= p0;

	return(out);
}
/*Convert to MSB*/

/*Enter lower and upper in MSB form*/
uint8_t hamming_byte_decoder(uint8_t lower, uint8_t upper) {
	uint8_t d0, d1, d2, d3;
	uint8_t p0 = 0, h0, h1, h2;
	uint8_t b0, b1, b2, b3;
	uint8_t s0, s1, s2, S;
	uint8_t e_low, e_up, E;
	uint8_t decode_byte;
	/*Start by decoding upper*/
	for (int i =0; i < 2; i++) {
		if (i == 0) {
			decode_byte = upper;
		} else if (i == 1) {
			decode_byte = lower;
		} else {

		}
		/*Extract bits from MSB byte*/
		p0 = !!(decode_byte & 0x1);
		h0 = !!(decode_byte & 0x2);
		h1 = !!(decode_byte & 0x4);
		h2 = !!(decode_byte & 0x8);
		d0 = !!(decode_byte & 0x16);
		d1 = !!(decode_byte & 0x32);
		d2 = !!(decode_byte & 0x64);
		d3 = !!(decode_byte & 0x128);
		/*Construct Syndrome*/
		s0 = d3 ^ d1 ^ d0 ^ h0;
		s1 = d3 ^ d2 ^ d0 ^ h1;
		s2 = d3 ^ d2 ^ d1 ^ h2;
		S = (s0 << 0) | (s1 << 1) | (s2 << 2);
		if (S == 0) {
			/*No hamming Error*/
			debug_printf("No error");
		}
	}
}
uint8_t hamming_hbyte_encoder(uint8_t in)
