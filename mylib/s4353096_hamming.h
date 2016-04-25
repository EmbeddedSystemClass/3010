/**
  ******************************************************************************
  * @file    s4353096_hamming.h
  * @author  Steffen Mitchell
  * @date    20042016
  * @brief   Hamming encoder and decoder
  *			 Bytes received are can use the below functions to be hamming encoded
	*			 or decoded.
  ******************************************************************************
	* EXTERNAL FUNCTIONS
  ******************************************************************************
  * extern uint16_t hamming_byte_encoder(uint8_t input) - Hamming encodes input
	* byte
  * extern uint8_t hamming_byte_decoder(uint8_t lower, uint8_t upper) - Hamming
	*	decodes input bytes
  ******************************************************************************
 */

extern uint16_t hamming_byte_encoder(uint8_t input);
uint8_t hamming_hbyte_encoder(uint8_t in);
extern uint8_t hamming_byte_decoder(uint8_t lower, uint8_t upper);
