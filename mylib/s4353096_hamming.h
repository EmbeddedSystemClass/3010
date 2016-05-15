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
/*struct Hamming {
  uint16_t hamming_encode; //10bits for
  uint16_t hamming_decode;
}*/
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FreeRTOS_CLI.h"
#define POLY 0x1021 // CRC-CCITT
extern uint16_t hamming_byte_encoder(uint8_t input);
uint8_t hamming_hbyte_encoder(uint8_t in);
extern uint8_t hamming_byte_decoder(uint8_t lower, uint8_t upper);
extern uint16_t crc_update(uint16_t crc, uint8_t c);
extern uint16_t crc_calculation(unsigned char *rxpacket);
