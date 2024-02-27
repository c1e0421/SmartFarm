/*
 * LPUART.h
 *
 *  Created on: Mar 17, 2016
 *      Author: B46911
 */

#ifndef LPUART_H_
#define LPUART_H_

void LPUART2_init(void);
void LPUART2_transmit_char(char send);
void LPUART2_transmit_string(char data_string[]);
char LPUART2_receive_char(void);
void LPUART2_recieve_and_echo_char(void);
#endif /* LPUART_H_ */
