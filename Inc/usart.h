#pragma once
#include <span>
#include <stdint.h>

// USART asynchronous communication
// Uses USART1, DMA1 Channel 4, GPIO C4, and GPIO C5

/* USART protocol
 * First byte of message is the packet type
 * Data is then sent afterwards (varies by packet type)
 * List of packet types:
 * 0: Console string - Prints a message to console
 * struct {
 *	uint16_t size;
 *	char data[size];
 * }
 * 1: Gyro data
 * struct {
 * 	float x, y, z;
 * }
 */

// Maximum size of the UART DMA buffer in bytes
constexpr uint32_t UART_BUFF_SIZE = 128;

// Creates span from string literal
template<size_t N>
constexpr std::span<const uint8_t> s2s(const char (&str)[N]) {
	return std::span{reinterpret_cast<const uint8_t *>(str), N};
}

// Initializes the USART bus
// REQUIREMENTS: DMA1, GPIOC, and USART1 clock enabled
void USART_init();

// Sends buffer via DMA
// Waits for current transfer to complete before sending
// If the size of buff > UART_BUFF_SIZE, UART_BUFF_SIZE bytes will be transfered
void USART_send_buff(std::span<const uint8_t> buff);

// Attempts to send buffer via DMA
// If there is an incomplete transfer, cancel the request
// If the size of buff > UART_BUFF_SIZE, UART_BUFF_SIZE bytes will be transfered
void USART_send_buff_cancelable(std::span<const uint8_t> buff);

// Sends buffer without checking if a transfer is occurring
// If the size of buff > UART_BUFF_SIZE, UART_BUFF_SIZE bytes will be transfered
void USART_send_buff_unsafe(std::span<const uint8_t> buff);

void USART_console_data(std::span<const uint8_t> buff);

void USART_gyro_data(float data[3]);

