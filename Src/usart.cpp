#include "usart.h"

#include "stm32f303xc.h"
#include <array>
#include <cstring>

static std::array<uint8_t, UART_BUFF_SIZE> UART_BUFF;

class Writer {
private:
	uint16_t _size;
public:
	Writer() : _size(0) {

	}

	template<class T>
	Writer& write(T&& v) {
		constexpr size_t s = sizeof(T);
		if(_size + s > UART_BUFF_SIZE) return *this;
		std::memcpy(UART_BUFF.data() + _size, (void*)&v, s);
		_size += s;
		return *this;
	}

	template<class T>
	Writer& write_ptr(const T* v, size_t n) {
		constexpr size_t s = sizeof(T);
		if(_size + s*n > UART_BUFF_SIZE) return *this;
		std::memcpy(UART_BUFF.data() + _size, (void*)v, s*n);
		_size += s*n;
		return *this;
	}

	uint16_t size() const {
		return _size;
	}
};

void USART_init() {
	// DMA Initialization
	// Set destination to be USART transfer register
	DMA1_Channel4->CPAR = reinterpret_cast<uint32_t>(&USART1->TDR);
	// Set DMA source to be UART_BUFF
	DMA1_Channel4->CMAR = reinterpret_cast<uint32_t>(UART_BUFF.data());
	// Set direction to go from memory to peripheral and enable automatic memory increment
	DMA1_Channel4->CCR |=  DMA_CCR_DIR|DMA_CCR_MINC;

	// GPIO Initialization
	// Set GPIO to be alternate pins
	GPIOC->MODER |= (2 << GPIO_MODER_MODER4_Pos) | (2 << GPIO_MODER_MODER5_Pos);
	// Set GPIO speed to high speed
	GPIOC->OSPEEDR |= (3 << GPIO_OSPEEDER_OSPEEDR4_Pos) | (3 << GPIO_OSPEEDER_OSPEEDR5_Pos);
	// Set alternate function to USART1
	GPIOC->AFR[0] |= (7 << GPIO_AFRL_AFRL4_Pos) | (7 << GPIO_AFRL_AFRL4_Pos);

	// Clear USART control register
	USART1->CR1 = 0;
	// Enable DMA
	USART1->CR3 |= USART_CR3_DMAT;
	// Set Baud rate to 38400
	USART1->BRR = 117 << 4 | 3; // BRR is 117.1875 for 38400 therefore mantissa 117 and frac 3
	// Enable USART1
	USART1->CR1 |= USART_CR1_UE;
	// Enable transfers
	USART1->CR1 |= USART_CR1_TE;
}

void USART_send_buff(std::span<const uint8_t> buff) {
	// Wait for current transfer to complete
	while(DMA1_Channel4->CNDTR != 0);
	USART_send_buff_unsafe(buff);
}

void USART_send_buff_cancelable(std::span<const uint8_t> buff) {
	// Cancel if current transfer is not complete
	if(DMA1_Channel4->CNDTR != 0) return;
	USART_send_buff_unsafe(buff);
}

void USART_send_buff_unsafe(std::span<const uint8_t> buff) {
	// Disable DMA channel
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	// Check if buff is bigger than DMA buffer
	uint32_t n = buff.size();
	if(n > UART_BUFF_SIZE) {
		n = UART_BUFF_SIZE;
	}
	// Copy buff to DMA buffer
	std::memcpy(UART_BUFF.data(), buff.data(), n);

	// Set number of bytes to be transfered
	DMA1_Channel4->CNDTR = n;
	// Re-enable DMA channel
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	// Reset USART transfer complete flag to start DMA transfer
	USART1->ICR |= USART_ICR_TCCF;
}

Writer get_writer() {
	// Wait for current transfer to complete
	while(DMA1_Channel4->CNDTR != 0);
	// Disable DMA channel
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	return Writer();
}

void submit_writer(Writer& writer) {
	// Set number of bytes to be transfered
	DMA1_Channel4->CNDTR = writer.size();
	// Re-enable DMA channel
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	// Reset USART transfer complete flag to start DMA transfer
	USART1->ICR |= USART_ICR_TCCF;
}

void USART_console_data(std::span<const uint8_t> buff) {
	Writer writer = get_writer();
	writer.write<uint8_t>(0) // Packet type console data
			.write<uint16_t>(buff.size()) // Write message size
		.write_ptr<uint8_t>(buff.data(), buff.size()); // Gyro Data
	submit_writer(writer);

}

void USART_gyro_data(float data[3]) {
	Writer writer = get_writer();
	writer.write<uint8_t>(1) // Packet type gyro data
		.write_ptr<float>(data, 3); // Gyro Data
	submit_writer(writer);
}




