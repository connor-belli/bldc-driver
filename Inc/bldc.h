#pragma once

#include "stm32f303xc.h"

constexpr uint16_t arr = 8192;


template<bool second,
	bool negative,
	uint32_t tim_addr,
	volatile uint32_t TIM_TypeDef::*lCCR,
	volatile uint32_t TIM_TypeDef::*hCCR,
	uint32_t lp,
	uint32_t hp>
void secondary_enable(uint16_t p, uint16_t f) {
	TIM_TypeDef* tim = reinterpret_cast<TIM_TypeDef*>(tim_addr);

	uint16_t freq;
	if(second) {
		if(negative) {
			tim->CCER |= lp;
			tim->CCER &= ~hp;
		} else {
			tim->CCER &= ~lp;
			tim->CCER |= hp;
		}
		if(negative) {
			freq = (f + p) >> 1;
		} else {
			freq = arr - ((f + p) >> 1);
		}
	} else {
		tim->CCER &= ~lp;
		tim->CCER &= ~hp;

		if(negative) {
			freq = arr - ((f + p) >> 1);
		} else {
			freq = (f + p) >> 1;
		}
	}
	if(negative) {
		tim->*lCCR = freq;
		tim->*hCCR = 0;
	} else {
		tim->*lCCR = 0xFFFF;
		tim->*hCCR = freq;
	}
}

template<bool negative,
	uint32_t tim_addr,
	volatile uint32_t TIM_TypeDef::*lCCR,
	volatile uint32_t TIM_TypeDef::*hCCR,
	uint32_t lp,
	uint32_t hp>
void primary_enable() {
	TIM_TypeDef* tim = reinterpret_cast<TIM_TypeDef*>(tim_addr);

	tim->CCER &= ~lp;
	tim->CCER &= ~hp;
	if(negative) {
		tim->*lCCR = 0;
		tim->*hCCR = 0;
	} else {
		tim->*lCCR = 0xFFFF;
		tim->*hCCR = 0xFFFF;
	}
}

template<bool second, bool negative>
void a_secondary(uint16_t p, uint16_t f) {
	secondary_enable<second,
		negative,
		TIM3_BASE,
		&TIM_TypeDef::CCR3,
		&TIM_TypeDef::CCR4,
		TIM_CCER_CC3P,
		TIM_CCER_CC4P>(p, f);
}

template<bool second, bool negative>
void b_secondary(uint16_t p, uint16_t f) {
	secondary_enable<second,
		negative,
		TIM3_BASE,
		&TIM_TypeDef::CCR1,
		&TIM_TypeDef::CCR2,
		TIM_CCER_CC1P,
		TIM_CCER_CC2P>(p, f);
}

template<bool second, bool negative>
void c_secondary(uint16_t p, uint16_t f) {
	secondary_enable<second,
		negative,
		TIM2_BASE,
		&TIM_TypeDef::CCR3,
		&TIM_TypeDef::CCR4,
		TIM_CCER_CC3P,
		TIM_CCER_CC4P>(p, f);
}

template<bool negative>
void a_primary() {
	primary_enable<negative,
		TIM3_BASE,
		&TIM_TypeDef::CCR3,
		&TIM_TypeDef::CCR4,
		TIM_CCER_CC3P,
		TIM_CCER_CC4P>();
}

template<bool negative>
void b_primary() {
	primary_enable<negative,
		TIM3_BASE,
		&TIM_TypeDef::CCR1,
		&TIM_TypeDef::CCR2,
		TIM_CCER_CC1P,
		TIM_CCER_CC2P>();
}

template<bool negative>
void c_primary() {
	primary_enable<negative,
		TIM2_BASE,
		&TIM_TypeDef::CCR3,
		&TIM_TypeDef::CCR4,
		TIM_CCER_CC3P,
		TIM_CCER_CC4P>();
}

