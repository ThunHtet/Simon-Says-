/**
    file: buzzer_file
    author: Austin Rood
    username = rooda
    date: 4/8/25

    This file include:
        ADC via PA1 to read a potentiometer for volume control
        DAC via PA4 to output square wave(sound) to aux port

*/
#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdio.h>

//global var for wavetable
#define N 1000
#define RATE 20000

//init func headers
void init_adc(void);
void init_dac();
void init_usart5();
void init_wavetable(void);
void init_tim6();
void TIM6_DAC_IRQHandler();

//func headers
void internal_clock();
void error_beep(); //output a beep sound for a couple seconds, then turn it off
uint16_t read_adc(void); //read adc value(pot for vol)
void set_volume(uint16_t adc_value); //set volume based on potentiometer

//universal volume int for whole file
volatile uint32_t volume = 2400;

//wavetable variables
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

int main(void) {
    internal_clock();
    init_adc();
    init_dac();
    init_usart5();
    init_wavetable();
    init_tim6();


    setbuf(stdout, NULL);
    printf("sound on\n");

    error_beep();
    printf("sound off\n");

    while(1) {
    }
}

void init_wavetable(void) {
    for(int i=0; i < N; i++)   
        wavetable[i] = (i < N/2) ? 32767 : -32767; //create wave for buzzer sound
}

void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

void error_beep(void) {
    set_freq(0, 250); //set standard freq, tried different frequencies, like this best
    volatile int x; //included because i kept getting overwritten when checking for set_volume
    for ( volatile int i = 0; i < 500000; i++) { //delay for a set time
        x = i % 1000; //mod i by 1000
        if (x == 0) { //if i mod 1000 is 0(basically, every 1000 ticks of i), then run set_volume(read_adc) to set the volume incrementally throughout the sound being played
            set_volume(read_adc()); //set volume based on potentiometer value
        }
    }
    set_freq(0, 0); //stop sound
}

int __io_putchar(int c) {
    while(!(USART5->ISR & USART_ISR_TXE));
    if(c == '\n') { //check if char is new line
        USART5->TDR = '\r'; //if char is new line, put carraige return first
        while(!(USART5->ISR & USART_ISR_TXE)) {
            //wait for TXE flag, meaning it has been passed
        }
    }
    USART5->TDR = c; //put char to output
    return c;
}

void init_usart5() {
    RCC->AHBENR |= (RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN); // enable port c and d clocks

    GPIOC->MODER &= ~(GPIO_MODER_MODER12); //reset PC12
    GPIOC->MODER |= (0x2 << (2*12)); //set PC12 function to alt func(2)

    GPIOC->AFR[1] |= (0x2 << (16)); //set AFR[1], bit shifted 16 bits, which represents AFR12(which means this is the set bit for AltFunc for PC12) to 0x2(representing AF2, which is USART5_TX)
    
    GPIOD->MODER &= ~(GPIO_MODER_MODER2); //reset PD2
    GPIOD->MODER |= (0x2 << (2*2)); //set PD2 function to alt func(2)

    GPIOD->AFR[0] |= (0x2 << (8)); //set AFR[0], bit shifted 8 bits, which represents AFR2(which means this is the set bit for AltFunc for PD2) to 0x2(representing AF2, which is USART5_RX)

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN; //enable RCC clock to USART5
    USART5->CR1 &= ~(USART_CR1_UE); //disable usart bit
    USART5->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1); //set size to 8bits(M0 and M1 must be set to 00)
    USART5->CR2 &= ~(USART_CR2_STOP); //set stop to be 00(representing 1 stop bit) ///this may be wrong???
    USART5->CR1 &= ~(USART_CR1_PCE); //set parity control enable to 0 for off
    USART5->CR1 &= ~(USART_CR1_OVER8); //set OVER8 to 0, representing 16x oversampling
    USART5->BRR = 0x1A1; //gotten from family reference for oversampling 16, desired baud rate of 115.2KBps

    USART5->CR1 |= USART_CR1_TE; //enable transmitter bit
    USART5->CR1 |= USART_CR1_RE; //enable reciever bit

    USART5->CR1 |= USART_CR1_UE; //enable usart bit
    while(!((USART5->ISR & USART_ISR_TEACK) && (USART5->ISR & USART_ISR_REACK))) {
        //wait until TE adn RE bits are acknowledged(TEACK and REACK and these flags), then the function can continue on to the end
    }
    // return; //end function
}

void init_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable PA1
    GPIOA->MODER |= (3 << (1 * 2)); //PA1 to analog mode
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; //PA1 as adc in

    ADC1->CFGR1 &= ~ADC_CFGR1_CONT; //single conversion mode
    ADC1->CR |= ADC_CR_ADEN; //enable adc
    while(!(ADC1->ISR & ADC_ISR_ADRDY)); //wait until ready, then leave function
}

uint16_t read_adc(void) {
    ADC1->CHSELR = ADC_CHSELR_CHSEL1; //select PA1
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC)); //wait for conversion
    return ADC1->DR;
}
void set_volume(uint16_t adc_value) {
    volume = 100 + ((adc_value * 3000) / 4095);
}

void init_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable GPIOA RCC clock
    GPIOA->MODER |= (0x3 << (2*4)); //set PA4 to DAC_OUT1: (0x3<<(2*4))= 11 00 00 00 00
    RCC->APB1ENR |= RCC_APB1ENR_DACEN; //enable DAC RCC clock

    DAC->CR &= ~DAC_CR_TSEL1; //select TRGO trigger fro TSEL field in CR register(bit 000(~0x7) shifted to the left 3 bits)

    DAC->CR |= DAC_CR_EN1; //enable DAC
}

void TIM6_DAC_IRQHandler() { //TIM7 ISR
    TIM6->SR &= ~TIM_SR_UIF; //clear UIF bit in SR register(acknowledge interrupt)

    offset0 += step0; //increment offset0 by step0
    offset1 += step1; //increment offset0 by step1

    if(offset0 >= (N <<16)) { 
        offset0 -= (N<<16); //decrement offset0 by (N<<16) if offset0 is greater than (N<<16)
    }
    if(offset1 >= (N <<16)) {
        offset1 -= (N<<16); //decrement offset1 by (N<<16) if offset1 is greater than (N<<16)
    }

    int samp = wavetable[offset0 >> 16] + wavetable[offset1 >> 16]; //samp is the sum of the wavetable of offset0 and offset1

    samp = (samp * volume) >> 15; //multiply samp by volume and shift right 17 bits

    samp += 1200; //increment samp by 2048

    DAC->DHR12R1 = (samp & 0xFFF);
    
    return;
}

void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //enable clock for TIM6
    TIM6->PSC = ((48000000 / (RATE*10))) - 1; //set PSC to 48000000/(RATE*10)
    TIM6->ARR = 10 -1; //set ARR to 9

    TIM6->DIER |= TIM_DIER_UIE;  //enable UIE bit in DIER to allow an interrupt to occur each time the counter reaches the ARR value, restarting it back at 0

    NVIC_EnableIRQ(TIM6_DAC_IRQn); //enable interrupt for TIM6

    TIM6->CR1 |= TIM_CR1_CEN; //enable TIM6 by setting CEN bit
    return;    
}