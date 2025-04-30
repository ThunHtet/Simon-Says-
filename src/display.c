#include "stm32f0xx.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>


//calculates the score
int score(int current){
    return (10*current);
}


void spi_cmd(unsigned int data) {
    while(!(SPI1->SR & SPI_SR_TXE)){ //wait till empty


    }
    SPI1->DR= data; //copy to SPI1 DR


}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
   
}
void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

//intalized the display
void spi1_init_oled(void) {
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    nano_wait(1000000);
    spi_cmd(0x38); //function set
    spi_cmd(0x08); //turn off display
    spi_cmd(0x01); //clear display
    nano_wait(2000000);
    spi_cmd(0x06);//entry mode
    spi_cmd(0x02);//cursor home postion
    spi_cmd(0x0c); //display on
   
}


//first row
void spi1_display1(const char *string) {
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    spi_cmd(0x02);
    while(*string != '\0'){ //for each character
        spi_data(*string); //call spi_data
        string ++;
    }
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}


//second row
void spi1_display2(const char *string) {
    spi_cmd(0xc0);//cursor second row
    while(*string != '\0'){ //for each character
        spi_data(*string); //call spi_data
        string ++;
    }
}

//intalized the display
void init_spi1(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~((3<<(15*2))|(3<<(5*2))|(3<<(7*2))); //clear bits 5,7,15
    GPIOA->MODER |= ((2<<(15*2))|(2<<(5*2))|(2<<(7*2))); //alternate function
    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (7 * 4)));
    GPIOA->AFR[0] |= ((0x0 << (5 * 4)) | (0x0 << (7 * 4)));
    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4));
    GPIOA->AFR[1] |=  (0x0 << ((15 - 8) * 4));


    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_SPE; //clear bit
    SPI1 ->CR1 |= SPI_CR1_BR;
    SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_0;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 |= SPI_CR2_SSOE;
    SPI1 -> CR2 |= SPI_CR2_NSSP;
    SPI1 -> CR2 |= SPI_CR2_TXDMAEN;
    SPI1 -> CR1 |= SPI_CR1_SPE;


   
}




//main function to use for testing

// int main(void){
//     internal_clock();
//     init_spi1();
//     spi1_init_oled();
//     spi1_display1("score");
//     int sc = score(10);
//     char scoreString[16];   // adjust buffer size as needed
//     sprintf(scoreString, "%d", sc);
//     spi1_display2(scoreString);
   
   
//     }
