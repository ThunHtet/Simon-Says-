#include "stm32f0xx.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>


// define led pins
#define LED_RED_PIN GPIO_PIN_0    // PC0
#define LED_GREEN_PIN GPIO_PIN_3  // PC3
#define LED_BLUE_PIN GPIO_PIN_6   // PC6
#define LED_ORANGE_PIN GPIO_PIN_9 // PC9

#define LED_PORT GPIOC

// define button pins on PB0-PB3
#define BUTTON_GREEN_PIN GPIO_PIN_0
#define BUTTON_ORANGE_PIN GPIO_PIN_1
#define BUTTON_RED_PIN GPIO_PIN_2
#define BUTTON_BLUE_PIN GPIO_PIN_3
#define BUTTON_PORT GPIOB

// game constants
#define MAX_LEVEL 10
#define LED_ON_TIME 500   // led on duration in milliseconds
#define BETWEEN_TIME 200  // delay between sequence steps
#define DEBOUNCE_DELAY 50 // debounce time for button input



// init func headers
void init_adc(void);
void init_dac();
void init_usart5();
void init_wavetable(void);
void init_tim6();
void TIM6_DAC_IRQHandler();

// func headers
void internal_clock();
void error_beep();                   // output a beep sound for a couple seconds, then turn it off
uint16_t read_adc(void);             // read adc value(pot for vol)
void set_volume(uint16_t adc_value); // set volume based on potentiometer




// game variables
uint8_t sequence[MAX_LEVEL]; // stores the full random pattern
uint8_t current_level = 0;   // current game level (starts from 0)
bool game_over = false;      // flag to check game over state
bool input_mode = false;     // true when waiting for player input
uint8_t current_input = 0;   // current position in input sequence

// function declarations
void GPIO_Configure(void);
void SysTick_Configure(void);
void PWM_Configure(void);
void generate_sequence(void);
void play_sequence(void);
void light_led(uint8_t led);
void set_led_brightness(uint8_t color, uint16_t brightness);
bool check_button(uint8_t button);
uint8_t get_button_press(void);
void delay_ms(uint32_t ms);

//diplay functions
int score(int current);
void spi_cmd(unsigned int data);
void spi_data(unsigned int data);
void nano_wait(unsigned int n);
void spi1_init_oled();
void spi1_display1(const char *string);
void spi1_display2(const char *string) ;
void init_spi1();

    
int main(void)
{
    
    // initialize ADC and DAC for sound gen
    internal_clock();
    
    init_adc();
    init_dac();
    init_usart5();
    init_wavetable();
    init_tim6();
   
    //initialize gpio, systick timer, and pwm output
    GPIO_Configure();
    SysTick_Configure();
    PWM_Configure();
    

    //initalize oled display
    init_spi1();
    spi1_init_oled();
    char str_oled[32]; //display for number
    spi1_display1("Start");
    int sc;

    // seed the random number generator using current systick value
    srand(SysTick->VAL);
    
    // generate the initial sequence of steps
    generate_sequence();
    
    
    // main game loop
    while (1)
    {
        
        if (!game_over)
        {
            // show the current pattern to the player
            play_sequence();

            // enable input mode and reset input index
            input_mode = true;
            current_input = 0;

            // loop while waiting for player to complete current sequence
            while (current_input < current_level && !game_over)
            {
                uint8_t button = get_button_press();
                if (button != 0)
                {
                    // briefly light the led corresponding to the button
                    light_led(button);

                    // check if the input matches the pattern
                    if (button != sequence[current_input])
                    {
                        // if incorrect, trigger game over sequence
                        game_over = true;
                        error_beep();

                        //oled display
                        sc = score(current_level);
                        sprintf(str_oled, "final score %d", sc);
                        spi1_display1("You Lose!");
                        spi1_display2(str_oled);

                        // flash all leds multiple times
                        for (int i = 0; i < 5; i++)
                        {
                            HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_RED_PIN | LED_BLUE_PIN, GPIO_PIN_SET);
                            delay_ms(200);
                            HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_RED_PIN | LED_BLUE_PIN, GPIO_PIN_RESET);
                            delay_ms(200);
                        }
                    }
                    else
                    {
                        // input is correct, move to next step
                        current_input++;

                         //diplay score
                         sc = score(current_level);
                         sprintf(str_oled, "current score %d", sc);
                         spi1_display1("Next Level");
                         spi1_display2(str_oled);
                    }
                }
            }

            input_mode = false;

            if (!game_over)
            {
                // flash leds quickly to indicate level success
                for (int i = 0; i < 3; i++)
                {
                    HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN, GPIO_PIN_SET);
                    delay_ms(100);
                    HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN, GPIO_PIN_RESET);
                    delay_ms(100);
                }

                // increase game level
                current_level++;

                // check for win condition
                if (current_level >= MAX_LEVEL)
                {
                    // flash leds repeatedly to indicate victory
                    for (int i = 0; i < 10; i++)
                    {
                        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN, GPIO_PIN_SET);
                        delay_ms(100);
                        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN, GPIO_PIN_RESET);
                        delay_ms(100);
                    }
                    //win display
                    sc = score(current_level);
                    sprintf(str_oled, "final score %d", sc);
                    spi1_display1("You Win!");
                    spi1_display2(str_oled);

                    // set game over flag
                    game_over = true;
                }
                else
                {
                    // add one more step to the sequence
                    sequence[current_level] = (rand() % 4) + 1;
                }
            }
        }
        else
        {
            // if game is over, wait for a button press to restart
            if (get_button_press() != 0)
            {
                game_over = false;
                current_level = 0;
                generate_sequence();
            }
        }
    }
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // enable clocks for GPIOC and GPIOB
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // configure PC0-PC3 (LEDs) as output
    GPIO_InitStruct.Pin = LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // configure PB0-PB3 (buttons) as input with pull-up
    GPIO_InitStruct.Pin = BUTTON_GREEN_PIN | BUTTON_ORANGE_PIN | BUTTON_RED_PIN | BUTTON_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
}

void SysTick_Configure(void)
{
    // configure systick to generate 1ms ticks
    HAL_SYSTICK_Config(SystemCoreClock / 1000);
}

void PWM_Configure(void)
{
    // enable clocks for TIM3 and GPIOC
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // configure PC6, PC7, PC8 as alternate function for TIM3 CH1–CH3
    GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN ;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // TIM3 configuration for PWM
    TIM_HandleTypeDef htim3;
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 48 - 1; // Prescaler to get 1 MHz
    htim3.Init.Period = 1000 - 1;  // 1 kHz PWM frequency
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    // PWM channel config template
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                        // Start with 0% brightness
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // Use LOW if you're using common anode LEDs
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    // configure all three channels for RGB
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1); // Red - PC6
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2); // Green - PC7
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3); // Blue - PC8

    // start PWM for all RGB channels
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void set_led_brightness(uint8_t color, uint16_t brightness)
{
    // brightness: 0 to 1000 (duty cycle for 1 kHz PWM)
    switch (color)
    {
    case 1: // Red
        TIM3->CCR1 = brightness;
        break;
    case 2: // Green
        TIM3->CCR2 = brightness;
        break;
    case 3: // Blue
        TIM3->CCR3 = brightness;
        break;
    default:
        break;
    }
}

void generate_sequence(void)
{
    // fill the sequence with random values from 1 to 4
    for (int i = 0; i < MAX_LEVEL; i++)
    {
        sequence[i] = (rand() % 4) + 1;
    }
}

void play_sequence(void)
{
    // show the sequence from the beginning up to current level
    for (int i = 0; i < current_level; i++)
    {
        light_led(sequence[i]);
        delay_ms(BETWEEN_TIME);
    }
}

void light_led(uint8_t led)
{
    // turn on led using pwm at full brightness
    set_led_brightness(led, 1000); // 1000 corresponds to full brightness

    // keep it on for a set time
    delay_ms(LED_ON_TIME);

    // turn off led by setting brightness to zero
    set_led_brightness(led, 0);
}

bool check_button(uint8_t button)
{
    // read the button state and return true if pressed
    GPIO_PinState state;
    switch (button)
    {
    case 1:
        state = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_GREEN_PIN);
        return (state == GPIO_PIN_RESET);
    case 2:
        state = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_ORANGE_PIN);
        return (state == GPIO_PIN_RESET);
    case 3:
        state = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_RED_PIN);
        return (state == GPIO_PIN_RESET);
    case 4:
        state = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_BLUE_PIN);
        return (state == GPIO_PIN_RESET);
    default:
        return false;
    }
}

uint8_t get_button_press(void)
{
    // wait until a button is pressed and released, then return its value
    while (1)
    {
        for (uint8_t i = 1; i <= 4; i++)
        {
            if (check_button(i))
            {
                delay_ms(DEBOUNCE_DELAY); // debounce delay
                if (check_button(i))
                {
                    while (check_button(i))
                        ; // wait for release
                    return i;
                }
            }
        }
        delay_ms(10); // prevent tight polling loop
    }
}

void delay_ms(uint32_t ms)
{
    // delay using hal systick timing
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms)
        ;
}


/*#include "stm32f0xx.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "fifo.h"
#include "tty.h"

#define FIFOSIZE 16
#define MAX_LEVEL 10
#define LED_ON_TIME 500   // LED on duration in milliseconds
#define BETWEEN_TIME 200  // Delay between sequence steps

// Game variables
uint8_t sequence[MAX_LEVEL]; // Stores the full random pattern
uint8_t current_level = 0;   // Current game level (starts from 0)
bool game_over = false;      // Flag to check game over state
bool input_mode = false;     // True when waiting for player input
uint8_t current_input = 0;   // Current position in input sequence

// Color definitions (mapped to characters)
const char colors[] = {'R', 'G', 'B', 'Y'}; // Red, Green, Blue, Yellow

// Function declarations
void internal_clock();
void init_usart5();
void enable_tty_interrupt();
char interrupt_getchar();
void generate_sequence(void);
void play_sequence(void);
void play_tone(int frequency, int duration);
void delay_ms(uint32_t ms);
void game_over_sequence(void);
void level_success_sequence(void);
void victory_sequence(void);

int __io_putchar(int c) {
    if (c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE)){}
        USART5->TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE)){}
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    return interrupt_getchar();
}

void USART3_8_IRQHandler(void) {
    while (DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo)) {
            insert_echo_char(serfifo[seroffset]);
        }
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}

void init_game() {
    srand(SysTick->VAL); // Seed random number generator
    current_level = 0;
    game_over = false;
    generate_sequence();
}

void generate_sequence(void) {
    // Fill the sequence with random values from 0 to 3 (matching colors array)
    for (int i = 0; i < MAX_LEVEL; i++) {
        sequence[i] = rand() % 4;
    }
}

void play_sequence(void) {
    printf("\nSimon says:\n");
    for (int i = 0; i <= current_level; i++) {
        printf("%c ", colors[sequence[i]]);
        play_tone(440 + (sequence[i] * 110), LED_ON_TIME); // Play different tones for each color
        delay_ms(BETWEEN_TIME);
    }
    printf("\nYour turn:\n");
}

void play_tone(int frequency, int duration) {
    // This would be implemented with actual hardware
    printf(" [Tone %dHz %dms] ", frequency, duration);
}

bool check_user_input() {
    for (int i = 0; i <= current_level; i++) {
        char expected = colors[sequence[i]];
        char received = interrupt_getchar();

        printf("%c", received); // Echo the input

        if (received != expected) {
            printf("\nWrong! Expected %c\n", expected);
            return false;
        }

        // Small delay between inputs
        delay_ms(200);
    }
    return true;
}

void game_over_sequence(void) {
    printf("\nGame Over! You reached level %d\n", current_level);
    for (int i = 0; i < 5; i++) {
        printf("X ");
        delay_ms(200);
        printf("O ");
        delay_ms(200);
    }
    printf("\nPress any key to play again...\n");
}

void level_success_sequence(void) {
    printf("\nCorrect! ");
    for (int i = 0; i < 3; i++) {
        printf(":) ");
        delay_ms(100);
        printf("   ");
        delay_ms(100);
    }
    printf("\n");
}

void victory_sequence(void) {
    printf("\nYOU WIN! ");
    for (int i = 0; i < 10; i++) {
        printf("\\o/ ");
        delay_ms(100);
        printf("    ");
        delay_ms(100);
    }
    printf("\nPress any key to play again...\n");
}

void delay_ms(uint32_t ms) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {}
}

int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();

    setbuf(stdin, 0);
    setbuf(stdout, 0);
    setbuf(stderr, 0);

    printf("Welcome to Simon Says!\n");
    printf("Colors: R(Red), G(Green), B(Blue), Y(Yellow)\n");
    printf("Press any key to start...\n");

    // Wait for any key to start
    interrupt_getchar();
    init_game();

    while(1) {
        if (!game_over) {
            play_sequence();

            if (!check_user_input()) {
                game_over_sequence();
                game_over = true;
            } else {
                level_success_sequence();

                // Increase game level
                current_level++;

                // Check for win condition
                if (current_level >= MAX_LEVEL) {
                    victory_sequence();
                    game_over = true;
                }
            }
        } else {
            // If game is over, wait for a key press to restart
            interrupt_getchar();
            init_game();
            printf("\nNew game started!\n");
        }
    }

    return 0;
}

// Include your existing USART5 initialization and configuration functions here
void init_usart5() {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIODEN;

    GPIOC->MODER &= ~(GPIO_MODER_MODER12);
    GPIOC->MODER |= GPIO_MODER_MODER12_1;
    GPIOC->AFR[1] |= (2 << (4 * (12 - 8)));

    GPIOD->MODER &= ~(GPIO_MODER_MODER2);
    GPIOD->MODER |= GPIO_MODER_MODER2_1;
    GPIOD->AFR[0] |= (2 << (4 * 2));

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    USART5->CR1 &= ~USART_CR1_UE;

    USART5->CR1 &= ~USART_CR1_M1;
    USART5->CR1 &= ~USART_CR1_M0;

    USART5->CR2 &= ~USART_CR2_STOP;

    USART5->CR1 &= ~USART_CR1_PCE;

    USART5->CR1 &= ~USART_CR1_OVER8;

    USART5->BRR = (48000000 / 115200);

    USART5->CR1 |= USART_CR1_TE;
    USART5->CR1 |= USART_CR1_RE;

    USART5->CR1 |= USART_CR1_UE;

    while(!(USART5->ISR & USART_ISR_TEACK));
    while(!(USART5->ISR & USART_ISR_REACK));
}

void enable_tty_interrupt(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    USART5->CR1 |= USART_CR1_RXNEIE;
    USART5->CR3 |= USART_CR3_DMAR;
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;
    DMA2_Channel2->CMAR = (uint32_t)serfifo;
    DMA2_Channel2->CPAR = (uint32_t)&USART5->RDR;
    DMA2_Channel2->CNDTR = FIFOSIZE;
    DMA2_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL | DMA_CCR_EN;
    NVIC->ISER[0] |= (1 << (USART3_8_IRQn));
}

char interrupt_getchar() {
    while (!fifo_newline(&input_fifo)){
        asm volatile ("wfi");
    }
    return fifo_remove(&input_fifo);
}

void internal_clock() {
    // Implement your clock initialization here
}*/