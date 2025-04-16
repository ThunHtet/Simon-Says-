// -----------------------------------------
// includes
// -----------------------------------------
#include "stm32f0xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// -----------------------------------------
// global timer handles
// -----------------------------------------
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

// -----------------------------------------
// led pin definitions
// -----------------------------------------
#define LED1_RED GPIO_PIN_0
#define LED1_GREEN GPIO_PIN_1
#define LED1_BLUE GPIO_PIN_2

#define LED2_RED GPIO_PIN_3
#define LED2_GREEN GPIO_PIN_4
#define LED2_BLUE GPIO_PIN_5

#define LED3_RED GPIO_PIN_6
#define LED3_GREEN GPIO_PIN_7
#define LED3_BLUE GPIO_PIN_8

#define LED4_RED GPIO_PIN_9
#define LED4_GREEN GPIO_PIN_10
#define LED4_BLUE GPIO_PIN_11

#define LED_PORT GPIOC

// -----------------------------------------
// button pin definitions
// -----------------------------------------
#define BUTTON_GREEN_PIN GPIO_PIN_0
#define BUTTON_ORANGE_PIN GPIO_PIN_1
#define BUTTON_RED_PIN GPIO_PIN_2
#define BUTTON_BLUE_PIN GPIO_PIN_3
#define BUTTON_PORT GPIOB

// -----------------------------------------
// game constants
// -----------------------------------------
#define MAX_LEVEL 10
#define LED_ON_TIME 500   // ms
#define BETWEEN_TIME 200  // ms
#define DEBOUNCE_DELAY 50 // ms

// -----------------------------------------
// wavetable constants and variables
// -----------------------------------------
#define N 1000
#define RATE 20000

short int wavetable[N];
int step0 = 0, offset0 = 0;
int step1 = 0, offset1 = 0;
volatile uint32_t volume = 2400;

// -----------------------------------------
// game state variables
// -----------------------------------------
uint8_t sequence[MAX_LEVEL]; // full pattern
uint8_t current_level = 0;
bool game_over = false;
bool input_mode = false;
uint8_t current_input = 0;

// -----------------------------------------
// init function prototypes
// -----------------------------------------
void GPIO_Configure(void);
void PWM_Configure(void);
void SysTick_Configure(void);

void init_adc(void);
void init_dac(void);
void init_usart5(void);
void init_wavetable(void);
void init_tim6(void);
void TIM6_DAC_IRQHandler(void);

// pwm timer inits
void init_tim2(void);  // LED 3 (red & green)
void init_tim3(void);  // LED 1 + partial LED 2
void init_tim14(void); // LED 4 red
void init_tim15(void); // LED 2 (green & blue)
void init_tim16(void); // LED 3 blue, LED 4 green
void init_tim17(void); // LED 4 blue

// -----------------------------------------
// system & game logic function prototypes
// -----------------------------------------
void internal_clock(void);
void error_beep(void);               // play error tone
uint16_t read_adc(void);             // read pot
void set_volume(uint16_t adc_value); // map adc to PWM

void generate_sequence(void);
void play_sequence(void);
void light_led(uint8_t led);
void light_led_color(uint8_t led, char color);
void set_led_brightness(uint8_t led, char color, uint16_t brightness);
bool check_button(uint8_t button);
uint8_t get_button_press(void);
void delay_ms(uint32_t ms);

int main(void)
{
    // Initialize system clock and peripherals
    internal_clock();
    init_adc();
    init_dac();
    init_usart5();
    init_wavetable();
    init_tim6();

    // Initialize GPIOs and timer-based PWM
    GPIO_Configure();
    SysTick_Configure();
    PWM_Configure(); // <-- This now calls init_tim3(), init_tim15(), etc.

    // Seed random number generator with systick value
    srand(SysTick->VAL);

    // Generate the pattern sequence
    generate_sequence();

    // --- MAIN GAME LOOP ---
    while (1)
    {
        if (!game_over)
        {
            play_sequence(); // Play back the LED sequence
            input_mode = true;
            current_input = 0;

            // Wait for user to input the full sequence
            while (current_input < current_level && !game_over)
            {
                uint8_t button = get_button_press();
                if (button != 0)
                {
                    light_led(button);

                    if (button != sequence[current_input])
                    {
                        game_over = true;
                        error_beep();

                        for (int i = 0; i < 5; i++)
                        {
                            // Flash all RGB LEDs in failure pattern
                            light_led(1);
                            light_led(2);
                            light_led(3);
                            light_led(4);
                            delay_ms(100);
                        }
                    }
                    else
                    {
                        current_input++;
                    }
                }
            }

            input_mode = false;

            if (!game_over)
            {
                // Level up pattern
                for (int i = 0; i < 3; i++)
                {
                    light_led(1);
                    light_led(2);
                    light_led(3);
                    light_led(4);
                    delay_ms(100);
                }

                current_level++;

                if (current_level >= MAX_LEVEL)
                {
                    // Victory celebration
                    for (int i = 0; i < 10; i++)
                    {
                        light_led(1);
                        light_led(2);
                        light_led(3);
                        light_led(4);
                        delay_ms(100);
                    }

                    game_over = true;
                }
                else
                {
                    sequence[current_level] = (rand() % 4) + 1;
                }
            }
        }
        else
        {
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

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = LED1_RED | LED1_GREEN | LED1_BLUE |
                          LED2_RED | LED2_GREEN | LED2_BLUE |
                          LED3_RED | LED3_GREEN | LED3_BLUE |
                          LED4_RED | LED4_GREEN | LED4_BLUE;
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
void init_tim3(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 48 - 1;
    htim3.Init.Period = 1000 - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void init_tim15(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM15_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_TIM15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    htim15.Instance = TIM15;
    htim15.Init.Prescaler = 48 - 1;
    htim15.Init.Period = 1000 - 1;
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim15);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}

void init_tim2(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 48 - 1;
    htim2.Init.Period = 1000 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void init_tim16(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_TIM16;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM16;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 48 - 1;
    htim16.Init.Period = 1000 - 1;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim16);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void init_tim17(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM17;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    htim17.Instance = TIM17;
    htim17.Init.Prescaler = 48 - 1;
    htim17.Init.Period = 1000 - 1;
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim17);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}

void init_tim14(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_TIM14;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 48 - 1;
    htim14.Init.Period = 1000 - 1;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim14);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}

void PWM_Configure(void)
{
    // This function delegates PWM initialization to specific timer setups
    init_tim3();  // LED1 (PC6-PC9)
    init_tim15(); // LED2 (PA14, PA1)
    init_tim2();  // LED3 (PA2, PA3)
    init_tim16(); // LED3 Blue (PA6) and LED4 Green (PB5)
    init_tim17(); // LED4 Blue (PB6)
    init_tim14(); // LED4 Red (PB4)
}

void set_led_brightness(uint8_t led, char color, uint16_t brightness)
{
    switch (led)
    {
    case 1: // LED1 — TIM3 CH1–CH3 (PC6-8)
        switch (color)
        {
        case 'R':
            TIM3->CCR1 = brightness;
            break;
        case 'G':
            TIM3->CCR2 = brightness;
            break;
        case 'B':
            TIM3->CCR3 = brightness;
            break;
        }
        break;

    case 2: // LED2 — Red: TIM3 CH4 (PC9), Green: TIM15 CH1 (PB14), Blue: TIM15 CH2 (PB15)
        switch (color)
        {
        case 'R':
            TIM3->CCR4 = brightness;
            break;
        case 'G':
            TIM15->CCR1 = brightness;
            break;
        case 'B':
            TIM15->CCR2 = brightness;
            break;
        }
        break;

    case 3: // LED3 — Red: TIM2 CH3 (PA2), Green: TIM2 CH4 (PA3), Blue: TIM16 CH1 (PA6)
        switch (color)
        {
        case 'R':
            TIM2->CCR3 = brightness;
            break;
        case 'G':
            TIM2->CCR4 = brightness;
            break;
        case 'B':
            TIM16->CCR1 = brightness;
            break;
        }
        break;

    case 4: // LED4 — Red: TIM14 CH1 (PB4), Green: TIM16 CH1 (PB5), Blue: TIM17 CH1 (PB6)
        switch (color)
        {
        case 'R':
            TIM14->CCR1 = brightness;
            break;
        case 'G':
            TIM16->CCR1 = brightness; // shared with LED3 blue
            break;
        case 'B':
            TIM17->CCR1 = brightness;
            break;
        }
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

void light_led_color(uint8_t led, char color)
{
    // Clear all PWM outputs (set brightness to 0)
    TIM3->CCR1 = 0;  // PC6  (LED1_RED or LED2_RED)
    TIM3->CCR2 = 0;  // PC7  (LED1_GREEN)
    TIM3->CCR3 = 0;  // PC8  (LED1_BLUE)
    TIM3->CCR4 = 0;  // PC9  (LED2_RED)
    TIM15->CCR1 = 0; // PA14 (LED2_GREEN)
    TIM15->CCR2 = 0; // PA1  (LED2_BLUE)

    // Determine RGB brightness values based on the color
    uint16_t r = 0, g = 0, b = 0;
    switch (color)
    {
    case 'R':
        r = 1000;
        break;
    case 'G':
        g = 1000;
        break;
    case 'B':
        b = 1000;
        break;
    case 'Y':
        r = 1000;
        g = 1000;
        break;
    default:
        break;
    }

    // Set the brightness on the specific LED
    switch (led)
    {
    case 1:
        TIM3->CCR1 = r; // Red - PC6 (CH1)
        TIM3->CCR2 = g; // Green - PC7 (CH2)
        TIM3->CCR3 = b; // Blue - PC8 (CH3)
        break;
    case 2:
        TIM3->CCR4 = r;  // Red - PC9 (CH4)
        TIM15->CCR1 = g; // Green - PA14 (CH1)
        TIM15->CCR2 = b; // Blue - PA1 (CH2)
        break;
    default:
        break;
    }

    delay_ms(LED_ON_TIME);

    // Turn off again
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    TIM15->CCR1 = 0;
    TIM15->CCR2 = 0;
}

void light_led(uint8_t led)
{
    switch (led)
    {
    case 1:
        light_led_color(1, 'R');
        break;
    case 2:
        light_led_color(2, 'G');
        break;
    case 3:
        light_led_color(3, 'B');
        break;
    case 4:
        light_led_color(4, 'Y');
        break;
    default:
        break;
    }
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

// int main(void) {
//     internal_clock();
//     init_adc();
//     init_dac();
//     init_usart5();
//     init_wavetable();
//     init_tim6();

//     setbuf(stdout, NULL);
//     printf("sound on\n");

//     error_beep();
//     printf("sound off\n");

//     while(1) {
//     }
// }

void init_wavetable(void)
{
    for (int i = 0; i < N; i++)
        wavetable[i] = (i < N / 2) ? 32767 : -32767; // create wave for buzzer sound
}

void set_freq(int chan, float f)
{
    if (chan == 0)
    {
        if (f == 0.0)
        {
            step0 = 0;
            offset0 = 0;
        }
        else
            step0 = (f * N / RATE) * (1 << 16);
    }
    if (chan == 1)
    {
        if (f == 0.0)
        {
            step1 = 0;
            offset1 = 0;
        }
        else
            step1 = (f * N / RATE) * (1 << 16);
    }
}

void error_beep(void)
{
    set_freq(0, 250); // set standard freq, tried different frequencies, like this best
    volatile int x;   // included because i kept getting overwritten when checking for set_volume
    for (volatile int i = 0; i < 500000; i++)
    {                 // delay for a set time
        x = i % 1000; // mod i by 1000
        if (x == 0)
        {                           // if i mod 1000 is 0(basically, every 1000 ticks of i), then run set_volume(read_adc) to set the volume incrementally throughout the sound being played
            set_volume(read_adc()); // set volume based on potentiometer value
        }
    }
    set_freq(0, 0); // stop sound
}

int __io_putchar(int c)
{
    while (!(USART5->ISR & USART_ISR_TXE))
        ;
    if (c == '\n')
    {                       // check if char is new line
        USART5->TDR = '\r'; // if char is new line, put carraige return first
        while (!(USART5->ISR & USART_ISR_TXE))
        {
            // wait for TXE flag, meaning it has been passed
        }
    }
    USART5->TDR = c; // put char to output
    return c;
}

void init_usart5()
{
    RCC->AHBENR |= (RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN); // enable port c and d clocks

    GPIOC->MODER &= ~(GPIO_MODER_MODER12); // reset PC12
    GPIOC->MODER |= (0x2 << (2 * 12));     // set PC12 function to alt func(2)

    GPIOC->AFR[1] |= (0x2 << (16)); // set AFR[1], bit shifted 16 bits, which represents AFR12(which means this is the set bit for AltFunc for PC12) to 0x2(representing AF2, which is USART5_TX)

    GPIOD->MODER &= ~(GPIO_MODER_MODER2); // reset PD2
    GPIOD->MODER |= (0x2 << (2 * 2));     // set PD2 function to alt func(2)

    GPIOD->AFR[0] |= (0x2 << (8)); // set AFR[0], bit shifted 8 bits, which represents AFR2(which means this is the set bit for AltFunc for PD2) to 0x2(representing AF2, which is USART5_RX)

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;          // enable RCC clock to USART5
    USART5->CR1 &= ~(USART_CR1_UE);                // disable usart bit
    USART5->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1); // set size to 8bits(M0 and M1 must be set to 00)
    USART5->CR2 &= ~(USART_CR2_STOP);              // set stop to be 00(representing 1 stop bit) ///this may be wrong???
    USART5->CR1 &= ~(USART_CR1_PCE);               // set parity control enable to 0 for off
    USART5->CR1 &= ~(USART_CR1_OVER8);             // set OVER8 to 0, representing 16x oversampling
    USART5->BRR = 0x1A1;                           // gotten from family reference for oversampling 16, desired baud rate of 115.2KBps

    USART5->CR1 |= USART_CR1_TE; // enable transmitter bit
    USART5->CR1 |= USART_CR1_RE; // enable reciever bit

    USART5->CR1 |= USART_CR1_UE; // enable usart bit
    while (!((USART5->ISR & USART_ISR_TEACK) && (USART5->ISR & USART_ISR_REACK)))
    {
        // wait until TE adn RE bits are acknowledged(TEACK and REACK and these flags), then the function can continue on to the end
    }
    // return; //end function
}

void init_adc(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable PA1
    GPIOA->MODER |= (3 << (1 * 2));    // PA1 to analog mode
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // PA1 as adc in

    ADC1->CFGR1 &= ~ADC_CFGR1_CONT; // single conversion mode
    ADC1->CR |= ADC_CR_ADEN;        // enable adc
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
        ; // wait until ready, then leave function
}

uint16_t read_adc(void)
{
    ADC1->CHSELR = ADC_CHSELR_CHSEL1; // select PA1
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC))
        ; // wait for conversion
    return ADC1->DR;
}
void set_volume(uint16_t adc_value)
{
    volume = 100 + ((adc_value * 3000) / 4095);
}

void init_dac(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable GPIOA RCC clock
    GPIOA->MODER |= (0x3 << (2 * 4));  // set PA4 to DAC_OUT1: (0x3<<(2*4))= 11 00 00 00 00
    RCC->APB1ENR |= RCC_APB1ENR_DACEN; // enable DAC RCC clock

    DAC->CR &= ~DAC_CR_TSEL1; // select TRGO trigger fro TSEL field in CR register(bit 000(~0x7) shifted to the left 3 bits)

    DAC->CR |= DAC_CR_EN1; // enable DAC
}

void TIM6_DAC_IRQHandler()
{                            // TIM7 ISR
    TIM6->SR &= ~TIM_SR_UIF; // clear UIF bit in SR register(acknowledge interrupt)

    offset0 += step0; // increment offset0 by step0
    offset1 += step1; // increment offset0 by step1

    if (offset0 >= (N << 16))
    {
        offset0 -= (N << 16); // decrement offset0 by (N<<16) if offset0 is greater than (N<<16)
    }
    if (offset1 >= (N << 16))
    {
        offset1 -= (N << 16); // decrement offset1 by (N<<16) if offset1 is greater than (N<<16)
    }

    int samp = wavetable[offset0 >> 16] + wavetable[offset1 >> 16]; // samp is the sum of the wavetable of offset0 and offset1

    samp = (samp * volume) >> 15; // multiply samp by volume and shift right 17 bits

    samp += 1200; // increment samp by 2048

    DAC->DHR12R1 = (samp & 0xFFF);

    return;
}

void init_tim6(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;         // enable clock for TIM6
    TIM6->PSC = ((48000000 / (RATE * 10))) - 1; // set PSC to 48000000/(RATE*10)
    TIM6->ARR = 10 - 1;                         // set ARR to 9

    TIM6->DIER |= TIM_DIER_UIE; // enable UIE bit in DIER to allow an interrupt to occur each time the counter reaches the ARR value, restarting it back at 0

    NVIC_EnableIRQ(TIM6_DAC_IRQn); // enable interrupt for TIM6

    TIM6->CR1 |= TIM_CR1_CEN; // enable TIM6 by setting CEN bit
    return;
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