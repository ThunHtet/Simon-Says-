// -----------------------------------------
// includes
// -----------------------------------------
#include "stm32f0xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// -----------------------------------------
// led pin definitions (remapped for PWM)
// -----------------------------------------
#define LED1_RED_PIN 6   // PC6 - TIM3_CH1
#define LED1_GREEN_PIN 7 // PC7 - TIM3_CH2
#define LED1_BLUE_PIN 8  // PC8 - TIM3_CH3

// -----------------------------------------
// GPIO pin definitions (CMSIS compliant)
// -----------------------------------------
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)

#define GPIO_PIN_RESET 0 // Use simple logic level
#define BUTTON_GREEN_PIN GPIO_PIN_0
#define BUTTON_ORANGE_PIN GPIO_PIN_1
#define BUTTON_RED_PIN GPIO_PIN_2
#define BUTTON_BLUE_PIN GPIO_PIN_3
#define BUTTON_PORT GPIOB

// -----------------------------------------
// game constants
// -----------------------------------------
#define MAX_LEVEL 10
#define LED_ON_TIME 500   // milliseconds
#define BETWEEN_TIME 200  // milliseconds
#define DEBOUNCE_DELAY 50 // milliseconds

// -----------------------------------------
// game logic helper function prototypes
// -----------------------------------------
void reset_game(void);
void handle_user_input(void);
void game_over_sequence(void);
void win_sequence(void);
void wait_for_restart(void);
void success_feedback(void);
void light_led_color(uint8_t led);

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
uint8_t sequence[MAX_LEVEL]; // Full sequence pattern
uint8_t current_level = 0;
bool game_over = false;
bool input_mode = false;
uint8_t current_input = 0;

// -----------------------------------------
// init function prototypes
// -----------------------------------------
void GPIO_Configure(void);    // Set up GPIO for LED outputs and button inputs
void PWM_Configure(void);     // Configure timers and channels for PWM LED control
void SysTick_Configure(void); // Configure SysTick for millisecond delay timing

void init_adc(void);            // Initialize ADC for potentiometer (volume control)
void init_dac(void);            // Initialize DAC for sound output (buzzer)
void init_usart5(void);         // Initialize USART5 for serial communication (debugging/output)
void init_wavetable(void);      // Generate wavetable for buzzer sound
void init_tim6(void);           // Configure TIM6 for DAC waveform timing
void TIM6_DAC_IRQHandler(void); // Interrupt handler for TIM6 (handles DAC output updates)

// -----------------------------------------
// pwm timer inits
// -----------------------------------------
void init_tim2(void);  // Configure TIM2 for LED3 (Red → PA5, Green → PA2, Blue → PA3)
void init_tim3(void);  // Configure TIM3 for LED1 (Red → PC6, Green → PC7, Blue → PC8)
                       // and LED2 Red channel (PC9, TIM3_CH4)
void init_tim15(void); // Configure TIM15 for LED2 Green (PA14) and Blue (PB15)

// -----------------------------------------
// system & game logic function prototypes
// -----------------------------------------
void internal_clock(void);           // Configure internal system clock to 48 MHz
void error_beep(void);               // Play error tone via DAC (wrong input buzzer)
uint16_t read_adc(void);             // Read ADC value from potentiometer (for volume)
void set_volume(uint16_t adc_value); // Scale ADC reading to volume value for buzzer

void generate_sequence(void); // Generate random sequence of LED/button pattern
void play_sequence(void);     // Play current sequence via LED light pattern
void light_led(uint8_t led);  // Light up the selected LED (1–4) with default color

void set_led_brightness(uint8_t led, char color, uint16_t brightness); // Set individual LED brightness via PWM
bool check_button(uint8_t button);                                     // Check if the specified button (1–4) is pressed
uint8_t get_button_press(void);                                        // Wait for valid button press and return which one was pressed
void delay_ms(uint32_t ms);                                            // Millisecond delay using SysTick

int main(void)
{
    // ----- Initialization -----
    internal_clock();
    init_adc();
    init_dac();
    init_usart5();
    init_wavetable();
    init_tim6();

    GPIO_Configure();
    SysTick_Configure();
    PWM_Configure();

    srand(SysTick->VAL); // Seed random generator

    while (1)
    {
        reset_game();
        play_sequence();

        input_mode = true;
        current_input = 0;

        while (input_mode && !game_over)
        {
            handle_user_input();
        }

        if (game_over)
        {
            game_over_sequence();
        }
        else if (current_level > MAX_LEVEL)
        {
            win_sequence();
        }
    }
}

void GPIO_Configure(void)
{
    // Enable GPIO clocks for A, B, C
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    // ------------------------------
    // Configure PB8–PB11 (Game Logic LEDs)
    // PB8  = White
    // PB9  = Red
    // PB10 = Green
    // PB11 = Blue
    // ------------------------------
    // Set as General Purpose Output Mode (01)
    GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)) |
                      (3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->MODER |= ((1U << (8 * 2)) | (1U << (9 * 2)) |
                     (1U << (10 * 2)) | (1U << (11 * 2)));

    // Output type: Push-pull (reset state, so no change needed)
    GPIOB->OTYPER &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));

    // Output speed: High speed (11)
    GPIOB->OSPEEDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)) |
                        (3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->OSPEEDR |= ((3U << (8 * 2)) | (3U << (9 * 2)) |
                       (3U << (10 * 2)) | (3U << (11 * 2)));

    // No pull-up/pull-down
    GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)) |
                      (3U << (10 * 2)) | (3U << (11 * 2)));

    // ------------------------------
    // KEEP your PWM setup here for the RGB LED (PC6-PC9, PA14, PB15)
    // ------------------------------
    // Already done properly in your existing code (don't delete your PWM config).
}

void SysTick_Configure(void)
{
    SysTick->LOAD = (48000 - 1);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

void init_tim3(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 47;
    TIM3->ARR = 999;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE_Msk |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE_Msk;
    TIM3->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE_Msk |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE_Msk;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1 = TIM_CR1_CEN;
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

void init_tim15(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->PSC = 47;
    TIM15->ARR = 999;
    TIM15->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE_Msk |
                   (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE_Msk;
    TIM15->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM15->CR1 = TIM_CR1_CEN;
}

void init_tim2(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 47;
    TIM2->ARR = 999;
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE_Msk |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE_Msk;
    TIM2->CCMR2 = (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE_Msk;
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM2->CR1 = TIM_CR1_CEN;
}

void PWM_Configure(void)
{
    // This function delegates PWM initialization to specific timer setups
    init_tim3();
    init_tim2();
    init_tim15();
}

void set_led_brightness(uint8_t led, char color, uint16_t brightness)
{
    switch (led)
    {
    case 1:
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
    case 2:
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
    case 3:
        switch (color)
        {
        case 'R':
            TIM2->CCR1 = brightness;
            break;
        case 'G':
            TIM2->CCR3 = brightness;
            break;
        case 'B':
            TIM2->CCR4 = brightness;
            break;
        }
        break;
    default:
        break;
    }
}

void light_led_color(uint8_t led)
{
    // Turn off all LEDs first
    GPIOB->ODR &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));

    // Light up the selected one
    switch (led)
    {
    case 1: // White
        GPIOB->ODR |= (1U << 8);
        break;
    case 2: // Red
        GPIOB->ODR |= (1U << 9);
        break;
    case 3: // Green
        GPIOB->ODR |= (1U << 10);
        break;
    case 4: // Blue
        GPIOB->ODR |= (1U << 11);
        break;
    default:
        break;
    }
}

void wait_for_restart(void)
{
    while (get_button_press() == 0)
    {
        // Wait here until any button is pressed
    }
    delay_ms(300); // Optional: small debounce / delay so the game doesn't restart immediately
}

void reset_game(void)
{
    current_level = 1;
    game_over = false;
    generate_sequence();
}

void handle_user_input(void)
{
    uint8_t button = get_button_press();

    if (button != 0)
    {
        light_led(button); // Feedback

        if (button != sequence[current_input])
        {
            game_over = true;
            input_mode = false;
        }
        else
        {
            current_input++;
            if (current_input >= current_level)
            {
                input_mode = false; // Completed this level successfully!
                success_feedback();
                current_level++;
            }
        }
    }
}

void success_feedback(void)
{
    for (int i = 0; i < 3; i++)
    {
        light_led(1);
        light_led(2);
        light_led(3);
        light_led(4);
        delay_ms(100);
        light_led(0); // Turn off
        delay_ms(100);
    }
}

void game_over_sequence(void)
{
    error_beep();
    for (int i = 0; i < 5; i++)
    {
        light_led(1);
        light_led(2);
        light_led(3);
        light_led(4);
        delay_ms(100);
        light_led(0);
        delay_ms(100);
    }

    wait_for_restart();
}

void win_sequence(void)
{
    for (int i = 0; i < 10; i++)
    {
        light_led(1);
        light_led(2);
        light_led(3);
        light_led(4);
        delay_ms(100);
        light_led(0);
        delay_ms(100);
    }

    game_over = true;
    wait_for_restart();
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
    for (int i = 0; i < current_level; i++)
    {
        light_led(sequence[i]); // turn on LED for this step
        delay_ms(LED_ON_TIME);  // keep it on for a bit
        light_led(0);           // turn all LEDs off
        delay_ms(BETWEEN_TIME); // wait between steps
    }
}

void light_led(uint8_t led)
{
    if (led == 0)
    {
        // Turn off all LEDs
        GPIOB->ODR &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));
        return;
    }

    // Light up the selected LED
    light_led_color(led);
}

bool check_button(uint8_t button)
{
    switch (button)
    {
    case 1:
        return !(BUTTON_PORT->IDR & (1 << 0)); // BUTTON_GREEN_PIN
    case 2:
        return !(BUTTON_PORT->IDR & (1 << 1)); // BUTTON_ORANGE_PIN
    case 3:
        return !(BUTTON_PORT->IDR & (1 << 2)); // BUTTON_RED_PIN
    case 4:
        return !(BUTTON_PORT->IDR & (1 << 3)); // BUTTON_BLUE_PIN
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
    volatile uint32_t delay_counter = ms;
    while (delay_counter--)
    {
        for (volatile uint32_t i = 0; i < 4800; i++)
            ; // ~1ms delay at 48MHz
    }
}

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