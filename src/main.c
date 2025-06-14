// ============================================
//                 INCLUDES
// ============================================
#include "stm32f0xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "display.h"
#include "buzzer.h"


// ============================================
//                  DEFINES
// ============================================


// LED pin definitions (PWM remapped)
#define LED1_RED_PIN 6   // PC6 - TIM3_CH1
#define LED1_GREEN_PIN 7 // PC7 - TIM3_CH2
#define LED1_BLUE_PIN 8  // PC8 - TIM3_CH3


// GPIO pin definitions
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_RESET 0
#define BUTTON_GREEN_PIN GPIO_PIN_0
#define BUTTON_ORANGE_PIN GPIO_PIN_1
#define BUTTON_RED_PIN GPIO_PIN_2
#define BUTTON_BLUE_PIN GPIO_PIN_3
#define BUTTON_PORT GPIOB


// Game constants
#define STARTING_SEQUENCE_LENGTH 4
#define MAX_SEQUENCE_LENGTH 10 // or whatever you'd like the max to be
#define MAX_LEVEL 8 //number of levels
#define LED_ON_TIME 500   // milliseconds
#define BETWEEN_TIME 200  // milliseconds
#define DEBOUNCE_DELAY 50 // milliseconds

// ============================================
//              GLOBAL VARIABLES
// ============================================
uint8_t sequence[MAX_SEQUENCE_LENGTH];
uint8_t current_level = 0;
bool game_over = false;
bool input_mode = false;
uint8_t current_input = 0;
bool game_started = false;


// ============================================
//              FUNCTION DECLARATIONS
// ============================================
// System setup
void internal_clock(void);
void GPIO_Configure(void);
void SysTick_Configure(void);

// Timer initializations
void init_tim1(void);
void init_tim2(void);
void init_tim3(void);
void init_tim15(void);

// RGB
void PWM_Configure_RGB(void);
void success_fade_green(void);
void failure_flash_red(void);
void celebration_sequence(void);

// LEDS
void light_led(uint8_t led);
void set_led_brightness(uint8_t led, char color, uint16_t brightness);

// Buttons
bool check_button(uint8_t button);
uint8_t get_button_press(void);
void delay_ms(uint32_t ms);
void flush_buttons(void);
void oled_clear(void);

// Game logic
void wait_for_game_start(void);
void reset_game(void);
void countdown(void);
void run_game_rounds(void);
void generate_sequence(void);
void play_sequence(void);
void handle_user_input(void);
void success_feedback(void);
void game_over_sequence(void);
void win_sequence(void);
void wait_for_restart(void);


// ============================================
//          MAIN FUNCTION
// ============================================
int main(void)
{
    internal_clock();
    GPIO_Configure();

    //Display configuration
    init_spi1();
    spi1_init_oled();

    //rgb configuration
    setup_tim1();

    //sound generation
    init_adc();
    init_dac();
    init_wavetable();
    init_tim6();

    while (1)
    {     
        TIM1->CCR1 = 2400; //red
        TIM1->CCR2 = 2400; //green
        TIM1->CCR3 = 2400; //blue
        wait_for_game_start();
        reset_game();
        run_game_rounds();
    }
}

// ============================================
//          FULL IMPLEMENTATION BELOW
// ============================================
// System setup
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
    // Set PB8–PB11 as OUTPUT
    GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)) | (3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->MODER |= ((1U << (8 * 2)) | (1U << (9 * 2)) | (1U << (10 * 2)) | (1U << (11 * 2)));

    // Push-pull output
    GPIOB->OTYPER &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));

    // Output speed: High speed (11)
    GPIOB->OSPEEDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)) |
                        (3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->OSPEEDR |= ((3U << (8 * 2)) | (3U << (9 * 2)) |
                       (3U << (10 * 2)) | (3U << (11 * 2)));


    // No pull-up/pull-down for LEDs
    GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)) |(3U << (10 * 2)) | (3U << (11 * 2)));

    // ------------------------------
    // Configure PB0–PB3 (buttons) as input with pull-up
    // ------------------------------
    // PB0–PB3 (buttons) as input with pull-down
    GPIOB->MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (2 * 2)) | (3U << (3 * 2))); // Input mode (00)
    GPIOB->PUPDR &= ~((3U<<(0*2))|(3U<<(1*2))|(3U<<(2*2))|(3U<<(3*2)));
    GPIOB->PUPDR |=  ((2U<<(0*2))|(2U<<(1*2))|(2U<<(2*2))|(2U<<(3*2)));
}

void SysTick_Configure(void)
{
    SysTick->LOAD = (48000 - 1);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


// PWM Configuration
void PWM_Configure_RGB(void)
{
    // Enable GPIOC clock and TIM3 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Set PC6, PC7, PC8 to Alternate Function mode (AF0 = TIM3)
    GPIOC->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)) | (3 << (8 * 2)));
    GPIOC->MODER |= ((2 << (6 * 2)) | (2 << (7 * 2)) | (2 << (8 * 2))); // AF mode

    // AFR settings: AF0 is 0, so technically no need to set AFRs. Safe to clear:
    GPIOC->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOC->AFR[1] &= ~(0xF << ((8 - 8) * 4)); // PC8 AFR[1] slot 0

    // Timer 3 setup
    TIM3->PSC = 47;  // 48MHz / (47 + 1) = 1MHz timer clock
    TIM3->ARR = 999; // 1kHz PWM frequency (adjust as needed)

    // Configure PWM mode 1 (OCxM = 110), preload enable
    TIM3->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE_Msk | // Channel 1 (PC6 Red)
                  (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE_Msk;  // Channel 2 (PC7 Green)
    TIM3->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE_Msk;  // Channel 3 (PC8 Blue)

    // Enable output on all three channels
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

    // Enable auto-reload preload
    TIM3->CR1 |= TIM_CR1_ARPE;

    // Enable timer
    TIM3->CR1 |= TIM_CR1_CEN;
}

void setup_tim1(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable clock for GPIOC

    GPIOA->MODER &= ~(0xFF<<(2*8)); //clear gpioc mode for PA8-11
    GPIOA->MODER |= (0xAA<<(2*8)); //set gpioc mode to alternative function for PC8-11(10101010<<16), 10 is alt func, applied to pc (18/2)=8,9,10,11

    GPIOA->AFR[1] &= ~(0xF<<(4*0)); //clear gpioc afr to 0 for PA8
    GPIOA->AFR[1] |= (0x2<<(4*0)); //set gpioc afr to 2 (alt func) for PA8

    GPIOA->AFR[1] &= ~(0xF<<(4*1)); //clear gpioc afr to 0 for PA9
    GPIOA->AFR[1] |= (0x2<<(4*1)); //set gpioc afr to 2 (alt func) for PA9
    
    GPIOA->AFR[1] &= ~(0xF<<(4*2)); //clear gpioc afr to 0 for PA10
    GPIOA->AFR[1] |= (0x2<<(4*2)); //set gpioc afr to 2 (alt func) for PA10

    GPIOA->AFR[1] &= ~(0xF<<(4*3)); //clear gpioc afr to 0 for PA11
    GPIOA->AFR[1] |= (0x2<<(4*3)); //set gpioc afr to 2 (alt func) for PA11

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //enable sys clock for TIM1

    TIM1->BDTR |= (0x1<<15); //enable MOE

    TIM1->PSC = 1 - 1; //set psc to be 1

    TIM1->ARR = 2400 - 1; //set freq to be 20kHz

    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE); // CH1
    TIM1->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE); // CH2
    TIM1->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE); // CH3
    TIM1->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE); // CH4

    TIM1->CCER |= TIM_CCER_CC1E; //enable output for ch1 active high output
    TIM1->CCER |= TIM_CCER_CC2E; //enable output for ch2 active high output
    TIM1->CCER |= TIM_CCER_CC3E; //enable output for ch3 active high output
    TIM1->CCER |= TIM_CCER_CC4E; //enable output for ch4 active high output

    TIM1->CR1 |= TIM_CR1_CEN; //enable timer 3
}

// Timers
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

// RGB Helpers
void set_led_brightness(uint8_t led, char color, uint16_t brightness)
{
    //which color is actice
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

//flash green LED
void success_fade_green(void)
{
    TIM1->CCR1 = 2400; //red
    TIM1->CCR2 = 0; //green
    TIM1->CCR3 = 2400; //blue
    delay_ms(1000);
    TIM1->CCR1 = 2400; //red
    TIM1->CCR2 = 2400; //green
    TIM1->CCR3 = 2400; //blue
}

//flash red LED
void failure_flash_red(void)
{
    TIM1->CCR1 = 0; //red
    TIM1->CCR2 = 2400; //green
    TIM1->CCR3 = 2400; //blue
    delay_ms(1000);
    TIM1->CCR1 = 2400; //red
    TIM1->CCR2 = 2400; //green
    TIM1->CCR3 = 2400; //blue
}

//Winning game sequence
void celebration_sequence(void)
{
    for (int i = 0; i < MAX_LEVEL; i++)
    {
        success_fade_green();
        delay_ms(100);
        failure_flash_red();
    }
}


uint8_t bcd2dec(uint8_t bcd) {
    // Lower digit
    uint8_t dec = bcd & 0xF;

    // Higher digit
    dec += 10 * (bcd >> 4);
    return dec;
}

void setrgb(int rgb) {
    uint8_t b = bcd2dec(rgb & 0xFF);
    uint8_t g = bcd2dec((rgb >> 8) & 0xFF);
    uint8_t r = bcd2dec((rgb >> 16) & 0xFF);

    TIM1->CCR1 = 2400- ((2400*r)/99); //2400 is max value, we want 2400(off) - (2400 * r as percentage) = inverse of value representative of 2400*percentage of r
    TIM1->CCR2 = 2400 - ((2400*g)/99); //2400 is max value, we want 2400(off) - (2400 * g as percentage) = inverse of value representative of 2400*percentage of g
    TIM1->CCR3 = 2400 - ((2400*b)/99); //2400 is max value, we want 2400(off) - (2400 * b as percentage) = inverse of value representative of 2400*percentage of b

}

// LED Helper
void light_led(uint8_t led)
{
    // First turn all LEDs OFF (ACTIVE-HIGH: clear bits)
    GPIOB->ODR &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));


    switch (led)
    {
    case 1:                       // Blue LED (PB11)
        GPIOB->ODR |= (1U << 11); // ON
        break;
    case 2:                       // Green LED (PB10)
        GPIOB->ODR |= (1U << 10); // ON
        break;
    case 3:                      // Red LED (PB9)
        GPIOB->ODR |= (1U << 9); // ON
        break;
    case 4:                      // White LED (PB8)
        GPIOB->ODR |= (1U << 8); // ON
        break;
    case 0: // All OFF (optional case 0 for OFF)
    default:
        // Do nothing extra — they're already off at the start.
        break;
    }
}


// check which buttons are active
bool check_button(uint8_t button)
{
    uint32_t mask;
    switch (button)
    {
        case 1: mask = GPIO_PIN_0; break;
        case 2: mask = GPIO_PIN_1; break;
        case 3: mask = GPIO_PIN_2; break;
        case 4: mask = GPIO_PIN_3; break;
        default: return false;
    }
    return (BUTTON_PORT->IDR & mask) != 0;
}

//check which button is being pressed
uint8_t get_button_press(void)
{
    uint8_t detected_button = 0;


    while (1)
    {
        for (uint8_t i = 1; i <= 4; i++)
        {
            if (check_button(i))
            {
                // THIS IS THE FIX: Wait until the button is released BEFORE returning.
                while (check_button(i))
                {
                    // Wait for button release
                }
                delay_ms(DEBOUNCE_DELAY); // Optional debounce after release
                return i;
            }
        }
        delay_ms(10); // Prevent tight loop
    }
    return detected_button;
}

void flush_buttons(void)
{
    for (uint8_t i = 1; i <= 4; i++)
    {
        while (check_button(i))
        {
            // Wait until released
        }
    }
    delay_ms(50); // debounce
}


// Delay Helper
void delay_ms(uint32_t ms)
{
    volatile uint32_t delay_counter = ms;
    while (delay_counter--)
    {
        for (volatile uint32_t i = 0; i < 4800; i++)
            ; // ~1ms delay at 48MHz
    }
}

//clears the display
void oled_clear(void)
{
    spi_cmd(0x01);      // Clear display
    nano_wait(2000000); // Wait for clear to complete
}


// Game Logic
void reset_game(void)
{
    current_level = 1;   // Reset to level 1
    current_input = 0;   // Reset input index
    game_over = false;   // Clear game over flag
    input_mode = false;  // Not waiting for input yet
    generate_sequence(); // Sequence generated ONCE at the start!
}

//start the game
void wait_for_game_start(void)
{
    oled_clear();
    delay_ms(10);
    spi1_display1("Press Button 1");
    spi1_display2("(Blue) to start!");


    while (1)
    {
        if (check_button(1))
        {
            while (check_button(1))
                ;         // Wait for release
            delay_ms(50); // Debounce
            break;        // Exit once Button 1 is pressed
        }
    }
}

//countdown for game to start
void countdown(void)
{
    oled_clear();
    delay_ms(1000); // Pause before countdown
    spi1_display1("Starting game...");
    delay_ms(700); // Pause before countdown


    for (int i = 5; i >= 1; i--)
    {
        char buffer[20];
        oled_clear();
        delay_ms(10);
        sprintf(buffer, "Starting in: %d", i);
        spi1_display1(buffer);
        delay_ms(700); 
        
    }
    oled_clear();
    delay_ms(10); // Pause before GO
    spi1_display1("GO!");
    delay_ms(500); 
}

//length of sequence
int sequence_length_for_level(int level)
{
    int length = STARTING_SEQUENCE_LENGTH + (level - 1);
    if (length > MAX_SEQUENCE_LENGTH){
        length = MAX_SEQUENCE_LENGTH;
    }
    return length;
}

//generates the sequence
void generate_sequence(void)
{
    // Hardcoded pattern for Level 1:
    sequence[0] = 4; // White
    sequence[1] = 3; // Red
    sequence[2] = 2; // Green
    sequence[3] = 1; // Blue


    // If you want random sequences for higher levels:
    for (int i = 4; i < MAX_SEQUENCE_LENGTH; i++)
    {
        sequence[i] = (rand() % 4) + 1;
    }
}

//plays the game sequence
void play_sequence(void)
{
    char buffer[20];
    int seq_len = sequence_length_for_level(current_level);


    oled_clear();
    delay_ms(10);
    sprintf(buffer, "Level %d:", current_level);
    spi1_display1(buffer);
    delay_ms(700); // Show level
    

    light_led(5);
    oled_clear();


    for (int i = 0; i < seq_len; i++)
    {
        // Ensure LEDs are OFF before showing each one:
        light_led(0);
        
        // Show the next LED and its label:
        switch (sequence[i])
        {
        case 1:
            oled_clear();
            delay_ms(10);
            spi1_display1("Simon Says:");
            light_led(1);
            spi1_display2("Blue");
            delay_ms(10);
            

            break;
        case 2:
            oled_clear();
            delay_ms(10);
            spi1_display1("Simon Says:");
            light_led(2);
            spi1_display2("Green");
            delay_ms(10);
            break;
        case 3:
            oled_clear();
            delay_ms(10);
            spi1_display1("Simon Says:");
            light_led(3);
            spi1_display2("Red");
            delay_ms(10);
            break;
        case 4:
            oled_clear();
            delay_ms(10);
            spi1_display1("Simon Says:");
            light_led(4);
            spi1_display2("White");
            delay_ms(10);
            break;
        }


        delay_ms(1000); // LED ON time
        light_led(0);   // Turn OFF after each flash!
    }


    oled_clear();
    delay_ms(10); // Pause between sequence steps
    
}

//checks for the user input
void handle_user_input(void)
{
    char buffer[20];
    int seq_len = sequence_length_for_level(current_level);

    oled_clear();
    delay_ms(50);
    spi1_display1("Your turn!");


    current_input = 0;
    flush_buttons();   // Clear any previous button presses
    input_mode = true; // Set input mode to true


    delay_ms(50); // Optional debounce


    while (current_input < seq_len && !game_over)
    {
        uint8_t button = get_button_press();
        light_led(0); // LED OFF


        light_led(button); // Feedback LED ON
        delay_ms(300);
        light_led(0); // LED OFF


        // Log the user's input to the screen:
        oled_clear();
        delay_ms(10);
        spi1_display1("You pressed:");
        delay_ms(10);


        switch (button)
        {
        case 1:
            sprintf(buffer, "Blue");
            delay_ms(10);
            break;
        case 2:
            sprintf(buffer, "Green");
            delay_ms(10);
            break;
        case 3:
            sprintf(buffer, "Red");
            delay_ms(10);
            break;
        case 4:
            sprintf(buffer, "White");
            delay_ms(10);
            break;
        default:
            sprintf(buffer, "?");
            break;
        }


        spi1_display2(buffer); // Actually print the color name
        delay_ms(800);         // Let the player read the feedback


        // Check correctness:
        if (button != sequence[current_input])
        {
            game_over = true;
            input_mode = false;
            
            return; // Wrong input → exit immediately
        }
        else
        {
            current_input++; // Move to next input
            if (current_input >= seq_len)
            {
                input_mode = false; // Successfully completed level!
                success_feedback();
                current_level++;
                return;
            }
        }
    }
}

//if you pass the level, success and score
void success_feedback(void)
{
    success_fade_green();
    oled_clear();
    delay_ms(10);
    int current_score = score(current_level);
    char score_buf[20];
    sprintf(score_buf, "Score: %d", current_score);
    spi1_display2(score_buf);
    delay_ms(1000);
}

//game over, stops the game
void game_over_sequence(void)
{
    // Stop in‐game input
    input_mode = false;

    // 1) Show Game Over on the LCD
    oled_clear();
    delay_ms(100);                 // ← give the clear time to take effect
    spi1_display1("Game Over!");
    {
        char buf[20];
        sprintf(buf, "Score: %d", score(current_level-1));
        spi1_display2(buf);
    }
    delay_ms(1000);        


    failure_flash_red();
    error_beep();


    wait_for_restart();
    
}

//shows that you passed all levels
void win_sequence(void)
{
    input_mode = false;

    oled_clear();
    delay_ms(10);
    spi1_display1("Congratulations!");
    spi1_display2("All levels passed!");

    celebration_sequence(); 


    game_over = true;
    wait_for_restart();
}

//generates the game
void run_game_rounds(void)
{
    game_over = false;
    current_level = 1;


    while (!game_over)
    {
        generate_sequence();
        
        countdown();
        play_sequence();
        handle_user_input(); // This may increment current_level or set game_over


        if (game_over)
        {
            game_over_sequence();
            break;
        }
        else if (current_level > MAX_LEVEL)
        {
            win_sequence();
            break;
        }
    }
}

//waits for the user to press a button to restart game
void wait_for_restart(void)
{
    while (get_button_press() == 0)
    {
        // Wait here until any button is pressed
    }
    delay_ms(300); // Debounce
}



