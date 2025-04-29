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
#define MAX_SEQUENCE_LENGTH 8 // or whatever you'd like the max to be
#define MAX_LEVEL 10
#define LED_ON_TIME 500   // milliseconds
#define BETWEEN_TIME 200  // milliseconds
#define DEBOUNCE_DELAY 50 // milliseconds

// ============================================
//              GLOBAL VARIABLES
// ============================================
uint8_t sequence[MAX_LEVEL];
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
void init_tim2(void);
void init_tim3(void);
void init_tim15(void);

// RGB
void PWM_Configure_RGB(void);
void success_fade_green(void);
void failure_flash_red(void);
void celebration_sequence(void);
void set_rgb_color(uint16_t red, uint16_t green, uint16_t blue);


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
    init_spi1();
    spi1_init_oled();


    while (1)
    {
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
    GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)) |
                      (3U << (10 * 2)) | (3U << (11 * 2)));


    // ------------------------------
    // Configure PB0–PB3 (buttons) as input with pull-up
    // ------------------------------
    // PB0–PB3 (buttons) as input with pull-up
    GPIOB->MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2)) |
                      (3U << (2 * 2)) | (3U << (3 * 2))); // Input mode (00)


    
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
void set_rgb_color(uint16_t red, uint16_t green, uint16_t blue)
{
    TIM3->CCR1 = red;   // PC6 - Red
    TIM3->CCR2 = green; // PC7 - Green
    TIM3->CCR3 = blue;  // PC8 - Blue
}
void success_fade_green(void)
{
    for (uint16_t i = 0; i < 1000; i += 10)
    {
        set_rgb_color(0, i, 0); // Green fade
        delay_ms(10);
    }
    set_rgb_color(0, 0, 0); // Turn off
}
void failure_flash_red(void)
{
    for (int i = 0; i < 5; i++)
    {
        set_rgb_color(1000, 0, 0); // Red ON
        delay_ms(200);
        set_rgb_color(0, 0, 0); // OFF
        delay_ms(200);
    }
}
void celebration_sequence(void)
{
    for (int i = 0; i < 10; i++)
    {
        set_rgb_color(1000, 0, 0); // Red
        delay_ms(200);
        set_rgb_color(0, 1000, 0); // Green
        delay_ms(200);
        set_rgb_color(0, 0, 1000); // Blue
        delay_ms(200);
        set_rgb_color(1000, 1000, 0); // Yellow
        delay_ms(200);
        set_rgb_color(0, 1000, 1000); // Cyan
        delay_ms(200);
        set_rgb_color(1000, 0, 1000); // Magenta
        delay_ms(200);
        set_rgb_color(0, 0, 0); // OFF
    }
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


// Button Helpers
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
void wait_for_game_start(void)
{
    oled_clear();
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
void countdown(void)
{
    oled_clear();
    delay_ms(1000); // Pause before countdown
    spi1_display1("Starting game...");
    delay_ms(1000); // Pause before countdown


    for (int i = 5; i >= 1; i--)
    {
        char buffer[20];
        sprintf(buffer, "Starting in: %d", i);
        spi1_display1(buffer);
        delay_ms(1000); // Give the LCD time to actually show it!
    }
    oled_clear();
    delay_ms(1000); // Pause before GO
    spi1_display1("GO!");
    delay_ms(1000); // Give the player a moment after "GO!"
}
int sequence_length_for_level(int level)
{
    int length = STARTING_SEQUENCE_LENGTH + (level - 1);
    if (length > MAX_SEQUENCE_LENGTH)
        length = MAX_SEQUENCE_LENGTH;
    return length;
}
void generate_sequence(void)
{
    // Hardcoded pattern for Level 1:
    sequence[0] = 4; // White
    sequence[1] = 3; // Red
    sequence[2] = 2; // Green
    sequence[3] = 1; // Blue


    // If you want random sequences for higher levels:
    for (int i = 4; i < MAX_LEVEL; i++)
    {
        sequence[i] = (rand() % 4) + 1;
    }
}
void play_sequence(void)
{
    char buffer[20];
    int seq_len = sequence_length_for_level(current_level);


    oled_clear();
    sprintf(buffer, "Level %d:", current_level);
    spi1_display1(buffer);
    delay_ms(1000); // Show level


    // 🟢 Ensure ALL LEDs are OFF at the start:
    light_led(5);
    oled_clear();


    for (int i = 0; i < seq_len; i++)
    {
        // Ensure LEDs are OFF before showing each one:
        light_led(0);
        spi1_display1("Simon Says:");
        delay_ms(1000); // Pause between sequence steps
        oled_clear();
        // Show the next LED and its label:
        switch (sequence[i])
        {
        case 1:
            light_led(1);
            spi1_display2("Blue");
            break;
        case 2:
            light_led(2);
            spi1_display2("Green");
            break;
        case 3:
            light_led(3);
            spi1_display2("Red");
            break;
        case 4:
            light_led(4);
            spi1_display2("White");
            break;
        }


        delay_ms(1000); // LED ON time
        light_led(0);   // Turn OFF after each flash!
    }


    oled_clear();
    delay_ms(1000); // Pause between sequence steps
    spi1_display1("Your turn!");
    delay_ms(500);
}
void handle_user_input(void)
{
    char buffer[20];
    int seq_len = sequence_length_for_level(current_level);


    oled_clear();
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
        spi1_display1("You pressed:");


        switch (button)
        {
        case 1:
            sprintf(buffer, "Blue");
            break;
        case 2:
            sprintf(buffer, "Green");
            break;
        case 3:
            sprintf(buffer, "Red");
            break;
        case 4:
            sprintf(buffer, "White");
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
void success_feedback(void)
{
    for (int i = 0; i < 3; i++)
    {
        light_led(1);
        light_led(2);
        light_led(3);
        light_led(4);
        delay_ms(100);
        light_led(0);
        delay_ms(100);
    }
    int current_score = score(current_level);
    char score_buf[20];
    sprintf(score_buf, "Score: %d", current_score);
    spi1_display2(score_buf);
}
void game_over_sequence(void)
{
    input_mode = false;
    error_beep();


    oled_clear();
    spi1_display1("Game Over!");
    int current_score = score(current_level);
    char score_buf[20];
    sprintf(score_buf, "Score: %d", current_score);
    spi1_display2(score_buf);


    failure_flash_red(); // 🟢 Replace GPIO toggling with your RGB helper


    set_rgb_color(0, 0, 0); // Make sure RGB turns OFF after flashing


    wait_for_restart();
}
void win_sequence(void)
{
    input_mode = false;


    oled_clear();
    spi1_display1("Congratulations!");
    spi1_display2("All levels passed!");


    celebration_sequence(); // 🟢 Replace game LED flashing with your RGB helper


    set_rgb_color(0, 0, 0); // Ensure RGB turns OFF at the end


    game_over = true;
    wait_for_restart();
}


void run_game_rounds(void)
{
    game_over = false;
    current_level = 1;


    while (!game_over)
    {
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
void wait_for_restart(void)
{
    while (get_button_press() == 0)
    {
        // Wait here until any button is pressed
    }
    delay_ms(300); // Debounce
}



