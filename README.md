# 🎮 Simon Says on STM32 with RGB LEDs + PWM

This project implements a classic **Simon Says memory game** using an STM32F0 microcontroller. It features four RGB LEDs controlled by **PWM** and four push buttons for user input. A buzzer (via DAC output) gives audio feedback for incorrect guesses. A potentiometer (via ADC) controls output volume.

---

## 🧠 Features

- ✅ RGB LED control using **PWM** across multiple timers (`TIM2`, `TIM3`, `TIM14–17`)
- ✅ Four-button input with **debounced** polling
- ✅ **Buzzer sound** generation using DAC and a square wave wavetable
- ✅ Potentiometer-based **volume control** using ADC
- ✅ Full gameplay logic for pattern generation, user response, and level progression

---

## 🧰 Hardware

- STM32 Nucleo Board (F091RC or compatible)
- 4x RGB LEDs (common cathode recommended)
- 4x Push buttons
- 1x Potentiometer (connected to PA1)
- 1x Buzzer (connected to DAC output on PA4)

---

## 📌 Pin Map

### Buttons (Input)

| Color   | Pin     |
|---------|---------|
| Green   | PB0     |
| Orange  | PB1     |
| Red     | PB2     |
| Blue    | PB3     |

### RGB LED Channels (PWM Output)

| LED     | Channel         | Timer   | GPIO   |
|---------|------------------|---------|--------|
| LED1    | R/G/B            | TIM3    | PC6–8  |
| LED2    | R/G/B            | TIM3/TIM15 | PC9, PA14, PA1 |
| LED3    | R/G/B            | TIM2/TIM16 | PA2, PA3, PA6 |
| LED4    | R/G/B            | TIM14/16/17 | PB4, PB5, PB6 |

### Audio + Volume

| Function     | Pin   | Peripheral |
|--------------|-------|------------|
| Buzzer Out   | PA4   | DAC        |
| Volume Input | PA1   | ADC        |

---

## 🔁 Game Flow

1. Game initializes all hardware.
2. A random LED sequence is shown.
3. Player repeats the pattern using buttons.
4. If correct, the level increases.
5. If wrong, the buzzer sounds and game resets.

---

## 📂 Project Structure

- `main.c` - core game loop, logic, and LED control
- `PWM_Configure()` - initializes all timers for LED control
- `set_led_brightness()` - sets per-color brightness via PWM
- `light_led_color()` - combines brightness values to light specific LED/color
- `play_sequence()` - plays current level's pattern
- `get_button_press()` - handles user input with debouncing
- `error_beep()` - plays tone on failure using DAC + wavetable

---

## 🧪 How to Test

- Confirm LEDs are wired to correct timer channels
- Connect buttons with pull-up logic (internal or external)
- Use a potentiometer on PA1 for real-time volume adjustment
- Watch USART5 debug output on PC12/PD2 (115200 baud)
- Check that each LED lights one at a time during sequence

---

## 🏁 Success Criteria

- Game reliably runs through 10 rounds
- Buttons detect correct vs incorrect input
- LED feedback matches intended colors
- Buzzer activates on error
- Game resets cleanly after finish or failure

---

## 📚 Dependencies

This project uses the **STM32Cube HAL** framework. All peripheral initialization is done through HAL functions, and no deprecated SPL calls are used.

PlatformIO build flags (in `platformio.ini`
