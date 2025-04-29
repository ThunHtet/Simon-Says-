# STM32 Simon Says Memory Game

Welcome to our ECE 362 final project — a fully-featured **Simon Says memory game**, implemented on the STM32F091RC Nucleo board using C and low-level hardware peripherals.

This project tests a player's memory by generating a flashing LED pattern that they must repeat using button presses. If they succeed, they move to the next level with longer sequences and increased difficulty. If they fail, a buzzer and RGB LED indicate failure, and the game restarts.

---

## Objectives Overview

### 1. LED Sequence Control (Ananya)

**Goal**: Flash single-color LEDs and RGB LEDs using GPIO and PWM to generate the game sequence and provide feedback.

- **Hardware**: PB8–PB11 (Game LEDs), PC6–PC8 (RGB LED)
- **Peripherals Used**: GPIO, TIM1/TIM3 (PWM)
- **Success Criteria**:
  - LEDs must flash in a consistent, repeatable sequence.
  - Sequences get longer as the user progresses through levels.
  - PWM on the RGB LED enables smooth color transitions (e.g., green fade on success, red flash on failure).

✅ Achieved: PWM-controlled RGB feedback, per-color LED flashing during sequence playback, with clearly timed visibility.

---

### 2. Keypad Input Handling (Mackenzie)

**Goal**: Detect user input from physical push buttons and compare it against the expected LED sequence.

- **Hardware**: PB0–PB3 (Push buttons)
- **Peripherals Used**: GPIO (input + pull-up), debounce logic
- **Success Criteria**:
  - Button presses must be debounced and matched reliably to game logic.
  - Player input must be validated in real time.
  - The game must proceed if input is correct or end on error.

✅ Achieved: Fully debounced input, flushed after startup, with user feedback shown on the OLED after every press (e.g., "You pressed: Green").

---

### 3. Dynamic Difficulty Scaling (Jason)

**Goal**: Make the game more challenging with each level by increasing sequence length and speed.

- **Mechanism**: Level-based logic, timer-based delays
- **Success Criteria**:
  - Starting with a hardcoded level 1 sequence.
  - Subsequent levels use dynamically generated sequences with increasing lengths.
  - Difficulty scales up to a defined maximum (MAX_LEVEL = 6).

✅ Achieved: Levels scale from 4 to 8 steps, each level more difficult than the last. Sequence playback uses consistent countdown and pacing.

---

### 4. Audio Feedback for Input (Austin)

**Goal**: Provide audio cues using a DAC-connected speaker to signal success or failure.

- **Hardware**: Speaker connected to DAC
- **Peripherals Used**: DAC, TIM6 interrupt for waveform generation
- **Success Criteria**:
  - Distinct tone on incorrect input.
  - Option for celebratory tone at the end of the game.

✅ Achieved: Interrupt-driven DAC output plays a buzzer tone on loss, integrated with failure state and visual red flash.

---

## 🎮 How It Works

1. **Startup**:

   - The OLED prompts the player to press the blue button to begin.

2. **Countdown**:

   - A short countdown is shown: “Starting in 5…” through “GO!”

3. **Sequence Playback**:

   - The game displays a sequence of LEDs (e.g., White → Red → Green → Blue) using PB8–PB11.
   - Each LED is labeled on the OLED for clarity.

4. **User Turn**:

   - Player must press buttons (PB0–PB3) in the same order.
   - Correct input = green fade + next round.
   - Incorrect input = buzzer, red flash, and game over.

5. **Win State**:
   - Completing all levels triggers a celebratory RGB light show and message.

---

## Build & Flash Instructions

> This project uses [PlatformIO](https://platformio.org/) in VS Code.

1. Clone the repo and open in VS Code with PlatformIO installed.
2. Plug in the STM32 Nucleo board via USB.
3. Build the firmware:
   ```bash
   pio run
   ```
