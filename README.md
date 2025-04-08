<h1 align="center">Simon Says 🎮 – Embedded Game on STM32</h1>

<p align="center">
  A memory game built on the STM32F0 using PWM-controlled LEDs and HAL. <br>
  Built with PlatformIO, using STM32Cube HAL, TIM3 PWM, and GPIO inputs.
</p>

<hr>

<h2>Project Overview</h2>

<ul>
  <li>Four colored LEDs (Green, Orange, Red, Blue) display a randomized light sequence.</li>
  <li>Four buttons let the player input their guess to match the pattern.</li>
  <li>Game becomes progressively harder as the sequence grows each round.</li>
  <li>Feedback is shown via LED flash animations.</li>
  <li>Built entirely with STM32 HAL drivers and PWM on TIM3.</li>
</ul>

<h2>🔧 Hardware Requirements</h2>

<ul>
  <li>STM32 Nucleo-F0 board (e.g., Nucleo-F091RC)</li>
  <li>4 push buttons (GPIOA pins PA0–PA3)</li>
  <li>4 LEDs (GPIOC pins PC8–PC11)</li>
  <li>Optional: DAC and speaker for sound feedback</li>
  <li>PlatformIO + STM32CubeF0 support</li>
</ul>

<h2>🛠 Technologies Used</h2>

<ul>
  <li><strong>STM32Cube HAL</strong> - Hardware Abstraction Layer</li>
  <li><strong>TIM3</strong> - Timer for PWM</li>
  <li><strong>SysTick</strong> - Timing and delays</li>
  <li><strong>PlatformIO</strong> - Build and upload environment</li>
</ul>

<h2>🚦 Game Logic</h2>

<ol>
  <li>Game initializes and generates a random LED sequence.</li>
  <li>LEDs are flashed using PWM to show the pattern.</li>
  <li>Player replicates the sequence using buttons.</li>
  <li>
    Correct input:
    <ul>
      <li>Moves to the next round.</li>
      <li>Pattern gets longer.</li>
    </ul>
  </li>
  <li>
    Incorrect input:
    <ul>
      <li>Triggers game over flash sequence.</li>
    </ul>
  </li>
  <li>
    If player reaches the maximum level:
    <ul>
      <li>Flashes victory pattern.</li>
    </ul>
  </li>
</ol>

<h2>🧩 Key Functions</h2>

<ul>
  <li><code>GPIO_Configure()</code> - Configures button and LED pins</li>
  <li><code>PWM_Configure()</code> - Sets up TIM3 for PWM output</li>
  <li><code>generate_sequence()</code> - Fills pattern with random values</li>
  <li><code>play_sequence()</code> - Plays current pattern via LEDs</li>
  <li><code>get_button_press()</code> - Captures debounced input</li>
  <li><code>light_led()</code> - Controls LED brightness via PWM</li>
</ul>
