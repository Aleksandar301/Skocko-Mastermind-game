# Skocko Game on STM32 Nucleo-L476RG

A hardware implementation of the classic **Skocko (Mastermind)** game on the **STM32 Nucleo-L476RG** board, featuring:

- **OLED display (SSD1306)** for text feedback
- **7-segment display** for live digit selection
- **Buttons SW1 & SW2** for locking digits and submitting guesses
- **Random number generation** for secret combination
- **ADC potentiometer input** for selecting digits

The project was initially implemented using **polling** as the microcontroller used was quite  fast for the project at hand, and later upgraded to **interrupt-driven** handling for buttons and ADC. The presented version is the pooling version.

---

## Table of Contents

- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [Game Rules](#game-rules)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Features](#features)
- [Future Improvements](#future-improvements)
- [License](#license)

---

## Hardware Setup

- **STM32 Nucleo-L476RG** board
- **SSD1306 OLED** connected via I2C2
- **7-segment display** connected to GPIO pins:
  - Segments: `SEG_A` to `SEG_G`
  - Digit selectors: `SEL1`, `SEL2`
- **Buttons**:
  - `SW1` → Lock current digit
  - `SW2` → Submit guess / continue
- **Potentiometer** connected to `ADC1 Channel 6` for selecting current digit

---

## Software Setup

1. Install **STM32CubeIDE** (latest version recommended).
2. Clone this repository:
   ```bash
   git clone https://github.com/<your-username>/<repo-name>.git
