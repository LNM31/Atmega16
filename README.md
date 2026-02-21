# ATmega16 Bare-Metal Embedded Applications

> A collection of bare-metal C applications for the ATmega16 microcontroller covering GPIO, timers, interrupts, UART serial communication, ADC, and 7-segment display driving — all written at the register level without abstraction libraries.

![C](https://img.shields.io/badge/Language-C-blue?logo=c&logoColor=white)
![AVR](https://img.shields.io/badge/MCU-ATmega16-red?logo=atmel&logoColor=white)
![Embedded](https://img.shields.io/badge/Embedded-Bare--Metal-orange)
![Atmel Studio](https://img.shields.io/badge/IDE-Atmel%20Studio%207-green)
![Clock](https://img.shields.io/badge/Clock-14.7456%20MHz-yellow)

---

## Overview

This repository contains **13 standalone applications** developed for the **ATmega16** microcontroller. Each program targets a specific hardware peripheral or embedded concept, progressing from basic GPIO manipulation to interrupt-driven UART and analog-to-digital conversion. All code operates directly on hardware registers — no HAL, no Arduino, no abstraction layers.

---

## Tech Stack

| Component | Detail |
|---|---|
| **Microcontroller** | ATmega16 (8-bit AVR, 16 KB Flash, 1 KB SRAM) |
| **Clock Source** | 14.7456 MHz external crystal |
| **Toolchain** | AVR-GCC + AVR Libc |
| **IDE** | Atmel Studio 7 |
| **Programming** | Direct register-level C (no HAL/Arduino) |

### Peripherals & Concepts Used

`GPIO` `Timer1 (16-bit)` `CTC Mode` `Interrupts (ISR)` `UART (USART)` `ADC` `7-Segment Display` `Polling` `Debouncing` `Volatile Flags` `Baud Rate Generation` `Prescaler Configuration`

---

## Applications

### 1. LED Blink — GPIO Fundamentals

**`42-LED-Blink/`** — Two variants of LED blinking using direct port manipulation.

| Variant | File | Blink Interval | Description |
|---|---|---|---|
| Assignment2 | `main.c` | 1000 ms | LED on PB0 toggles every 1 second |
| Assignment3 | `main.c` | 500 ms | LED on PB0 toggles every 500 ms |

**How it works:** Configures `DDRB` to set PB0 as output, then alternates `PORTB` high/low in an infinite loop with `_delay_ms()` for timing. Demonstrates the fundamental DDRx/PORTx register pattern for GPIO output control.

```c
DDRB  |= (1 << PB0);       // Data Direction: PB0 = output
PORTB |= (1 << PB0);       // Drive HIGH  (LED on)
PORTB &= ~(1 << PB0);      // Drive LOW   (LED off)
```

---

### 2. Push Button Input — GPIO with Debouncing

**`43-Push-Buttons/`** — Reading digital inputs with software debounce and state management.

| Variant | Buttons | LEDs | Description |
|---|---|---|---|
| Assignment1 | PD6 | PB0 | Toggle LED on/off with each button press |
| Assignment3 | PD5, PD6 | PB0–PB7 | Shift a lit LED left/right across 8 LEDs |

**Assignment1 — Toggle with Debounce:**
Reads the `PIND` register to detect a button press on PD6. Implements a two-stage debounce: first checks if the pin is low, waits 1 ms, checks again, then blocks until release before toggling. Uses a `static` variable to track toggle state.

```c
if ((PIND & (1 << PD6)) == 0)       // First read — button pressed?
{
    _delay_ms(1);                     // Debounce delay
    if ((PIND & (1 << PD6)) == 0)    // Second read — confirmed?
    {
        while ((PIND & (1 << PD6)) == 0);  // Wait for release
        led_toggle();
    }
}
```

**Assignment3 — LED Shift Register:**
Drives all 8 pins of PORTB as outputs, each connected to an LED. Two buttons on PD5 and PD6 shift the active LED left or right with circular wraparound using bitwise shift operations on the entire port register.

---

### 3. Timer/Counter — Compare Match & Interrupts

**`44-Timer-CompareMatch-Interrupts/`** — Three progressive implementations of Timer1 in CTC (Clear Timer on Compare) mode.

| Variant | Technique | ISR? | Description |
|---|---|---|---|
| Assignment1 | Polling | No | Polls the OCF1A flag in TIFR to detect compare match |
| Assignment2 | Interrupt | Yes | ISR toggles LED directly on compare match |
| Assignment3 | Interrupt + Flag | Yes | ISR sets a volatile flag; main loop handles the toggle |

**Timer Configuration (shared across all three):**

```c
// Timer1 CTC Mode — Compare value: 0x7080 (28,800 decimal)
// Clock: 14.7456 MHz / 256 prescaler = 57,600 Hz tick rate
// Match interval: 28,800 / 57,600 = 0.5 seconds

OCR1AH = (value >> 8) & 0xFF;   // Compare register high byte
OCR1AL = value & 0xFF;           // Compare register low byte
TCCR1B |= (1 << WGM12) | (1 << CS12);  // CTC mode + prescaler 256
```

**Assignment1 — Polling approach:** The main loop continuously checks `TIFR & (1 << OCF1A)` and manually clears the flag by writing 1 to it. Simple but CPU-bound.

**Assignment2 — Direct ISR:** Enables global interrupts with `sei()` and the Timer1 compare match interrupt via `TIMSK |= (1 << OCIE1A)`. The `ISR(TIMER1_COMPA_vect)` handler toggles the LED directly — the main loop is empty.

**Assignment3 — Flag-based ISR (best practice):** The ISR only sets a `volatile` flag (`LEDF`). The main loop checks the flag, performs the toggle, and clears it. This pattern minimizes time spent in the interrupt context and is the recommended approach for real embedded systems.

---

### 4. 7-Segment Display — Interrupt-Driven Multiplexing

**`45-2Digits-7SegmentDisplay/`** — Drives a common-anode 7-segment display with timer-based digit cycling.

**How it works:** Uses PORTA (7 pins) to drive the segment lines of the display. A lookup table maps digits 0–9 to their 7-segment encoding. Timer1 fires an interrupt at a fixed interval; each time the ISR sets a flag, the main loop advances to the next digit.

```c
// 7-segment encoding (common anode — active LOW)
static const uint8_t digitmap[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99,   // 0, 1, 2, 3, 4
    0x92, 0x82, 0xF8, 0x80, 0x90    // 5, 6, 7, 8, 9
};
```

| Segment | a | b | c | d | e | f | g |
|---|---|---|---|---|---|---|---|
| **Port Pin** | PA0 | PA1 | PA2 | PA3 | PA4 | PA5 | PA6 |

---

### 5. UART Serial Communication

**`46-UART/`** — Four applications demonstrating UART from basic transmission to interrupt-driven receive.

| Variant | Baud Rate | Direction | Technique | Description |
|---|---|---|---|---|
| Assignment1 | 9600 | TX only | Polling | Transmits character `'a'` every 1 second |
| Assignment5 | 115200 | TX only | Polling | Transmits formatted counter string (`"some text N"`) |
| Assignment7 | 115200 | TX + RX | Blocking | Echo — waits for received byte, immediately retransmits |
| Assignment9 | 115200 | TX + RX | Interrupt | Echo — ISR captures byte into volatile, main loop retransmits |

**UART Register Configuration:**

```c
// Baud rate: UBRR = F_OSC / (16 * BAUD) - 1
// At 14.7456 MHz: 9600 baud -> UBRR = 95, 115200 baud -> UBRR = 7
// (14.7456 MHz chosen specifically for 0% baud rate error)

UBRRH = (unsigned char)(ubrr >> 8);       // Baud rate high byte
UBRRL = (unsigned char) ubrr;              // Baud rate low byte
UCSRB |= (1 << TXEN) | (1 << RXEN);      // Enable TX and RX
UCSRC |= (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);  // 8-bit data, 1 stop, no parity
```

**Assignment7 — Blocking Echo:** Uses `while(!(UCSRA & (1 << RXC)))` to block until a byte arrives, then immediately transmits it back. Simple but blocks the CPU entirely.

**Assignment9 — Interrupt Echo:** Enables the UART RX Complete interrupt (`RXCIE`). The `ISR(USART_RXC_vect)` reads `UDR` into a volatile variable and sets a flag. The main loop checks the flag and transmits — fully non-blocking.

---

### 6. ADC — Analog-to-Digital Conversion

**`48-ADC/`** — Reads an analog voltage on PA0, converts it to millivolts, and transmits the result over UART.

**How it works:**
1. Configures the ADC module: enables it via `ADEN`, sets prescaler for proper sampling clock, selects AVCC as voltage reference via `ADMUX`
2. Triggers a single conversion by setting `ADSC`, then polls until the conversion completes
3. Reads the 10-bit result from `ADCL`/`ADCH` (low byte first, per datasheet requirement)
4. Converts the raw ADC value to millivolts: `value = result * (AVCC / 1023)`
5. Formats the result as a string and transmits it over UART at 115200 baud

```c
ADCSRA |= (1 << ADEN);                    // Enable ADC
ADCSRA |= (1 << ADPS1) | (1 << ADPS0);   // Prescaler = 8
ADMUX  |= (1 << REFS0);                   // Reference = AVCC (5V)

ADCSRA |= (1 << ADSC);                    // Start conversion
while ((ADCSRA & (1 << ADSC)) != 0);      // Wait for completion

result  = (uint16_t)ADCL;                 // Read low byte first
result |= (uint16_t)(ADCH << 8);          // Then high byte
```

---

## Peripheral Usage Map

| Application | GPIO | Timer1 | CTC | ISR | UART TX | UART RX | ADC | 7-Seg |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| LED Blink | x | | | | | | | |
| Push Buttons | x | | | | | | | |
| Timer — Polling | x | x | x | | | | | |
| Timer — ISR Direct | x | x | x | x | | | | |
| Timer — ISR Flag | x | x | x | x | | | | |
| 7-Segment Display | x | x | x | x | | | | x |
| UART Basic TX | x | | | | x | | | |
| UART String TX | x | | | | x | | | |
| UART Blocking Echo | x | | | | x | x | | |
| UART Interrupt Echo | x | | | x | x | x | | |
| ADC Voltage Reader | x | | | | x | | x | |

---

## Pin Configuration

```
                    ATmega16 DIP-40
                   ┌────────────────┐
          (PB0) ── │ 1   PB0  PA0 40│ ── ADC Channel 0 / 7-Seg a
          (PB1) ── │ 2   PB1  PA1 39│ ── 7-Seg b
          (PB2) ── │ 3   PB2  PA2 38│ ── 7-Seg c
          (PB3) ── │ 4   PB3  PA3 37│ ── 7-Seg d
          (PB4) ── │ 5   PB4  PA4 36│ ── 7-Seg e
          (PB5) ── │ 6   PB5  PA5 35│ ── 7-Seg f
          (PB6) ── │ 7   PB6  PA6 34│ ── 7-Seg g
          (PB7) ── │ 8   PB7  PA7 33│
                   │ 9   RST AREF 32│ ── AVCC Reference
                   │10   VCC  GND 31│
                   │11   GND AVCC 30│ ── 5V Analog Supply
          XTAL1 ── │12  XTAL2 PC7 29│
          XTAL2 ── │13  XTAL1 PC6 28│
       UART RX  ── │14   PD0  PC5 27│
       UART TX  ── │15   PD1  PC4 26│
                   │16   PD2  PC3 25│
                   │17   PD3  PC2 24│
                   │18   PD4  PC1 23│
        Button  ── │19   PD5  PC0 22│
        Button  ── │20   PD6  PD7 21│
                   └────────────────┘

   PB0      : Status LED (all LED apps)
   PB0–PB7  : 8-LED array (shift register app)
   PA0–PA6  : 7-segment display segments
   PA0      : ADC analog input
   PD0      : UART RX
   PD1      : UART TX
   PD5, PD6 : Push button inputs (active LOW)
```

---

## Project Structure

```
Atmega16/
├── 42-LED-Blink/
│   ├── Assignment2/              # LED blink — 1s interval
│   │   └── Assignment2/main.c
│   └── Assignment3/              # LED blink — 500ms interval
│       └── Assignement3/main.c
├── 43-Push-Buttons/
│   ├── Assignment1/              # Single button toggle with debounce
│   │   └── Assignment1/main.c
│   └── Assignment3/              # Dual-button 8-LED shift
│       └── Assignment3/main.c
├── 44-Timer-CompareMatch-Interrupts/
│   ├── Assignment1/              # Timer1 CTC — polling OCF1A flag
│   │   └── Assignment1/
│   │       ├── main.c
│   │       └── Timer-Header/
│   │           ├── timer.h
│   │           └── timer.c
│   ├── Assignment2/              # Timer1 CTC — ISR toggles LED directly
│   │   └── Assignment2/
│   │       ├── main.c
│   │       └── Timer-Header/
│   │           ├── timer.h
│   │           └── timer.c
│   └── Assignment3/              # Timer1 CTC — ISR sets flag, main loop acts
│       └── Assignment2/
│           ├── main.c
│           └── Timer-Header/
│               ├── timer.h       # Exports volatile LEDF flag
│               └── timer.c       # Defines volatile LEDF flag
├── 45-2Digits-7SegmentDisplay/
│   └── Assignment1/              # Timer-driven 7-segment digit counter
│       └── Assignment1/main.c
├── 46-UART/
│   ├── Assignment1/              # Basic TX — single char at 9600 baud
│   │   └── Assignment1/main.c
│   ├── Assignment5/              # String TX with counter at 115200 baud
│   │   └── Assignment5/main.c
│   ├── Assignment7/              # Blocking echo (RX → TX)
│   │   └── Assignment7/main.c
│   └── Assignment9/              # Interrupt-driven echo (RX ISR → TX)
│       └── Assignment9/main.c
└── 48-ADC/
    └── GccApplication1/          # ADC read → mV conversion → UART output
        └── GccApplication1/main.c
```

---

## Key Embedded Concepts Demonstrated

| Concept | Where Applied |
|---|---|
| **Register-level GPIO** | DDRx for direction, PORTx for output, PINx for input |
| **Software debouncing** | Double-read with delay + wait-for-release in push button apps |
| **Timer CTC mode** | TCCR1B WGM12 bit, OCR1A compare register configuration |
| **Polling vs. Interrupts** | Timer apps progress from polling TIFR to ISR-driven design |
| **Volatile flag pattern** | ISR sets minimal flag, main loop acts — avoids long ISR execution |
| **Baud rate calculation** | UBRR = F_OSC/(16*BAUD)-1 with crystal chosen for 0% error |
| **UART framing** | 8N1 config via UCSRC (UCSZ1:0 for 8-bit, defaults for 1 stop, no parity) |
| **ADC conversion** | Single-shot mode, AVCC reference, 10-bit result assembly from ADCL/ADCH |
| **Modular timer library** | Separate timer.h/timer.c with extern volatile shared state |
