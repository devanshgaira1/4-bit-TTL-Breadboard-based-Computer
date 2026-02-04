# 🧠 4-bit TTL Breadboard-Based Computer

*A fully discrete 4-bit TTL computer built on breadboards using 74-series logic ICs, with Arduino-assisted RAM/ROM emulation.*

---

## 📌 Overview

This project documents the design and construction of a **4-bit, bus-based CPU** built entirely from **74-series TTL logic ICs**, spread across **seven full-size breadboards**.

The goal of this build is **not performance or compactness**, but **hands-on learning** — understanding how a CPU works at the signal, clock, and bus level using real hardware.

---

## 💡 Design Inspiration

The design is inspired by classic educational CPUs (Ben Eater–style, *NAND to Tetris* concepts), and extends them with:

- **Shared 4-bit data bus** using `74LS245` transceivers  
- **74LS181 ALU** supporting arithmetic and logical operations  
- **Split registers (low/high nibble)** to scale beyond trivial demos  

---

## ▶️ Execution Modes

The CPU supports **two execution modes**:

### 🔹 Manual / Button-Driven Mode
- Single-step execution using pushbuttons  
- DIP switches for instruction and data entry  
- Ideal for debugging and learning  

### 🔹 Arduino-Controlled Mode
- Arduino acts as **ROM, RAM, and instruction sequencer**  
- CPU fetches instructions directly from Arduino  
- Allows running full programs without DIP switches  

---

## ✅ What This CPU *Is* (and Isn’t)

### This CPU **is**:
- Fully synchronous  
- TTL-based and breadboarded  
- Designed for learning, debugging, and experimentation  

### This CPU **is not**:
- Optimized  
- Compact  
- Neat  

And that’s **exactly the point**.

---

## 🧩 Breadboard Layout Overview (BB1–BB7)

This CPU is built across **seven breadboards**, each dedicated to a subsystem:

| Breadboard | Function |
|-----------|---------|
| **BB1** | Power & I/O – +5 V rails, DIP switches, LEDs |
| **BB2** | Clock Generator – manual STEP + NE555 auto clock |
| **BB3** | Program Counter – 4-bit counter (74LS161) |
| **BB4** | Instruction Register & Decoder |
| **BB5** | Register A (Accumulator) |
| **BB6** | Register B |
| **BB7** | ALU & Bus Interface (74LS181) |

All boards are connected via:
- A **shared 4-bit data/address bus**
- **Control lines** (CLK, RESET, ENABLE, etc.)

⚠️ **Important:**  
Floating TTL inputs are extremely noise-sensitive.  
Every IC **must** have:
- Unused inputs tied to `5V` or `GND`
- A `0.1 µF` decoupling capacitor close to VCC/GND

---

## 🕹️ Option 1: Manual Control Mode

In this mode:
- Instructions are entered using DIP switches
- CPU advances one cycle per **STEP** button press
- **RESET** clears PC and registers
- **HALT** stops execution until reset

### Wiring Notes
- STEP button must generate a **single clean pulse**
- RESET must asynchronously clear:
  - Program Counter
  - Registers A and B
- Unused control lines must be tied to known states

---

## 🤖 Option 2: Arduino-Controlled Mode

In this mode:
- Arduino emulates **ROM + RAM**
- Arduino outputs instructions on the data bus
- CPU executes instructions normally

### Typical Connections
- Arduino → Address Bus (A0–A3)
- Arduino → Data Bus (D0–D3)
- Arduino → Clock / WAIT / READY lines
- Arduino GND connected to CPU GND

The Arduino monitors memory read signals and places the correct instruction nibble on the bus.

---

## 🧪 Breadboard-by-Breadboard Details

### 🔌 BB1: Power & I/O Board
**Goal:** Stable power distribution and manual inputs.

- `74LS245` – bus buffering
- `74LS04` – signal cleanup
- DIP switches with pull-down resistors
- LEDs for bus monitoring
- RESET & HALT buttons
- One `0.1 µF` capacitor per IC

*(Detailed wiring retained as-is)*

---

### ⏱️ BB2: Clock Generator
- NE555 for automatic clock
- STEP button for manual pulses
- `74LS00` NAND gates for gating
- AUTO / MANUAL selector switch

---

### 🔢 BB3: Program Counter
- `74LS161` 4-bit counter
- `74LS245` to drive address bus
- CLR tied to RESET
- CLK tied to system clock

---

### 📥 BB4: Instruction Register & Decoder
- `74LS173` instruction latch
- Logic gates (`74LS08`, `74LS32`, `74LS04`) for decoding
- HALT signal gates the clock

---

### 🅰️ BB5: Register A
- `74LS173` accumulator
- `74LS245` bus interface
- Controlled by `LOAD_A` and `A_OUT`

---

### 🅱️ BB6: Register B
- Same structure as Register A
- Controlled by `LOAD_B` and `B_OUT`

---

### ➕ BB7: ALU & Bus Interface
- `74LS181` ALU
- `74LS245` to drive ALU result onto bus
- Supports arithmetic & logic modes
- Optional flags: Carry, Zero, Overflow

---

## 📜 Instruction Set

| Opcode | Mnemonic | Description | Effect |
|------|---------|------------|--------|
| 0x0 | LDI | Load Immediate → A | A ← operand |
| 0x1 | LDB | Load Immediate → B | B ← operand |
| 0x2 | ADD | Add | A ← A + B |
| 0x3 | SUB | Subtract | A ← A − B |
| 0x4 | AND | Bitwise AND | A ← A AND B |
| 0x5 | OR | Bitwise OR | A ← A OR B |
| 0x6 | XOR | Bitwise XOR | A ← A XOR B |
| 0x7 | NOT | Bitwise NOT | A ← NOT A |
| 0xF | HALT | Halt execution | Stop clock |

---

## 🧠 Arduino Integration (Example)

```cpp
const uint8_t program[] = { 0x0, 0x1, 0x2, 0xF };

void loop() {
  if (digitalRead(MREQ) == LOW && digitalRead(RD) == LOW) {
    uint8_t addr = readAddressBus();
    writeDataBus(program[addr]);
  }
}
