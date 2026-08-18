# lwipenc

lwipenc is a lightweight ENC28J60 Ethernet driver and example application that integrates the Microchip ENC28J60 (SPI Ethernet controller) with LwIP and libopencm3-based firmware. The project includes a minimal TCP echo server (port 7) and demonstrates using the ENC28J60 as a network interface with DHCP.

This repository contains:

- enc28j60.c / enc28j60.h / enc28j60.glue.c — ENC28J60 driver and platform glue code
- main.c — example application: lwIP TCP echo server and initialization
- Makefile / mk/ — build infrastructure that ties in libopencm3, lwIP and project-specific make fragments
- submodules referenced in the tree: libopencm3, lwip, printf, rtt, bgrt (real-time kernel used by this project)

Features
- ENC28J60 MAC + PHY driver adapted for LwIP
- SPI DMA-based transfers and external interrupt handling (platform glue in enc28j60.glue.c)
- Example echo server (TCP) that uses DHCP to obtain an IP address

Requirements
- GNU make
- arm-none-eabi toolchain (gcc, ld, objcopy) or a compatible cross-compiler
- git (to initialize submodules) 
- Submodules used by the build: libopencm3, lwip, printf, rtt, bgrt/bugurt (these are expected as git submodules)

Quick build
1. Clone the repository and initialize submodules:

   git clone --recurse-submodules https://github.com/beefdeadbeef/lwipenc.git
   cd lwipenc

   If you forgot --recurse-submodules, run:

   git submodule update --init --recursive

2. Build:

   make

   The build produces the firmware artifacts named `fw.elf` and `fw.bin` as configured by the Makefile.

Flashing
- The exact flash procedure depends on your board. The produced `fw.bin` and `fw.elf` can be used with OpenOCD, a vendor programmer tool, or any supported flashing utility for your board.

Supported hardware / pinout
- This project targets an AT32F403-style device (Makefile DEVICE = at32f403acgu / libopencm3 target `at32/f40x`).
- ENC28J60 wiring used by enc28j60.glue.c (GPIOB pins):
  - SCK  : PB4
  - NSS  : PB6
  - SCKIN: PB7
  - MISO : PB8
  - MOSI : PB9
  - EXTI (interrupt from ENC28J60): PB5 (falling edge)

Behavior
- On boot the firmware runs a tiny network stack (LwIP) and requests an IP address using DHCP.
- A TCP echo server listens on port 7 and echoes received data back to clients.

Notes and credits
- ENC28J60 driver originally derived from Linux driver sources. LwIP adaptation and project integration by Sergey Bolshakov <beefdeadbeef@gmail.com> (see source headers).
- The code uses libopencm3 for MCU support and a small real-time kernel (bgrt/bugurt) for threading primitives.

License
- Files in this repository are provided under GPL-2.0 (see SPDX headers in source files). By contributing or using this code you agree to the terms of that license.

Contributing
- Please open issues or pull requests for bug reports, improvements or platform ports.
- When submitting patches, follow the existing style and add a short description and testing notes.

Contact
- For questions or patches you can reach the original author shown in source headers or open an issue in this repository.
