# Nebula-Core

This is the repository from Nebula Core Microarchitecture from Alchemist RV SoC.

## Overview

The Nebula Core is a RISC-V Core that uses RV64I ISA and Dual ALU Operations with Dual Issue (Two instructions per clock). The core is a Little Core with Low Power Efficiency.

## Features

* **Instruction Set Architecture (ISA):** RISC-V (RV64I)
* **Pipeline:** 8 stage in-order
* **Additional Resources:**
  * Cache L1 and L2
  * Linux Support (RV64)

## Repository Structure
/
├── doc/                # Project Documentation
├── rtl/                # Source-code in SystemVerilog/VHDL
│   ├── core/           # The Core code
│   └── tb/             # Testbenches
├── tools/              # Auxiliary tools and scripts
└── README.md           # This file

## License
This project is licensed under a license (MIT). See the LICENSE file for more details. Free all for use.
