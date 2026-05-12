# MIRA 2.0 — Fault-Tolerant Multimodal Presence Verification

CPRE 5450: Dependable Computing Systems, Iowa State University, Spring 2026

Extends MIRA (CPRE 5750) with a 24 GHz mmWave UART sensor and a fault-tolerant activation state machine (FT controller).

## New files (CPRE 5450 additions)
- \main/mmwave_sensor.h\ / \mmwave_sensor.c\ — Seeed MR24HPC1 UART driver
- \main/ft_activation.h\ / \t_activation.c\ — 7-state FT activation FSM

## Build
\\\ash
idf.py build
idf.py flash monitor
\\\`n
Source: https://github.com/Shi6aSK/MIRA-2.0
