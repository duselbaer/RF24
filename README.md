Driver for NRF24L01
===================

Based on the version provided by maniacbug on github.

This driver for the NRF24L01 is a C++ class which is hardware independent.
When creating this driver the goal was to have a single driver which can
be used on different platforms like

   * MSP430
   * maybe AVR
   * Cortex M0/M3
   * Raspberry

through having the hardware dependency completely moved to a communication
class injected to the driver.
