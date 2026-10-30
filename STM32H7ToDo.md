* ~~Board pin definitions~~
* ~~Extra SPI definitions SPI5 & 6~~
* ~~Allow SPI to work with/without DMA to unaligned/aligned addresses~~
* ~~SPI DMA - Fix to work when no recv buffer & DMA~~
* ~~ADC calibration constants seem to be using 16bit samples but we expect 12bit~~
* ~~SDIO card interface~~
* ~~DMA memory buffers (re-enable dcache currently disabled)~~
* ~~TMC2209 interface~~
* ~~Test TMC5160 interface~~
* ~~Check neopixel timing (consider using dma)~~
* ~~Sort out STM32F4 and STM32H7 defines consider having base STM32 define~~
* ~~WiFi interface~~
* ~~Check clock timing~~
* Enable 16bit ADC samples (currently 12 bit with oversampling to 14)
* Use processor d-ram for stacks etc.
* ~~Check UART code~~
* Consider using FIFOs for UARTS
* ~~Common STM32 Core?~~
* ~~Crash dump flash interface~~
* ~~CAN support~~
* Investigate SD Card SPI mode high speed switching
* Test SBC mode
* Build H7 IAP for SBC mode
* ~~Sort out a H7 bootloader for Fly boards~~
* ~~Check H7 timer allocation~~
* ~~use QSPI for neoplixels or use DMA~~
* Allow use of shared timers per pin (find best match)
* Update hardware usage for H7
* Allow DMA with SPI1, SPI5, SPI6
* ~~Replace ST USB library~~
* Allow use of embeded config
* ~~Set initial pin state from board.txt~~
* ~~Add support for humidity sensors~~
* ~~Allow setting of neopixel pins~~
* Fix F4 PWM pin timer allocation for pin PD_13 to match H7
* ~~Allow default thermistor resistor value to be changed on a per board basis (for PT1000 etc.)~~
* Consider making default thermistor value part of the per board configuration
