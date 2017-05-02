# SPI Bootloader Host example
This example project acts as a STM32 bootloader SPI host to communicate with
another STM32 MCU configured to boot in system memory. The code provided in
this example can be programmed onto a STM32L476RG-Nucleo and will use SPI1 as a
default interface. The bootloader library can be used with any other STM32 MCU.


## SPI Bootloader library functions:
```c
void     BL_Init(SPI_HandleTypeDef *hspi);
void     BL_Get_Command(uint8_t *pData);
uint8_t  BL_GetVersion_Command(void);
uint16_t BL_GetID_Command(void);
void     BL_ReadMemory_Command(uint32_t address, uint8_t nob, uint8_t *pData);
void     BL_Go_Command(uint32_t address);
void     BL_WriteMemory_Command(uint32_t address, uint8_t nob, uint8_t *pData);
void     BL_EraseMemory_Command(uint16_t nb, uint8_t code);
void     BL_WriteProtect_Command(uint8_t nb, uint8_t *codes);
void     BL_WriteUnprotect_Command(void);
void     BL_ReadoutProtect_Command(void);
void     BL_ReadoutUnprotect_Command(void);
```
More details about the functions listed above can be found in [AN4286 SPI protocol used in the STM32 bootloader].

# Requirements
* two STM32
### Windows
* IAR Embedded Workbench
* STM32 ST-LINK utility

## Enable target STM32 MCU to boot in system memory mode
### NUCLEO board
Connect BOOT0 to VDD (Place a jumper on pin CN7-5 and CN7-7)
Connect SPI1_NSS to GND (Connect CN7-32 to GND)

Refer to [AN2606 STM32 microcontroller system memory boot mode]
for more details on how to enable system memory boot on a particular STM32.

## Build
### Windows
1. Double-click on `EWARM/Project.eww` to open the workspace.

2. Make

3. Project > Download > Download active application

## SPI Connection
The example project is configured to use SPI1. On a NUCLEO board, the two boards
can be connected using the following pins:
* SPI1_SCK  PA5 (CN5-6 or CN10-11)
* SPI1_MISO PA6 (CN5-5 or CN10-13)
* SPI1_MOSI PA7 (CN5-4 or CN10-15)
* GND

    [AN2606 STM32 microcontroller system memory boot mode]: <http://www.st.com/content/ccc/resource/technical/document/application_note/b9/9b/16/3a/12/1e/40/0c/CD00167594.pdf/files/CD00167594.pdf/jcr:content/translations/en.CD00167594.pdf>
    [AN4286 SPI protocol used in the STM32 bootloader]: <http://www.st.com/content/ccc/resource/technical/document/application_note/7a/8a/0a/8f/8f/38/47/c0/DM00081379.pdf/files/DM00081379.pdf/jcr:content/translations/en.DM00081379.pdf>
