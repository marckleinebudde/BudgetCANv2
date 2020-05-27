# BudgetCANv2

Refactoring of the candleLightFW for use on STM32 G4 microcontroller.

Utilizes the STM32CubeMX libraries to implement FDCAN and USB using the STM32 HAL

Build environment is currently VisualGDB for ease of my testing with the STLinkv3 in VisualStudio.  Should compile under other toolchains with some TLC.

2020-05-26:
- Currently only non-FDCAN is implemented as there is no gs_usb driver for FDCAN
- CAN device error returns are sketchy as I still need to map the new G4 status register flags to the exisiting gs_usb errors
- Currenly only lightly tested so use at your own risk

Tested using the following HW:

http://github.com/ryedwards/BudgetCANv2-HW
