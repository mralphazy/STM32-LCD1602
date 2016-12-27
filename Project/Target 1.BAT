SET PATH=C:\Keil\ARM\BIN40\;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\Program Files\Common Files\Thunder Network\KanKan\Codecs;C:\Program Files\Flash Magic;D:\Program Files\Altium Designer 6\System;C:\Program Files\ATMEL Corporation\SAM-BA v2.10\drv\;C:\Program Files\ATMEL Corporation\SAM-BA v2.10;C:\Program Files\StormII\Codec;C:\Program Files\StormII
SET RVCT40INC=C:\Keil\ARM\RV31\INC
SET RVCT40LIB=C:\Keil\ARM\RV31\LIB
SET CPU_TYPE=STM32F103ZE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Target 1
SET CPU_CLOCK=0x00000000
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\core_cm3._ip"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\core_cm3.__i"
"C:\Keil\ARM\BIN40\ArmAsm" --Via ".\obj\startup_stm32f10x_hd._ia"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\SYSTEM~2._IP"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\system_stm32f10x.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\main._ip"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\main.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\STM32F~3._IP"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_it.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\STM32F~2._IP"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_gpio.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\STM32F~4._IP"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_rcc.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\misc._ip"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\misc.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\STM32F~1._IP"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_flash.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\gpio_led._ip"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\gpio_led.__i"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\List\SYSTEM~1._IP"
"C:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\systemclock.__i"
"C:\Keil\ARM\BIN40\ArmLink" --Via ".\Obj\Gpio.lnp"
C:\Keil\ARM\BIN40\fromelf.exe ".\Obj\Gpio.axf" --i32combined --output ".\Obj\Gpio.hex"
