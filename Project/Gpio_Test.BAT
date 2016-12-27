SET PATH=d:\Keil\ARM\ARMCC\bin\;C:\ProgramData\Oracle\Java\javapath;C:\Program Files (x86)\Intel\iCLS Client\;C:\Program Files\Intel\iCLS Client\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\Lenovo\Fingerprint Manager Pro\;C:\Program Files (x86)\Common Files\Adobe\AGL;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\QuickTime\QTSystem\;
SET CPU_TYPE=STM32F103ZE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Gpio_Test
SET CPU_CLOCK=0x007A1200
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\core_cm3._ip"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\core_cm3.__i"
"d:\Keil\ARM\ARMCC\bin\ArmAsm" --Via ".\obj\startup_stm32f10x_hd._ia"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\SYSTEM~2._IP"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\system_stm32f10x.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\main._ip"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\main.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\STM32F~3._IP"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\stm32f10x_it.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\STM32F~2._IP"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\stm32f10x_gpio.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\STM32F~4._IP"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\stm32f10x_rcc.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\misc._ip"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\misc.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\STM32F~1._IP"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\stm32f10x_flash.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\gpio_led._ip"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\gpio_led.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\List\SYSTEM~1._IP"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\obj\systemclock.__i"
"d:\Keil\ARM\ARMCC\bin\ArmLink" --Via ".\Obj\Gpio.lnp"
d:\Keil\ARM\ARMCC\bin\fromelf.exe ".\Obj\Gpio.axf" --i32combined --output ".\Obj\Gpio.hex"
