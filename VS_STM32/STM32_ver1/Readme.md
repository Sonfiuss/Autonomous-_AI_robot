Step 1: cd /mnt/d/dev/SuperProject/VS_STM32/STM32_ver1/

//Install
wsl: Ubuntu: 22.4.05 LTS
pass: Sonk51c@1
sudo apt-get install gcc-arm-none-eabi
sudo apt-get install gdb-multiarch


//Step build
1. cmake -S . -B build
2. cmake --build build

Check connect ST_link
STM32_Programmer_CLI --list

//Note 
Nếu IDE chưa biên dịch thì phải include Path vào file c_cpp_properties.json

Truyền code 
STM32_Programmer_CLI --connect port=swd --download build/debug/STM32_ver1.elf -hardRst -rst --start
      -------------------------------------------------------------------
                       STM32CubeProgrammer v2.19.0
      -------------------------------------------------------------------

ST-LINK SN  : 1200290006000059334D524E
ST-LINK FW  : V2J39S7
Board       : --
Voltage     : 3.25V
SWD freq    : 4000 KHz
Connect mode: Normal
Reset mode  : Software reset
Device ID   : 0x410
Revision ID : Rev X
Device name : STM32F101/F102/F103 Medium-density
Flash size  : 128 KBytes
Device type : MCU
Device CPU  : Cortex-M3
BL Version  : --

Opening and parsing file: STM32_ver1.elf


Memory Programming ...
  File          : STM32_ver1.elf
  Size          : 832.00 B
  Address       : 0x08000000



Erasing memory corresponding to sector 0:
Erasing internal memory sector 0
Download in Progress:
ÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛÛ 100%

File download complete
Time elapsed during download operation: 00:00:00.174

Hard reset is performed

MCU Reset

Software reset is performed

RUNNING Program ...
  Address:      : 0x8000000
Application is running, Please Hold on...
Start operation achieved successfully

D:\dev\SuperProject\VS_STM32\STM32_ver1>

