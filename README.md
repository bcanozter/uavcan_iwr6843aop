# uavcan_iwr6843aop (WIP)
![dronecan_gui_tool](/docs/dronecan_gui_tool.png)

This is a work in progress project. Integrating uavcan protocol to TI mmwave software.

## Hardware

- [IWR6843AOPEVM](https://www.ti.com/product/IWR6843AOP)


<span style="font-size: 1.5em;">IMPORTANT: You must set the S2.1 switch to `ON` position in order to enable CAN0 output.</span>

![xWR6843AOPEVM](/docs/iwr6843aop_switches.png)

## Tools

### UNIFLASH

* [UniFlash flash programming tool](https://www.ti.com/tool/UNIFLASH)

### dronecan_gui_tool

* [DroneCAN GUI Tool](https://dronecan.github.io/GUI_Tool/Overview/)

### ccs 20.2.0

* [Code Composer Studio™ integrated development environment (IDE)](https://www.ti.com/tool/download/CCSTUDIO/20.2.0)
#### Radar Toolbox CAN Integration

* [CAN Integration Example 6843](https://dev.ti.com/tirex/explore/node?node=A__AKHzboWwnc47XghFDOzP5g__radar_toolbox__1AslXXD__LATEST)

## Dependencies

- XDCtools 3.50.8.24_core
- [SYS/BIOS 6.73.1.01](https://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/bios/sysbios/6_73_01_01/index_FDS.html)
- [mmWave SDK 3.6.0.00-LTS](https://www.ti.com/tool/download/MMWAVE-SDK/03.06.00.00-LTS)
- Radar Toolbox 3.20.0.04
- SysConfig 1.24.0

### Compiler

- [TI ARM-CFG v20.2.7-LTS](https://www.ti.com/tool/download/ARM-CGT/20.2.7.LTS)
- gcc 11.4.0

## References

- https://github.com/ArduPilot/ardupilot
- https://github.com/dronecan/DSDL
- https://github.com/dronecan/dronecan_dsdlc
- https://github.com/dronecan/libcanard
- https://github.com/dronecan/pydronecan
- https://www.ti.com/lit/ug/swru546e/swru546e.pdf
- [CAN Integration User Guide](https://dev.ti.com/tirex/content/radar_toolbox_3_20_00_04/source/ti/examples/Fundamentals/CAN_Data_Output/CAN_Integration/docs/CAN_Integration_User_Guide.html)
