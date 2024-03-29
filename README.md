# MobaStation
<p align="center">
  <img src="/Housing/Front.jpg" width="500">
  <img src="/Housing/Back.jpg" width="500">
</p>

MobaStation is a open source project to create a full digital model railroad control system based on Arduino.
Main goal of the project is to use only cheap, easy to get components that can easy assembled.

MobaStation works with the Z21-LAN-Protocol. So you can connect it as Z21 to control systems like rocrail, traincontroller (not tested yet) and others. You can also use mobile-apps like the official Z21 app.

For DCC we use the <a href="https://github.com/Maggge/DCC">Maggge/DCC</a>-Library wich is extracted from <a href="https://github.com/DCC-EX/CommandStation-EX">DCC-EX</a>-Library and slightly modified.

For accessories like turnouts, sensors and others we use the <a href="https://github.com/Maggge/MobaBus">Maggge/MobaBus</a>-Library.</br>
Coming soon: XpressNet to connect Roco multiMAUS controller.

<h2>Installation</h2>

  1) Build the MobaStation:</br>
      1)For Network connectivity are different possibilities:
        - <a href="https://www.makershop.de/plattformen/arduino/mega2560-wifi-r3"/>Arduino Mega with integrated Wifi-Module </a>(the ESP needs to  be flashed with the AT 1.7 firmware)
        - Arduino Mega with ESP-01 connected to Serial3 (the ESP needs to  be flashed with the AT 1.7 firmware)
        - Arduino Mega with Ethernet Modul
        - Arduino Mega with Ethernet and Wifi (the ESP needs to  be flashed with the AT 1.7 firmware)
        <p align="center">
          <img src="/schematics/Mega-Wifi-Ethernet.jpg" width="500"> 
        </p>
                 
      2)Arduino Motorshield for DCC      
    
      3)Optional: MCP2515 Can-Bus-Modul for MobaBus  
      <p align="center">
        <img src="/schematics/Mega-Can.jpg" width="500"> 
      </p>
      4)Optional: MAX485 RS485-Modul connected to Serial1 for XPressNet (at the moment not activated)
      <p align="center">
        <img src="/schematics/Mega-RS485.jpg" width="500"> 
      </p>
      5)Optional: Track-Power-Button (connected to Pin 24 and GND) and Track-Status-Led (connected to Pin 25 and GND). I use a Button with integrated LED-Ring
      6)Optional: Track-Voltage meter (Voltage divider on MotorDriver Input. Connected to Pin A15. Resistor - values will follow).
      
  
  2) Download the latest <a href=https://github.com/Maggge/MobaStation/releases>Release<a> (not the source code!) and extract it. 
  3) Connect the Arduino Mega to USB.
  4) Open the MobaStation Config Tool.
    <p align="center">
      <img src="/ConfigTool-images/ConfigTool.PNG" width="500"> 
    </p>
  5) Open "Flash device" in Menu.
    <p align="center">
      <img src="/ConfigTool-images/ConfigTool_flash.PNG" width="500"> 
    </p>
  6) Select the COM Port of the Arduino and the firmware contained in Package and "Flash device".
  7) Now you can connect to MobaStation in Menu
  8) Open the config window in Menu.
    <p align="center">
      <img src="/ConfigTool-images/ConfigTool_config.PNG" width="500"> 
    </p>   
   8) Enter your settigns. Press "Write config" and voilà. Your MobaStation is now ready to drive your trains!
