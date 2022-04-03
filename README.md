# MobaStation
<p align="center">
  <img src="/Housing/Front.jpg" width="500">
  <img src="/Housing/Back.jpg" width="500">
</p>

MobaStation is a open source project to create a full digital model railroad control system based on Arduino.
Main goal of the project is to use only cheap, easy to get components that can easy assembled.

MobaStation works with the Z21-LAN-Protocol. So you can connect it as Z21 to control systems like rocrail, traincontroller (not tested yet) and others. You can also use mobile-apps like the official Z21 app.

For DCC we use the <a href="https://github.com/Maggge/DCC">Maggge/DCC</a>-Library wich is extracted from <a href="https://github.com/DCC-EX/CommandStation-EX">DCC-EX</a>-Library and slightly modified.

For accessories like turnouts, sensors and others we use the <a href="https://github.com/Maggge/MobaBus">MobaBus</a>-Library and XpressNet (not working yet) to connect Roco multiMAUS controller.

<h2>Installation</h2>

  1) Build the MobaStation to your needs (Instructions coming soon)
  2) Download the latest <a href=https://github.com/Maggge/MobaStation/releases>Release<a> (not the source code!) an extract it. 
  3) Connect the Arduino Mega to USB.
  4) Open the MobaStation Config Tool.
  5) Open Flash device in Menu.
  6) Select the COM Port of the Arduino,select the firmware contained in Package and Flash device.
  7) Now you can connect in Menu and config the MobaStation.
