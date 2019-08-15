# flyLinuxServer
Linux Server application for manage airplane

<br>
If you want to run this application, you need to have:
- Linux board(preferably with wifi)
- UART to rf board (stm8s003_and_si4463 example)
- USB joustick
<br>
<br>
The application (flyLinuxServer) was tested on devices Raspberry Pi3 and Banana Pro.
<br>
How to use (example for Raspberry Pi3):
>sudo apt-get install nodejs
>cd /home/pi/
>mkdir Project
>cd Project
>git clone https://github.com/fademike/flyLinuxServer
>make
>cd cliect
>make
>cd ../
>chmod +x run_nodejs.sh
>./run_nodejs.sh &
>./flyLinuxServer
<br>
For convenience, you can transfer the application to Linux startup.
<br>
If the path to the folder with the application is not the same, then you need to fix the path in node/index.js
<br>

Application flyLinuxServer use UART (for exchange with airplane), Joystick and local network (TCP connection for send information to display on Web page through client and nodejs)

<br>
Example Web page
![Image alt](https://github.com/fademike/flyLinuxServer/raw/master/flyLinuxServerWeb.png)
<br>
if indication = 255 means the application (flyLinuxServer) is not running
