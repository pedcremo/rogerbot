Robot velocista (linetracker or line follower) Rogerbot
-------------------------------------------------------
More info:
 http://pere.bocairent.net/wiki/index.php/Experiments_i_apunts_rob%C3%B2tica#Rogerbot_2:_Robot_seguidor_de_l.C3.ADnia_i_altres_prop.C3.B2sits

With this robot we went to desafio robot 2015 contest.
It contains some design flaws.

How to program Rogerbot with Ubuntu or other debian based distributions
-----------------------------------------------------------------------
First of all we should install the following packages
<pre>
sudo apt-get install gcc-avr binutils-avr avr-libc avrdude"
</pre>

These packages are the crosscompiler for AVR microcontrollers, link utilities, avr C library and a program to transfer binaries to the EEPROM microcontroller.

To compile:
<pre>make</pre>

To compile and tranfer the binary to our Robot using an ISP programmer:
<pre>make program</pre>


We can do it manually too without "make program". Introduce the following in the command line.

"avrdude -c avrispv2 -p m328p -P /dev/ttyACM0 -U flash:w:main.hex"


Youtube Robot videos at working
---------------------------------------------

*Line following https://www.youtube.com/watch?v=3Vng8f2Hjuk

*Line following https://www.youtube.com/watch?v=mVdQlziA4T0

*Line following https://www.youtube.com/watch?v=nVbRpd_NfEo

*Rescue contest https://www.youtube.com/watch?v=M8LMy-KG1w0  

*Rescue contest https://www.youtube.com/watch?v=BnAOamVrsOA

TODO
----
Fix lot of design flaws with a new version
