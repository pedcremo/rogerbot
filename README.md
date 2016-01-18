Robot velocista (linetracker or line follower) Rogerbot
-------------------------------------------------------
More info http://pere.bocairent.net/wiki/index.php/Experiments_i_apunts_rob%C3%B2tica#Rogerbot_2:_Robot_seguidor_de_l.C3.ADnia_i_altres_prop.C3.B2sits
With this robot we went to desafio robot 2015 contest.
It contains some design flaws.

How to program Rogerbot with Ubuntu or other debian based distributions
-----------------------------------------------------------------------
First of all we should install the following packages
<pre>
sudo apt-get install gcc-avr binutils-avr avr-libc avrdude"
</pre>

Aquests paquets són el compilador creuar per a microcontroladors AVR, utilitats de link, la llibreria de C per a estos microcontroladors i el programa per transferir els binaris a la pròpia EEPROM del microcontrolador.

To compile:
<pre>make</pre>

To compile and tranfer the binary to our Robot using an ISP programmer:
<pre>make program</pre>

Manualment podriem transferir el programa al microcontrolador sense fer "make program" de la següent forma:

"avrdude -c avrispv2 -p m328p -P /dev/ttyACM0 -U flash:w:main.hex"


Videos de Youtube del robot en funcionament
---------------------------------------------

Seguiment de línia https://www.youtube.com/watch?v=3Vng8f2Hjuk
Seguiment de línia https://www.youtube.com/watch?v=mVdQlziA4T0
Seguiment de línia https://www.youtube.com/watch?v=nVbRpd_NfEo
Prova de rescat https://www.youtube.com/watch?v=M8LMy-KG1w0  
Prova de rescat https://www.youtube.com/watch?v=BnAOamVrsOA

TODO
----
Lectures digitals o analògiques dels sensors?
Hi ha gran diferència?
Com fan les lectures digitals amb la llibreria de pololu?
