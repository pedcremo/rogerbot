Ubuntu o altres distribucions tipus debian
-------------------------------------------
Instal·lar paquets "sudo apt-get install gcc-avr binutils-avr avr-libc avrdude". Aquests paquets són el compilador creuar per a microcontroladors avsr, utilitats de link, la llibreria de C per a estos microcontroladors i el programa per transferir els binaris a la pròpia EEPROM del microcontrolador.

make

make program

Manualment podriem transferir el programa al microcontrolador sense fer "make program" de la següent forma:

"avrdude -c avrispv2 -p m328p -P /dev/ttyACM0 -U flash:w:main.hex"


Videos de Youtube del robot en funcionament
---------------------------------------------

Seguiment de línia https://www.youtube.com/watch?v=3Vng8f2Hjuk
Seguiment de línia https://www.youtube.com/watch?v=mVdQlziA4T0
Seguiment de línia https://www.youtube.com/watch?v=nVbRpd_NfEo
Prova de rescat https://www.youtube.com/watch?v=M8LMy-KG1w0  
Prova de rescat https://www.youtube.com/watch?v=BnAOamVrsOA
