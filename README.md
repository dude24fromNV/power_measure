# Power measure

 *It is the prototype device what can measure power and collect statistics.*
 
Max parameters for measuring: DC 50V & 30A.
Instantaneous measuring power show in 7 segment display.
It can collect statistic used power for last hour and 15 days (24 readings per day).
For see statistic you must connect to device via uart. And enter one of two commands: "stat per hour" or "all stat".
Nowaday device measure power at once you power on the device. 
For avoid collision in save statistic, device have 2 buttons for reset. 
One button reset statistic per hour, second reset all statistic.


For show in [7 segment display](https://drive.google.com/drive/folders/1RTUUCOk_01ovHiSp5Q3dC73Gxl-3k437)
and connect via [uart](https://drive.google.com/drive/folders/1fqAAm-uNajGP5BuFKkavCXQf70DaVqD3), i used code from lab material THodnev.