# RaspberryPi_FBTFT_fbflex_35_lcd
Raspberry Pi FBTFT fbflex 3.5 inch TFT LCD module V5.0 SPI Control program

---
## Raspberry Pi用 FBTFT fbflex対応の汎用 3.5インチ TFT液晶を自前のプログラムで制御する方法
http://www.neko.ne.jp/~freewing/raspberry_pi/raspberry_pi_3_tft_lcd_35inch_fbflex_2/

You can choose any control method .  
1. bcm2835 version  
2. The pigpio library version  
3. Linux spidev version  
4. WiringPi library version  

## FREE WING Homepage
http://www.neko.ne.jp/~freewing/

---
## References（参考文献）
### KeDei 3.5 inch 480x320 TFT lcd from ali
https://www.raspberrypi.org/forums/viewtopic.php?p=1019562  
 by Conjur - Mon Aug 22, 2016 2:12 am - Final post on the KeDei v5.0 code.

### l0nley/kedei35
https://github.com/l0nley/kedei35

---
## Build bcm2835 version
##### # C library for Broadcom BCM 2835 as used in Raspberry Pi
##### # http://www.airspayce.com/mikem/bcm2835/

#### # Enable SPI
sudo raspi-config

#### # build bcm2835 lib.
cd  
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.52.tar.gz  
tar -zxf bcm2835-1.52.tar.gz  
cd bcm2835-1.52  
./configure  
sudo make install

#### # git clone
cd  
git clone https://github.com/FREEWING-JP/RaspberryPi_FBTFT_fbflex_35_lcd.git

#### # compile
cd ~/RaspberryPi_FBTFT_fbflex_35_lcd  
gcc -o fbflex_lcd_pi fbflex_lcd_pi.c -lbcm2835  

#### # execute !
sudo ./fbflex_lcd_pi


---
## Build The pigpio library version
##### # The pigpio library
##### # http://abyz.co.uk/rpi/pigpio/

#### # Enable SPI
sudo raspi-config

#### # git clone
cd  
git clone https://github.com/FREEWING-JP/RaspberryPi_FBTFT_fbflex_35_lcd.git

#### # compile
cd ~/RaspberryPi_FBTFT_fbflex_35_lcd  
gcc -o fbflex_lcd_pi_pigpio fbflex_lcd_pi_pigpio.c -lpigpio -lrt -lpthread  

#### # execute !
sudo ./fbflex_lcd_pi_pigpio


---
## Build Linux spidev version
##### # SPIdev - The Linux Kernel Archives
##### # https://www.kernel.org/doc/Documentation/spi/spidev

#### # Enable SPI
sudo raspi-config

#### # git clone
cd  
git clone https://github.com/FREEWING-JP/RaspberryPi_FBTFT_fbflex_35_lcd.git

#### # compile
cd ~/RaspberryPi_FBTFT_fbflex_35_lcd  
gcc -o fbflex_lcd_pi_spidev fbflex_lcd_pi_spidev.c

#### # execute !
sudo ./fbflex_lcd_pi_spidev


---
## Build WiringPi library version
##### # WiringPi - GPIO Interface library for the Raspberry Pi
##### # http://wiringpi.com/

#### # Enable SPI
sudo raspi-config

#### # git clone
cd  
git clone https://github.com/FREEWING-JP/RaspberryPi_FBTFT_fbflex_35_lcd.git

#### # compile
cd ~/RaspberryPi_FBTFT_fbflex_35_lcd  
gcc -o fbflex_lcd_pi_wiringpi fbflex_lcd_pi_wiringpi.c -lwiringPi

#### # execute !
sudo ./fbflex_lcd_pi_wiringpi


---
## Picture

![Raspberry Pi FBTFT fbflex 3.5 inch LCD module Control program](/fbtft_fbflex_35_lcd_1.jpg)

![Raspberry Pi FBTFT fbflex 3.5 inch LCD module](/fbtft_fbflex_35_lcd_2.jpg)

