 // Original code - KeDei V5.0 code - Conjur
 //  https://www.raspberrypi.org/forums/viewtopic.php?p=1019562
 //  Mon Aug 22, 2016 2:12 am - Final post on the KeDei v5.0 code.
 // References code - Uladzimir Harabtsou l0nley
 //  https://github.com/l0nley/kedei35
 //
 // FBTFT fbflex 3.5inch LCD module for Raspberry Pi
 // Modified by FREE WING, Y.Sakamoto
 // http://www.neko.ne.jp/~freewing/
 //
 // gcc -o fbflex_lcd_pi fbflex_lcd_pi.c -lbcm2835
 // sudo ./fbflex_lcd_pi

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <bcm2835.h>
#include <time.h>

#define LCD_CS 0
#define TOUCH_CS 1
#define LCD_WIDTH  480
#define LCD_HEIGHT 320

//	D/C ------- GPIO24 ______ Pin 18
#define GPIO_LCD_DC RPI_GPIO_P1_18
//	RESET ----- GPIO25 ______ Pin 22
#define GPIO_LCD_RST RPI_GPIO_P1_22


uint8_t lcd_rotations[4] = {
	0b00101000,	//   0
	0b10001000,	//  90
	0b11101000,	// 180
	0b01001000	// 270
};

volatile uint8_t color;
volatile uint8_t lcd_rotation;
volatile uint16_t lcd_h;
volatile uint16_t lcd_w;

uint16_t colors[16] = {
	0b0000000000000000,				/* BLACK	000000 */
	0b0000000000010000,				/* NAVY		000080 */
	0b0000000000011111,				/* BLUE		0000ff */
	0b0000010011000000,				/* GREEN	009900 */
	0b0000010011010011,				/* TEAL		009999 */
	0b0000011111100000,				/* LIME		00ff00 */
	0b0000011111111111,				/* AQUA		00ffff */
	0b1000000000000000,				/* MAROON	800000 */
	0b1000000000010000,				/* PURPLE	800080 */
	0b1001110011000000,				/* OLIVE	999900 */
	0b1000010000010000,				/* GRAY		808080 */
	0b1100111001111001,				/* SILVER	cccccc */
	0b1111100000000000,				/* RED		ff0000 */
	0b1111100000011111,				/* FUCHSIA	ff00ff */
	0b1111111111100000,				/* YELLOW	ffff00 */
	0b1111111111111111				/* WHITE	ffffff */
};


void delayms(int ms) {
	delay(ms);
}


int lcd_open(void) {
	int r;
	uint32_t v;
	r = bcm2835_init();
	if(r!=1) return -1;

	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
	// 8=NG
	// 16=OK, 32=6MHz, 64=4MHz, 128=2MHz, 256=1.5MHz, 512=0.8MHz
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);

	bcm2835_gpio_fsel(GPIO_LCD_RST, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_LCD_DC, BCM2835_GPIO_FSEL_OUTP);

	return 0;
}

int lcd_close(void) {
	bcm2835_spi_end();
	int r = bcm2835_close();
	if(r!=1) return -1;

	return 0;
}

void spi_transmit(int devsel, uint8_t *data, int len) {
	if(devsel == 0)
	{
		bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	} else {
		bcm2835_spi_chipSelect(BCM2835_SPI_CS1);
	}
	bcm2835_spi_transfern((char*)data, len);
}

void lcd_rst(void) {
	bcm2835_gpio_write(GPIO_LCD_RST, LOW);
	delayms(150);

	bcm2835_gpio_write(GPIO_LCD_RST, HIGH);
	delayms(250);
}


void lcd_cmd(uint8_t cmd) {
	uint8_t b1[2];

	bcm2835_gpio_write(GPIO_LCD_DC, LOW);

	b1[0] = 0x00;
	b1[1] = cmd;
	spi_transmit(LCD_CS, &b1[0], sizeof(b1));

	bcm2835_gpio_write(GPIO_LCD_DC, HIGH);
}


void lcd_data(uint8_t dat) {
	uint8_t b1[2];

	bcm2835_gpio_write(GPIO_LCD_DC, HIGH);

	b1[0] = 0x00;
	b1[1] = dat;
	spi_transmit(LCD_CS, &b1[0], sizeof(b1));
}


void lcd_color(uint16_t col) {
	uint8_t b1[2];

	bcm2835_gpio_write(GPIO_LCD_DC, HIGH);

	// RGB565
	// 0xF800 R(R4-R0, DB15-DB11)
	// 0x07E0 G(G5-G0, DB10- DB5)
	// 0x001F B(B4-B0, DB4 - DB0)
	b1[0]= col>>8;
	b1[1]= col&0x00FF;
	spi_transmit(LCD_CS, &b1[0], sizeof(b1));
}


uint16_t colorRGB(uint8_t r, uint8_t g, uint8_t b) {

	// RGB565
	// 0xF800 R(R4-R0, DB15-DB11)
	// 0x07E0 G(G5-G0, DB10- DB5)
	// 0x001F B(B4-B0, DB4 - DB0)
	uint16_t col = ((r<<8) & 0xF800) | ((g<<3) & 0x07E0) | ((b>>3) & 0x001F);
//	printf("%02x %02x %02x %04x\n", r, g, b, col);

	return col;
}


// 18bit color mode
void lcd_colorRGB(uint8_t r, uint8_t g, uint8_t b) {

	// RGB565
	lcd_color(colorRGB(r, g, b));
}


void lcd_setrotation(uint8_t m) {
	lcd_cmd(0x36); lcd_data(lcd_rotations[m]);
	if (m&1) {
		lcd_h = LCD_WIDTH;
		lcd_w = LCD_HEIGHT;
	} else {
		lcd_h = LCD_HEIGHT;
		lcd_w = LCD_WIDTH;
	}
}

void lcd_init(void) {
	//reset display
	lcd_rst();

	/*
	Initialize Parameter FBTFT flexfb
	flexfb fbtft_reset()
	flexfb init: write(0xB0) 0x00
	flexfb init: write(0x11)
	flexfb init: mdelay(250)
	flexfb init: write(0x3A) 0x55
	flexfb init: write(0xC2) 0x44
	flexfb init: write(0xC5) 0x00 0x00 0x00 0x00
	flexfb init: write(0xE0) 0x0F 0x1F 0x1C 0x0C 0x0F 0x08 0x48 0x98 0x37 0x0A 0x13 0x04 0x11 0x0D 0x00
	flexfb init: write(0xE1) 0x0F 0x32 0x2E 0x0B 0x0D 0x05 0x47 0x75 0x37 0x06 0x10 0x03 0x24 0x20 0x00
	flexfb init: write(0xE2) 0x0F 0x32 0x2E 0x0B 0x0D 0x05 0x47 0x75 0x37 0x06 0x10 0x03 0x24 0x20 0x00
	flexfb init: write(0x36) 0x28
	flexfb init: write(0x11)
	flexfb init: write(0x29)
	*/

	lcd_cmd(0xB0); lcd_data(0x00);

	lcd_cmd(0x11);		// Sleep Out
	delayms(250);

	lcd_cmd(0x3A); lcd_data(0x55);
	lcd_cmd(0xC2); lcd_data(0x44);
	lcd_cmd(0xC5); lcd_data(0x00); lcd_data(0x00); lcd_data(0x00); lcd_data(0x00);
	lcd_cmd(0xE0); lcd_data(0x0F); lcd_data(0x1F); lcd_data(0x1C); lcd_data(0x0C); lcd_data(0x0F); lcd_data(0x08); lcd_data(0x48); lcd_data(0x98); lcd_data(0x37); lcd_data(0x0A); lcd_data(0x13); lcd_data(0x04); lcd_data(0x11); lcd_data(0x0D); lcd_data(0x00);
	lcd_cmd(0xE1); lcd_data(0x0F); lcd_data(0x32); lcd_data(0x2E); lcd_data(0x0B); lcd_data(0x0D); lcd_data(0x05); lcd_data(0x47); lcd_data(0x75); lcd_data(0x37); lcd_data(0x06); lcd_data(0x10); lcd_data(0x03); lcd_data(0x24); lcd_data(0x20); lcd_data(0x00);
	lcd_cmd(0xE2); lcd_data(0x0F); lcd_data(0x32); lcd_data(0x2E); lcd_data(0x0B); lcd_data(0x0D); lcd_data(0x05); lcd_data(0x47); lcd_data(0x75); lcd_data(0x37); lcd_data(0x06); lcd_data(0x10); lcd_data(0x03); lcd_data(0x24); lcd_data(0x20); lcd_data(0x00);


//	lcd_cmd(0x36); lcd_data(0x28);
	lcd_setrotation(0);

	lcd_cmd(0x11);
	delayms(250);

	lcd_cmd(0x29);
	delayms(150);
}

void lcd_setframe(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
	lcd_cmd(0x2A);
	lcd_data(x>>8); lcd_data(x&0xFF);
	lcd_data(((w+x)-1)>>8); lcd_data(((w+x)-1)&0xFF);
	lcd_cmd(0x2B);
	lcd_data(y>>8); lcd_data(y&0xFF);
	lcd_data(((h+y)-1)>>8); lcd_data(((h+y)-1)&0xFF);
	lcd_cmd(0x2C);
}

//lcd_fillframe
//fills an area of the screen with a single color.
void lcd_fillframe(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t col) {
	int span=h*w;
	lcd_setframe(x,y,w,h);
	int q;
	for(q=0;q<span;q++) { lcd_color(col); }
}

void lcd_fill(uint16_t col) {
	lcd_fillframe(0, 0, lcd_w, lcd_h, col);
}


void lcd_fillframeRGB(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
	int span=h*w;
	lcd_setframe(x,y,w,h);
	int q;
	for(q=0;q<span;q++) { lcd_colorRGB(r, g, b); }
}

void lcd_fillRGB(uint8_t r, uint8_t g, uint8_t b) {
	lcd_fillframeRGB(0, 0, lcd_w, lcd_h, r, g, b);
}


void lcd_img(char *fname, uint16_t x, uint16_t y) {

	uint8_t buf[54];
	uint16_t p, c;
	uint32_t isize, ioffset, iwidth, iheight, ibpp, fpos, rowbytes;

	FILE *f = fopen(fname, "rb");
	if (f != NULL) {
		fseek(f, 0L, SEEK_SET);
		fread(buf, 30, 1, f);

		isize =	 buf[2] + (buf[3]<<8) + (buf[4]<<16) + (buf[5]<<24);
		ioffset = buf[10] + (buf[11]<<8) + (buf[12]<<16) + (buf[13]<<24);
		iwidth =	buf[18] + (buf[19]<<8) + (buf[20]<<16) + (buf[21]<<24);
		iheight = buf[22] + (buf[23]<<8) + (buf[24]<<16) + (buf[25]<<24);
		ibpp =		buf[28] + (buf[29]<<8);

		printf("\n\n");
		printf("File Size: %u\nOffset: %u\nWidth: %u\nHeight: %u\nBPP: %u\n\n",isize,ioffset,iwidth,iheight,ibpp);

		lcd_setframe(x,y,iwidth,iheight); //set the active frame...
		rowbytes=(iwidth*3) + 4-((iwidth*3)%4);
		for (p=iheight-1;p>0;p--) {
			// p = relative page address (y)
			fpos = ioffset+(p*rowbytes);
			fseek(f, fpos, SEEK_SET);
			for (c=0;c<iwidth;c++) {
				// c = relative column address (x)
				fread(buf, 3, 1, f);

				// B buf[0]
				// G buf[1]
				// R buf[2]
				// 18bit color mode
				lcd_colorRGB(buf[2], buf[1], buf[0]);
			}
		}

		fclose(f);
	}
}


void loop() {

	//Update rotation
	lcd_setrotation(lcd_rotation);

	//Fill entire screen with new color
	lcd_fillframe(0,0,lcd_w,lcd_h,colors[color]);

	//Make a color+1 box, 5 pixels from the top-left corner, 20 pixels high, 95 (100-5) pixels from right border.
	lcd_fillframe(5,5,lcd_w-100,20,colors[(color+1) & 0xF]);

	//increment color
	color++;
	//if color is overflowed, reset to 0
	if (color==16) {color=0;}

	//increment rotation
	lcd_rotation++;

	//if rotation is overflowed, reset to 0
	if (lcd_rotation==4) lcd_rotation=0;

	delayms(500);
}


int main(int argc,char *argv[]) {

	lcd_open();

	lcd_init();

	lcd_fill(0); //black out the screen.
	// 24bit Bitmap only
	lcd_img("fbflex_lcd_pi.bmp", 50, 5);
	delayms(500);

	lcd_fillRGB(0xFF, 0x00, 0x00);
	lcd_fillRGB(0x00, 0xFF, 0x00);
	lcd_fillRGB(0xFF, 0xFF, 0x00);
	lcd_fillRGB(0x00, 0x00, 0xFF);
	lcd_fillRGB(0xFF, 0x00, 0xFF);
	lcd_fillRGB(0x00, 0xFF, 0xFF);
	lcd_fillRGB(0xFF, 0xFF, 0xFF);
	lcd_fillRGB(0x00, 0x00, 0x00);

	// 24bit Bitmap only
	lcd_img("fbflex_lcd_pi.bmp", 50, 5);
	delayms(500);

	// Demo
	color=0;
	lcd_rotation=0;
	loop();	loop();	loop();	loop();
	loop();	loop();	loop();	loop();
	loop();	loop();	loop();	loop();
	loop();	loop();	loop();	loop();
	loop();

	// 24bit Bitmap only
	lcd_img("fbflex_lcd_pi.bmp", 50, 5);

	lcd_close();
}

