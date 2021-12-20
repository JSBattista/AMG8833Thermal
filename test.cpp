// to compile and link this, use the following command.
// g++ test.cpp AMG8833IR.cpp -Wall -std=c++11 -fpermissive -o testamg
/*
	https://github.com/nox771/i2c_t3/issues/16
	https://github.com/threebrooks/AdafruitStepperMotorHAT_CPP/issues/1
	https://www.raspberrypi.org/forums/viewtopic.php?t=189709
*/
#include <stdio.h>
#include "AMG8833IR.h"
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
//#include <linux/ioctl.h>
#include <sys/ioctl.h>  // one or the other will work
// 'global' variables to store screen info
int fbfd = 0;
char *fbp = 0;
struct fb_var_screeninfo vinfo;
struct fb_fix_screeninfo finfo;
int page_size = 0;
int cur_page = 0;
void clear_screen(int c) {
    memset(fbp + cur_page * page_size, c, page_size);
}
void put_pixel(int x, int y, int c)
{
    // calculate the pixel's byte offset inside the buffer
    unsigned int pix_offset = x + y * finfo.line_length;
    // offset by the current buffer start
    pix_offset += cur_page * page_size;
    // now this is about the same as 'fbp[pix_offset] = value'
    *((char*)(fbp + pix_offset)) = c;
}
// helper function to draw a rectangle in given color
void fill_rect(int x, int y, int w, int h, int c) {
    int cx, cy;
    for (cy = 0; cy < h; cy++) {
        for (cx = 0; cx < w; cx++) {
            put_pixel(x + cx, y + cy, c);
        }
    }
}
int main(){
	AMG8833IR ir;
	ir.begin(0x69);
	ir.wake();
	ir.setFramerate10FPS();
	float devt =  ir.getDeviceTemperature();
	float f = ir.getDeviceTemperatureFahrenheit();	
	printf("Device temp = %f, %f\n", devt, f);
	int16_t rawt = ir.getDeviceTemperatureRaw();
	printf("Device temp raw = %i\n", rawt);
    	struct fb_var_screeninfo orig_vinfo;
    	long int screensize = 0;
    	// Open the file for reading and writing
    	fbfd = open("/dev/fb0", O_RDWR);
    	//fbfd = open("/dev/fb1", O_RDWR);
    	if (fbfd == -1) {
      		printf("Error: cannot open framebuffer device.\n");
      		return(1);
    	}
    	printf("The framebuffer device was opened successfully.\n");
    	// Get variable screen information
    	if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo)) {
      		printf("Error reading variable information.\n");
    	}
    	printf("Original %dx%d, %dbpp\n", vinfo.xres, vinfo.yres, 
       	vinfo.bits_per_pixel );
    	// Store for reset (copy vinfo to vinfo_orig)
    	memcpy(&orig_vinfo, &vinfo, sizeof(struct fb_var_screeninfo));
    	// Change variable info
    	vinfo.bits_per_pixel = 16;//8;
    	//vinfo.bits_per_pixel = 8;
    	vinfo.xres = 100;//320;//1280;
    	vinfo.yres = 100;//240;//1024;
    	vinfo.xres_virtual = vinfo.xres;
    	vinfo.yres_virtual = vinfo.yres * 2;
    	if (ioctl(fbfd, FBIOPUT_VSCREENINFO, &vinfo)) {
      		printf("Error setting variable information.\n");
    	}
    	// Get fixed screen information
    	if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo)) {
      		printf("Error reading fixed information.\n");
    	}
    	//printf("Fixed info: smem_len %d, line_length %d\n", finfo.smem_len, finfo.line_length);
    	page_size = finfo.line_length * vinfo.yres;
    	// map fb to user mem 
    	screensize = finfo.smem_len;
    	fbp = (char*)mmap(0, 
              screensize, 
              PROT_READ | PROT_WRITE, 
              MAP_SHARED, 
              fbfd, 
              0);
    	if ((int)fbp == -1) {
        	printf("Failed to mmap.\n");
    	}
    	else {
    		int fps = 50;
    		int secs = 5;
    		// loop for a while
		float r = 0;
		float irreads[64] = {
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0
		};
		int offsetx = 30;
		int offsety = 20;
		int indexr = 0;
		int rectx = 50;
		int recty = 50;
		int spacing = 20;
    		for (int i = 0; i < (fps * secs); i++) {
        		// change page to draw to (between 0 and 1)
        		cur_page = (cur_page + 1) % 2;
        		// clear the previous image (= fill entire screen)
        		clear_screen(8);
			indexr = 0;
			for (unsigned char i = TEMPERATURE_REGISTER_START; i < TEMPERATURE_REGISTER_START + 64; ++i) {
				//r = ir.getPixelTemperatureFahrenheit(i);
				r = ir.getPixelTemperature(i);
				irreads[indexr] = r;
				indexr++;
			}  // for
			indexr = 0;
			// Borrows from the Rectangle fuction from Charlie actual display
    			for (unsigned int cy = 0; cy < 8; cy++) {
        			for (unsigned int cx = 0; cx < 8; cx++) {
            				put_pixel(offsetx + cx, offsety + cy, (unsigned int)irreads[indexr++]);  // really small
        			}  // for
    			}  // for
        		// switch page
        		vinfo.yoffset = cur_page * vinfo.yres;
        		vinfo.activate = FB_ACTIVATE_VBL;
        		if (ioctl(fbfd, FBIOPAN_DISPLAY, &vinfo)) {
            			printf("Error panning display.\n");
        		}  // if
        		usleep(1000000 / fps);
    	    	}  // for
    	}
    	// cleanup
    	// unmap fb file from memory
    	munmap(fbp, screensize);
    	// reset the display mode
    	if (ioctl(fbfd, FBIOPUT_VSCREENINFO, &orig_vinfo)) {
        	printf("Error re-setting variable information.\n");
    	}
    	// close fb file    
    	close(fbfd);
	return 0; 
}
