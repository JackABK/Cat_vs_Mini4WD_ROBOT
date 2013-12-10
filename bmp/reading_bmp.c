/*
#include <png.h>

int main(){
	png_structp png_ptr;
	png_inforp info_ptr;



	png_read_png(png_ptr,info_ptr,PNG_TRANSFORM_IDENTITY,NULL);


	return 0;
}*/


#include "bmpfile.h"
#include "reading_bmp.h"
#include "fat_filelib.h"

#define DEPTH 24
#define HEIGHT 100
#define WIDTH 100
#define USE_FILELIB_STDIO_COMPAT_NAMES


bmpfile_t * readBMP(char* filename)
{
    int i,j;
    bmpfile_t *bmp;
	rgb_pixel_t pixel;
    FL_FILE* f = fl_fopen(filename, "rb");
    unsigned char info[54];
    fl_fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

    // extract image height and width from header
    int width = *(int*)&info[18];
    int height = *(int*)&info[22];
    int depth = *(uint16_t*)&info[28];

    int size = depth/8 * width * height;
    unsigned char* data = pvPortMalloc( sizeof(unsigned char)*size  ); // allocate 3 bytes per pixel
    fl_fread(data, sizeof(unsigned char), size, f); // read the rest of the data at once
    fl_fclose(f);




	bmp=bmp_create(width, height,depth);
	for (j = 0; j < height;j++) {
		for(i=0;i<width;i++){
			memcpy((uint8_t*)(&pixel),(uint8_t*)(&data[(i+j*width)*depth/8]),depth/8);
			bmp_set_pixel(bmp, i,height-j, pixel);
		}

	}
	vPortFree(data);


    return bmp;
}



rgb_pixel_t get_avg_pixel(bmpfile_t *bmp, uint32_t x, uint32_t y){
	rgb_pixel_t ret_pixel;
	rgb_pixel_t* pixel_ptr;
	int  width,height;
	int counter=0;
	int blue=0,red=0,green=0,alpha=0;

	width=bmp_get_width(bmp);
	height=bmp_get_height(bmp);
	
	

	pixel_ptr=bmp_get_pixel(bmp,x-1,y);
	if(pixel_ptr!=NULL){
		blue+=pixel_ptr->blue;
		red+=pixel_ptr->red;
		green+=pixel_ptr->green;
		alpha+=pixel_ptr->alpha;
		counter++;
	}
	pixel_ptr=bmp_get_pixel(bmp,x+1,y);
	if(pixel_ptr!=NULL){
		blue+=pixel_ptr->blue;
		red+=pixel_ptr->red;
		green+=pixel_ptr->green;
		alpha+=pixel_ptr->alpha;
		counter++;
	}
	pixel_ptr=bmp_get_pixel(bmp,x,y-1);
	if(pixel_ptr!=NULL){
		blue+=pixel_ptr->blue;
		red+=pixel_ptr->red;
		green+=pixel_ptr->green;
		alpha+=pixel_ptr->alpha;
		counter++;
	}
	pixel_ptr=bmp_get_pixel(bmp,x,y+1);
	if(pixel_ptr!=NULL){
		blue+=pixel_ptr->blue;
		red+=pixel_ptr->red;
		green+=pixel_ptr->green;
		alpha+=pixel_ptr->alpha;
		counter++;
	}
	blue/=counter;
	red/=counter;
	green/=counter;
	alpha/=counter;
	ret_pixel.blue=blue;
	ret_pixel.red=red;
	ret_pixel.green=green;
	ret_pixel.alpha=alpha;

	return ret_pixel;
}



void	example_bmp(){

	bmpfile_t *bmp;
	int i, j;
	rgb_pixel_t pixel ;

/*
	 bmp=bmp_create(WIDTH, HEIGHT,DEPTH);



	for (i = 10, j = 10; j < HEIGHT; ++i, ++j) {
		bmp_set_pixel(bmp, i, j, pixel);
		pixel.red++;
		pixel.green++;
		pixel.blue++;
		bmp_set_pixel(bmp, i + 1, j, pixel);
		bmp_set_pixel(bmp, i, j + 1, pixel);
	}

	bmp_save(bmp, "hello.bmp");
	bmp_destroy(bmp);
	*/
	bmp=readBMP("/input.bmp");
	for (j = 0; j < bmp_get_height(bmp);j++) {
		for(i=0;i<bmp_get_width(bmp);i++){
			bmp_set_pixel(bmp, i,j, get_avg_pixel(bmp,i,j) );

		}

	}
	bmp_save(bmp, "/try.bmp");
	bmp_destroy(bmp);
	fl_shutdown();
}

