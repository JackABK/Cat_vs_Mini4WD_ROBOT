#include "fat_filelib.h"                                                                
#include "test_bmp.h"

void test_bmp()
{
	
	FL_FILE *read , *write;
	int i , j , w, h ; 
		/*Create the Bmp Header file structure*/	
		#pragma pack(1)
		static struct BMP_file_header {   //Total 54 bytes
				unsigned char Identifier[2];             		
				unsigned long  File_Size;            
				unsigned long  Reserved;          
				unsigned long  Bitmap_Data_Offset;  
				unsigned long   Bitmap_Header_Size;   
				unsigned long   Width;
				unsigned long   Height;       
				unsigned short  Planes;        
				unsigned short  Bits_Per_Pixel;      
				unsigned long   Compression;      
				unsigned long   Bitmap_Data_Size;  
				unsigned long   H_Resolution;        
				unsigned long   V_Resolution;         
				unsigned long   Used_Colors;           
				unsigned long   Important_Colors;  
		} BMP_header;  
		#pragma pack()



		/*read input image*/
		read = fl_fopen("/input.bmp" , "rb");
		write = fl_fopen("/22.bmp" , "wb");


		//copy the Bmp_header
		static struct BMP_file_header *header_temp; 
		header_temp=&BMP_header;
		fl_fread(header_temp,54,sizeof(unsigned char),read);                        
		fl_fwrite(header_temp,54,sizeof(unsigned char),write);

		//copy the color table BGRA total 256*4=1024 bytes                                                                       
		unsigned char color_table_temp[1024]={0}; 
		fl_fread(color_table_temp,1024,sizeof(unsigned char),read);
		fl_fwrite(color_table_temp,1024,sizeof(unsigned char),write);  


		//grab the width and height by header file  
		w = header_temp->Width ;
		h=  header_temp->Height;
		unsigned char imageData_old[w*h];

		//grab the imageData                                                                             
		fl_fread(imageData_old , w*h , sizeof(unsigned char) , read);
		fl_fwrite(imageData_old , w*h , sizeof(unsigned char) , write);
		printf("w is : %d\n" ,  w);
		printf("h is : %d\n" ,  h);
		printf("file size is %d\n" , (header_temp->File_Size));
		
		fl_fclose(read);
		fl_fclose(write);

		fl_shutdown();
}
