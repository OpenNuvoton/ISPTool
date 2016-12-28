#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "diskio.h"
#include "NUC230_240.h"
#include "ISP_BSP.H"
#include "ISP_FILE_ACCESS.h"
FATFS Fs[1];		/* File system object for logical drive */
FIL file1;

void put_rc (FRESULT rc)
{
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");

    uint32_t i;
    for (i = 0; (i != (UINT)rc) && *p; i++) {
        while(*p++) ;
    }
    dbg_printf(_T("rc=%u FR_%s\n"), (UINT)rc, p);
}

unsigned int Check_SDCARD(void)		   
{
    FRESULT res; 	
    //mount file system
	res	=f_mount(&Fs[0],"0:",1); 		
    return res;	 
}


unsigned int GET_FILE_SIZE(void)
{
     
	 unsigned int file_size=0;
	FRESULT res; 	
	  res=f_open(&file1,BIN_FILE,FA_OPEN_EXISTING | FA_READ);								
		 if(res == FR_OK)
		 {
		  file_size=file1.fsize;            
		}			 					
	   f_close(&file1); 
	  return file_size;
}


unsigned int GET_FILE_CHECKSUM(void)
{
	
	FRESULT res; 	
unsigned int binfile_checksum,s1;
unsigned int temp_ct;
unsigned char Buff[512];
binfile_checksum=0;

	  res=f_open(&file1,BIN_FILE,FA_OPEN_EXISTING | FA_READ);								
		 if(res == FR_OK)
		 {			 			
		 for(;;)
			 {
			 res=f_read(&file1,Buff,sizeof(Buff),&s1);
			 if (res || s1 == 0) break;
				for(temp_ct=0;temp_ct<s1;temp_ct++)
				{
					binfile_checksum=binfile_checksum+Buff[temp_ct];
				}
			 }
			 
			
	   f_close(&file1); 
		 }
		 return binfile_checksum;
}

void write_log(char* logdata)
{	
FRESULT res; 	
unsigned int s1;
unsigned int l=0;
unsigned char buffer[256];
	  res=f_open(&file1,log_FILE,FA_WRITE);								
		 if(res == FR_OK)
		 {			 						 
    while(*logdata){
        if(*logdata<0x80)
					{  
            buffer[l]=	*logdata;					
            logdata++;
            l++;
				  }
			
		}
		//for next line
		buffer[l]=0x0d;
		l++;
		buffer[l]=0x0a;
		l++;
	 res=f_lseek(&file1, file1.fsize);//to file end
		res=f_write(&file1, buffer, l, &s1);	
		if(res==FR_OK)
	 f_close(&file1); 
		}
		 
}

