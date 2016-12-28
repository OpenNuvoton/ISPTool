#include "stdafx.h"
#include "ISP_COMMAND.h"
#include <time.h>


int _tmain(int argc, _TCHAR* argv[])
{
	clock_t start_time, end_time;
	float total_time = 0;
	start_time = clock(); /* mircosecond */

	ISP_COMMAND *ISP = new ISP_COMMAND();

	if (ISP->OPEN_USBPORT() != RES_CONNECT)
	{
		printf("USB NO FOUND\n\r");
		goto EXIT;
	}
	ISP->USB_TO_UART_AUTO_DETECT(); //THIS IS FOR AUTO DETECT

	//START ;
	ISP->SN_PACKAGE();

	ISP->SN_PACKAGE();
	//CHECK FW
	printf("FW version: 0x%x \n\r", ISP->READFW_VERSION());

	//CHECK PID
	if (ISP->READ_PID() == RES_FALSE)
	{
		printf("CHIP NO FOUND\n\r");
		goto EXIT;
	}

	//READ CONFIG
	printf("config \n\r");
	ISP->READ_CONFIG();


	if (ISP->File_Open_APROM(argv[1]) == RES_FILE_NO_FOUND)
	{
		printf("FILE NO FOUND\n\r");
		goto EXIT;
	}

	printf("File name: %s\n\r", argv[1]);
	printf("File size: %d\n\r", ISP->file_size);

	//test updata aprom
	ISP->UPDATE_APROM();

	//reboot mcu to aprom
	ISP->RUN_TO_APROM();

EXIT:
	//close usb port
	ISP->CLOSE_USBPORT();

	delete ISP;
	end_time = clock();
	/* CLOCKS_PER_SEC is defined at time.h */
	total_time = (float)(end_time - start_time) / CLOCKS_PER_SEC;

	printf("Time : %f sec \n", total_time);

	return 0;
	
}
