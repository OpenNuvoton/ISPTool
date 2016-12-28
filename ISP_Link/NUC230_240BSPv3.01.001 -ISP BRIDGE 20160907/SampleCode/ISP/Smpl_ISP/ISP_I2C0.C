#include "NUC230_240.h"
#include "ISP_BSP.h"

void I2C0_INIT(void)
{
    /* Set GPA10,11 multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFP |= SYS_GPA_MFP_PA8_I2C0_SDA | SYS_GPA_MFP_PA9_I2C0_SCL;

    /* Enable I2C0 clock */
    CLK->APBCLK |= CLK_APBCLK_I2C0_EN_Msk;

    /* Reset I2C0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;
    I2C_Open(I2C0, 100000);    
}


#define Device_Addr0  0x36
unsigned char I2C_MasterRcvDataT1(void)
{
	uint32_t temp_count;
	I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_SI);
	while (I2C_GetIntFlag(I2C0) == 0);
	
	I2C_SetData(I2C0,(Device_Addr0<<1)|0x01);
	I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	while (I2C_GetIntFlag(I2C0) == 0);
	if(I2C_GetStatus(I2C0)!=0x40)
		return FALSE;
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{
		
		I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
		while (I2C_GetIntFlag(I2C0) == 0);		
	  rcvbuf[temp_count]=I2C_GetData(I2C0);
	}
	I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
	while((I2C0->I2CON & I2C_I2CON_STO_Msk) == I2C_I2CON_STO_Msk);
	return TRUE;
}

unsigned  char I2C_MasterSendDataT1(void)
{
	uint32_t temp_count;
	I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_SI);
	while (I2C_GetIntFlag(I2C0) == 0);
	
	I2C_SetData(I2C0,Device_Addr0<<1);
	I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	while (I2C_GetIntFlag(I2C0) == 0);
	if(I2C_GetStatus(I2C0)!=0x18)
		return FALSE;
	for (temp_count = 0; temp_count<Protocol_package; temp_count++)
	{		
		I2C_SetData(I2C0,sendbuf[temp_count]);
		I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
		while (I2C_GetIntFlag(I2C0) == 0);
	}
	I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
	while((I2C0->I2CON & I2C_I2CON_STO_Msk) == I2C_I2CON_STO_Msk);
	return TRUE;
}
