#define I2C0    0
#define I2C1    1

#define      I2C_R    1
#define      I2C_W		0

void I2C_Open(unsigned char u8I2CSel, unsigned long u32SYSCLK, unsigned long u32I2CCLK);
void I2C_Close(unsigned char u8I2CSel);
void I2C_EnableInt(unsigned char u8I2CSel);
void I2C_DisableInt(unsigned char u8I2CSel);
unsigned char I2C_GetStatus(unsigned char u8I2CSel);
void I2C_SetSlaveAddrMask(unsigned char u8I2CSel, unsigned char u8SlaveNo, unsigned char u8SlaveAddrMask);
void I2C_EnableTimeout(unsigned char u8I2CSel);
void I2C_DisableTimeout(unsigned char u8I2CSel);
void I2C_ClearTimeoutFlag(unsigned char u8I2CSel);