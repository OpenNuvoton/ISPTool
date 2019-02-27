

#define     PAGE_SIZE               128
extern unsigned char xdata DIDBuffer[4];
extern unsigned char xdata UIDBuffer[9];
extern unsigned char xdata UCIDBuffer[12];
extern unsigned char xdata IAPDataBuf[128];
extern unsigned char xdata IAPCFBuf[5];

void Trigger_IAP(void);
void Erase_LDROM(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize);
void LDROM_Blank_Check(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize);
void LDROM_Program(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize);
void LDROM_Read_Verify(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize);
void Erase_APROM(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize);
void APROM_Blank_Check(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize);
void APROM_Program(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize);
void APROM_Read_Verify(unsigned int u16IAPStartAddress, unsigned int u16IAPDataSize);
void Modify_CONFIG(unsigned char u8CF0,unsigned char u8CF1,unsigned char u8CF2,unsigned char u8CF3,unsigned char u8CF4);
void UID_Read(void);
void UCID_Read(void);
void DID_Read(void);