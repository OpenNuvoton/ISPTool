extern void Open_SPI_Flash(void);
extern unsigned int SpiFlash_ReadMidDid(void);
extern void SpiFlash_ChipErase(void);
extern void SpiFlash_PageProgram(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount);
extern void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount);

