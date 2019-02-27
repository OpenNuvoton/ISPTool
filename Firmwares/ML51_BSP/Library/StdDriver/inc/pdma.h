#define   PDMA0       0
#define   PDMA1       1
#define   PDMA2       2
#define   PDMA3       3
#define   XRAM_XRAM   0   
#define   SPI0RX      1   
#define   SMCRX       2   
#define   SPI1RX      3  
#define   SPI0TX      5   
#define   SMCTX       6
#define   SPI1TX      7
#define   PDMAFULLINT 1
#define   PDMAHALFINT 2
#define   PDMAALLINT  3


void PDMA_Open( unsigned char u8PDMASel, unsigned char u8PDMASourceSel,unsigned int  u16PDMABAddress,unsigned char u8PDMACOUNT);
void PDMA_Interrupt_Enable(unsigned char u8PDMASel, unsigned char u8PDMAINTSel);
void PDMA_MTM_DestinationAddress(unsigned char u8PDMASel, unsigned int u16PDMADAddress);
void PDMA_Run(unsigned char u8PDMASel);
void PDMA_Close(unsigned char u8PDMASel);
void PDMA_Interrupt_Enable(unsigned char u8PDMASel,unsigned char u8PDMAINTSel);