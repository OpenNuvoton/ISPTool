#define  VBOD18    6
#define  VBOD20    5
#define  VBOD24    4
#define  VBOD27    3
#define  VBOD30    2
#define  VBOD37    1
#define  VBOD44    0

#define  BOD_RESET_DISABLE  0
#define  BOD_RESET_ENABLE   1

#define  LPBOD_MODE0    0
#define  LPBOD_MODE1    1
#define  LPBOD_MODE2    2
#define  LPBOD_MODE3    3

#define  BOD_FT_DISABLE     0
#define  BOD_FT_ENABLE      1

void BOD_Disable(void);
void BOD_Enable(unsigned char u8BODLEVEL,unsigned char u8RSTENABLE);
void BOD_LowPower_Enable(unsigned char u8BODLPMODE,unsigned char u8BODFTEN);
void POR_Disable(void);
void POR_Enable(void);
void LVR_Disable(void);
void LVR_Enable(void);
void LowPowerLVR_Enable(void);
void BIAS_ALL_DIGITAL(void);