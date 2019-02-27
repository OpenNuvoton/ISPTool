#define GPIO_MODE_QUASI  0
#define	GPIO_MODE_PUSHPULL 1
#define GPIO_MODE_INPUT  2
#define GPIO_MODE_OPENDRAIN  3

#define  Port0     0
#define  Port1     1
#define  Port2     2
#define  Port3     3
#define  Port4     4
#define  Port5     5

#define  PullUp     0
#define  PullDown   1

void GPIO_SetMode(unsigned char u8Port, unsigned char u8PinMask, unsigned char u8Mode);
void GPIO_Pull_Enable(unsigned char u8Port, unsigned char u8PinMask, unsigned char u8PullMode);
void GPIO_Pull_Disable(unsigned char u8Port, unsigned char u8PinMask, unsigned char u8PullMode);
