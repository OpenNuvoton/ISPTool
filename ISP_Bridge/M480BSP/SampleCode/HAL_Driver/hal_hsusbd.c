#include <stdio.h>
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

void _EP_HID_IN_Handler(uint32_t u32Ep, uint8_t *pu8Buf, uint32_t u32rLen)  /* Interrupt IN handler */
{
    uint32_t i;

    /* Trigger HID IN */
    for (i = 0; i < u32rLen; i++) {
        HSUSBD->EP[u32Ep].EPDAT_BYTE = pu8Buf[i];
    }

    HSUSBD->EP[u32Ep].EPTXCNT = 64;
    HSUSBD_ENABLE_EP_INT(u32Ep, HSUSBD_EPINTEN_INTKIEN_Msk);
}

void _EP_HID_OUT_Handler(uint32_t u32Ep, uint8_t *pu8Buf)  /* Interrupt OUT handler */
{
    uint32_t len, i;
    len = HSUSBD->EP[u32Ep].EPDATCNT & 0xffff;

    for (i = 0; i < len; i++) {
        pu8Buf[i] = HSUSBD->EP[u32Ep].EPDAT_BYTE;
    }
}

#ifdef __cplusplus
}
#endif
