/**************************************************************************//**
 * @file     usbd_reg.h
 * @version  V1.00
 * @brief    USBD register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __USBD_REG_H__
#define __USBD_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */



/*---------------------- Universal Serial Bus Device Controller -------------------------*/
/**
    @addtogroup USBD Universal Serial Bus Device Controller (USBD)
    Memory Mapped Structure for USBD Controller
@{ */


typedef struct
{


/**
 * @var USBD_EP_T::BUFSEG
 * Offset: 0x500/0x510/0x520/0x530/0x540/0x550/0x560/0x570  Endpoint 0~7 Buffer Segmentation Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:3]   |BUFSEG    |Endpoint Buffer Segmentation
 * |        |          |It is used to indicate the offset address for each endpoint with the USB SRAM starting address
 * |        |          |The effective starting address of the endpoint is
 * |        |          |USB_SRAM address + { BUFSEG[8:3], 3'b000}
 * |        |          |Where the USB_SRAM address = USBD_BA+0x100h.
 * |        |          |Refer to the section 5.4.4.7 for the endpoint SRAM structure and its description.
 * @var USBD_EP_T::MXPLD
 * Offset: 0x504/0x514/0x524/0x534/0x544/0x554/0x564/0x574  Endpoint 0~7 Maximal Payload Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:0]   |MXPLD     |Maximal Payload
 * |        |          |Define the data length which is transmitted to host (IN token) or the actual data length which
 * |        |          |is received from the host (OUT token).
 * |        |          |It also used to indicate that the endpoint is ready to be transmitted in IN token or received in
 * |        |          |OUT token.
 * |        |          |(1) When the register is written by CPU,
 * |        |          |For IN token, the value of MXPLD is used to define the data length to be transmitted and
 * |        |          |indicate the data buffer is ready.
 * |        |          |For OUT token, it means that the controller is ready to receive data from the host and the value
 * |        |          |of MXPLD is the maximal data length comes from host.
 * |        |          |(2) When the register is read by CPU,
 * |        |          |For IN token, the value of MXPLD is indicated by the data length be transmitted to host
 * |        |          |For OUT token, the value of MXPLD is indicated the actual data length receiving from host.
 * |        |          |Note: Once MXPLD is written, the data packets will be transmitted/received immediately after
 * |        |          |IN/OUT token arrived.
 * @var USBD_EP_T::CFG
 * Offset: 0x508/0x518/0x528/0x538/0x548/0x558/0x568/0x578  Endpoint 0~7 Configuration Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |EP_NUM    |Endpoint Number
 * |        |          |These bits are used to define the endpoint number of the current endpoint.
 * |[4]     |ISOCH     |Isochronous Endpoint
 * |        |          |This bit is used to set the endpoint as Isochronous endpoint, no handshake.
 * |        |          |0 = No Isochronous endpoint.
 * |        |          |1 = Isochronous endpoint.
 * |[6:5]   |STATE     |Endpoint STATE
 * |        |          |00 = Endpoint is Disabled.
 * |        |          |01 = Out endpoint.
 * |        |          |10 = IN endpoint.
 * |        |          |11 = Undefined.
 * |[7]     |DSQ_SYNC  |Data Sequence Synchronization
 * |        |          |0 = DATA0 PID.
 * |        |          |1 = DATA1 PID.
 * |        |          |Note: It is used to specify the DATA0 or DATA1 PID in the following IN token transaction.
 * |        |          |Hardware will toggle automatically in IN token base on the bit.
 * |[9]     |CSTALL    |Clear STALL Response
 * |        |          |0 = Disable the device to clear the STALL handshake in setup stage.
 * |        |          |1 = Clear the device to response STALL handshake in setup stage.
 * @var USBD_EP_T::CFGP
 * Offset: 0x50C/0x51C/0x52C/0x53C/0x54C/0x55C/0x56C/0x57C  Endpoint 0~7 Set Stall and Clear In/Out Ready Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CLRRDY    |Clear Ready
 * |        |          |When the USB_MXPLD register is set by user, it means that the endpoint is ready to transmit or
 * |        |          |receive data.
 * |        |          |If the user wants to turn off this transaction before the transaction start, users can set this
 * |        |          |bit to 1 to turn it off and it will be cleared to 0 automatically.
 * |        |          |For IN token, write '1' to clear the IN token had ready to transmit the data to USB.
 * |        |          |For OUT token, write '1' to clear the OUT token had ready to receive the data from USB.
 * |        |          |This bit is write 1 only and is always 0 when it is read back.
 * |[1]     |SSTALL    |Set STALL
 * |        |          |0 = Disable the device to response STALL.
 * |        |          |1 = Set the device to respond STALL automatically.
 */

    __IO uint32_t BUFSEG;        /* Offset: 0x500/0x510/0x520/0x530/0x540/0x550/0x560/0x570  Endpoint 0~7 Buffer Segmentation Register */
    __IO uint32_t MXPLD;         /* Offset: 0x504/0x514/0x524/0x534/0x544/0x554/0x564/0x574  Endpoint 0~7 Maximal Payload Register */
    __IO uint32_t CFG;           /* Offset: 0x508/0x518/0x528/0x538/0x548/0x558/0x568/0x578  Endpoint 0~7 Configuration Register */
    __IO uint32_t CFGP;          /* Offset: 0x50C/0x51C/0x52C/0x53C/0x54C/0x55C/0x56C/0x57C  Endpoint 0~7 Set Stall and Clear In/Out Ready Control Register */

} USBD_EP_T;






typedef struct
{


/**
 * @var USBD_T::INTEN
 * Offset: 0x00  USB Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BUS_IE    |Bus Event Interrupt Enable
 * |        |          |0 = BUS event interrupt Disabled.
 * |        |          |1 = BUS event interrupt Enabled.
 * |[1]     |USB_IE    |USB Event Interrupt Enable
 * |        |          |0 = USB event interrupt Disabled.
 * |        |          |1 = USB event interrupt Enabled.
 * |[2]     |FLDET_IE  |Floating Detection Interrupt Enable
 * |        |          |0 = Floating detection Interrupt Disabled.
 * |        |          |1 = Floating detection Interrupt Enabled.
 * |[3]     |WAKEUP_IE |USB Wake-Up Interrupt Enable
 * |        |          |0 = Wake-up Interrupt Disabled.
 * |        |          |1 = Wake-up Interrupt Enabled.
 * |[8]     |WAKEUP_EN |Wake-Up Function Enable
 * |        |          |0 = USB wake-up function Disabled.
 * |        |          |1 = USB wake-up function Enabled.
 * |[15]    |INNAK_EN  |Active NAK Function And Its Status In IN Token
 * |        |          |0 = When device responds NAK after receiving IN token, IN NAK status will not be
 * |        |          |    updated to USBD_EPSTS register, so that the USB interrupt event will not be asserted.
 * |        |          |1 = IN NAK status will be updated to USBD_EPSTS register and the USB interrupt event
 * |        |          |    will be asserted, when the device responds NAK after receiving IN token.
 * @var USBD_T::INTSTS
 * Offset: 0x04  USB Interrupt Event Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BUS_STS   |BUS Interrupt Status
 * |        |          |The BUS event means that there is one of the suspense or the resume function in the bus.
 * |        |          |0 = No BUS event occurred.
 * |        |          |1 = Bus event occurred; check USB_ATTR[3:0] to know which kind of bus event was occurred,
 * |        |          |cleared by write 1 to USB_INTSTS[0].
 * |[1]     |USB_STS   |USB Event Interrupt Status
 * |        |          |The USB event includes the SETUP Token, IN Token, OUT ACK, ISO IN, or ISO OUT events in the bus.
 * |        |          |0 = No USB event occurred.
 * |        |          |1 = USB event occurred, check EPSTS0~7 to know which kind of USB event occurred.
 * |        |          |Cleared by write 1 to USB_INTSTS[1] or EPEVT0~7 and SETUP (USB_INTSTS[31]).
 * |[2]     |FLDET_STS |Floating Detection Interrupt Status
 * |        |          |0 = There is not attached/detached event in the USB.
 * |        |          |1 = There is attached/detached event in the USB bus and it is cleared by write 1 to
 * |        |          |USB_INTSTS[2].
 * |[3]     |WAKEUP_STS|Wake-Up Interrupt Status
 * |        |          |0 = No Wake-up event occurred.
 * |        |          |1 = Wake-up event occurred, cleared by write 1 to USB_INTSTS[3].
 * |[16]    |EPEVT0    |Endpoint 0's USB Event Status
 * |        |          |0 = No event occurred on endpoint 0.
 * |        |          |1 = USB event occurred on Endpoint 0, check USB_EPSTS[10:8] to know which kind of USB event was
 * |        |          |occurred, cleared by write 1 to USB_INTSTS[16] or USB_INTSTS[1].
 * |[17]    |EPEVT1    |Endpoint 1's USB Event Status
 * |        |          |0 = No event occurred on endpoint 1.
 * |        |          |1 = USB event occurred on Endpoint 1, check USB_EPSTS[13:11] to know which kind of USB event was
 * |        |          |occurred, cleared by write 1 to USB_INTSTS[17] or USB_INTSTS[1].
 * |[18]    |EPEVT2    |Endpoint 2's USB Event Status
 * |        |          |0 = No event occurred on endpoint 2.
 * |        |          |1 = USB event occurred on Endpoint 2, check USB_EPSTS[16:14] to know which kind of USB event was
 * |        |          |occurred, cleared by write 1 to USB_INTSTS[18] or USB_INTSTS[1].
 * |[19]    |EPEVT3    |Endpoint 3's USB Event Status
 * |        |          |0 = No event occurred on endpoint 3.
 * |        |          |1 = USB event occurred on Endpoint 3, check USB_EPSTS[19:17] to know which kind of USB event was
 * |        |          |occurred, cleared by write 1 to USB_INTSTS[19] or USB_INTSTS[1].
 * |[20]    |EPEVT4    |Endpoint 4's USB Event Status
 * |        |          |0 = No event occurred on endpoint 4.
 * |        |          |1 = USB event occurred on Endpoint 4, check USB_EPSTS[22:20] to know which kind of USB event was
 * |        |          |occurred, cleared by write 1 to USB_INTSTS[20] or USB_INTSTS[1].
 * |[21]    |EPEVT5    |Endpoint 5's USB Event Status
 * |        |          |0 = No event occurred on endpoint 5.
 * |        |          |1 = USB event occurred on Endpoint 5, check USB_EPSTS[25:23] to know which kind of USB event was
 * |        |          |occurred, cleared by write 1 to USB_INTSTS[21] or USB_INTSTS[1].
 * |[22]    |EPEVT6    |Endpoint 6's USB Event Status
 * |        |          |0 = No event occurred on endpoint 6.
 * |        |          |1 = USB event occurred on Endpoint 6, check USB_EPSTS[28:26] to know which kind of USB event was
 * |        |          |occurred, cleared by write 1 to USB_INTSTS[22] or USB_INTSTS[1].
 * |[23]    |EPEVT7    |Endpoint 7's USB Event Status
 * |        |          |0 = No event occurred on endpoint 7.
 * |        |          |1 = USB event occurred on Endpoint 7, check USB_EPSTS[31:29] to know which kind of USB event was
 * |        |          |occurred, cleared by write 1 to USB_INTSTS[23] or USB_INTSTS[1].
 * |[31]    |SETUP     |Setup Event Status
 * |        |          |0 = No Setup event.
 * |        |          |1 = SETUP event occurred, cleared by write 1 to USB_INTSTS[31].
 * @var USBD_T::FADDR
 * Offset: 0x08  USB Device Function Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[6:0]   |FADDR     |USB Device Function Address
 * @var USBD_T::EPSTS
 * Offset: 0x0C  USB Endpoint Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7]     |OVERRUN   |Overrun
 * |        |          |It indicates that the received data is over the maximum payload number or not.
 * |        |          |0 = No overrun.
 * |        |          |1 = Out Data is more than the Max Payload in MXPLD register or the Setup Data is more than 8
 * |        |          |Bytes.
 * |[10:8]  |EPSTS0    |Endpoint 0 Bus Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |000 = In ACK.
 * |        |          |001 = In NAK.
 * |        |          |010 = Out Packet Data0 ACK.
 * |        |          |110 = Out Packet Data1 ACK.
 * |        |          |011 = Setup ACK.
 * |        |          |111 = Isochronous transfer end.
 * |[13:11] |EPSTS1    |Endpoint 1 Bus Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |000 = In ACK.
 * |        |          |001 = In NAK.
 * |        |          |010 = Out Packet Data0 ACK.
 * |        |          |110 = Out Packet Data1 ACK.
 * |        |          |011 = Setup ACK.
 * |        |          |111 = Isochronous transfer end.
 * |[16:14] |EPSTS2    |Endpoint 2 Bus Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |000 = In ACK.
 * |        |          |001 = In NAK.
 * |        |          |010 = Out Packet Data0 ACK.
 * |        |          |110 = Out Packet Data1 ACK.
 * |        |          |011 = Setup ACK.
 * |        |          |111 = Isochronous transfer end.
 * |[19:17] |EPSTS3    |Endpoint 3 Bus Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |000 = In ACK.
 * |        |          |001 = In NAK.
 * |        |          |010 = Out Packet Data0 ACK.
 * |        |          |110 = Out Packet Data1 ACK.
 * |        |          |011 = Setup ACK.
 * |        |          |111 = Isochronous transfer end.
 * |[22:20] |EPSTS4    |Endpoint 4 Bus Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |000 = In ACK.
 * |        |          |001 = In NAK.
 * |        |          |010 = Out Packet Data0 ACK.
 * |        |          |110 = Out Packet Data1 ACK.
 * |        |          |011 = Setup ACK.
 * |        |          |111 = Isochronous transfer end.
 * |[25:23] |EPSTS5    |Endpoint 5 Bus Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |000 = In ACK.
 * |        |          |001 = In NAK.
 * |        |          |010 = Out Packet Data0 ACK.
 * |        |          |110 = Out Packet Data1 ACK.
 * |        |          |011 = Setup ACK.
 * |        |          |111 = Isochronous transfer end.
 * |[28:26] |EPSTS6    |Endpoint 6 Bus Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |000 = In ACK.
 * |        |          |001 = In NAK.
 * |        |          |010 = Out Packet Data0 ACK.
 * |        |          |110 = Out Packet Data1 ACK.
 * |        |          |011 = Setup ACK.
 * |        |          |111 = Isochronous transfer end.
 * |[31:29] |EPSTS7    |Endpoint 7 Bus Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |000 = In ACK.
 * |        |          |001 = In NAK.
 * |        |          |010 = Out Packet Data0 ACK.
 * |        |          |110 = Out Packet Data1 ACK.
 * |        |          |011 = Setup ACK.
 * |        |          |111 = Isochronous transfer end.
 * @var USBD_T::ATTR
 * Offset: 0x10  USB Bus Status and Attribution Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |USBRST    |USB Reset Status
 * |        |          |0 = Bus no reset.
 * |        |          |1 = Bus reset when SE0 (single-ended 0) is presented more than 2.5us.
 * |        |          |Note: This bit is read only.
 * |[1]     |SUSPEND   |Suspend Status
 * |        |          |0 = Bus no suspend.
 * |        |          |1 = Bus idle more than 3ms, either cable is plugged off or host is sleeping.
 * |        |          |Note: This bit is read only.
 * |[2]     |RESUME    |Resume Status
 * |        |          |0 = No bus resume.
 * |        |          |1 = Resume from suspend.
 * |        |          |Note: This bit is read only.
 * |[3]     |TIMEOUT   |Time-Out Status
 * |        |          |0 = No time-out.
 * |        |          |1 = No Bus response more than 18 bits time.
 * |        |          |Note: This bit is read only.
 * |[4]     |PHY_EN    |PHY Transceiver Function Enable
 * |        |          |0 = PHY transceiver function Disabled.
 * |        |          |1 = PHY transceiver function Enabled.
 * |[5]     |RWAKEUP   |Remote Wake-Up
 * |        |          |0 = Release the USB bus from K state.
 * |        |          |1 = Force USB bus to K (USB_D+ low, USB_D- high) state, used for remote wake-up.
 * |[7]     |USB_EN    |USB Controller Enable
 * |        |          |0 = USB Controller Disabled.
 * |        |          |1 = USB Controller Enabled.
 * |[8]     |DPPU_EN   |Pull-Up Resistor On USB_D+ Enable
 * |        |          |0 = Pull-up resistor in USB_D+ pin Disabled.
 * |        |          |1 = Pull-up resistor in USB_D+ pin Enabled.
 * |[9]     |PWRDN     |Power-Down PHY Transceiver, Low Active
 * |        |          |0 = Power-down related circuit of PHY transceiver.
 * |        |          |1 = Turn-on related circuit of PHY transceiver.
 * |[10]    |BYTEM     |CPU Access USB SRAM Size Mode Selection
 * |        |          |0 = Word mode: The size of the transfer from CPU to USB SRAM can be Word only.
 * |        |          |1 = Byte mode: The size of the transfer from CPU to USB SRAM can be Byte only.
 * @var USBD_T::FLDET
 * Offset: 0x14  USB Floating Detection Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FLDET     |Device Floating Detected
 * |        |          |0 = Controller is not attached into the USB host.
 * |        |          |1 =Controller is attached into the BUS.
 * @var USBD_T::STBUFSEG
 * Offset: 0x18  Setup Token Buffer Segmentation Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:3]   |STBUFSEG  |Setup Token Buffer Segmentation
 * |        |          |It is used to indicate the offset address for the SETUP token with the USB Device SRAM starting
 * |        |          |address The effective starting address is
 * |        |          |USB_SRAM address + {STBUFSEG[8:3], 3'b000}
 * |        |          |Where the USB_SRAM address = USBD_BA+0x100h.
 * |        |          |Note: It is used for SETUP token only.
 * @var USBD_T::DRVSE0
 * Offset: 0x90  USB Drive SE0 Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DRVSE0    |Drive Single Ended Zero In USB Bus
 * |        |          |The Single Ended Zero (SE0) is when both lines (USB_D+ and USB_D-) are being pulled low.
 * |        |          |0 = None.
 * |        |          |1 = Force USB PHY transceiver to drive SE0.
 */

    __IO uint32_t INTEN;         /* Offset: 0x00  USB Interrupt Enable Register                                      */
    __IO uint32_t INTSTS;        /* Offset: 0x04  USB Interrupt Event Status Register                                */
    __IO uint32_t FADDR;         /* Offset: 0x08  USB Device Function Address Register                               */
    __I  uint32_t EPSTS;         /* Offset: 0x0C  USB Endpoint Status Register                                       */
    __IO uint32_t ATTR;          /* Offset: 0x10  USB Bus Status and Attribution Register                            */
    __I  uint32_t FLDET;         /* Offset: 0x14  USB Floating Detection Register                                    */
    __IO uint32_t STBUFSEG;      /* Offset: 0x18  Setup Token Buffer Segmentation Register                           */
    __I  uint32_t RESERVE1[29];  
    __IO uint32_t DRVSE0;        /* Offset: 0x90  USB Drive SE0 Control Register                                     */
    __I  uint32_t RESERVE2[283];
        USBD_EP_T EP[8];         /* Offset: 0x500 Endpoint 0~7 Control Registers                                     */

} USBD_T;


/**
    @addtogroup USBD_CONST USBD Bit Field Definition
    Constant Definitions for USBD Controller
@{ */


/* USBD INTEN Bit Field Definitions */
#define USBD_INTEN_INNAK_EN_Pos    15                                    /*!< USBD_T::INTEN: INNAK_EN Position */
#define USBD_INTEN_INNAK_EN_Msk    (1ul << USBD_INTEN_INNAK_EN_Pos)      /*!< USBD_T::INTEN: INNAK_EN Mask */

#define USBD_INTEN_WAKEUP_EN_Pos   8                                     /*!< USBD_T::INTEN: RWAKEUP Position */
#define USBD_INTEN_WAKEUP_EN_Msk   (1ul << USBD_INTEN_WAKEUP_EN_Pos)     /*!< USBD_T::INTEN: RWAKEUP Mask */

#define USBD_INTEN_WAKEUP_IE_Pos   3                                     /*!< USBD_T::INTEN: WAKEUP_IE Position */
#define USBD_INTEN_WAKEUP_IE_Msk   (1ul << USBD_INTEN_WAKEUP_IE_Pos)     /*!< USBD_T::INTEN: WAKEUP_IE Mask */

#define USBD_INTEN_FLDET_IE_Pos    2                                     /*!< USBD_T::INTEN: FLDET_IE Position */
#define USBD_INTEN_FLDET_IE_Msk    (1ul << USBD_INTEN_FLDET_IE_Pos)      /*!< USBD_T::INTEN: FLDET_IE Mask */

#define USBD_INTEN_USB_IE_Pos      1                                     /*!< USBD_T::INTEN: USB_IE Position */
#define USBD_INTEN_USB_IE_Msk      (1ul << USBD_INTEN_USB_IE_Pos)        /*!< USBD_T::INTEN: USB_IE Mask */

#define USBD_INTEN_BUS_IE_Pos      0                                     /*!< USBD_T::INTEN: BUS_IE Position */
#define USBD_INTEN_BUS_IE_Msk      (1ul << USBD_INTEN_BUS_IE_Pos)        /*!< USBD_T::INTEN: BUS_IE Mask */

/* USBD INTSTS Bit Field Definitions */
#define USBD_INTSTS_SETUP_Pos        31                                  /*!< USBD_T::INTSTS: SETUP Position */
#define USBD_INTSTS_SETUP_Msk        (1ul << USBD_INTSTS_SETUP_Pos)      /*!< USBD_T::INTSTS: SETUP Mask */

#define USBD_INTSTS_EPEVT7_Pos       23                                  /*!< USBD_T::INTSTS: EPEVT7 Position */
#define USBD_INTSTS_EPEVT7_Msk       (0x1ul << USBD_INTSTS_EPEVT7_Pos)   /*!< USBD_T::INTSTS: EPEVT7 Mask     */

#define USBD_INTSTS_EPEVT6_Pos       22                                  /*!< USBD_T::INTSTS: EPEVT6 Position */
#define USBD_INTSTS_EPEVT6_Msk       (0x1ul << USBD_INTSTS_EPEVT6_Pos)   /*!< USBD_T::INTSTS: EPEVT6 Mask     */

#define USBD_INTSTS_EPEVT5_Pos       21                                  /*!< USBD_T::INTSTS: EPEVT5 Position */
#define USBD_INTSTS_EPEVT5_Msk       (0x1ul << USBD_INTSTS_EPEVT5_Pos)   /*!< USBD_T::INTSTS: EPEVT5 Mask     */

#define USBD_INTSTS_EPEVT4_Pos       20                                  /*!< USBD_T::INTSTS: EPEVT4 Position */
#define USBD_INTSTS_EPEVT4_Msk       (0x1ul << USBD_INTSTS_EPEVT4_Pos)   /*!< USBD_T::INTSTS: EPEVT4 Mask     */

#define USBD_INTSTS_EPEVT3_Pos       19                                  /*!< USBD_T::INTSTS: EPEVT3 Position */
#define USBD_INTSTS_EPEVT3_Msk       (0x1ul << USBD_INTSTS_EPEVT3_Pos)   /*!< USBD_T::INTSTS: EPEVT3 Mask     */

#define USBD_INTSTS_EPEVT2_Pos       18                                  /*!< USBD_T::INTSTS: EPEVT2 Position */
#define USBD_INTSTS_EPEVT2_Msk       (0x1ul << USBD_INTSTS_EPEVT2_Pos)   /*!< USBD_T::INTSTS: EPEVT2 Mask     */

#define USBD_INTSTS_EPEVT1_Pos       17                                  /*!< USBD_T::INTSTS: EPEVT1 Position */
#define USBD_INTSTS_EPEVT1_Msk       (0x1ul << USBD_INTSTS_EPEVT1_Pos)   /*!< USBD_T::INTSTS: EPEVT1 Mask     */

#define USBD_INTSTS_EPEVT0_Pos       16                                  /*!< USBD_T::INTSTS: EPEVT0 Position */
#define USBD_INTSTS_EPEVT0_Msk       (0x1ul << USBD_INTSTS_EPEVT0_Pos)   /*!< USBD_T::INTSTS: EPEVT0 Mask     */

#define USBD_INTSTS_WAKEUP_STS_Pos   3                                   /*!< USBD_T::INTSTS: WAKEUP_STS Position */
#define USBD_INTSTS_WAKEUP_STS_Msk   (1ul << USBD_INTSTS_WAKEUP_STS_Pos) /*!< USBD_T::INTSTS: WAKEUP_STS Mask */

#define USBD_INTSTS_FLDET_STS_Pos    2                                   /*!< USBD_T::INTSTS: FLDET_STS Position */
#define USBD_INTSTS_FLDET_STS_Msk    (1ul << USBD_INTSTS_FLDET_STS_Pos)  /*!< USBD_T::INTSTS: FLDET_STS Mask */

#define USBD_INTSTS_USB_STS_Pos      1                                   /*!< USBD_T::INTSTS: USB_STS Position */
#define USBD_INTSTS_USB_STS_Msk      (1ul << USBD_INTSTS_USB_STS_Pos)    /*!< USBD_T::INTSTS: USB_STS Mask */

#define USBD_INTSTS_BUS_STS_Pos      0                                   /*!< USBD_T::INTSTS: BUS_STS Position */
#define USBD_INTSTS_BUS_STS_Msk      (1ul << USBD_INTSTS_BUS_STS_Pos)    /*!< USBD_T::INTSTS: BUS_STS Mask */

/* USBD FADDR Bit Field Definitions */
#define USBD_FADDR_FADDR_Pos     0                                       /*!< USBD_T::FADDR: FADDR Position */
#define USBD_FADDR_FADDR_Msk     (0x7Ful << USBD_FADDR_FADDR_Pos)        /*!< USBD_T::FADDR: FADDR Mask */

/* USBD EPSTS Bit Field Definitions */
#define USBD_EPSTS_EPSTS5_Pos    23                                      /*!< USBD_T::EPSTS: EPSTS5 Position */
#define USBD_EPSTS_EPSTS5_Msk    (7ul << USBD_EPSTS_EPSTS5_Pos)          /*!< USBD_T::EPSTS: EPSTS5 Mask */

#define USBD_EPSTS_EPSTS4_Pos    20                                      /*!< USBD_T::EPSTS: EPSTS4 Position */
#define USBD_EPSTS_EPSTS4_Msk    (7ul << USBD_EPSTS_EPSTS4_Pos)          /*!< USBD_T::EPSTS: EPSTS5 Mask */

#define USBD_EPSTS_EPSTS3_Pos    17                                      /*!< USBD_T::EPSTS: EPSTS3 Position */
#define USBD_EPSTS_EPSTS3_Msk    (7ul << USBD_EPSTS_EPSTS3_Pos)          /*!< USBD_T::EPSTS: EPSTS3 Mask */

#define USBD_EPSTS_EPSTS2_Pos    14                                      /*!< USBD_T::EPSTS: EPSTS2 Position */
#define USBD_EPSTS_EPSTS2_Msk    (7ul << USBD_EPSTS_EPSTS2_Pos)          /*!< USBD_T::EPSTS: EPSTS2 Mask */

#define USBD_EPSTS_EPSTS1_Pos    11                                      /*!< USBD_T::EPSTS: EPSTS1 Position */
#define USBD_EPSTS_EPSTS1_Msk    (7ul << USBD_EPSTS_EPSTS1_Pos)          /*!< USBD_T::EPSTS: EPSTS1 Mask */

#define USBD_EPSTS_EPSTS0_Pos    8                                       /*!< USBD_T::EPSTS: EPSTS0 Position */
#define USBD_EPSTS_EPSTS0_Msk    (7ul << USBD_EPSTS_EPSTS0_Pos)          /*!< USBD_T::EPSTS: EPSTS0 Mask */

#define USBD_EPSTS_OVERRUN_Pos   7                                       /*!< USBD_T::EPSTS: OVERRUN Position */
#define USBD_EPSTS_OVERRUN_Msk   (1ul << USBD_EPSTS_OVERRUN_Pos)         /*!< USBD_T::EPSTS: OVERRUN Mask */

/* USBD ATTR Bit Field Definitions */
#define USBD_ATTR_BYTEM_Pos      10                                      /*!< USBD_T::ATTR: BYTEM Position */
#define USBD_ATTR_BYTEM_Msk      (1ul << USBD_ATTR_BYTEM_Pos)            /*!< USBD_T::ATTR: BYTEM Mask */

#define USBD_ATTR_PWRDN_Pos      9                                       /*!< USBD_T::ATTR: PWRDN Position */
#define USBD_ATTR_PWRDN_Msk      (1ul << USBD_ATTR_PWRDN_Pos)            /*!< USBD_T::ATTR: PWRDN Mask */

#define USBD_ATTR_DPPU_EN_Pos    8                                       /*!< USBD_T::ATTR: DPPU_EN Position */
#define USBD_ATTR_DPPU_EN_Msk    (1ul << USBD_ATTR_DPPU_EN_Pos)          /*!< USBD_T::ATTR: DPPU_EN Mask */

#define USBD_ATTR_USB_EN_Pos     7                                       /*!< USBD_T::ATTR: USB_EN Position */
#define USBD_ATTR_USB_EN_Msk     (1ul << USBD_ATTR_USB_EN_Pos)           /*!< USBD_T::ATTR: USB_EN Mask */

#define USBD_ATTR_RWAKEUP_Pos    5                                       /*!< USBD_T::ATTR: RWAKEUP Position */
#define USBD_ATTR_RWAKEUP_Msk    (1ul << USBD_ATTR_RWAKEUP_Pos)          /*!< USBD_T::ATTR: RWAKEUP Mask */

#define USBD_ATTR_PHY_EN_Pos     4                                       /*!< USBD_T::ATTR: PHY_EN Position */
#define USBD_ATTR_PHY_EN_Msk     (1ul << USBD_ATTR_PHY_EN_Pos)           /*!< USBD_T::ATTR: PHY_EN Mask */

#define USBD_ATTR_TIMEOUT_Pos    3                                       /*!< USBD_T::ATTR: TIMEOUT Position */
#define USBD_ATTR_TIMEOUT_Msk    (1ul << USBD_ATTR_TIMEOUT_Pos)          /*!< USBD_T::ATTR: TIMEOUT Mask */

#define USBD_ATTR_RESUME_Pos     2                                       /*!< USBD_T::ATTR: RESUME Position */
#define USBD_ATTR_RESUME_Msk     (1ul << USBD_ATTR_RESUME_Pos)           /*!< USBD_T::ATTR: RESUME Mask */

#define USBD_ATTR_SUSPEND_Pos    1                                       /*!< USBD_T::ATTR: SUSPEND Position */
#define USBD_ATTR_SUSPEND_Msk    (1ul << USBD_ATTR_SUSPEND_Pos)          /*!< USBD_T::ATTR: SUSPEND Mask */

#define USBD_ATTR_USBRST_Pos     0                                       /*!< USBD_T::ATTR: USBRST Position */
#define USBD_ATTR_USBRST_Msk     (1ul << USBD_ATTR_USBRST_Pos)           /*!< USBD_T::ATTR: USBRST Mask */

/* USBD FLDET Bit Field Definitions */
#define USBD_FLDET_FLDET_Pos     0                                       /*!< USBD_T::FLDET: FLDET Position */
#define USBD_FLDET_FLDET_Msk     (1ul << USBD_FLDET_FLDET_Pos)           /*!< USBD_T::FLDET: FLDET Mask */

/* USBD STBUFSEG Bit Field Definitions */
#define USBD_STBUFSEG_STBUFSEG_Pos   3                                        /*!< USBD_T::STBUFSEG: STBUFSEG Position */
#define USBD_STBUFSEG_STBUFSEG_Msk   (0x3Ful << USBD_STBUFSEG_STBUFSEG_Pos)   /*!< USBD_T::STBUFSEG: STBUFSEG Mask */

/* USBD BUFSEG Bit Field Definitions */
#define USBD_BUFSEG_BUFSEG_Pos   3                                       /*!< USBD_EP_T::BUFSEG: BUFSEG Position */
#define USBD_BUFSEG_BUFSEG_Msk   (0x3Ful << USBD_BUFSEG_BUFSEG_Pos)      /*!< USBD_EP_T::BUFSEG: BUFSEG Mask */

/* USBD MXPLD Bit Field Definitions */
#define USBD_MXPLD_MXPLD_Pos    0                                        /*!< USBD_EP_T::MXPLD: MXPLD Position */
#define USBD_MXPLD_MXPLD_Msk    (0x1FFul << USBD_MXPLD_MXPLD_Pos)        /*!< USBD_EP_T::MXPLD: MXPLD Mask */

/* USBD CFG Bit Field Definitions */
#define USBD_CFG_CSTALL_Pos     9                                        /*!< USBD_EP_T::CFG: CSTALL Position */
#define USBD_CFG_CSTALL_Msk     (1ul << USBD_CFG_CSTALL_Pos)             /*!< USBD_EP_T::CFG: CSTALL Mask */

#define USBD_CFG_DSQ_SYNC_Pos   7                                        /*!< USBD_EP_T::CFG: DSQ_SYNC Position */
#define USBD_CFG_DSQ_SYNC_Msk   (1ul << USBD_CFG_DSQ_SYNC_Pos)           /*!< USBD_EP_T::CFG: DSQ_SYNC Mask */

#define USBD_CFG_STATE_Pos      5                                        /*!< USBD_EP_T::CFG: STATE Position */
#define USBD_CFG_STATE_Msk      (3ul << USBD_CFG_STATE_Pos)              /*!< USBD_EP_T::CFG: STATE Mask */

#define USBD_CFG_ISOCH_Pos      4                                        /*!< USBD_EP_T::CFG: ISOCH Position */
#define USBD_CFG_ISOCH_Msk      (1ul << USBD_CFG_ISOCH_Pos)              /*!< USBD_EP_T::CFG: ISOCH Mask */

#define USBD_CFG_EP_NUM_Pos     0                                        /*!< USBD_EP_T::CFG: EP_NUM Position */
#define USBD_CFG_EP_NUM_Msk     (0xFul << USBD_CFG_EP_NUM_Pos)           /*!< USBD_EP_T::CFG: EP_NUM Mask */

/* USBD CFGP Bit Field Definitions */
#define USBD_CFGP_SSTALL_Pos    1                                        /*!< USBD_EP_T::CFGP: SSTALL Position */
#define USBD_CFGP_SSTALL_Msk    (1ul << USBD_CFGP_SSTALL_Pos)            /*!< USBD_EP_T::CFGP: SSTALL Mask */

#define USBD_CFGP_CLRRDY_Pos    0                                        /*!< USBD_EP_T::CFGP: CLRRDY Position */
#define USBD_CFGP_CLRRDY_Msk    (1ul << USBD_CFGP_CLRRDY_Pos)            /*!< USBD_EP_T::CFGP: CLRRDY Mask */

/* USBD DRVSE0 Bit Field Definitions */
#define USBD_DRVSE0_DRVSE0_Pos   0                                       /*!< USBD_T::DRVSE0: DRVSE0 Position */
#define USBD_DRVSE0_DRVSE0_Msk   (1ul << USBD_DRVSE0_DRVSE0_Pos)         /*!< USBD_T::DRVSE0: DRVSE0 Mask */
/*@}*/ /* end of group USBD_CONST */
/*@}*/ /* end of group USBD */
/**@}*/ /* end of REGISTER group */


#endif /* __USBD_REG_H__ */
