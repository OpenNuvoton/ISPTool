// DialogChipSetting_CFG_M460.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "FlashInfo.h"
#include "DialogChipSetting_CFG_M460.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M460, CDialog)

CDialogChipSetting_CFG_M460::CDialogChipSetting_CFG_M460(CWnd *pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_DataFlashBase(16, 8)
    , m_ALOCK(16, 2)
    , m_KSPLOCK(16, 2)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M460)
    m_nRadioCWDTEN	= -1;
    m_nRadioCBOV	= -1;
    m_nRadioCIOINI	= -1;
    m_nRadioCBS		= -1;
    m_nRadioSPIMPSL	= -1;
    m_nRadioUART1PSL = -1;
    m_bCheckCBORST	= FALSE;
    m_bCheckCBODEN	= FALSE;
    m_bCheckICELOCK	= FALSE;
    m_bCheckLOCK	= FALSE;
    m_bCheckDFEN	= FALSE;
    m_bCheckISP_UART = TRUE;
    m_bCheckISP_USB	= FALSE;
    m_bCheckISP_CAN	= TRUE;
    m_bCheckISP_I2C	= TRUE;
    m_bCheckISP_SPI	= TRUE;
    m_sDataFlashBase = _T("");
    m_sDataFlashSize = _T("");
    m_sConfigValue0	= _T("");
    m_sConfigValue1	= _T("");
    m_sConfigValue2	= _T("");
    m_sConfigValue3	= _T("");
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M460::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_CFG_M460)
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE,			m_nRadioCWDTEN);
    DDX_Radio(pDX, IDC_RADIO_BOV_7,					m_nRadioCBOV);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI,				m_nRadioCIOINI);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM_LDROM,		m_nRadioCBS);
    DDX_Radio(pDX, IDC_RADIO_SPIM_SEL3,				m_nRadioSPIMPSL);
    DDX_Radio(pDX, IDC_RADIO_UART1_SEL3,			m_nRadioUART1PSL);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET,		m_bCheckCBORST);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT,		m_bCheckCBODEN);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK,				m_bCheckICELOCK);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK,			m_bCheckLOCK);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE,		m_bCheckDFEN);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_UART,			m_bCheckISP_UART);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_USB,			m_bCheckISP_USB);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_CAN,			m_bCheckISP_CAN);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_I2C,			m_bCheckISP_I2C);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_SPI,			m_bCheckISP_SPI);
    DDX_Control(pDX, IDC_EDIT_FLASH_ADVANCE_LOCK,	m_ALOCK);
    DDX_Control(pDX, IDC_EDIT_FLASH_KEYSTORE_LOCK,	m_KSPLOCK);
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,	m_DataFlashBase);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE,		m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE,		m_SpinDataFlashSize);
    DDX_Text(pDX, IDC_EDIT_FLASH_ADVANCE_LOCK,		m_sALOCK);
    DDX_Text(pDX, IDC_EDIT_FLASH_KEYSTORE_LOCK,		m_sKSPLOCK);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,		m_sDataFlashBase);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE,			m_sDataFlashSize);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0,		m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1,		m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2,		m_sConfigValue2);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_3,		m_sConfigValue3);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_CFG_M460, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_CFG_M460)
    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP,			OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_STOP,			OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_0,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL3,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL2,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL1,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL0,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL3,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL2,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL1,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL0,					OnRadioClick)

    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK,					OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,			OnCheckClick)

    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_UART,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_USB,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_CAN,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_I2C,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_SPI,				OnCheckClick)

    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_ADVANCE_LOCK,		OnKillfocusEditAdvanceLock)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_KEYSTORE_LOCK,		OnKillfocusEditKeyStoreLock)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,		OnKillfocusEditDataFlashBase)
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE,	OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 message handlers

BOOL CDialogChipSetting_CFG_M460::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    //CHIP_INFO_NUMICRO_T chipInfo;
    //GetChipInfo_NuMicro(m_uPID, m_uDID, 0xFFFFFFFF, 0xFFFFFFFF, false, &chipInfo);
    //m_uProgramMemorySize = chipInfo.uProgramMemorySize;
    //m_uFlashPageSize	= (1 << chipInfo.uFlash_PageSize);
    //m_bSupportSPIM		= (chipInfo.uSPIM_Size > 0) ? true : false;
    m_uProgramMemorySize = 256 * 1024;
    m_uFlashPageSize	= NUMICRO_FLASH_PAGE_SIZE_4K;
    m_bSupportSPIM		= true;
    UpdateUI();
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);
    ConfigToGUI();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;	// return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_CFG_M460::UpdateUI()
{
    // TODO: Add your control notification handler code here
    if (!m_bSupportSPIM) {
        GetDlgItem(IDC_GROUP_SPIM_SELECT)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_RADIO_SPIM_SEL3)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_RADIO_SPIM_SEL2)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_RADIO_SPIM_SEL1)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_RADIO_SPIM_SEL0)->ShowWindow(SW_HIDE);
        RECT rcTmp, rcGroupSPIMPSL, rcGroupUART1PSL;
        LONG lDiff;
        GetDlgItem(IDC_GROUP_SPIM_SELECT)->GetWindowRect(&rcGroupSPIMPSL);
        GetDlgItem(IDC_GROUP_UART1_SELECT)->GetWindowRect(&rcGroupUART1PSL);
        lDiff = rcGroupUART1PSL.bottom - rcGroupSPIMPSL.bottom;
        int i, nIDs[] = {IDC_GROUP_UART1_SELECT, IDC_RADIO_UART1_SEL3, IDC_RADIO_UART1_SEL2, IDC_RADIO_UART1_SEL1, IDC_RADIO_UART1_SEL0,
                         IDC_GROUP_ISP_MODE, IDC_CHECK_ISP_MODE_UART, IDC_CHECK_ISP_MODE_USB, IDC_CHECK_ISP_MODE_CAN, IDC_CHECK_ISP_MODE_I2C, IDC_CHECK_ISP_MODE_SPI,
                         IDC_GROUP_CONFIG_VALUE, IDC_STATIC_CONFIG_0, IDC_STATIC_CONFIG_VALUE_0, IDC_STATIC_CONFIG_1, IDC_STATIC_CONFIG_VALUE_1, IDC_STATIC_CONFIG_2, IDC_STATIC_CONFIG_VALUE_2, IDC_STATIC_CONFIG_3, IDC_STATIC_CONFIG_VALUE_3
                        };

        for (i = 0; i < _countof(nIDs); i++) {
            GetDlgItem(nIDs[i])->GetWindowRect(&rcTmp);
            this->ScreenToClient(&rcTmp);
            GetDlgItem(nIDs[i])->SetWindowPos(NULL, rcTmp.left, rcTmp.top - lDiff, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
        }

        this->GetWindowRect(&rcTmp);
        SetWindowPos(this, 0, 0, rcTmp.right - rcTmp.left, rcTmp.bottom - rcTmp.top - lDiff, SWP_NOZORDER | SWP_NOMOVE);
    }
}

void CDialogChipSetting_CFG_M460::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];
    m_uConfigValue_t[3] = m_uConfigValue[3];
    CFG2GUI_CWDT();
    CFG2GUI_CBOD_8();
    CFG2GUI_ICELOCK();
    CFG2GUI_CIOINI();
    CFG2GUI_CBS_2();
    CFG2GUI_DFEN();
    CFG2GUI_KSPLOCK();
    m_bCheckISP_UART = ((m_uConfigValue_t[3] & M460_FLASH_CONFIG_UARTISPEN) != 0 ? TRUE : FALSE);
    m_bCheckISP_USB	= ((m_uConfigValue_t[3] & M460_FLASH_CONFIG_USBISPDIS) == 0 ? TRUE : FALSE);
    m_bCheckISP_CAN	= ((m_uConfigValue_t[3] & M460_FLASH_CONFIG_CANISPEN) != 0 ? TRUE : FALSE);
    m_bCheckISP_I2C	= ((m_uConfigValue_t[3] & M460_FLASH_CONFIG_I2CISPEN) != 0 ? TRUE : FALSE);
    m_bCheckISP_SPI	= ((m_uConfigValue_t[3] & M460_FLASH_CONFIG_SPIISPEN) != 0 ? TRUE : FALSE);

    /* SPIM multi-function pin selection */
    switch (m_uConfigValue_t[3] & M460_FLASH_CONFIG_SPIMPSL) {
        case M460_FLASH_CONFIG_SPIMPSL_SEL0:
            m_nRadioSPIMPSL = 3;
            break;

        case M460_FLASH_CONFIG_SPIMPSL_SEL1:
            m_nRadioSPIMPSL = 2;
            break;

        case M460_FLASH_CONFIG_SPIMPSL_SEL2:
            m_nRadioSPIMPSL = 1;
            break;

        case M460_FLASH_CONFIG_SPIMPSL_SEL3:
            m_nRadioSPIMPSL = 0;
            break;
    }

    /* UART1 multi-function pin selection */
    switch (m_uConfigValue_t[3] & M460_FLASH_CONFIG_UART1PSL) {
        case M460_FLASH_CONFIG_UART1PSL_SEL0:
            m_nRadioUART1PSL = 3;
            break;

        case M460_FLASH_CONFIG_UART1PSL_SEL1:
            m_nRadioUART1PSL = 2;
            break;

        case M460_FLASH_CONFIG_UART1PSL_SEL2:
            m_nRadioUART1PSL = 1;
            break;

        case M460_FLASH_CONFIG_UART1PSL_SEL3:
            m_nRadioUART1PSL = 0;
            break;
    }

    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);
    m_sConfigValue3.Format(_T("0x%08X"), m_uConfigValue_t[3]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M460::GUIToConfig()
{
    UpdateData(TRUE);
    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];
    m_uConfigValue_t[3] = 0xFFFFFFFF;
    GUI2CFG_CWDT();
    GUI2CFG_CBOD_8();
    GUI2CFG_ICELOCK();
    GUI2CFG_CIOINI();
    GUI2CFG_CBS_2();
    GUI2CFG_DFEN();
    GUI2CFG_KSPLOCK();

    if (m_bCheckISP_UART) {
        m_uConfigValue_t[3] |=  M460_FLASH_CONFIG_UARTISPEN;
    } else {
        m_uConfigValue_t[3] &= ~M460_FLASH_CONFIG_UARTISPEN;
    }

    if (!m_bCheckISP_USB) {
        m_uConfigValue_t[3] |=  M460_FLASH_CONFIG_USBISPDIS;
    } else {
        m_uConfigValue_t[3] &= ~M460_FLASH_CONFIG_USBISPDIS;
    }

    if (m_bCheckISP_CAN) {
        m_uConfigValue_t[3] |=  M460_FLASH_CONFIG_CANISPEN;
    } else {
        m_uConfigValue_t[3] &= ~M460_FLASH_CONFIG_CANISPEN;
    }

    if (m_bCheckISP_I2C) {
        m_uConfigValue_t[3] |=  M460_FLASH_CONFIG_I2CISPEN;
    } else {
        m_uConfigValue_t[3] &= ~M460_FLASH_CONFIG_I2CISPEN;
    }

    if (m_bCheckISP_SPI) {
        m_uConfigValue_t[3] |=  M460_FLASH_CONFIG_SPIISPEN;
    } else {
        m_uConfigValue_t[3] &= ~M460_FLASH_CONFIG_SPIISPEN;
    }

    if (m_bSupportSPIM) {
        m_uConfigValue_t[3] &= ~M460_FLASH_CONFIG_SPIMPSL;

        switch (m_nRadioSPIMPSL) {
            case 0:
                m_uConfigValue_t[3] |= M460_FLASH_CONFIG_SPIMPSL_SEL3;
                break;

            case 1:
                m_uConfigValue_t[3] |= M460_FLASH_CONFIG_SPIMPSL_SEL2;
                break;

            case 2:
                m_uConfigValue_t[3] |= M460_FLASH_CONFIG_SPIMPSL_SEL1;
                break;

            case 3:
            default:
                if (m_nRadioUART1PSL != 1) {
                    m_uConfigValue_t[3] |= M460_FLASH_CONFIG_SPIMPSL_SEL0;
                } else {
                    m_uConfigValue_t[3] |= (m_uConfigValue[3] & M460_FLASH_CONFIG_SPIMPSL);
                }
        }
    }

    m_uConfigValue_t[3] &= ~M460_FLASH_CONFIG_UART1PSL;

    switch (m_nRadioUART1PSL) {
        case 0:
            m_uConfigValue_t[3] |= M460_FLASH_CONFIG_UART1PSL_SEL3;
            break;

        case 1:
            if (m_nRadioSPIMPSL != 3) {
                m_uConfigValue_t[3] |= M460_FLASH_CONFIG_UART1PSL_SEL2;
            } else {
                m_uConfigValue_t[3] |= (m_uConfigValue[3] & M460_FLASH_CONFIG_UART1PSL);
            }

            break;

        case 2:
            m_uConfigValue_t[3] |= M460_FLASH_CONFIG_UART1PSL_SEL1;
            break;

        case 3:
        default:
            m_uConfigValue_t[3] |= M460_FLASH_CONFIG_UART1PSL_SEL0;
    }

    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
    m_uConfigValue[3] = m_uConfigValue_t[3];
}

void CDialogChipSetting_CFG_M460::CFG2GUI_CWDT()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN:
            if (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTPDEN) {
                m_nRadioCWDTEN = 2;
            } else {
                m_nRadioCWDTEN = 1;
            }

            break;

        default:
            m_nRadioCWDTEN = 1;
    }
}

void CDialogChipSetting_CFG_M460::CFG2GUI_CBOD_8()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL) {
        case NUMICRO_FLASH_CONFIG_CBOV_7:
            m_nRadioCBOV = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_6:
            m_nRadioCBOV = 1;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_5:
            m_nRadioCBOV = 2;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_4:
            m_nRadioCBOV = 3;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_3:
            m_nRadioCBOV = 4;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_2:
            m_nRadioCBOV = 5;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_1:
            m_nRadioCBOV = 6;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_0:
            m_nRadioCBOV = 7;
            break;

        default:
            m_nRadioCBOV = 0;
    }

    m_bCheckCBORST = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckCBODEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M460::CFG2GUI_CIOINI()
{
    m_nRadioCIOINI = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CIOINI) ? 0 : 1);
}

void CDialogChipSetting_CFG_M460::CFG2GUI_CBS_2()
{
    m_nRadioCBS = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBS_2_MODE) ? 0 : 1);
}

void CDialogChipSetting_CFG_M460::CFG2GUI_ICELOCK()
{
    m_bCheckICELOCK	= ((m_uConfigValue_t[0] & M480_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M460::CFG2GUI_DFEN()
{
    unsigned int uDataFlashBase, uDataFlashSize;
    m_bCheckDFEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);

    if (m_bCheckDFEN) {
        m_SpinDataFlashSize.EnableWindow(TRUE);
        m_DataFlashBase.EnableWindow(TRUE);
        uDataFlashBase = m_uConfigValue_t[1] & ~(m_uFlashPageSize - 1);
        uDataFlashSize = m_uProgramMemorySize - uDataFlashBase;

        if (uDataFlashSize > m_uProgramMemorySize) {
            uDataFlashSize = 0;
        }
    } else {
        m_SpinDataFlashSize.EnableWindow(FALSE);
        m_DataFlashBase.EnableWindow(FALSE);
        uDataFlashBase = 0xFFFFFFFF;
        uDataFlashSize = 0;
    }

    m_sDataFlashBase.Format(_T("%X"), m_uConfigValue_t[1]);
    m_sDataFlashSize.Format(_T("%.2fK"), uDataFlashSize / 1024.);
}

void CDialogChipSetting_CFG_M460::CFG2GUI_KSPLOCK()
{
    m_bCheckLOCK = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_sALOCK.Format(_T("%02X"), m_uConfigValue_t[2] & 0xFF);
    m_sKSPLOCK.Format(_T("%02X"), (m_uConfigValue_t[2] >> 8) & 0xFF);
}

void CDialogChipSetting_CFG_M460::GUI2CFG_CWDT()
{
    switch (m_nRadioCWDTEN) {
        case 0:
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        case 2:
            m_uConfigValue_t[0] &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            m_uConfigValue_t[0] |= (NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            break;

        default:
            if (((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE) ||
                    (((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN) && (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTPDEN))) {
                m_uConfigValue_t[0] &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
                m_uConfigValue_t[0] |= (NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            }
    }
}

void CDialogChipSetting_CFG_M460::GUI2CFG_CBOD_8()
{
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL;

    switch (m_nRadioCBOV) {
        case 0:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_7;
            break;

        case 1:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_6;
            break;

        case 2:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_5;
            break;

        case 3:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_4;
            break;

        case 4:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_3;
            break;

        case 5:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_2;
            break;

        case 6:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_1;
            break;

        case 7:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_0;
            break;

        default:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_3;
    }

    if (m_bCheckCBORST) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBORST;
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CBORST;
    }

    if (m_bCheckCBODEN) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBODEN;
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CBODEN;
    }
}

void CDialogChipSetting_CFG_M460::GUI2CFG_CIOINI()
{
    if (m_nRadioCIOINI == 0) {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CIOINI;
    } else {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CIOINI;
    }
}

void CDialogChipSetting_CFG_M460::GUI2CFG_CBS_2()
{
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBS_2_MODE;

    if (m_nRadioCBS == 0) {
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
    } else {
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
    }
}

void CDialogChipSetting_CFG_M460::GUI2CFG_ICELOCK()
{
    if (m_bCheckICELOCK) {
        m_uConfigValue_t[0] &= ~M480_FLASH_CONFIG_ICELOCK;
    } else {
        m_uConfigValue_t[0] |=  M480_FLASH_CONFIG_ICELOCK;
    }
}

void CDialogChipSetting_CFG_M460::GUI2CFG_DFEN()
{
    if (m_bCheckDFEN) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_DFEN;
        m_uConfigValue_t[1] &= ~(m_uFlashPageSize - 1);

        if (m_uConfigValue_t[1] < m_uFlashPageSize) {
            m_uConfigValue_t[1] = m_uFlashPageSize;
        }

        if (m_uConfigValue_t[1] > (m_uProgramMemorySize - m_uFlashPageSize)) {
            m_uConfigValue_t[1] = (m_uProgramMemorySize - m_uFlashPageSize);
        }
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_DFEN;

        if (m_uConfigValue_t[1] != 0xFFFFFFFF) {
            m_uConfigValue_t[1]  = 0xFFFFFFFF;
        }
    }
}

void CDialogChipSetting_CFG_M460::GUI2CFG_KSPLOCK()
{
    bool bModify0 = false, bModify1 = false;

    if (m_bCheckLOCK) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;

        if ((m_uConfigValue_t[2] & 0x5A) == 0x5A) {
            m_uConfigValue_t[2] &= ~0xFF;
            bModify0 = true;
        }
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;

        if ((m_uConfigValue_t[2] & 0x5A) != 0x5A) {
            m_uConfigValue_t[2] = (m_uConfigValue_t[2] & ~0xFF) | 0x5A;
            bModify0 = true;
        }

        if ((m_uConfigValue_t[2] & 0x5A00) != 0x5A00) {
            m_uConfigValue_t[2] = (m_uConfigValue_t[2] & ~0xFF00) | 0x5A00;
            bModify1 = true;
        }
    }

    if (!bModify0) {
        unsigned int uALOCK = ::_tcstoul(m_sALOCK, NULL, 16) & 0x5A;
        m_uConfigValue_t[2] = (m_uConfigValue_t[2] & ~0xFF) | uALOCK;

        if (uALOCK == 0x5A) {
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;

            if ((m_uConfigValue_t[2] & 0x5A00) != 0x5A00) {
                m_uConfigValue_t[2] = (m_uConfigValue_t[2] & ~0xFF00) | 0x5A00;
                bModify1 = true;
            }
        } else {
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;
        }
    }

    if (!bModify1) {
        unsigned int uKSPLOCK = ::_tcstoul(m_sKSPLOCK, NULL, 16) & 0x5A;
        m_uConfigValue_t[2] = (m_uConfigValue_t[2] & ~0xFF00) | (uKSPLOCK << 8);

        if ((m_uConfigValue_t[2] & 0x5A00) != 0x5A00) {
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;
            m_uConfigValue_t[2] &= ~0xFF;
        }
    }
}

void CDialogChipSetting_CFG_M460::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M460::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M460::OnKillfocusEditAdvanceLock()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M460::OnKillfocusEditKeyStoreLock()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M460::OnKillfocusEditDataFlashBase()
{
    UpdateData(TRUE);
    unsigned int uDataFlashBase = ::_tcstoul(m_sDataFlashBase, NULL, 16);
    uDataFlashBase &= ~(m_uFlashPageSize - 1);

    if (uDataFlashBase < m_uFlashPageSize) {
        uDataFlashBase = m_uFlashPageSize;
    }

    if (uDataFlashBase > (m_uProgramMemorySize - m_uFlashPageSize)) {
        uDataFlashBase = (m_uProgramMemorySize - m_uFlashPageSize);
    }

    m_uConfigValue[1] = uDataFlashBase;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M460::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
    unsigned int uDataFlashBase = ::_tcstoul(m_sDataFlashBase, NULL, 16);
    uDataFlashBase &= ~(m_uFlashPageSize - 1);

    if (pNMUpDown->iDelta == 1) {
        uDataFlashBase += m_uFlashPageSize;
    } else if (pNMUpDown->iDelta == -1) {
        uDataFlashBase -= m_uFlashPageSize;
    }

    if (uDataFlashBase < m_uFlashPageSize) {
        uDataFlashBase = m_uFlashPageSize;
    }

    if (uDataFlashBase > (m_uProgramMemorySize - m_uFlashPageSize)) {
        uDataFlashBase = (m_uProgramMemorySize - m_uFlashPageSize);
    }

    m_uConfigValue[1] = uDataFlashBase;
    ConfigToGUI();
    *pResult = 0;
}

void CDialogChipSetting_CFG_M460::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_CFG_M460::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}
