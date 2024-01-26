// DialogChipSetting_CFG_M460.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "FlashInfo.h"
#include "DialogChipSetting_CFG_M55M1.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M55M1, CDialog)

CDialogChipSetting_CFG_M55M1::CDialogChipSetting_CFG_M55M1(CWnd *pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M460)
    m_nRadioCWDTEN	= -1;
    m_nRadioCBOV	= -1;
    m_nRadioCBS		= -1;
    m_nRadioUART    = -1;
    m_bCheckCBORST	= FALSE;
    m_bCheckCBODEN	= FALSE;
    m_bCheckICELOCK	= FALSE;
    m_bCheckSCEN    = FALSE;
    m_bTamperPowerDown = FALSE;
    m_bCheckISP_UART = TRUE;
    m_bCheckISP_USB	= FALSE;
    m_bCheckISP_CAN	= TRUE;
    m_bCheckISP_I2C	= TRUE;
    m_bCheckISP_SPI	= TRUE;
    m_bCheckNSCRLOCK = FALSE;
    m_bCheckSCRLOCK = FALSE;
    m_bCheckARLOCK = FALSE;
    m_bMirror = FALSE;
    m_sSecureConcealBase = _T("");
    m_sSecureConcealPageSize = _T("");
    m_sNSCRBase = _T("");
    m_sConfigValue0	= _T("");
    m_sConfigValue3	= _T("");
    m_sConfigValue4 = _T("");
    m_sConfigValue5 = _T("");
    m_sConfigValue6 = _T("");
    m_sConfigValue11 = _T("");
    m_sConfigValue12 = _T("");
    m_sConfigValue13 = _T("");
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M55M1::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_CFG_M55M1)
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE,			m_nRadioCWDTEN);
    DDX_Radio(pDX, IDC_RADIO_BOV_7,					m_nRadioCBOV);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM_LDROM,		m_nRadioCBS);
    DDX_Radio(pDX, IDC_RADIO_UART1_SEL1,            m_nRadioUART);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET,		m_bCheckCBORST);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT,		m_bCheckCBODEN);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK,				m_bCheckICELOCK);
    DDX_Check(pDX, IDC_CHECK_SECURE_CONCEAL_ENABLE, m_bCheckSCEN);
    DDX_Check(pDX, IDC_CHECK_TAMPER_POWER_DOWN,     m_bTamperPowerDown);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_UART,			m_bCheckISP_UART);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_CAN,			m_bCheckISP_CAN);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_I2C,			m_bCheckISP_I2C);
    DDX_Check(pDX, IDC_CHECK_NSCR_LOCK,             m_bCheckNSCRLOCK);
    DDX_Check(pDX, IDC_CHECK_SCR_LOCK,              m_bCheckSCRLOCK);
    DDX_Check(pDX, IDC_CHECK_AR_LOCK,               m_bCheckARLOCK);
    DDX_Check(pDX, IDC_CHECK_MIRROR_BOUNDARY,       m_bMirror);
    DDX_Text(pDX, IDC_EDIT_SECURE_CONCEAL_BASE_ADDRESS, m_sSecureConcealBase);
    DDX_Text(pDX, IDC_EDIT_SECURE_CONCEAL_SIZE,     m_sSecureConcealPageSize);
    DDX_Text(pDX, IDC_EDIT_NSCR_BASE_ADDRESS,       m_sNSCRBase);
    DDX_Control(pDX, IDC_EDIT_SECURE_CONCEAL_BASE_ADDRESS, m_SecureConcealBase);
    DDX_Control(pDX, IDC_EDIT_SECURE_CONCEAL_SIZE,  m_SecureConcealPageSize);
    DDX_Control(pDX, IDC_EDIT_NSCR_BASE_ADDRESS,    m_NSCRBase);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0,		m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_3,		m_sConfigValue3);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_4,        m_sConfigValue4);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_5,        m_sConfigValue5);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_6,        m_sConfigValue6);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_11,       m_sConfigValue11);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_12,       m_sConfigValue12);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_13,       m_sConfigValue13);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_CFG_M55M1, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_CFG_M55M1)
    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP,			OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_STOP,            OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL0,                 OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL1,                 OnRadioClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK,					OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_NSCR_LOCK,                  OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SCR_LOCK,				    OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_AR_LOCK,                    OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_MIRROR_BOUNDARY,            OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURE_CONCEAL_ENABLE,      OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_TAMPER_POWER_DOWN,          OnCheckClick)

    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_UART,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_CAN,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_I2C,				OnCheckClick)

    ON_EN_KILLFOCUS(IDC_EDIT_SECURE_CONCEAL_BASE_ADDRESS, OnKillfocusEditSCBase)
    ON_EN_KILLFOCUS(IDC_EDIT_SECURE_CONCEAL_SIZE,       OnKillfocusEditSCSize)
    ON_EN_KILLFOCUS(IDC_EDIT_NSCR_BASE_ADDRESS,         OnKillfocusEditNSCRBase)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 message handlers

BOOL CDialogChipSetting_CFG_M55M1::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    //CHIP_INFO_NUMICRO_T chipInfo;
    //GetChipInfo_NuMicro(m_uPID, m_uDID, 0xFFFFFFFF, 0xFFFFFFFF, false, &chipInfo);
    //m_uProgramMemorySize = chipInfo.uProgramMemorySize;
    //m_uFlashPageSize	= (1 << chipInfo.uFlash_PageSize);
    UpdateUI();
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_bCheckNSCRLOCK = 0;
    GetDlgItem(IDC_CHECK_MIRROR_BOUNDARY)->EnableWindow(m_bCheckNSCRLOCK);
    GetDlgItem(IDC_EDIT_NSCR_BASE_ADDRESS)->EnableWindow(m_bCheckNSCRLOCK);
    ConfigToGUI();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;	// return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_CFG_M55M1::UpdateUI()
{
    // TODO: Add your control notification handler code here
}

void CDialogChipSetting_CFG_M55M1::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[3] = m_uConfigValue[3];
    m_uConfigValue_t[4] = m_uConfigValue[4];
    m_uConfigValue_t[5] = m_uConfigValue[5];
    m_uConfigValue_t[6] = m_uConfigValue[6];
    m_uConfigValue_t[11] = m_uConfigValue[11];
    m_uConfigValue_t[12] = m_uConfigValue[12];
    m_uConfigValue_t[13] = m_uConfigValue[13];
    CFG2GUI_CWDT();
    CFG2GUI_CBOD_8();
    CFG2GUI_ICELOCK();
    CFG2GUI_UART1();
    CFG2GUI_CBS_2();
    CFG2GUI_SCEN();
    CFG2GUI_NSCR();
    m_bTamperPowerDown = ((m_uConfigValue_t[3] & M2354_FLASH_CONFIG_TMPPD) == M2354_FLASH_CONFIG_TMPPD_CODE ? TRUE : FALSE);
    m_bCheckISP_UART = ((m_uConfigValue_t[3] & M2L31_FLASH_CONFIG_UARTISPEN) != 0 ? TRUE : FALSE);
    m_bCheckISP_CAN	= ((m_uConfigValue_t[3] & M2L31_FLASH_CONFIG_CANISPEN) != 0 ? TRUE : FALSE);
    m_bCheckISP_I2C	= ((m_uConfigValue_t[3] & M2L31_FLASH_CONFIG_I2CISPEN) != 0 ? TRUE : FALSE);
    m_bCheckSCRLOCK = ((m_uConfigValue_t[11] & 0x000000FFUL) != 0x0000005AUL ? TRUE : FALSE);
    m_bCheckARLOCK = ((m_uConfigValue_t[13] & 0x000000FFUL) != 0x0000005AUL ? TRUE : FALSE);
    m_bMirror = ((m_uConfigValue_t[12] & 0x80000000UL) != 0 ? TRUE : FALSE);
    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue3.Format(_T("0x%08X"), m_uConfigValue_t[3]);
    m_sConfigValue4.Format(_T("0x%08X"), m_uConfigValue_t[4]);
    m_sConfigValue5.Format(_T("0x%08X"), m_uConfigValue_t[5]);
    m_sConfigValue6.Format(_T("0x%08X"), m_uConfigValue_t[6]);
    m_sConfigValue11.Format(_T("0x%08X"), m_uConfigValue_t[11]);
    m_sConfigValue12.Format(_T("0x%08X"), m_uConfigValue_t[12]);
    m_sConfigValue13.Format(_T("0x%08X"), m_uConfigValue_t[13]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M55M1::GUIToConfig()
{
    UpdateData(TRUE);
    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[3] = 0xFFFFFFFF;
    m_uConfigValue_t[4] = m_uConfigValue[4];
    m_uConfigValue_t[5] = m_uConfigValue[5];
    m_uConfigValue_t[6] = m_uConfigValue[6];
    m_uConfigValue_t[11] = m_uConfigValue[11];
    m_uConfigValue_t[12] = m_uConfigValue[12];
    m_uConfigValue_t[13] = m_uConfigValue[13];

    GetDlgItem(IDC_CHECK_MIRROR_BOUNDARY)->EnableWindow(m_bCheckNSCRLOCK);
    GetDlgItem(IDC_EDIT_NSCR_BASE_ADDRESS)->EnableWindow(m_bCheckNSCRLOCK);

    GUI2CFG_CWDT();
    GUI2CFG_CBOD_8();
    GUI2CFG_ICELOCK();
    GUI2CFG_UART1();
    GUI2CFG_CBS_2();
    GUI2CFG_SCEN();
    GUI2CFG_NSCR();

    m_uConfigValue_t[3] &= ~M2354_FLASH_CONFIG_TMPPD;

    if (m_bTamperPowerDown) {
        m_uConfigValue_t[3] |= M2354_FLASH_CONFIG_TMPPD_CODE;
    }
    else {
        if ((m_uConfigValue[3] & M2354_FLASH_CONFIG_TMPPD) != M2354_FLASH_CONFIG_TMPPD_CODE) {
            m_uConfigValue_t[3] |= (m_uConfigValue[3] & M2354_FLASH_CONFIG_TMPPD);
        }
        else {
            m_uConfigValue_t[3] |= M2354_FLASH_CONFIG_TMPPD;
        }
    }

    if (m_bCheckISP_UART) {
        m_uConfigValue_t[3] |=  M2L31_FLASH_CONFIG_UARTISPEN;
    } else {
        m_uConfigValue_t[3] &= ~M2L31_FLASH_CONFIG_UARTISPEN;
    }

    if (m_bCheckISP_CAN) {
        m_uConfigValue_t[3] |=  M2L31_FLASH_CONFIG_CANISPEN;
    } else {
        m_uConfigValue_t[3] &= ~M2L31_FLASH_CONFIG_CANISPEN;
    }

    if (m_bCheckISP_I2C) {
        m_uConfigValue_t[3] |=  M2L31_FLASH_CONFIG_I2CISPEN;
    } else {
        m_uConfigValue_t[3] &= ~M2L31_FLASH_CONFIG_I2CISPEN;
    }

    m_uConfigValue_t[11] &= ~0x000000FFUL;

    if (!m_bCheckSCRLOCK) {
        m_uConfigValue_t[11] |= 0x0000005AUL;
    }
    else {
        if ((m_uConfigValue[11] & 0x000000FFUL) != 0x0000005AUL) {
            m_uConfigValue_t[11] |= (m_uConfigValue[11] & 0x000000FFUL);
        }
        else {
            m_uConfigValue_t[11] |= 0x000000FFUL;
        }
    }

    m_uConfigValue_t[13] &= ~0x000000FFUL;

    if (!m_bCheckARLOCK) {
        m_uConfigValue_t[13] |= 0x0000005AUL;
    }
    else {
        if ((m_uConfigValue[13] & 0x000000FFUL) != 0x0000005AUL) {
            m_uConfigValue_t[13] |= (m_uConfigValue[13] & 0x000000FFUL);
        }
        else {
            m_uConfigValue_t[13] |= 0x000000FFUL;
        }
    }

    if (m_bMirror) {
        m_uConfigValue_t[12] |= 0x80000000UL;
    }
    else {
        m_uConfigValue_t[12] &= ~0x80000000UL;
    }

    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[3] = m_uConfigValue_t[3];
    m_uConfigValue[4] = m_uConfigValue_t[4];
    m_uConfigValue[5] = m_uConfigValue_t[5];
    m_uConfigValue[6] = m_uConfigValue_t[6];
    m_uConfigValue[11] = m_uConfigValue_t[11];
    m_uConfigValue[12] = m_uConfigValue_t[12];
    m_uConfigValue[13] = m_uConfigValue_t[13];
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_CWDT()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN:
            if (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTPDEN) {
                m_nRadioCWDTEN = 2;
            }
            else {
                m_nRadioCWDTEN = 1;
            }

            break;

        default:
            m_nRadioCWDTEN = 1;
    }
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_CBOD_8()
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

        default:
            m_nRadioCBOV = 0;
    }

    m_bCheckCBORST = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckCBODEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_CBS_2()
{
    m_nRadioCBS = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBS_2_MODE) ? 0 : 1);
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_ICELOCK()
{
    m_bCheckICELOCK = ((m_uConfigValue_t[0] & M480_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_UART1()
{
    switch (m_uConfigValue_t[3] & M2351_FLASH_CONFIG_UART1PSL_SEL3) {
    case M2351_FLASH_CONFIG_UART1PSL_SEL0:
        m_nRadioUART = 0;
        break;

    case M2351_FLASH_CONFIG_UART1PSL_SEL1:
        m_nRadioUART = 1;
        break;

    default:
        m_nRadioUART = 0;
    }
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_SCEN()
{
    unsigned int uSCBase, uSCSize;
    m_bCheckSCEN = ((m_uConfigValue_t[6] == M2L31_SECURE_CONCEAL_ENABLE)? TRUE : FALSE);

    if (m_bCheckSCEN) {
        m_SecureConcealBase.EnableWindow(TRUE);
        m_SecureConcealPageSize.EnableWindow(TRUE);
        uSCBase = m_uConfigValue_t[4] & ~(m_uFlashPageSize - 1);
        //uSCSize = m_uProgramMemorySize - uSCBase;
        uSCSize = m_uConfigValue_t[5];

        if (uSCSize > (m_uProgramMemorySize - uSCBase) / 8182) {
            uSCSize = (m_uProgramMemorySize - uSCBase) / 8182;
        }

        if (uSCSize > m_uProgramMemorySize) {
            uSCSize = 0;
        }
    }
    else {
        m_SecureConcealBase.EnableWindow(FALSE);
        m_SecureConcealPageSize.EnableWindow(FALSE);
        uSCBase = 0xFFFFFFFF;
        uSCSize = 0;
    }

    m_sSecureConcealBase.Format(_T("%X"), m_uConfigValue_t[4]);
    m_sSecureConcealPageSize.Format(_T("%d"), uSCSize);
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_NSCR()
{
    unsigned int uSCBase, uSCSize;

    uSCBase = m_uConfigValue_t[12] & 0x7FFFFFFFUL;

    uSCBase = (uSCBase/0x2000) * 0x2000;

    if (uSCBase > 0x2FE000) {
        uSCBase = 0x2FE000;
    }

    if (uSCBase < 0x102000) {
        uSCBase = 0x102000;
    }

    if (!m_bCheckNSCRLOCK) {
        uSCBase = 0xFFFFFF;
    }
    m_sNSCRBase.Format(_T("%X"), uSCBase);
}

void CDialogChipSetting_CFG_M55M1::GUI2CFG_CWDT()
{
    switch (m_nRadioCWDTEN) {
        case 0:
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
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

void CDialogChipSetting_CFG_M55M1::GUI2CFG_CBOD_8()
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

        default:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_7;
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

void CDialogChipSetting_CFG_M55M1::GUI2CFG_CBS_2()
{
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBS_2_MODE;

    if (m_nRadioCBS == 0) {
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
    } else {
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
    }
}

void CDialogChipSetting_CFG_M55M1::GUI2CFG_ICELOCK()
{
    if (m_bCheckICELOCK) {
        m_uConfigValue_t[0] &= ~M480_FLASH_CONFIG_ICELOCK;
    } else {
        m_uConfigValue_t[0] |=  M480_FLASH_CONFIG_ICELOCK;
    }
}

void CDialogChipSetting_CFG_M55M1::GUI2CFG_UART1()
{
    m_uConfigValue_t[3] &= ~M2351_FLASH_CONFIG_UART1PSL_SEL3;

    switch (m_nRadioUART) {
    case 0:
        m_uConfigValue_t[3] |= M2351_FLASH_CONFIG_UART1PSL_SEL0;
        break;

    case 1:
        m_uConfigValue_t[3] |= M2351_FLASH_CONFIG_UART1PSL_SEL1;
        break;

    default:
        m_uConfigValue_t[3] |= M2351_FLASH_CONFIG_UART1PSL_SEL1;
    }
}

void CDialogChipSetting_CFG_M55M1::GUI2CFG_SCEN()
{
    if (m_bCheckSCEN) {
        m_uConfigValue_t[6] = M2L31_SECURE_CONCEAL_ENABLE;
        m_uConfigValue_t[4] &= ~(m_uFlashPageSize - 1);

        if (m_uConfigValue_t[4] < m_uFlashPageSize) {
            m_uConfigValue_t[4] = m_uFlashPageSize;
        }

        if (m_uConfigValue_t[4] > (m_uProgramMemorySize - m_uFlashPageSize)) {
            m_uConfigValue_t[4] = (m_uProgramMemorySize - m_uFlashPageSize);
        }

        if (m_uConfigValue_t[5] > (m_uProgramMemorySize - m_uConfigValue_t[4]) / 8182) {
            m_uConfigValue_t[5] = (m_uProgramMemorySize - m_uConfigValue_t[4]) / 8182;
        }

    }
    else {
        m_uConfigValue_t[6] = 0xFFFFFFFF;

        if (m_uConfigValue_t[4] != 0xFFFFFFFF) {
            m_uConfigValue_t[4] = 0xFFFFFFFF;
        }

        if (m_uConfigValue_t[5] != 0xFFFFFFFF) {
            m_uConfigValue_t[5] = 0xFFFFFFFF;
        }
    }
}

void CDialogChipSetting_CFG_M55M1::GUI2CFG_NSCR()
{
    unsigned int uNSCR;
    m_uConfigValue_t[12] &= ~(m_uFlashPageSize - 1);
    uNSCR = m_uConfigValue_t[12] & ~(0x80000000UL);

    if (uNSCR < 0x102000) {
        uNSCR = 0x102000;
    }

    if (uNSCR > (0x2FE000)) {
        uNSCR = (0x2FE000);
    }

    m_uConfigValue_t[12] |= uNSCR;
    if (!m_bCheckNSCRLOCK) {
        m_uConfigValue_t[12] = 0xFFFFFFFF;
    }
}

void CDialogChipSetting_CFG_M55M1::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1::OnKillfocusEditSCBase()
{
    UpdateData(TRUE);
    unsigned int uSCBase = ::_tcstoul(m_sSecureConcealBase, NULL, 16);
    uSCBase &= ~(m_uFlashPageSize - 1);

    if (uSCBase < m_uFlashPageSize) {
        uSCBase = m_uFlashPageSize;
    }

    if (uSCBase > (m_uProgramMemorySize - m_uFlashPageSize)) {
        uSCBase = (m_uProgramMemorySize - m_uFlashPageSize);
    }

    m_uConfigValue_t[4] = uSCBase;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1::OnKillfocusEditSCSize()
{
    UpdateData(TRUE);
    unsigned int uSCSize = ::_tcstoul(m_sSecureConcealPageSize, NULL, 10);

    if (uSCSize > (m_uProgramMemorySize - m_uConfigValue_t[4]) / 8182) {
        uSCSize = (m_uProgramMemorySize - m_uConfigValue_t[4]) / 8182;
    }

    if (uSCSize < 1) {
        uSCSize = 1;
    }

    m_uConfigValue_t[5] = uSCSize;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1::OnKillfocusEditNSCRBase()
{
    UpdateData(TRUE);
    unsigned int uSCBase = ::_tcstoul(m_sNSCRBase, NULL, 16);
    uSCBase &= ~(m_uFlashPageSize - 1);
    uSCBase &= ~(0x80000000UL);

    if (uSCBase < 0x102000) {
        uSCBase = 0x102000;
    }

    if (uSCBase > (0x2FE000)) {
        uSCBase = (0x2FE000);
    }

    m_uConfigValue_t[12] |= uSCBase;
    if (!m_bCheckNSCRLOCK) {
        m_uConfigValue_t[12] = 0xFFFFFFFF;
    }
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_CFG_M55M1::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}
