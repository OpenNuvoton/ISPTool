// DialogConfiguration_M2351.cpp : implementation file
//
#include "stdafx.h"
#include "DialogConfiguration_M2351.h"
#include "ChipDefs.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M2351 dialog
IMPLEMENT_DYNAMIC(CDialogConfiguration_M2351, CDialog)

CDialogConfiguration_M2351::CDialogConfiguration_M2351(CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_M2351::IDD, pParent)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_M2351)
    m_nRadioCBOV	= -1;
    m_nRadioCBS		= -1;
    m_nRadioCFGXT1	= -1;
    m_nRadioCIOINI	= -1;
    m_nRadioUART	= -1;
    m_nRadioCWDTEN	= -1;
    m_bDisableICE	= FALSE;
    m_bCheckMBS		= FALSE;
    m_bCheckCBODEN	= FALSE;
    m_bCheckCBORST	= FALSE;
    m_bTamperPowerDown	= FALSE;
    m_uConfigValue[0] = 0xFFFFFFFF;
    m_uConfigValue[1] = 0xFFFFFFFF;
    m_uConfigValue[2] = 0xFFFFFFFF;
    m_uConfigValue[3] = 0xFFFFFFFF;
    m_sConfigValue0 = _T("");
    m_sConfigValue3 = _T("");
    //}}AFX_DATA_INIT
}

CDialogConfiguration_M2351::~CDialogConfiguration_M2351()
{
}

void CDialogConfiguration_M2351::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_M2351)
    DDX_Radio(pDX, IDC_RADIO_BOV_7, m_nRadioCBOV);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM_LDROM, m_nRadioCBS);
    DDX_Radio(pDX, IDC_RADIO_GPF_CRYSTAL, m_nRadioCFGXT1);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioCIOINI);
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE, m_nRadioCWDTEN);
    DDX_Radio(pDX, IDC_RADIO_UART1_SEL4, m_nRadioUART);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckCBODEN);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckCBORST);
    DDX_Check(pDX, IDC_CHECK_BS_MKROM, m_bCheckMBS);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK, m_bDisableICE);
    DDX_Check(pDX, IDC_CHECK_TAMPER_POWER_DOWN, m_bTamperPowerDown);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_3, m_sConfigValue3);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M2351, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_M2351)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7, OnRadioClick)

    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnCheckClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM, OnRadioClick)
    ON_BN_CLICKED(IDC_CHECK_BS_MKROM, OnCheckClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_CRYSTAL, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_GPIO, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL0, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL1, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL2, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL3, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL4, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_STOP, OnRadioClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_TAMPER_POWER_DOWN, OnCheckClick)
    //ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M2351 message handlers

BOOL CDialogConfiguration_M2351::OnInitDialog()
{
    CDialog::OnInitDialog();

    // TODO: Add extra initialization here

    if (m_uChipType == NUC_CHIP_TYPE_M2354) {
        GetDlgItem(IDC_CHECK_TAMPER_POWER_DOWN)->ShowWindow(SW_SHOW);
    } else { //if (m_uChipType == NUC_CHIP_TYPE_M2351)
        GetDlgItem(IDC_CHECK_TAMPER_POWER_DOWN)->ShowWindow(SW_HIDE);
    }

    ConfigToGUI();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;	// return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_M2351::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogConfiguration_M2351::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogConfiguration_M2351::ConfigToGUI()
{
    unsigned int uConfig0 = m_uConfigValue[0];
    unsigned int uConfig3 = m_uConfigValue[3];

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL) {
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
        default:
            m_nRadioCBOV = 7;
            break;
    }

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN:
            if (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTPDEN) {
                m_nRadioCWDTEN = 2;
            } else {
                m_nRadioCWDTEN = 1;
            }

            break;

        default:
            m_nRadioCWDTEN = 1;
    }

    m_nRadioCBS		= ((uConfig0 & NUMICRO_FLASH_CONFIG_CBS_2_MODE) ? 0 : 1);
    m_nRadioCFGXT1	= ((uConfig0 & M2351_FLASH_CONFIG_CFGXT1) ? 0 : 1);
    m_nRadioCIOINI	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CIOINI) ? 0 : 1);

    if (m_uChipType == NUC_CHIP_TYPE_M2354) {
        m_bCheckMBS = TRUE;
    } else {
        m_bCheckMBS = ((uConfig0 & M2351_FLASH_CONFIG_MBS) == 0 ? TRUE : FALSE);
    }

    m_bCheckCBODEN	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckCBORST	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bDisableICE = ((uConfig0 & M2351_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);

    switch (uConfig3 & M2351_FLASH_CONFIG_UART1PSL) {
        case M2351_FLASH_CONFIG_UART1PSL_SEL0:
            m_nRadioUART = 4;
            break;

        case M2351_FLASH_CONFIG_UART1PSL_SEL1:
            m_nRadioUART = 3;
            break;

        case M2351_FLASH_CONFIG_UART1PSL_SEL2:
            m_nRadioUART = 2;
            break;

        case M2351_FLASH_CONFIG_UART1PSL_SEL3:
            m_nRadioUART = 1;
            break;

        default:
            m_nRadioUART = 0;
    }

    m_bTamperPowerDown = ((uConfig3 & M2354_FLASH_CONFIG_TMPPD) == M2354_FLASH_CONFIG_TMPPD_CODE ? TRUE : FALSE);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue3.Format(_T("0x%08X"), uConfig3);
    UpdateData(FALSE);
}

void CDialogConfiguration_M2351::GUIToConfig()
{
    UpdateData(TRUE);
    unsigned int uConfig0 = m_uConfigValue[0];
    unsigned int uConfig3 = m_uConfigValue[3];
    uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL;

    switch (m_nRadioCBOV) {
        case 0:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_7;
            break;

        case 1:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_6;
            break;

        case 2:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_5;
            break;

        case 3:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_4;
            break;

        case 4:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_3;
            break;

        case 5:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_2;
            break;

        case 6:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_1;
            break;

        case 7:
        default:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_0;
            break;
    }

    switch (m_nRadioCWDTEN) {
        case 0:
            uConfig0 &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            uConfig0 |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        case 2:
            uConfig0 &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            uConfig0 |= (NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            break;

        default:
            if (((uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE) ||
                    (((uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN) && (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTPDEN))) {
                uConfig0 &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
                uConfig0 |= (NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            }
    }

    uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBS_2_MODE;

    if (m_nRadioCBS == 0) {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
    }

    if (m_nRadioCFGXT1 == 0) {
        uConfig0 |=  M2351_FLASH_CONFIG_CFGXT1;
    } else {
        uConfig0 &= ~M2351_FLASH_CONFIG_CFGXT1;
    }

    if (m_nRadioCIOINI == 0) {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CIOINI;
    }

    if (m_uChipType == NUC_CHIP_TYPE_M2354) {
        uConfig0 |=  M2351_FLASH_CONFIG_MBS;
    } else {
        if (m_bCheckMBS) {
            uConfig0 &= ~M2351_FLASH_CONFIG_MBS;
        } else {
            uConfig0 |=  M2351_FLASH_CONFIG_MBS;
        }
    }

    if (m_bCheckCBODEN) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckCBORST) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CBORST;
    }

    if (m_bDisableICE) {
        uConfig0 &= ~M2351_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |=  M2351_FLASH_CONFIG_ICELOCK;
    }

    uConfig3 &= ~M2351_FLASH_CONFIG_UART1PSL;

    switch (m_nRadioUART) {
        case 4:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL0;
            break;

        case 3:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL1;
            break;

        case 2:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL2;
            break;

        case 1:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL3;
            break;

        default:
            if ((m_uConfigValue[3] & M2351_FLASH_CONFIG_UART1PSL) > M2351_FLASH_CONFIG_UART1PSL_SEL3) {
                uConfig3 |= (m_uConfigValue[3] & M2351_FLASH_CONFIG_UART1PSL);    /* Keep old value */
            } else {
                uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL4;
            }
    }

    uConfig3 &= ~M2354_FLASH_CONFIG_TMPPD;

    if (m_bTamperPowerDown) {
        uConfig3 |= M2354_FLASH_CONFIG_TMPPD_CODE;
    } else {
        if ((m_uConfigValue[3] & M2354_FLASH_CONFIG_TMPPD) != M2354_FLASH_CONFIG_TMPPD_CODE) {
            uConfig3 |= (m_uConfigValue[3] & M2354_FLASH_CONFIG_TMPPD);
        } else {
            uConfig3 |= M2354_FLASH_CONFIG_TMPPD;
        }
    }

    m_uConfigValue[0] = uConfig0;
    m_uConfigValue[3] = uConfig3;
}

