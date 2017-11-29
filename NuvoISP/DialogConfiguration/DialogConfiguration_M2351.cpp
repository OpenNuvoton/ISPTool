// DialogConfiguration_M2351.cpp : implementation file
//
#include "stdafx.h"
#include "DialogConfiguration_M2351.h"
#include "ChipDefs.h"

#define FLASH_CONFIG_CWDTEN				0x80000018
#define FLASH_CONFIG_CWDTEN_INACTIVE	0x80000018
#define FLASH_CONFIG_CWDTEN_BY_LIRCEN	0x00000018
#define FLASH_CONFIG_CWDTEN_ACTIVE		0x00000000
#define FLASH_CONFIG_CWDTPDEN			0x40000000


/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M2351 dialog
IMPLEMENT_DYNAMIC(CDialogConfiguration_M2351, CDialog)

CDialogConfiguration_M2351::CDialogConfiguration_M2351(CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_M2351::IDD, pParent)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_M2351)
    m_nRadioBov		= -1;
    m_nRadioBS		= -1;
    m_nRadioGPG		= -1;
    m_nRadioIO		= -1;
    m_nRadioUART	= -1;
    m_nRadioWDT		= -1;
    m_bDisableICE	= FALSE;
    m_bCheckMBS		= FALSE;
    m_bCheckBrownOutDetect	= FALSE;
    m_bCheckBrownOutReset	= FALSE;
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
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM_LDROM, m_nRadioBS);
    DDX_Radio(pDX, IDC_RADIO_GPF_CRYSTAL, m_nRadioGPG);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE, m_nRadioWDT);
    DDX_Radio(pDX, IDC_RADIO_UART1_SEL4, m_nRadioUART);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_BS_MKROM, m_bCheckMBS);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK, m_bDisableICE);
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
    ON_WM_SIZE()
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

    switch (uConfig0 & TC8226_FLASH_CONFIG_CBOV) {
        case TC8226_FLASH_CONFIG_CBOV_30:
            m_nRadioBov = 0;
            break;

        case TC8226_FLASH_CONFIG_CBOV_28:
            m_nRadioBov = 1;
            break;

        case TC8226_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 2;
            break;

        case TC8226_FLASH_CONFIG_CBOV_24:
            m_nRadioBov = 3;
            break;

        case TC8226_FLASH_CONFIG_CBOV_22:
            m_nRadioBov = 4;
            break;

        case TC8226_FLASH_CONFIG_CBOV_20:
            m_nRadioBov = 5;
            break;

        case TC8226_FLASH_CONFIG_CBOV_18:
            m_nRadioBov = 6;
            break;

        case TC8226_FLASH_CONFIG_CBOV_16:
            m_nRadioBov = 7;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_uConfigValue[0] & TC8226_FLASH_CONFIG_CBOV);
    }

    switch (uConfig0 & FLASH_CONFIG_CWDTEN) {
        case FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioWDT = 0;
            break;

        case FLASH_CONFIG_CWDTEN_BY_LIRCEN:
            if (uConfig0 & FLASH_CONFIG_CWDTPDEN) {
                m_nRadioWDT = 2;
            } else {
                m_nRadioWDT = 1;
            }

            break;

        default:
            m_nRadioWDT = 1;
    }

    m_nRadioBS = ((uConfig0 & TC8226_FLASH_CONFIG_CBS) ? 0 : 1);
    m_nRadioGPG = ((uConfig0 & TC8226_FLASH_CONFIG_CGPFMFP) ? 0 : 1);
    m_nRadioIO = ((uConfig0 & TC8226_FLASH_CONFIG_CIOINI) ? 0 : 1);
    m_bCheckMBS = ((uConfig0 & TC8226_FLASH_CONFIG_MBS) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutDetect = ((uConfig0 & TC8226_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & TC8226_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
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

    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue3.Format(_T("0x%08X"), uConfig3);
    UpdateData(FALSE);
}

void CDialogConfiguration_M2351::GUIToConfig()
{
    UpdateData(TRUE);
    unsigned int uConfig0 = m_uConfigValue[0];
    unsigned int uConfig3 = m_uConfigValue[3];
    uConfig0 &= ~TC8226_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_30;
            break;

        case 1:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_28;
            break;

        case 2:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_24;
            break;

        case 4:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_22;
            break;

        case 5:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_20;
            break;

        case 6:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_18;
            break;

        case 7:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_16;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_uConfigValue[0] & TC8226_FLASH_CONFIG_CBOV);
    }

    switch (m_nRadioWDT) {
        case 0:
            uConfig0 &= ~FLASH_CONFIG_CWDTEN;
            uConfig0 |= FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        case 2:
            uConfig0 &= ~(FLASH_CONFIG_CWDTEN | FLASH_CONFIG_CWDTPDEN);
            uConfig0 |= (FLASH_CONFIG_CWDTEN_BY_LIRCEN | FLASH_CONFIG_CWDTPDEN);
            break;

        default:
            if (((uConfig0 & FLASH_CONFIG_CWDTEN) == FLASH_CONFIG_CWDTEN_INACTIVE) ||
                    (((uConfig0 & FLASH_CONFIG_CWDTEN) == FLASH_CONFIG_CWDTEN_BY_LIRCEN) && (uConfig0 & FLASH_CONFIG_CWDTPDEN))) {
                uConfig0 &= ~(FLASH_CONFIG_CWDTEN | FLASH_CONFIG_CWDTPDEN);
                uConfig0 |= (FLASH_CONFIG_CWDTEN_ACTIVE | FLASH_CONFIG_CWDTPDEN);
            }
    }

    if (m_nRadioBS == 0) {
        uConfig0 |=  TC8226_FLASH_CONFIG_CBS;
    } else {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CBS;
    }

    if (m_nRadioGPG == 0) {
        uConfig0 |=  TC8226_FLASH_CONFIG_CGPFMFP;
    } else {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CGPFMFP;
    }

    if (m_nRadioIO == 0) {
        uConfig0 |=  TC8226_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CIOINI;
    }

    if (m_bCheckMBS) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_MBS;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_MBS;
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_CBORST;
    }

    if (m_bDisableICE) {
        uConfig0 &= ~M2351_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |= M2351_FLASH_CONFIG_ICELOCK;
    }

    uConfig3 &= ~M2351_FLASH_CONFIG_UART1PSL;

    switch (m_nRadioUART) {
        case 0:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL4;
            break;

        case 1:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL3;
            break;

        case 2:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL2;
            break;

        case 3:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL1;
            break;

        case 4:
            uConfig3 |= M2351_FLASH_CONFIG_UART1PSL_SEL0;
            break;

        default:
            /* Keep old value */
            uConfig3 |= (m_uConfigValue[3] & M2351_FLASH_CONFIG_UART1PSL);
    }

    m_uConfigValue[0] = uConfig0;
    m_uConfigValue[3] = uConfig3;
}


