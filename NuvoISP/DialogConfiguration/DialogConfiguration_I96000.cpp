// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "Lang.h"
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_I96000.h"
#include <cassert>

#ifdef _DEBUG
    #define new DEBUG_NEW
    #undef THIS_FILE
    static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_I96000 dialog


CDialogConfiguration_I96000::CDialogConfiguration_I96000(CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_I96000::IDD, pParent)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_I96000)
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bSecurityLock = FALSE;
    m_bICELock = FALSE;
    m_bCheckJTAGBootFail = FALSE;
    m_bCheckClearRstSts = FALSE;
    m_bCheckJTAGBootSuccess = FALSE;
    m_bPDWakeupPin = FALSE;
    m_bPCWakeupPin = FALSE;
    m_bPBWakeupPin = FALSE;
    m_bPAWakeupPin = FALSE;
    m_nRadioBov = -1;
    m_nRadioIO = -1;
    m_nRadioStrapSel = -1;
    m_nRadioSPIMPinSel = -1;
    m_nRadioAppLoadFail = -1;
    //}}AFX_DATA_INIT
}


void CDialogConfiguration_I96000::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_I96000)
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    DDX_Radio(pDX, IDC_RADIO_STRAP_PIN, m_nRadioStrapSel);
    DDX_Radio(pDX, IDC_RADIO_SPIM_SEL0, m_nRadioSPIMPinSel);
    DDX_Radio(pDX, IDC_RADIO_LOAD_FAIL_DISABLE, m_nRadioAppLoadFail);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK, m_bICELock);
    DDX_Check(pDX, IDC_CHECK_JTAG_BOOTFAIL, m_bCheckJTAGBootFail);
    DDX_Check(pDX, IDC_CHECK_CLEAR_RSTSTS, m_bCheckClearRstSts);
    DDX_Check(pDX, IDC_CHECK_JTAG_BOOTSUCCESS, m_bCheckJTAGBootSuccess);
    DDX_Check(pDX, IDC_CHECK_PIN_WAKEUP_0, m_bPDWakeupPin);
    DDX_Check(pDX, IDC_CHECK_PIN_WAKEUP_1, m_bPCWakeupPin);
    DDX_Check(pDX, IDC_CHECK_PIN_WAKEUP_2, m_bPBWakeupPin);
    DDX_Check(pDX, IDC_CHECK_PIN_WAKEUP_3, m_bPAWakeupPin);
    DDX_CBIndex(pDX, IDC_COMBO_FUSE_RETRY, m_nComboboxRetryCount);
    DDX_CBIndex(pDX, IDC_COMBO_PIN_WAKEUP_0, m_nComboboxPDWakeupPinSel);
    DDX_CBIndex(pDX, IDC_COMBO_PIN_WAKEUP_1, m_nComboboxPCWakeupPinSel);
    DDX_CBIndex(pDX, IDC_COMBO_PIN_WAKEUP_2, m_nComboboxPBWakeupPinSel);
    DDX_CBIndex(pDX, IDC_COMBO_PIN_WAKEUP_3, m_nComboboxPAWakeupPinSel);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2, m_sConfigValue2);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_I96000, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_I96000)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_STRAP_PIN, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_STRAP_FUSE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_JTAG_BOOTFAIL, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CLEAR_RSTSTS, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_JTAG_BOOTSUCCESS, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_LOAD_FAIL_DISABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_LOAD_FAIL_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_PIN_WAKEUP_0, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_PIN_WAKEUP_1, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_PIN_WAKEUP_2, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_PIN_WAKEUP_3, OnButtonClick)
    ON_CBN_SELCHANGE(IDC_COMBO_FUSE_RETRY, OnButtonClick)
    ON_CBN_SELCHANGE(IDC_COMBO_PIN_WAKEUP_0, OnButtonClick)
    ON_CBN_SELCHANGE(IDC_COMBO_PIN_WAKEUP_1, OnButtonClick)
    ON_CBN_SELCHANGE(IDC_COMBO_PIN_WAKEUP_2, OnButtonClick)
    ON_CBN_SELCHANGE(IDC_COMBO_PIN_WAKEUP_3, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_I96000 message handlers

BOOL CDialogConfiguration_I96000::OnInitDialog()
{
    CDialog::OnInitDialog();

    ConfigToGUI(0);

    UpdateData(FALSE);

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;
}

void CDialogConfiguration_I96000::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];
    unsigned int uConfig2 = m_ConfigValue.m_value[2];

    switch (uConfig0 & I96000_FLASH_CONFIG_CBOV)
    {
        case I96000_FLASH_CONFIG_CBOV_7:
            m_nRadioBov = 0;
            break;

        case I96000_FLASH_CONFIG_CBOV_6:
            m_nRadioBov = 1;
            break;

        case I96000_FLASH_CONFIG_CBOV_5:
            m_nRadioBov = 2;
            break;

        case I96000_FLASH_CONFIG_CBOV_4:
            m_nRadioBov = 3;
            break;

        case I96000_FLASH_CONFIG_CBOV_3:
            m_nRadioBov = 4;
            break;

        case I96000_FLASH_CONFIG_CBOV_2:
            m_nRadioBov = 5;
            break;

        case I96000_FLASH_CONFIG_CBOV_1:
            m_nRadioBov = 6;
            break;

        case I96000_FLASH_CONFIG_CBOV_0:
            m_nRadioBov = 7;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & I96000_FLASH_CONFIG_CBOV);
    }

    m_bCheckBrownOutDetect = ((uConfig0 & I96000_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & I96000_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & I96000_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bICELock = ((uConfig0 & I96000_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    m_nRadioIO = ((uConfig0 & I96000_FLASH_CONFIG_CIOINI) == 0 ? 1 : 0);


    /* Config 1 */
    m_nRadioStrapSel        = ((uConfig1 & I96000_FLASH_CONFIG_BOOTSEL) == 0 ? 1 : 0);
    m_nRadioSPIMPinSel      = ((uConfig1 & I96000_FLASH_CONFIG_SPIMPINSEL) == 0 ? 1 : 0);
    m_nRadioAppLoadFail     = ((uConfig1 & I96000_FLASH_CONFIG_ACT) == 0 ? 1 : 0);
    m_bCheckJTAGBootFail    = ((uConfig1 & I96000_FLASH_CONFIG_JTAGFAIL) == 0 ? TRUE : FALSE);
    m_bCheckClearRstSts     = ((uConfig1 & I96000_FLASH_CONFIG_CLRRSTSTS) == 0 ? TRUE : FALSE);
    m_bCheckJTAGBootSuccess = ((uConfig1 & I96000_FLASH_CONFIG_JTAGSUCCESS) == 0 ? TRUE : FALSE);
    m_nComboboxRetryCount   = (uConfig1 & I96000_FLASH_CONFIG_RETRY) >> 16;

    /* Config 2 */
    m_bPDWakeupPin  = ((uConfig2 & I96000_FLASH_CONFIG_PDWKEN) == 0 ? TRUE : FALSE);
    m_bPCWakeupPin  = ((uConfig2 & I96000_FLASH_CONFIG_PCWKEN) == 0 ? TRUE : FALSE);
    m_bPBWakeupPin  = ((uConfig2 & I96000_FLASH_CONFIG_PBWKEN) == 0 ? TRUE : FALSE);
    m_bPAWakeupPin  = ((uConfig2 & I96000_FLASH_CONFIG_PAWKEN) == 0 ? TRUE : FALSE);
    m_nComboboxPDWakeupPinSel = (uConfig2 & I96000_FLASH_CONFIG_PDWKPSEL) >> 24;
    m_nComboboxPCWakeupPinSel = (uConfig2 & I96000_FLASH_CONFIG_PCWKPSEL) >> 16;
    m_nComboboxPBWakeupPinSel = (uConfig2 & I96000_FLASH_CONFIG_PBWKPSEL) >> 8;
    m_nComboboxPAWakeupPinSel = (uConfig2 & I96000_FLASH_CONFIG_PAWKPSEL);

    GetDlgItem(IDC_CHECK_PIN_WAKEUP_0)->EnableWindow(m_nRadioAppLoadFail);
    GetDlgItem(IDC_CHECK_PIN_WAKEUP_1)->EnableWindow(m_nRadioAppLoadFail);
    GetDlgItem(IDC_CHECK_PIN_WAKEUP_2)->EnableWindow(m_nRadioAppLoadFail);
    GetDlgItem(IDC_CHECK_PIN_WAKEUP_3)->EnableWindow(m_nRadioAppLoadFail);
    GetDlgItem(IDC_COMBO_PIN_WAKEUP_0)->EnableWindow(m_nRadioAppLoadFail);
    GetDlgItem(IDC_COMBO_PIN_WAKEUP_1)->EnableWindow(m_nRadioAppLoadFail);
    GetDlgItem(IDC_COMBO_PIN_WAKEUP_2)->EnableWindow(m_nRadioAppLoadFail);
    GetDlgItem(IDC_COMBO_PIN_WAKEUP_3)->EnableWindow(m_nRadioAppLoadFail);

    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
    m_sConfigValue2.Format(_T("0x%08X"), uConfig2);
}

void CDialogConfiguration_I96000::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];
    unsigned int uConfig2 = m_ConfigValue.m_value[2];

    uConfig0 &= ~I96000_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov)
    {
        case 0:
            uConfig0 |= I96000_FLASH_CONFIG_CBOV_7;
            break;

        case 1:
            uConfig0 |= I96000_FLASH_CONFIG_CBOV_6;
            break;

        case 2:
            uConfig0 |= I96000_FLASH_CONFIG_CBOV_5;
            break;

        case 3:
            uConfig0 |= I96000_FLASH_CONFIG_CBOV_4;
            break;

        case 4:
            uConfig0 |= I96000_FLASH_CONFIG_CBOV_3;
            break;

        case 5:
            uConfig0 |= I96000_FLASH_CONFIG_CBOV_2;
            break;

        case 6:
            uConfig0 |= I96000_FLASH_CONFIG_CBOV_1;
            break;

        case 7:
            uConfig0 |= I96000_FLASH_CONFIG_CBOV_0;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & I96000_FLASH_CONFIG_CBOV);
    }

    if (m_bCheckBrownOutDetect)
        uConfig0 &= ~I96000_FLASH_CONFIG_CBODEN;
    else
        uConfig0 |= I96000_FLASH_CONFIG_CBODEN;

    if (m_bCheckBrownOutReset)
        uConfig0 &= ~I96000_FLASH_CONFIG_CBORST;
    else
        uConfig0 |= I96000_FLASH_CONFIG_CBORST;

    if (m_bSecurityLock)
        uConfig0 &= ~I96000_FLASH_CONFIG_LOCK;
    else
        uConfig0 |= I96000_FLASH_CONFIG_LOCK;

    if (m_bICELock)
        uConfig0 &= ~I96000_FLASH_CONFIG_ICELOCK;
    else
        uConfig0 |= I96000_FLASH_CONFIG_ICELOCK;

    if (m_nRadioIO)
        uConfig0 &= ~I96000_FLASH_CONFIG_CIOINI;
    else
        uConfig0 |= I96000_FLASH_CONFIG_CIOINI;

    /* Config 1 */
    if (m_nRadioStrapSel)
        uConfig1 &= ~I96000_FLASH_CONFIG_BOOTSEL;
    else
        uConfig1 |= I96000_FLASH_CONFIG_BOOTSEL;

    if (m_nRadioSPIMPinSel)
        uConfig1 &= ~I96000_FLASH_CONFIG_SPIMPINSEL;
    else
        uConfig1 |= I96000_FLASH_CONFIG_SPIMPINSEL;

    if (m_nRadioAppLoadFail)
        uConfig1 &= ~I96000_FLASH_CONFIG_ACT;
    else
        uConfig1 |= I96000_FLASH_CONFIG_ACT;

    uConfig1 &= ~I96000_FLASH_CONFIG_RETRY;
    uConfig1 |= (m_nComboboxRetryCount << 16);

    if (m_bCheckJTAGBootFail)
        uConfig1 &= ~I96000_FLASH_CONFIG_JTAGFAIL;
    else
        uConfig1 |= I96000_FLASH_CONFIG_JTAGFAIL;

    if (m_bCheckClearRstSts)
        uConfig1 &= ~I96000_FLASH_CONFIG_CLRRSTSTS;
    else
        uConfig1 |= I96000_FLASH_CONFIG_CLRRSTSTS;

    if (m_bCheckJTAGBootSuccess)
        uConfig1 &= ~I96000_FLASH_CONFIG_JTAGSUCCESS;
    else
        uConfig1 |= I96000_FLASH_CONFIG_JTAGSUCCESS;

    /* Config2 */
    if (m_bPDWakeupPin)
        uConfig2 &= ~I96000_FLASH_CONFIG_PDWKEN;
    else
        uConfig2 |= I96000_FLASH_CONFIG_PDWKEN;

    uConfig2 &= ~I96000_FLASH_CONFIG_PDWKPSEL;
    uConfig2 |= (m_nComboboxPDWakeupPinSel << 24);

    if (m_bPCWakeupPin)
        uConfig2 &= ~I96000_FLASH_CONFIG_PCWKEN;
    else
        uConfig2 |= I96000_FLASH_CONFIG_PCWKEN;

    uConfig2 &= ~I96000_FLASH_CONFIG_PCWKPSEL;
    uConfig2 |= (m_nComboboxPCWakeupPinSel << 16);

    if (m_bPBWakeupPin)
        uConfig2 &= ~I96000_FLASH_CONFIG_PBWKEN;
    else
        uConfig2 |= I96000_FLASH_CONFIG_PBWKEN;

    uConfig2 &= ~I96000_FLASH_CONFIG_PBWKPSEL;
    uConfig2 |= (m_nComboboxPBWakeupPinSel << 8);

    if (m_bPAWakeupPin)
        uConfig2 &= ~I96000_FLASH_CONFIG_PAWKEN;
    else
        uConfig2 |= I96000_FLASH_CONFIG_PAWKEN;

    uConfig2 &= ~I96000_FLASH_CONFIG_PAWKPSEL;
    uConfig2 |= m_nComboboxPAWakeupPinSel;

    m_ConfigValue.m_value[0] = uConfig0;
    m_ConfigValue.m_value[1] = uConfig1;
    m_ConfigValue.m_value[2] = uConfig2;
}

void CDialogConfiguration_I96000::OnGUIEvent(int nEventID)
{
    UpdateData(TRUE);

    GUIToConfig(nEventID);
    ConfigToGUI(nEventID);

    UpdateData(FALSE);
}

void CDialogConfiguration_I96000::OnButtonClick()
{
    OnGUIEvent();
}

void CDialogConfiguration_I96000::OnOK()
{
    UpdateData(TRUE);

    GUIToConfig(0);

    CDialog::OnOK();
}

CString CDialogConfiguration_I96000::GetConfigWarning(const CAppConfig::I96000_configs_t &config)
{
    CString str;
    unsigned int uConfig0 = config.m_value[0];

    BOOL bSecurityLock = ((uConfig0 & I96000_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);

    if (!bSecurityLock)
        str += _T("   ") + _I(IDS_DISABLE_SECURITY_LOCK);

    return str;
}

void CDialogConfiguration_I96000::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    if (pScrollBar != NULL)
        return;

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}
