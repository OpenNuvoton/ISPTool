// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_M05x.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M05x dialog

CDialogConfiguration_M05x::CDialogConfiguration_M05x(UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialogResize(nIDTemplate, pParent)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_M05x)
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bClockFilterEnable = FALSE;
    m_bSecurityLock = FALSE;
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_M05x::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_M05x)
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_CLOCK_FILTER_ENABLE, m_bClockFilterEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M05x, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_M05x)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CLOCK_FILTER_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M05x message handlers

BOOL CDialogConfiguration_M05x::OnInitDialog()
{
    CDialog::OnInitDialog();
    ConfigToGUI();
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_M05x::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];

    switch (uConfig0 & M05X_FLASH_CONFIG_CBOV) {
        case M05X_FLASH_CONFIG_CBOV_45:
            m_nRadioBov = 0;
            break;

        case M05X_FLASH_CONFIG_CBOV_38:
            m_nRadioBov = 1;
            break;

        case M05X_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 2;
            break;

        case M05X_FLASH_CONFIG_CBOV_22:
        default:
            m_nRadioBov = 3;
            break;
    }

    m_nRadioBS = ((uConfig0 & M05X_FLASH_CONFIG_CBS) == 0 ? 0 : 1);
    m_bCheckBrownOutDetect = ((uConfig0 & M05X_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & M05X_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bClockFilterEnable = ((uConfig0 & M05X_FLASH_CONFIG_CKF) == M05X_FLASH_CONFIG_CKF ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_sConfigValue0.Format(_T("0x%08X"), m_ConfigValue.m_value[0]);
}

void CDialogConfiguration_M05x::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    uConfig0 &= ~M05X_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= M05X_FLASH_CONFIG_CBOV_45;
            break;

        case 1:
            uConfig0 |= M05X_FLASH_CONFIG_CBOV_38;
            break;

        case 2:
            uConfig0 |= M05X_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= M05X_FLASH_CONFIG_CBOV_22;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M05X_FLASH_CONFIG_CBOV);
    }

    if (m_nRadioBS == 0) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CBS;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CBS;
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CBORST;
    }

    if (m_bClockFilterEnable) {
        uConfig0 |= M05X_FLASH_CONFIG_CKF;
    } else {
        uConfig0 &= ~M05X_FLASH_CONFIG_CKF;
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_LOCK;
    }

    uConfig0 |= M05X_FLASH_CONFIG_CWDTEN;
    m_ConfigValue.m_value[0] = uConfig0;
}

void CDialogConfiguration_M05x::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig();
    ConfigToGUI();
    UpdateData(FALSE);
}

void CDialogConfiguration_M05x::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    GUIToConfig();
    CDialog::OnOK();
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M05XAN
/////////////////////////////////////////////////////////////////////////////

CDialogConfiguration_M05XAN::CDialogConfiguration_M05XAN(CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M05x(IDD_DIALOG_CONFIGURATION_M051, pParent)
{
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M05XBN
/////////////////////////////////////////////////////////////////////////////
CDialogConfiguration_M05XBN::CDialogConfiguration_M05XBN(CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M05x(IDD_DIALOG_CONFIGURATION_M051, pParent)
{
}

BOOL CDialogConfiguration_M05XBN::OnInitDialog()
{
    GetDlgItem(IDC_RADIO_BOV_1)->SetWindowText(_T("3.7V"));
    GetDlgItem(IDC_RADIO_BOV_0)->SetWindowText(_T("4.4V"));
    return CDialogConfiguration_M05x::OnInitDialog();
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M05XDN
/////////////////////////////////////////////////////////////////////////////

CDialogConfiguration_M05XDN::CDialogConfiguration_M05XDN(UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M05x(nIDTemplate, pParent)
{
    m_nRadioIO = -1;
    m_bWDTEnable = FALSE;
    m_bWDTPowerDown = FALSE;
}

void CDialogConfiguration_M05XDN::DoDataExchange(CDataExchange *pDX)
{
    CDialogConfiguration_M05x::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_M05XDN)
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    DDX_Check(pDX, IDC_CHECK_WDT_ENABLE, m_bWDTEnable);
    DDX_Check(pDX, IDC_CHECK_WDT_POWER_DOWN, m_bWDTPowerDown);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M05XDN, CDialogConfiguration_M05x)
    //{{AFX_MSG_MAP(CDialogConfiguration_M05XDN)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_POWER_DOWN, OnCheckClickWDTPD)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM, OnButtonClick)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

void CDialogConfiguration_M05XDN::OnCheckClickWDTPD()
{
    UpdateData(TRUE);
    GUIToConfig(IDC_CHECK_WDT_POWER_DOWN);
    ConfigToGUI(IDC_CHECK_WDT_POWER_DOWN);
    UpdateData(FALSE);
}

void CDialogConfiguration_M05XDN::ConfigToGUI(int nEventID)
{
    CDialogConfiguration_M05x::ConfigToGUI(nEventID);
    unsigned int uConfig0 = m_ConfigValue.m_value[0];

    switch (uConfig0 & M05X_FLASH_CONFIG_CBS2) {
        case M05X_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case M05X_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case M05X_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case M05X_FLASH_CONFIG_CBS_AP_LD:
        default:
            m_nRadioBS = 3;
            break;
    }

    m_nRadioIO = ((uConfig0 & M05X_FLASH_CONFIG_CIOINI) == 0 ? 0 : 1);
    m_bWDTPowerDown = ((uConfig0 & M05X_FLASH_CONFIG_CWDTPDEN) == 0 ? TRUE : FALSE);
    m_bWDTEnable = ((uConfig0 & M05X_FLASH_CONFIG_CWDTEN) == 0 ? TRUE : FALSE);;

    if (!m_bWDTEnable) {
        m_bWDTPowerDown = FALSE;
    }
}

void CDialogConfiguration_M05XDN::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    uConfig0 &= ~M05X_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= M05X_FLASH_CONFIG_CBOV_45;
            break;

        case 1:
            uConfig0 |= M05X_FLASH_CONFIG_CBOV_38;
            break;

        case 2:
            uConfig0 |= M05X_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= M05X_FLASH_CONFIG_CBOV_22;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M05X_FLASH_CONFIG_CBOV);
    }

    uConfig0 &= ~M05X_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= M05X_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= M05X_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= M05X_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= M05X_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M05X_FLASH_CONFIG_CBS2);
    }

    if (m_nRadioIO == 0) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CIOINI;
    }

    if (m_bWDTPowerDown) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CWDTPDEN;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CWDTPDEN;
    }

    if (m_bWDTEnable) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CWDTEN;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CWDTEN;
    }

    if (nEventID == IDC_CHECK_WDT_POWER_DOWN) {
        if (m_bWDTPowerDown) {
            uConfig0 &= ~M05X_FLASH_CONFIG_CWDTEN;
        }
    } else {
        if (!m_bWDTEnable) {
            uConfig0 |= M05X_FLASH_CONFIG_CWDTPDEN;
        }
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CBORST;
    }

    if (m_bClockFilterEnable) {
        uConfig0 |= M05X_FLASH_CONFIG_CKF;
    } else {
        uConfig0 &= ~M05X_FLASH_CONFIG_CKF;
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_LOCK;
    }

    m_ConfigValue.m_value[0] = uConfig0;
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M058SAN
/////////////////////////////////////////////////////////////////////////////

CDialogConfiguration_M058SAN::CDialogConfiguration_M058SAN(CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M05XDN(IDD_DIALOG_CONFIGURATION_M058, pParent)
{
    m_nRadioGP7 = -1;
}

void CDialogConfiguration_M058SAN::DoDataExchange(CDataExchange *pDX)
{
    CDialogConfiguration_M05XDN::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_M05XDN)
    DDX_Radio(pDX, IDC_RADIO_GPF_GPIO, m_nRadioGP7);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M058SAN, CDialogConfiguration_M05XDN)
    //{{AFX_MSG_MAP(CDialogConfiguration_M05XDN)
    ON_BN_CLICKED(IDC_RADIO_GPF_GPIO, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_CRYSTAL, OnButtonClick)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

void CDialogConfiguration_M058SAN::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    m_nRadioGP7 = ((uConfig0 & M05X_FLASH_CONFIG_CGP7MFP) == 0 ? 0 : 1);
    CDialogConfiguration_M05XDN::ConfigToGUI(nEventID);
}

void CDialogConfiguration_M058SAN::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];

    if (m_nRadioGP7 == 0) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CGP7MFP;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CGP7MFP;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    CDialogConfiguration_M05XDN::GUIToConfig(nEventID);
}
