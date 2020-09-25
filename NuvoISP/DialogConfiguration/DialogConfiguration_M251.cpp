// DialogConfiguration_M251.cpp : implementation file
//
#include "stdafx.h"
#include "ChipDefs.h"
#include "DialogConfiguration_M251.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define page_size NUMICRO_FLASH_PAGE_SIZE_512

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M251 dialog

CDialogConfiguration_M251::CDialogConfiguration_M251(unsigned int uProgramMemorySize,
        UINT nIDTemplate,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(nIDTemplate, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bSecurityLock = FALSE;
    m_bICELock = FALSE;
    m_bWDTEnable = FALSE;
    m_bWDTPowerDown = FALSE;
    m_nRadioIO = -1;
}

void CDialogConfiguration_M251::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK, m_bICELock);
    DDX_Check(pDX, IDC_CHECK_WDT_ENABLE, m_bWDTEnable);
    DDX_Check(pDX, IDC_CHECK_WDT_POWER_DOWN, m_bWDTPowerDown);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M251, CDialog)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_POWER_DOWN, OnCheckClickWDTPD)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M251 message handlers

BOOL CDialogConfiguration_M251::OnInitDialog()
{
    CDialog::OnInitDialog();
    ConfigToGUI(0);
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;
}

void CDialogConfiguration_M251::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];

    switch (uConfig0 & M480_FLASH_CONFIG_CBOV) {
        case M480_FLASH_CONFIG_CBOV_30:
            m_nRadioBov = 0;
            break;

        case M480_FLASH_CONFIG_CBOV_28:
            m_nRadioBov = 1;
            break;

        case M480_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 2;
            break;

        case M480_FLASH_CONFIG_CBOV_24:
            m_nRadioBov = 3;
            break;

        case M480_FLASH_CONFIG_CBOV_22:
            m_nRadioBov = 4;
            break;

        case M480_FLASH_CONFIG_CBOV_20:
            m_nRadioBov = 5;
            break;

        case M480_FLASH_CONFIG_CBOV_18:
            m_nRadioBov = 6;
            break;

        case M480_FLASH_CONFIG_CBOV_16:
            m_nRadioBov = 7;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M480_FLASH_CONFIG_CBOV);
    }

    switch (uConfig0 & M480_FLASH_CONFIG_CBS2) {
        case M480_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case M480_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case M480_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case M480_FLASH_CONFIG_CBS_AP_LD:
            m_nRadioBS = 3;
            break;

        default:
            m_nRadioBS = 1;
            break;
    }

    m_nRadioIO = ((uConfig0 & M480_FLASH_CONFIG_CIOINI) == 0 ? 1 : 0);
    m_bWDTPowerDown = ((uConfig0 & M480_FLASH_CONFIG_CWDTPDEN) == 0 ? TRUE : FALSE);
    m_bWDTEnable = ((uConfig0 & M480_FLASH_CONFIG_CWDTEN) == 0 ? TRUE : FALSE);;

    if (!m_bWDTEnable) {
        m_bWDTPowerDown = FALSE;
    }

    m_bCheckBrownOutDetect = ((uConfig0 & M480_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & M480_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bICELock = ((uConfig0 & M251_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
}

void CDialogConfiguration_M251::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    uConfig0 &= ~M480_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_30;
            break;

        case 1:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_28;
            break;

        case 2:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_24;
            break;

        case 4:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_22;
            break;

        case 5:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_20;
            break;

        case 6:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_18;
            break;

        case 7:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_16;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M480_FLASH_CONFIG_CBOV);
    }

    uConfig0 &= ~M480_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= M480_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= M480_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= M480_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= M480_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M480_FLASH_CONFIG_CBS2);
    }

    if (m_nRadioIO == 0) {
        uConfig0 |= M480_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~M480_FLASH_CONFIG_CIOINI;
    }

    if (m_bWDTPowerDown) {
        uConfig0 &= ~M480_FLASH_CONFIG_CWDTPDEN;
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CWDTPDEN;
    }

    if (m_bWDTEnable) {
        uConfig0 &= ~M480_FLASH_CONFIG_CWDTEN;

        if (!m_bWDTPowerDown) {
            uConfig0 &= ~M480_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 &= ~M480_FLASH_CONFIG_CWDTEN_BIT0;
        }
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CWDTEN;
    }

    if (nEventID == IDC_CHECK_WDT_POWER_DOWN) {
        if (m_bWDTPowerDown) {
            uConfig0 &= ~M480_FLASH_CONFIG_CWDTEN;
            uConfig0 |= M480_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 |= M480_FLASH_CONFIG_CWDTEN_BIT0;
        }
    } else {
        if (!m_bWDTEnable) {
            uConfig0 |= M480_FLASH_CONFIG_CWDTPDEN;
            uConfig0 |= M480_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 |= M480_FLASH_CONFIG_CWDTEN_BIT0;
        }
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~M480_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~M480_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CBORST;
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_LOCK;
    }

    if (m_bICELock) {
        uConfig0 &= ~M251_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |= M251_FLASH_CONFIG_ICELOCK;
    }

    m_ConfigValue.m_value[0] = uConfig0;
}

void CDialogConfiguration_M251::OnGUIEvent(int nEventID)
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig(nEventID);
    ConfigToGUI(nEventID);
    UpdateData(FALSE);
}

void CDialogConfiguration_M251::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}

void CDialogConfiguration_M251::OnCheckClickWDTPD()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent(IDC_CHECK_WDT_POWER_DOWN);
}

void CDialogConfiguration_M251::OnOK()
{
    // TODO: Add extra validation here
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogConfiguration_M251::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel ();
}

void CDialogConfiguration_M251::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M258
/////////////////////////////////////////////////////////////////////////////
CDialogConfiguration_M258::CDialogConfiguration_M258(unsigned int uProgramMemorySize, UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M251(uProgramMemorySize, nIDTemplate, pParent)
{
    m_nRadioBootClkSel = -1;
}

void CDialogConfiguration_M258::DoDataExchange(CDataExchange *pDX)
{
    CDialogConfiguration_M251::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_M258)
    DDX_Radio(pDX, IDC_RADIO_BOOT_CLOCK_SELECT_0, m_nRadioBootClkSel);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M258, CDialogConfiguration_M251)
    //{{AFX_MSG_MAP(CDialogConfiguration_M258)
    ON_BN_CLICKED(IDC_RADIO_BOOT_CLOCK_SELECT_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOOT_CLOCK_SELECT_1, OnButtonClick)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

void CDialogConfiguration_M258::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    m_nRadioBootClkSel = ((uConfig0 & M258_FLASH_CONFIG_BOOTCLOCKSEL) == 0 ? 0 : 1);
    CDialogConfiguration_M251::ConfigToGUI(nEventID);
}

void CDialogConfiguration_M258::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];

    if (m_nRadioBootClkSel == 0) {
        uConfig0 &= ~M258_FLASH_CONFIG_BOOTCLOCKSEL;
    } else {
        uConfig0 |= M258_FLASH_CONFIG_BOOTCLOCKSEL;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    CDialogConfiguration_M251::GUIToConfig(nEventID);
}