// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_M2003.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define page_size NUMICRO_FLASH_PAGE_SIZE_512

CDialogConfiguration_M2003::CDialogConfiguration_M2003(unsigned int uProgramMemorySize,
    unsigned int uPID,
    UINT nIDTemplate,
    CWnd* pParent /*=NULL*/)
    : CDialogResize(nIDTemplate, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uPID(uPID)
    , m_ALOCK(16, 2)
{
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_nRadioSPIM = -1;
    m_nRadioUART = -1;
    m_nRadioWDT = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue2 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bSecurityLock = FALSE;
    m_bICELock = FALSE;
    m_nRadioIO = -1;
}

void CDialogConfiguration_M2003::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE, m_nRadioWDT);
    DDX_Radio(pDX, IDC_RADIO_SPIM_SEL0, m_nRadioSPIM);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2, m_sConfigValue2);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK, m_bICELock);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Control(pDX, IDC_EDIT_FLASH_ADVANCE_LOCK, m_ALOCK);
    DDX_Text(pDX, IDC_EDIT_FLASH_ADVANCE_LOCK, m_sALOCK);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    //}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDialogConfiguration_M2003, CDialog)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_STOP, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL1, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_ADVANCE_LOCK, OnKillfocusEditAdvanceLock)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE, OnDeltaposSpinDataFlashSize)
END_MESSAGE_MAP()

BOOL CDialogConfiguration_M2003::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    ConfigToGUI(0);
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_M2003::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig2 = m_ConfigValue.m_value[2];

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL) {
    case NUMICRO_FLASH_CONFIG_CBOV_7:
    case NUMICRO_FLASH_CONFIG_CBOV_6:
        m_nRadioBov = 0;
        break;

    case NUMICRO_FLASH_CONFIG_CBOV_5:
    case NUMICRO_FLASH_CONFIG_CBOV_4:
        m_nRadioBov = 1;
        break;

    case NUMICRO_FLASH_CONFIG_CBOV_3:
    case NUMICRO_FLASH_CONFIG_CBOV_2:
        m_nRadioBov = 2;
        break;

    case NUMICRO_FLASH_CONFIG_CBOV_1:
    case NUMICRO_FLASH_CONFIG_CBOV_0:
        m_nRadioBov = 3;
        break;

    default:
        /* Keep old value */
        uConfig0 |= (m_ConfigValue.m_value[0] & NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL);
    }

    switch (uConfig0 & M480_FLASH_CONFIG_CBS2) {
    case M480_FLASH_CONFIG_CBS_LD:
        m_nRadioBS = 0;
        break;

    case M480_FLASH_CONFIG_CBS_AP:
        m_nRadioBS = 1;
        break;

    default:
        m_nRadioBS = 1;
        break;
    }

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) {
    case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
        m_nRadioWDT = 0;
        break;

    case NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN:
        if (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTPDEN) {
            m_nRadioWDT = 2;
        }
        else {
            m_nRadioWDT = 1;
        }

        break;

    default:
        m_nRadioWDT = 1;
    }

    m_nRadioIO = ((uConfig0 & M480_FLASH_CONFIG_CIOINI) == 0 ? 1 : 0);
    m_nRadioSPIM = ((uConfig0 & M0A21_FLASH_CONFIG_CFGRPS) == 0 ? 1 : 0);

    m_bCheckBrownOutDetect = ((uConfig0 & M480_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & M480_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bICELock = ((uConfig0 & M480_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);

    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_sALOCK.Format(_T("%02X"), uConfig2 & 0xFF);

    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue2.Format(_T("0x%08X"), uConfig2);

}

void CDialogConfiguration_M2003::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig2 = m_ConfigValue.m_value[2];
    uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL;

    switch (m_nRadioBov) {
    case 0:
        uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_7;
        break;

    case 1:
        uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_5;
        break;

    case 2:
        uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_3;
        break;

    case 3:
        uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_1;
        break;

    default:
        /* Keep old value */
        uConfig0 |= (m_ConfigValue.m_value[0] & NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL);
    }

    uConfig0 &= ~M480_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
    case 0:
        uConfig0 |= M480_FLASH_CONFIG_CBS_LD;
        break;

    case 1:
        uConfig0 |= M480_FLASH_CONFIG_CBS_AP;
        break;

    default:
        /* Keep old value */
        uConfig0 |= (m_ConfigValue.m_value[0] & M480_FLASH_CONFIG_CBS2);
    }

    uConfig0 |= M480_FLASH_CONFIG_CWDTPDEN;

    uConfig0 |= M480_FLASH_CONFIG_CWDTEN;

    switch (m_nRadioWDT) {
    case 0:
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
        uConfig0 |= NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
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

    if (m_nRadioIO == 0) {
        uConfig0 |= M480_FLASH_CONFIG_CIOINI;
    }
    else {
        uConfig0 &= ~M480_FLASH_CONFIG_CIOINI;
    }

    if (m_nRadioSPIM == 0) {
        uConfig0 |= M0A21_FLASH_CONFIG_CFGRPS;
    }
    else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_CFGRPS;
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~M480_FLASH_CONFIG_CBODEN;
    }
    else {
        uConfig0 |= M480_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~M480_FLASH_CONFIG_CBORST;
    }
    else {
        uConfig0 |= M480_FLASH_CONFIG_CBORST;
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
        uConfig2 &= ~M480_FLASH_CONFIG_ALOCK;
        unsigned int uALOCK = ::_tcstoul(m_sALOCK, NULL, 16) & 0x5A;
        uConfig2 = 0xFFFFFF00 | uALOCK;
    }
    else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_LOCK;
        uConfig2 &= ~M480_FLASH_CONFIG_ALOCK;
        uConfig2 |= 0x0000005A;
    }

    if (m_bICELock) {
        uConfig0 &= ~M480_FLASH_CONFIG_ICELOCK;
    }
    else {
        uConfig0 |= M480_FLASH_CONFIG_ICELOCK;
    }

    m_ConfigValue.m_value[0] = uConfig0;

    m_ConfigValue.m_value[2] = uConfig2;
}

void CDialogConfiguration_M2003::OnGUIEvent(int nEventID)
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig(nEventID);
    ConfigToGUI(nEventID);
    UpdateData(FALSE);
}

void CDialogConfiguration_M2003::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}

void CDialogConfiguration_M2003::OnKillfocusEditAdvanceLock()
{
    OnGUIEvent();
}

void CDialogConfiguration_M2003::OnOK()
{
    UpdateData(TRUE);
    GUIToConfig(0);
    CDialog::OnOK();
}

void CDialogConfiguration_M2003::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}

///////////////////////////////////////////////////////////////////////////////