// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_M031.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define page_size NUMICRO_FLASH_PAGE_SIZE_512

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M031 dialog

CDialogConfiguration_M031::CDialogConfiguration_M031(unsigned int uProgramMemorySize,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_M031)
    m_nRadioLVR = -1;
    m_nRadioXT1 = -1;
    m_nRadioBov = -1;
    m_nRadioIO = -1;
    m_nRadioRstExt = -1;
    m_nRadioRstWSel = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_sFlashBaseAddress = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bDisableICE = FALSE;
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_M031::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_M031)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_LVR_LEVEL_0, m_nRadioLVR);
    DDX_Radio(pDX, IDC_RADIO_HXT_MODE_0, m_nRadioXT1);
    DDX_Radio(pDX, IDC_RADIO_CHIPRESET_TIMEEXT_0, m_nRadioRstExt);
    DDX_Radio(pDX, IDC_RADIO_RST_PIN_WIDTH_0, m_nRadioRstWSel);
    DDX_Radio(pDX, IDC_RADIO_IO_BI, m_nRadioIO);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK, m_bDisableICE);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M031, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_M031)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_LVR_LEVEL_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_LVR_LEVEL_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_HXT_MODE_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_HXT_MODE_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_CHIPRESET_TIMEEXT_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_CHIPRESET_TIMEEXT_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_RST_PIN_WIDTH_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_RST_PIN_WIDTH_1, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE, OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M031 message handlers

BOOL CDialogConfiguration_M031::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);
    ConfigToGUI();
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_M031::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];

    switch (uConfig0 & M031_FLASH_CONFIG_CBOV) {
        case M031_FLASH_CONFIG_CBOV_0:
            m_nRadioBov = 0;
            break;

        case M031_FLASH_CONFIG_CBOV_1:
            m_nRadioBov = 1;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M031_FLASH_CONFIG_CBOV);
    }

    switch (uConfig0 & MINI51_FLASH_CONFIG_CBS2) {
        case MINI51_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case MINI51_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case MINI51_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case MINI51_FLASH_CONFIG_CBS_AP_LD:
            m_nRadioBS = 3;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & MINI51_FLASH_CONFIG_CBS2);
    }

    m_nRadioLVR = ((uConfig0 & M031_FLASH_CONFIG_LVRLVSEL) ? 1 : 0);
    m_nRadioXT1 = ((uConfig0 & M031_FLASH_CONFIG_CFGXT1) ? 1 : 0);
    m_nRadioIO = ((uConfig0 & M031_FLASH_CONFIG_CIOINI) ? 1 : 0);
    m_nRadioRstExt = ((uConfig0 & M031_FLASH_CONFIG_RSTEXT) ? 1 : 0);
    m_nRadioRstWSel = ((uConfig0 & M031_FLASH_CONFIG_RSTWSEL) ? 1 : 0);
    m_bCheckBrownOutDetect = ((uConfig0 & M031_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & M031_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bDisableICE = ((uConfig0 & M031_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & MINI51_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & MINI51_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    unsigned int uFlashBaseAddress = uConfig1;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    unsigned int uPageNum = uFlashBaseAddress / NUMICRO_FLASH_PAGE_SIZE_512;
    unsigned int uLimitNum = m_uProgramMemorySize / NUMICRO_FLASH_PAGE_SIZE_512;
    unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * NUMICRO_FLASH_PAGE_SIZE_512) : 0;
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
    m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable ? TRUE : FALSE);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
}

void CDialogConfiguration_M031::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1;
    uConfig0 &= ~M031_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= M031_FLASH_CONFIG_CBOV_0;
            break;

        case 1:
            uConfig0 |= M031_FLASH_CONFIG_CBOV_1;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M031_FLASH_CONFIG_CBOV);
    }

    uConfig0 &= ~MINI51_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= MINI51_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= MINI51_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= MINI51_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= MINI51_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & MINI51_FLASH_CONFIG_CBS2);
    }

    if (m_nRadioLVR == 0) {
        uConfig0 &= ~M031_FLASH_CONFIG_LVRLVSEL;
    } else {
        uConfig0 |= M031_FLASH_CONFIG_LVRLVSEL;
    }

    if (m_nRadioXT1 == 0) {
        uConfig0 &= ~M031_FLASH_CONFIG_CFGXT1;
    } else {
        uConfig0 |= M031_FLASH_CONFIG_CFGXT1;
    }

    if (m_nRadioIO == 0) {
        uConfig0 &= ~M031_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 |= M031_FLASH_CONFIG_CIOINI;
    }

    if (m_nRadioRstExt == 0) {
        uConfig0 &= ~M031_FLASH_CONFIG_RSTEXT;
    } else {
        uConfig0 |= M031_FLASH_CONFIG_RSTEXT;
    }

    if (m_nRadioRstWSel == 0) {
        uConfig0 &= ~M031_FLASH_CONFIG_RSTWSEL;
    } else {
        uConfig0 |= M031_FLASH_CONFIG_RSTWSEL;
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~M031_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= M031_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~M031_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= M031_FLASH_CONFIG_CBORST;
    }

    if (m_bDisableICE) {
        uConfig0 &= ~M031_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |= M031_FLASH_CONFIG_ICELOCK;
    }

    if (m_bDataFlashEnable) {
        uConfig0 &= ~MINI51_FLASH_CONFIG_DFEN;
    } else {
        uConfig0 |= MINI51_FLASH_CONFIG_DFEN;
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~MINI51_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= MINI51_FLASH_CONFIG_LOCK;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;
}

void CDialogConfiguration_M031::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig();
    ConfigToGUI();
    UpdateData(FALSE);
}

void CDialogConfiguration_M031::OnKillfocusEditFlashBaseAddress()
{
    // TODO: If this is a RICHEDIT control, the control will not
    // send this notification unless you override the CDialog::OnInitDialog()
    // function and call CRichEditCtrl().SetEventMask()
    // with the ENM_CHANGE flag ORed into the mask.
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    TCHAR *pEnd;
    unsigned int uFlashBaseAddress = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);

    if (m_bDataFlashEnable) {
        if (!((uFlashBaseAddress >= page_size) && (uFlashBaseAddress < m_uProgramMemorySize))) {
            uFlashBaseAddress = m_uProgramMemorySize - page_size;
        }

        uFlashBaseAddress &= ~(page_size - 1);
        m_sDataFlashSize.Format(_T("%.2fK"), (uFlashBaseAddress < m_uProgramMemorySize) ? ((m_uProgramMemorySize - uFlashBaseAddress) / 1024.) : 0.);
    }

    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    m_sConfigValue1.Format(_T("0x%08X"), uFlashBaseAddress);
    UpdateData(FALSE);
}

void CDialogConfiguration_M031::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    GUIToConfig();
    CDialog::OnOK();
}

void CDialogConfiguration_M031::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    TCHAR *pEnd;
    unsigned int uFlashBaseAddress = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    unsigned int uPageNum = uFlashBaseAddress / NUMICRO_FLASH_PAGE_SIZE_512;
    unsigned int uLimitNum = m_uProgramMemorySize / NUMICRO_FLASH_PAGE_SIZE_512;

    if (pNMUpDown->iDelta == 1) {
        uPageNum += 1;
    } else if (pNMUpDown->iDelta == -1 && uPageNum > 0) {
        uPageNum -= 1;
    }

    uFlashBaseAddress = 0 + min(uPageNum, uLimitNum) * NUMICRO_FLASH_PAGE_SIZE_512;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    m_sConfigValue1.Format(_T("0x%08X"), uFlashBaseAddress);
    unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * NUMICRO_FLASH_PAGE_SIZE_512) : 0;
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
    UpdateData(FALSE);
    *pResult = 0;
}

void CDialogConfiguration_M031::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}
