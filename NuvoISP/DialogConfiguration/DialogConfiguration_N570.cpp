// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_N570.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_N570 dialog

CDialogConfiguration_N570::CDialogConfiguration_N570(unsigned int uProgramMemorySize,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_N570::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_N570)
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bCheckLowVolResetEnable = FALSE;
    m_bCheckBrownOutHysteresis = FALSE;
    m_bCheckBrownOutEnable = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_sFlashBaseAddress = _T("");
    m_uPageSize = NUMICRO_FLASH_PAGE_SIZE_512;
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_N570::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_N570)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_ENABLE, m_bCheckBrownOutEnable);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_LVOL_RSTEN, m_bCheckLowVolResetEnable);
    DDX_Check(pDX, IDC_CHECK_BOD_HYS_EN, m_bCheckBrownOutHysteresis);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_N570, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_N570)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_8, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_9, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_A, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_B, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_C, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_D, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_E, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_F, OnButtonClick)

    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_CHECK_LVOL_RSTEN, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BOD_HYS_EN, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)

    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE, OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_N570 message handlers

BOOL CDialogConfiguration_N570::OnInitDialog()
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

void CDialogConfiguration_N570::ConfigToGUI()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];

    switch (uConfig0 & N570_FLASH_CONFIG_CBOV) {
        case N570_FLASH_CONFIG_CBOV_46:
            m_nRadioBov = 0;
            break;

        case N570_FLASH_CONFIG_CBOV_42:
            m_nRadioBov = 1;
            break;

        case N570_FLASH_CONFIG_CBOV_39:
            m_nRadioBov = 2;
            break;

        case N570_FLASH_CONFIG_CBOV_37:
            m_nRadioBov = 3;
            break;

        case N570_FLASH_CONFIG_CBOV_36:
            m_nRadioBov = 4;
            break;

        case N570_FLASH_CONFIG_CBOV_34:
            m_nRadioBov = 5;
            break;

        case N570_FLASH_CONFIG_CBOV_31:
            m_nRadioBov = 6;
            break;

        case N570_FLASH_CONFIG_CBOV_30:
            m_nRadioBov = 7;
            break;

        case N570_FLASH_CONFIG_CBOV_28:
            m_nRadioBov = 8;
            break;

        case N570_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 9;
            break;

        case N570_FLASH_CONFIG_CBOV_24:
            m_nRadioBov = 10;
            break;

        case N570_FLASH_CONFIG_CBOV_22:
            m_nRadioBov = 11;
            break;

        case N570_FLASH_CONFIG_CBOV_21:
            m_nRadioBov = 12;
            break;

        case N570_FLASH_CONFIG_CBOV_20:
            m_nRadioBov = 13;
            break;

        case N570_FLASH_CONFIG_CBOV_19:
            m_nRadioBov = 14;
            break;

        case N570_FLASH_CONFIG_CBOV_18:
            m_nRadioBov = 15;
            break;

        default:
            m_nRadioBov = 0;
            break;
    }

    m_bCheckBrownOutReset = ((uConfig0 & N570_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutEnable = ((uConfig0 & N570_FLASH_CONFIG_CBOVEN) == 0 ? TRUE : FALSE);
    m_nRadioBS = ((uConfig0 & N570_FLASH_CONFIG_CBS) == 0 ? 0 : 1);
    m_bCheckLowVolResetEnable = ((uConfig0 & N570_FLASH_CONFIG_CLVR) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutHysteresis = ((uConfig0 & N570_FLASH_CONFIG_CBHYS) == 0 ? FALSE : TRUE);
    m_bDataFlashEnable = ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    unsigned int uFlashBaseAddress = uConfig1 & 0xFFFFF;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    unsigned int uPageNum = uFlashBaseAddress / m_uPageSize;
    unsigned int uLimitNum = m_uProgramMemorySize / m_uPageSize;
    unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * m_uPageSize) : m_uPageSize;
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
    m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable ? TRUE : FALSE);
    GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS)->EnableWindow(m_bDataFlashEnable);

    if (m_bDataFlashEnable) {
        uFlashBaseAddress = m_uProgramMemorySize - uDataFlashSize;
        m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
        uConfig1 = uFlashBaseAddress;
    }

    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
}

void CDialogConfiguration_N570::GUIToConfig()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1;
    uConfig0 &= ~N570_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_46;
            break;

        case 1:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_42;
            break;

        case 2:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_39;
            break;

        case 3:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_37;
            break;

        case 4:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_36;
            break;

        case 5:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_34;
            break;

        case 6:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_31;
            break;

        case 7:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_30;
            break;

        case 8:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_28;
            break;

        case 9:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_26;
            break;

        case 10:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_24;
            break;

        case 11:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_22;
            break;

        case 12:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_21;
            break;

        case 13:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_20;
            break;

        case 14:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_19;
            break;

        case 15:
            uConfig0 |= N570_FLASH_CONFIG_CBOV_18;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & N570_FLASH_CONFIG_CBOV);
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~N570_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= N570_FLASH_CONFIG_CBORST;
    }

    if (m_nRadioBS == 0) {
        uConfig0 &= ~N570_FLASH_CONFIG_CBS;
    } else {
        uConfig0 |= N570_FLASH_CONFIG_CBS;
    }

    if (m_bCheckBrownOutEnable) {
        uConfig0 &= ~N570_FLASH_CONFIG_CBOVEN;
    } else {
        uConfig0 |= N570_FLASH_CONFIG_CBOVEN;
    }

    if (m_bDataFlashEnable) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_DFEN;
    } else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_DFEN;
        m_sFlashBaseAddress = "FFFFF";
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_LOCK;
    }

    if (m_bCheckLowVolResetEnable) {
        uConfig0 &= ~N570_FLASH_CONFIG_CLVR;
    } else {
        uConfig0 |= N570_FLASH_CONFIG_CLVR;
    }

    if (m_bCheckBrownOutHysteresis) {
        uConfig0 |= N570_FLASH_CONFIG_CBHYS;
    } else {
        uConfig0 &= ~N570_FLASH_CONFIG_CBHYS;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;// | 0xFFF00000;
}

void CDialogConfiguration_N570::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig();
    ConfigToGUI();
    UpdateData(FALSE);
}

void CDialogConfiguration_N570::OnKillfocusEditFlashBaseAddress()
{
    UpdateData(TRUE);
    CDialogResize::OnKillfocusEditFlashBaseAddress(m_bDataFlashEnable, m_uProgramMemorySize, m_uPageSize);
}

void CDialogConfiguration_N570::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    OnKillfocusEditFlashBaseAddress();
    GUIToConfig();
    CDialog::OnOK();
}

void CDialogConfiguration_N570::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    CDialogResize::OnDeltaposSpinDataFlashSize(pNMHDR, pResult, m_bDataFlashEnable, m_uProgramMemorySize, m_uPageSize);
}

void CDialogConfiguration_N570::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}

