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
// CDialogConfiguration_M05X dialog


CDialogConfiguration_M05X::CDialogConfiguration_M05X(UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialogResize(nIDTemplate, pParent)
{
}

void CDialogConfiguration_M05X::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M05X, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_M05X)
    ON_BN_CLICKED(IDC_RADIO_BOV_45, OnRadioBov)
    ON_BN_CLICKED(IDC_RADIO_CLK_E12M, OnRadioClk)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnRadioBs)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE, OnCheckClickWDT)
    ON_BN_CLICKED(IDC_CHECK_WDT_POWER_DOWN, OnCheckClickWDTPD)
    ON_BN_CLICKED(IDC_RADIO_BOV_38, OnRadioBov)
    ON_BN_CLICKED(IDC_RADIO_BOV_27, OnRadioBov)
    ON_BN_CLICKED(IDC_RADIO_BOV_22, OnRadioBov)
    ON_BN_CLICKED(IDC_RADIO_CLK_I22M, OnRadioClk)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnRadioBs)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_CLOCK_FILTER_ENABLE, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_WATCHDOG_ENABLE, OnCheckClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnRadioIO)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnRadioIO)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM, OnRadioBs)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM, OnRadioBs)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M05X message handlers

BOOL CDialogConfiguration_M05X::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    ConfigToGUI(0);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_M05X::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];

    /* Clock Source Selection */
    if ((uConfig0 & M05X_FLASH_CONFIG_CFOSC) == M05X_FLASH_CONFIG_E12M) {
        ((CButton *)GetDlgItem(IDC_RADIO_CLK_E12M))->SetCheck(TRUE);
    } else if ((uConfig0 & M05X_FLASH_CONFIG_CFOSC) == M05X_FLASH_CONFIG_CFOSC) {
        ((CButton *)GetDlgItem(IDC_RADIO_CLK_I22M))->SetCheck(TRUE);
    }

    /* Brown Out Voltage */
    switch (uConfig0 & M05X_FLASH_CONFIG_CBOV) {
        case M05X_FLASH_CONFIG_CBOV_22:
            ((CButton *)GetDlgItem(IDC_RADIO_BOV_22))->SetCheck(TRUE);
            break;

        case M05X_FLASH_CONFIG_CBOV_26:
            ((CButton *)GetDlgItem(IDC_RADIO_BOV_27))->SetCheck(TRUE);
            break;

        case M05X_FLASH_CONFIG_CBOV_38:
            ((CButton *)GetDlgItem(IDC_RADIO_BOV_38))->SetCheck(TRUE);
            break;

        case M05X_FLASH_CONFIG_CBOV_45:
            ((CButton *)GetDlgItem(IDC_RADIO_BOV_45))->SetCheck(TRUE);
            break;
    }

    /* Boot Select */
    switch (uConfig0 & M05X_FLASH_CONFIG_CBS2) {
        case M05X_FLASH_CONFIG_CBS_LD_AP:
            CheckDlgButton(IDC_RADIO_BS_LDROM_APROM, TRUE);
            break;

        case M05X_FLASH_CONFIG_CBS_LD:
            CheckDlgButton(IDC_RADIO_BS_LDROM, TRUE);
            break;

        case M05X_FLASH_CONFIG_CBS_AP_LD:
            CheckDlgButton(IDC_RADIO_BS_APROM_LDROM, TRUE);
            break;

        case M05X_FLASH_CONFIG_CBS_AP:
            CheckDlgButton(IDC_RADIO_BS_APROM, TRUE);
            break;
    }

    /* Brown Out Detector Enable */
    if (uConfig0 & M05X_FLASH_CONFIG_CBODEN) {
        ((CButton *)GetDlgItem(IDC_CHECK_BROWN_OUT_DETECT))->SetCheck(FALSE);
    } else {
        ((CButton *)GetDlgItem(IDC_CHECK_BROWN_OUT_DETECT))->SetCheck(TRUE);
    }

    /* Brown Out Reset Enable */
    if (uConfig0 & M05X_FLASH_CONFIG_CBORST) {
        ((CButton *)GetDlgItem(IDC_CHECK_BROWN_OUT_RESET))->SetCheck(FALSE);
    } else {
        ((CButton *)GetDlgItem(IDC_CHECK_BROWN_OUT_RESET))->SetCheck(TRUE);
    }

    /* Security Lock */
    if (uConfig0 & M05X_FLASH_CONFIG_LOCK) {
        ((CButton *)GetDlgItem(IDC_CHECK_SECURITY_LOCK))->SetCheck(FALSE);
    } else {
        ((CButton *)GetDlgItem(IDC_CHECK_SECURITY_LOCK))->SetCheck(TRUE);
    }

    /* Watchdog Enable */
    if (uConfig0 & M05X_FLASH_CONFIG_CWDTEN) {
        CheckDlgButton(IDC_CHECK_WDT_ENABLE, FALSE);
    } else {
        CheckDlgButton(IDC_CHECK_WDT_ENABLE, TRUE);
    }

    /* Watchdog Clock Power Down Enable */
    if (uConfig0 & M05X_FLASH_CONFIG_CWDTPDEN) {
        CheckDlgButton(IDC_CHECK_WDT_POWER_DOWN, FALSE);
    } else {
        CheckDlgButton(IDC_CHECK_WDT_POWER_DOWN, TRUE);
    }

    /* I/O Initial State Select */
    if (uConfig0 & M05X_FLASH_CONFIG_CIOINI) {
        CheckDlgButton(IDC_RADIO_IO_BI, TRUE);
    } else {
        CheckDlgButton(IDC_RADIO_IO_TRI, TRUE);
    }

    /* Config0 Value */
    CString str;
    str.Format(_T("0x%08X"), uConfig0);
    ((CEdit *)GetDlgItem(IDC_STATIC_CONFIG_VALUE_0))->SetWindowText(str);
}

void CDialogConfiguration_M05X::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    /* Clock Source Selection */
    uConfig0 &= ~M05X_FLASH_CONFIG_CFOSC;

    if (IsDlgButtonChecked(IDC_RADIO_CLK_E12M)) {
        uConfig0 |= M05X_FLASH_CONFIG_E12M;
    } else if (IsDlgButtonChecked(IDC_RADIO_CLK_I22M)) {
        uConfig0 |= M05X_FLASH_CONFIG_CFOSC;
    }

    /* Brown Out Voltage */
    uConfig0 &= ~M05X_FLASH_CONFIG_CBOV;

    if (IsDlgButtonChecked(IDC_RADIO_BOV_45)) {
        uConfig0 |= M05X_FLASH_CONFIG_CBOV_45;
    } else if (IsDlgButtonChecked(IDC_RADIO_BOV_38)) {
        uConfig0 |= M05X_FLASH_CONFIG_CBOV_38;
    } else if (IsDlgButtonChecked(IDC_RADIO_BOV_27)) {
        uConfig0 |= M05X_FLASH_CONFIG_CBOV_26;
    } else if (IsDlgButtonChecked(IDC_RADIO_BOV_22)) {
        uConfig0 |= M05X_FLASH_CONFIG_CBOV_22;
    }

    /* Boot Select */
    uConfig0 &= ~M05X_FLASH_CONFIG_CBS2;

    if (IsDlgButtonChecked(IDC_RADIO_BS_LDROM)) {
        uConfig0 |= M05X_FLASH_CONFIG_CBS_LD;
    } else if (IsDlgButtonChecked(IDC_RADIO_BS_APROM)) {
        uConfig0 |= M05X_FLASH_CONFIG_CBS_AP;
    } else if (IsDlgButtonChecked(IDC_RADIO_BS_LDROM_APROM)) {
        uConfig0 |= M05X_FLASH_CONFIG_CBS_LD_AP;
    } else if (IsDlgButtonChecked(IDC_RADIO_BS_APROM_LDROM)) {
        uConfig0 |= M05X_FLASH_CONFIG_CBS_AP_LD;
    }

    /* Brown Out Detector Enable */
    if (IsDlgButtonChecked(IDC_CHECK_BROWN_OUT_DETECT)) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CBODEN;
    }

    /* Brown Out Reset Enable */
    if (IsDlgButtonChecked(IDC_CHECK_BROWN_OUT_RESET)) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_CBORST;
    }

    /* Security Lock */
    if (IsDlgButtonChecked(IDC_CHECK_SECURITY_LOCK)) {
        uConfig0 &= ~M05X_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= M05X_FLASH_CONFIG_LOCK;
    }

    /* Watchdog Enable */
    /* Watchdog Clock Power Down Enable */
    if (GetDlgItem(IDC_CHECK_WDT_ENABLE) != NULL) {
        if (IsDlgButtonChecked(IDC_CHECK_WDT_ENABLE)) {
            if (IsDlgButtonChecked(IDC_CHECK_WDT_POWER_DOWN)) {
                uConfig0 &= ~M05X_FLASH_CONFIG_CWDTEN;
                uConfig0 &= ~M05X_FLASH_CONFIG_CWDTPDEN;
            } else {
                uConfig0 &= ~M05X_FLASH_CONFIG_CWDTEN;
                uConfig0 |= M05X_FLASH_CONFIG_CWDTPDEN;
            }
        } else {
            if ((nEventID == IDC_CHECK_WDT_POWER_DOWN) && (IsDlgButtonChecked(IDC_CHECK_WDT_POWER_DOWN))) {
                uConfig0 &= ~M05X_FLASH_CONFIG_CWDTEN;
                uConfig0 &= ~M05X_FLASH_CONFIG_CWDTPDEN;
            } else {
                uConfig0 |= M05X_FLASH_CONFIG_CWDTEN;
                uConfig0 |= M05X_FLASH_CONFIG_CWDTPDEN;
            }
        }
    }

    /* I/O Initial State Select */
    if (IsDlgButtonChecked(IDC_RADIO_IO_BI)) {
        uConfig0 |= M05X_FLASH_CONFIG_CIOINI;
    } else if (IsDlgButtonChecked(IDC_RADIO_IO_TRI)) {
        uConfig0 &= ~M05X_FLASH_CONFIG_CIOINI;
    }

    m_ConfigValue.m_value[0] = uConfig0;
}

void CDialogConfiguration_M05X::OnGUIEvent(int nEventID)
{
    // TODO: Add your control notification handler code here
    GUIToConfig(nEventID);
    ConfigToGUI(nEventID);
}

void CDialogConfiguration_M05X::OnRadioBov()
{
    OnGUIEvent();
}

void CDialogConfiguration_M05X::OnRadioClk()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}

void CDialogConfiguration_M05X::OnRadioBs()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}

void CDialogConfiguration_M05X::OnRadioIO()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}

void CDialogConfiguration_M05X::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}


void CDialogConfiguration_M05X::OnCheckClickWDTPD()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent(IDC_CHECK_WDT_POWER_DOWN);
}

void CDialogConfiguration_M05X::OnCheckClickWDT()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent(IDC_CHECK_WDT_ENABLE);
}

void CDialogConfiguration_M05X::OnOK()
{
    GUIToConfig(0);
    CDialog::OnOK();
}

CDialogConfiguration_M05XBN::CDialogConfiguration_M05XBN(bool bIsM05xBN, UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M05X(nIDTemplate, pParent)
    , m_bIsM05xBN(bIsM05xBN)
{
}

BOOL CDialogConfiguration_M05XBN::OnInitDialog()
{
    if (m_bIsM05xBN) {
        SetDlgItemText(IDC_RADIO_BOV_45, _T("4.4V"));
        SetDlgItemText(IDC_RADIO_BOV_38, _T("3.7V"));
    }

    return CDialogConfiguration_M05X::OnInitDialog();
}
