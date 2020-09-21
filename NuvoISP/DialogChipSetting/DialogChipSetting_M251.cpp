// CDialogChipSetting_M251.cpp : implementation file
//
#include "stdafx.h"
#include "DialogChipSetting_M251.h"

// CDialogChipSetting_M251 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M251, CDialog)

CDialogChipSetting_M251::CDialogChipSetting_M251(int uParam, unsigned int uProgramMemorySize, UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialogResize(nIDTemplate, pParent)
    , m_uParam(uParam)
    , m_uProgramMemorySize(uProgramMemorySize)
{
}

CDialogChipSetting_M251::~CDialogChipSetting_M251()
{
}

void CDialogChipSetting_M251::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_M251, CDialog)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_M251::OnTcnSelchangeTabChipsetting)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_M251::OnOk)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()

// CDialogChipSetting_M251 message handlers
BOOL CDialogChipSetting_M251::OnInitDialog()
{
    CDialog::OnInitDialog();
    m_TabChipSetting.InsertItem(0, _T("Configuration"));

    if (m_uParam == 0) {
        m_ConfigOptionM251.m_ConfigValue.m_value[0] = m_uConfigOption_ConfigValue[0];
        m_ConfigOptionM251.m_uProgramMemorySize = m_uProgramMemorySize;
        m_ConfigOptionM251.Create(CDialogConfiguration_M251::IDD, &m_TabChipSetting);
        CRect rcClient;
        m_TabChipSetting.GetClientRect(rcClient);
        m_TabChipSetting.AdjustRect(FALSE, rcClient);
        m_TabChipSetting.SetCurSel(0);
        m_ConfigOptionM251.MoveWindow(rcClient);
        m_ConfigOptionM251.ShowWindow(TRUE);
    } else {
        m_ConfigOptionM258.m_ConfigValue.m_value[0] = m_uConfigOption_ConfigValue[0];
        m_ConfigOptionM258.m_uProgramMemorySize = m_uProgramMemorySize;
        m_ConfigOptionM258.Create(CDialogConfiguration_M258::IDD, &m_TabChipSetting);
        CRect rcClient;
        m_TabChipSetting.GetClientRect(rcClient);
        m_TabChipSetting.AdjustRect(FALSE, rcClient);
        m_TabChipSetting.SetCurSel(0);
        m_ConfigOptionM258.MoveWindow(rcClient);
        m_ConfigOptionM258.ShowWindow(TRUE);
    }

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;
}
void CDialogChipSetting_M251::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSettingM251[] = {
        &m_ConfigOptionM251,
    };
    CDialog *pChipSettingM258[] = {
        &m_ConfigOptionM258,
    };
    m_nSel = m_TabChipSetting.GetCurSel();

    if (m_uParam == 0) {
        for (int i = 0; i < sizeof(pChipSettingM251) / sizeof(pChipSettingM251[0]); ++i) {
            if (i != m_nSel) {
                pChipSettingM251[i]->ShowWindow(FALSE);
            }
        }

        pChipSettingM251[m_nSel]->ShowWindow(TRUE);
    } else {
        for (int i = 0; i < sizeof(pChipSettingM258) / sizeof(pChipSettingM258[0]); ++i) {
            if (i != m_nSel) {
                pChipSettingM258[i]->ShowWindow(FALSE);
            }
        }

        pChipSettingM258[m_nSel]->ShowWindow(TRUE);
    }

    *pResult = 0;
}

void CDialogChipSetting_M251::OnOk()
{
    if (m_uParam == 0) {
        m_ConfigOptionM251.UpdateData(TRUE);
        m_ConfigOptionM251.GUIToConfig(0);
        m_uConfigOption_ConfigValue[0] = m_ConfigOptionM251.m_ConfigValue.m_value[0];
    } else {
        m_ConfigOptionM258.UpdateData(TRUE);
        m_ConfigOptionM258.GUIToConfig(0);
        m_uConfigOption_ConfigValue[0] = m_ConfigOptionM258.m_ConfigValue.m_value[0];
    }

    CDialog::OnOK();
}
