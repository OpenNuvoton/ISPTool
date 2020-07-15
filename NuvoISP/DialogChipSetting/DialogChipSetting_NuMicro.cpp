// DialogChipSetting_NuMicro.cpp : implementation file
//
#include "stdafx.h"

#include <deque>
#include <string>
#include <utility>
#include "DialogChipSetting_NuMicro.h"

// CDialogChipSetting_NuMicro dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_NuMicro, CDialog)

CDialogChipSetting_NuMicro::CDialogChipSetting_NuMicro(unsigned int uProgramMemorySize, unsigned int uFlashPageSize, unsigned int uChipSeries, CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_NuMicro::IDD, pParent)
    , m_nSel(0)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uFlashPageSize(uFlashPageSize)
    , m_uChipSeries(uChipSeries)
    , m_uShowFlag(0x01)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_NuMicro)
    // NOTE: the ClassWizard will add member initialization here
    m_uConfigValue[0]	= 0xFFFFFFFF;
    m_uConfigValue[1]	= 0xFFFFFFFF;
    m_uConfigValue[2]	= 0xFFFFFFFF;
    m_uConfigValue[3]	= 0xFFFFFFFF;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_NuMicro::~CDialogChipSetting_NuMicro()
{
    if (m_uShowFlag & 0x01) {
        delete m_pChipSetting_CFG;
    }
}

void CDialogChipSetting_NuMicro::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_NuMicro, CDialog)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_NuMicro::OnTcnSelchangeTabChipsetting)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_NuMicro::OnOk)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()

BOOL CDialogChipSetting_NuMicro::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    int nItem = 0;

    if (m_uShowFlag & 0x01) {
        m_TabChipSetting.InsertItem(nItem++, _T("Configuration"));

        if (m_uChipSeries == NUC_CHIP_TYPE_M031) {
            m_pChipSetting_CFG = new CDialogChipSetting_CFG_M031();
        }

        if (m_uChipSeries == NUC_CHIP_TYPE_M0A21) {
            m_pChipSetting_CFG = new CDialogChipSetting_CFG_M0A21();
        }

        if (m_uChipSeries == NUC_CHIP_TYPE_M030G) {
            m_pChipSetting_CFG = new CDialogChipSetting_CFG_M030G();
        }

        if (m_uChipSeries == NUC_CHIP_TYPE_M451) {
            m_pChipSetting_CFG = new CDialogChipSetting_CFG_M451();
        }

        if (m_uChipSeries == NUC_CHIP_TYPE_M471) {
            m_pChipSetting_CFG = new CDialogChipSetting_CFG_M471();
        }

        m_pChipSetting_CFG->m_uProgramMemorySize	= m_uProgramMemorySize;
        m_pChipSetting_CFG->m_uFlashPageSize		= m_uFlashPageSize;
        m_pChipSetting_CFG->m_uConfigValue[0]		= m_uConfigValue[0];
        m_pChipSetting_CFG->m_uConfigValue[1]		= m_uConfigValue[1];
        m_pChipSetting_CFG->m_uConfigValue[2]		= m_uConfigValue[2];
        m_pChipSetting_CFG->m_uConfigValue[3]		= m_uConfigValue[3];
        m_pChipSetting_CFG->Create(CDialogChipSetting_CFG_M0A21::IDD, &m_TabChipSetting);
    }

    CRect rcTmpTab, rcTmpCFG;
    m_TabChipSetting.GetWindowRect(&rcTmpTab);
    m_pChipSetting_CFG->GetWindowRect(&rcTmpCFG);
    LONG lDiff = rcTmpTab.bottom - rcTmpCFG.bottom;

    if (lDiff > 25) {
        lDiff -= 25;
        CRect rcTmp;
        m_TabChipSetting.GetWindowRect(&rcTmp);
        ScreenToClient(rcTmp);
        m_TabChipSetting.SetWindowPos(NULL, rcTmp.left, rcTmp.top, rcTmp.right - rcTmp.left, rcTmp.bottom - rcTmp.top - lDiff, SWP_NOZORDER);
        GetDlgItem(IDOK)->GetWindowRect(&rcTmp);
        this->ScreenToClient(&rcTmp);
        GetDlgItem(IDOK)->SetWindowPos(NULL, rcTmp.left, rcTmp.top - lDiff, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
        GetDlgItem(IDCANCEL)->GetWindowRect(&rcTmp);
        this->ScreenToClient(&rcTmp);
        GetDlgItem(IDCANCEL)->SetWindowPos(NULL, rcTmp.left, rcTmp.top - lDiff, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
        this->GetWindowRect(&rcTmp);
        SetWindowPos(this, 0, 0, rcTmp.right - rcTmp.left, rcTmp.bottom - rcTmp.top - lDiff, SWP_NOZORDER | SWP_NOMOVE);
    }

    CRect rcClient;
    m_TabChipSetting.GetClientRect(rcClient);
    m_TabChipSetting.AdjustRect(FALSE, rcClient);
    CDialog *pChipSetting[] = {
        m_pChipSetting_CFG,
    };
    m_TabChipSetting.SetCurSel(m_nSel);
    nItem = 0;

    for (int i = 0; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); i++) {
        if (m_uShowFlag & (1 << i)) {
            pChipSetting[i]->MoveWindow(rcClient);

            if (m_nSel == nItem) {
                pChipSetting[i]->ShowWindow(TRUE);
            } else {
                pChipSetting[i]->ShowWindow(FALSE);
            }

            nItem++;
        }
    }

    CenterWindow();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;
}

void CDialogChipSetting_NuMicro::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSetting[] = {
        m_pChipSetting_CFG,
    };
    m_nSel = m_TabChipSetting.GetCurSel();
    int nItem = 0;

    for (int i = 0; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); i++) {
        if (m_uShowFlag & (1 << i)) {
            if (m_nSel == nItem) {
                pChipSetting[i]->ShowWindow(TRUE);
            } else {
                pChipSetting[i]->ShowWindow(FALSE);
            }

            nItem++;
        }
    }

    *pResult = 0;
}

void CDialogChipSetting_NuMicro::OnOk()
{
    // TODO: Add extra validation here
    this->SetFocus();

    if (m_uShowFlag & 0x01) {
        m_uConfigValue[0] = m_pChipSetting_CFG->m_uConfigValue[0];
        m_uConfigValue[1] = m_pChipSetting_CFG->m_uConfigValue[1];
        m_uConfigValue[2] = m_pChipSetting_CFG->m_uConfigValue[2];
        m_uConfigValue[3] = m_pChipSetting_CFG->m_uConfigValue[3];
    }

    CDialog::OnOK();
}
