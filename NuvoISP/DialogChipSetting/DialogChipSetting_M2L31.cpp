// DialogChipSetting_M460.cpp : implementation file
//
#include "stdafx.h"

#include <deque>
#include <string>
#include <utility>
#include "DialogChipSetting_M2L31.h"

// CDialogChipSetting_M460 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M2L31, CDialog)

CDialogChipSetting_M2L31::CDialogChipSetting_M2L31(unsigned int uProgramMemorySize, unsigned int uFlashPageSize, unsigned int uPID, unsigned int uDID, unsigned int uChipSeries, CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_M2L31::IDD, pParent)
    , m_nSel(0)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uFlashPageSize(uFlashPageSize)
    , m_uPID(uPID)
    , m_uDID(uDID)
    , m_uChipSeries(uChipSeries)
    , m_uShowFlag(0x03)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_M460)
    // NOTE: the ClassWizard will add member initialization here
    m_uConfigValue[0]	= 0xFFFFFFFF;
    m_uConfigValue[1]	= 0xFFFFFFFF;
    m_uConfigValue[2]	= 0xFFFFFFFF;
    m_uConfigValue[3]	= 0xFFFFFFFF;
    m_uConfigValue[4]   = 0xFFFFFFFF;
    m_uConfigValue[5]   = 0xFFFFFFFF;
    m_uConfigValue[6]   = 0xFFFFFFFF;

    m_uConfigValue[8] = 0xFFFFFFFF;
    m_uConfigValue[9] = 0xFFFFFFFF;
    m_uConfigValue[10] = 0xFFFFFFFF;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_M2L31::~CDialogChipSetting_M2L31()
{
    if (m_uShowFlag & 0x01) {
        delete m_pChipSetting_CFG;
    }

    if (m_uShowFlag & 0x02) {
        delete m_pChipSetting_CFG_2;
    }
}

void CDialogChipSetting_M2L31::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_M2L31, CDialog)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_M2L31::OnTcnSelchangeTabChipsetting)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_M2L31::OnOk)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()

BOOL CDialogChipSetting_M2L31::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    int nItem = 0;

    if (m_uShowFlag & 0x01) {
        m_TabChipSetting.InsertItem(nItem++, _T("Configuration"));
        m_pChipSetting_CFG = new CDialogChipSetting_CFG_M2L31();
        m_pChipSetting_CFG->m_uProgramMemorySize = m_uProgramMemorySize;
        m_pChipSetting_CFG->m_uFlashPageSize = m_uFlashPageSize;
        m_pChipSetting_CFG->m_uPID					= m_uPID;
        m_pChipSetting_CFG->m_uDID					= m_uDID;
        m_pChipSetting_CFG->m_uConfigValue[0]		= m_uConfigValue[0];
        m_pChipSetting_CFG->m_uConfigValue[1]		= m_uConfigValue[1];
        m_pChipSetting_CFG->m_uConfigValue[2]		= m_uConfigValue[2];
        m_pChipSetting_CFG->m_uConfigValue[3]		= m_uConfigValue[3];
        m_pChipSetting_CFG->m_uConfigValue[4]       = m_uConfigValue[4];
        m_pChipSetting_CFG->m_uConfigValue[5]       = m_uConfigValue[5];
        m_pChipSetting_CFG->m_uConfigValue[6]       = m_uConfigValue[6];
        m_pChipSetting_CFG->Create(CDialogChipSetting_CFG_M2L31::IDD, &m_TabChipSetting);
    }

    if (m_uShowFlag & 0x02) {
        m_TabChipSetting.InsertItem(nItem++, _T("Configuration"));
        m_pChipSetting_CFG_2 = new CDialogChipSetting_CFG_M2L31_2();
        m_pChipSetting_CFG_2->m_uProgramMemorySize = m_uProgramMemorySize;
        m_pChipSetting_CFG_2->m_uFlashPageSize = m_uFlashPageSize;
        m_pChipSetting_CFG_2->m_uPID = m_uPID;
        m_pChipSetting_CFG_2->m_uDID = m_uDID;
        m_pChipSetting_CFG_2->m_uConfigValue[0] = m_uConfigValue[8];
        m_pChipSetting_CFG_2->m_uConfigValue[1] = m_uConfigValue[9];
        m_pChipSetting_CFG_2->m_uConfigValue[2] = m_uConfigValue[10];
        m_pChipSetting_CFG_2->Create(CDialogChipSetting_CFG_M2L31_2::IDD, &m_TabChipSetting);
    }

    CRect rcTmpTab, rcTmpCFG;
    m_TabChipSetting.GetWindowRect(&rcTmpTab);
    m_pChipSetting_CFG->GetWindowRect(&rcTmpCFG);
    m_pChipSetting_CFG_2->GetWindowRect(&rcTmpCFG);
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
        m_pChipSetting_CFG_2,
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

void CDialogChipSetting_M2L31::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSetting[] = {
        m_pChipSetting_CFG,
        m_pChipSetting_CFG_2,
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

void CDialogChipSetting_M2L31::OnOk()
{
    // TODO: Add extra validation here
    this->SetFocus();

    if (m_uShowFlag & 0x01) {
        m_uConfigValue[0] = m_pChipSetting_CFG->m_uConfigValue[0];
        m_uConfigValue[1] = m_pChipSetting_CFG->m_uConfigValue[1];
        m_uConfigValue[2] = m_pChipSetting_CFG->m_uConfigValue[2];
        m_uConfigValue[3] = m_pChipSetting_CFG->m_uConfigValue[3];
        m_uConfigValue[4] = m_pChipSetting_CFG->m_uConfigValue[4];
        m_uConfigValue[5] = m_pChipSetting_CFG->m_uConfigValue[5];
        m_uConfigValue[6] = m_pChipSetting_CFG->m_uConfigValue[6];
    }

    if (m_uShowFlag & 0x02) {
        m_uConfigValue[8] = m_pChipSetting_CFG_2->m_uConfigValue[0];
        m_uConfigValue[9] = m_pChipSetting_CFG_2->m_uConfigValue[1];
        m_uConfigValue[10] = m_pChipSetting_CFG_2->m_uConfigValue[2];
    }

    CDialog::OnOK();
}
