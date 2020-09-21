// DialogChipSetting_M480LD.cpp : implementation file
//
#include "stdafx.h"

#include <deque>
#include <string>
#include <utility>
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogChipSetting_M480LD.h"

// CDialogChipSetting_M480LD dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M480LD, CDialog)

CDialogChipSetting_M480LD::CDialogChipSetting_M480LD(unsigned int uAPROMSize, CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_M480LD::IDD, pParent)
    , m_nSel(0)
    , m_nSelOffset(0)
    , m_uAPROM_Size(uAPROMSize)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_M480LD)
    // NOTE: the ClassWizard will add member initialization here
    //}}AFX_DATA_INIT
}

CDialogChipSetting_M480LD::~CDialogChipSetting_M480LD()
{
}

void CDialogChipSetting_M480LD::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_M480LD, CDialog)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_M480LD::OnTcnSelchangeTabChipsetting)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_M480LD::OnOk)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()

BOOL CDialogChipSetting_M480LD::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    m_ChipSetting_CFG.m_uProgramMemorySize = m_uAPROM_Size;
    m_ChipSetting_CFG.m_uConfigValue[0]	= m_uConfigValue[0];
    m_ChipSetting_CFG.m_uConfigValue[1]	= m_uConfigValue[1];
    m_ChipSetting_CFG.m_uConfigValue[2]	= m_uConfigValue[2];
    unsigned int i = 0;

    if (m_nSelOffset == 0) {
        m_TabChipSetting.InsertItem(i++, _T("Configuration"));
        m_ChipSetting_CFG.Create(CDialogChipSetting_CFG_M480LD::IDD, &m_TabChipSetting);
    }

    CRect rcClient;
    m_TabChipSetting.GetClientRect(rcClient);
    m_TabChipSetting.AdjustRect(FALSE, rcClient);
    CDialog *pChipSetting[] = {
        &m_ChipSetting_CFG,
    };
    m_TabChipSetting.SetCurSel(m_nSel);

    for (i = m_nSelOffset; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); ++i) {
        pChipSetting[i]->MoveWindow(rcClient);

        if (m_nSel == (i - m_nSelOffset)) {
            pChipSetting[i]->ShowWindow(TRUE);
        } else {
            pChipSetting[i]->ShowWindow(FALSE);
        }
    }

    CenterWindow();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;
}

void CDialogChipSetting_M480LD::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSetting[] = {
        &m_ChipSetting_CFG,
    };
    m_nSel = m_TabChipSetting.GetCurSel();

    for (int i = m_nSelOffset; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); ++i) {
        if (m_nSel == (i - m_nSelOffset)) {
            pChipSetting[i]->ShowWindow(TRUE);
        } else {
            pChipSetting[i]->ShowWindow(FALSE);
        }
    }

    *pResult = 0;
}

void CDialogChipSetting_M480LD::OnOk()
{
    // TODO: Add extra validation here
    this->SetFocus();
    m_uConfigValue[0] = m_ChipSetting_CFG.m_uConfigValue[0];
    m_uConfigValue[1] = m_ChipSetting_CFG.m_uConfigValue[1];
    m_uConfigValue[2] = m_ChipSetting_CFG.m_uConfigValue[2];
    CDialog::OnOK();
}

