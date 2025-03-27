// DialogChipSetting_M480LD.cpp : implementation file
//
#include "stdafx.h"

#include <deque>
#include <string>
#include <utility>
#include "Lang.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogChipSetting_M480LD.h"


// CDialogChipSetting_M480LD dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M480LD, CDialog)

CDialogChipSetting_M480LD::CDialogChipSetting_M480LD(unsigned int uAPROMSize, CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_M480LD::IDD, pParent)
    , m_nSel(0)
    , m_uAPROM_Size(uAPROMSize)
    , m_uShowFlag(0x1)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_M480LD)
    // NOTE: the ClassWizard will add member initialization here
    //}}AFX_DATA_INIT
}

CDialogChipSetting_M480LD::~CDialogChipSetting_M480LD()
{
    if (m_uShowFlag & 0x01)
    {
        delete m_ChipSetting_CFG;
    }
}

void CDialogChipSetting_M480LD::DoDataExchange(CDataExchange* pDX)
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

    int i, nItem = 0;

    if (m_uShowFlag & 0x01)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("Configuration"));

        m_ChipSetting_CFG.m_uProgramMemorySize = m_uAPROM_Size;
        m_ChipSetting_CFG.m_uConfigValue[0] = m_uConfigValue[0];
        m_ChipSetting_CFG.m_uConfigValue[1] = m_uConfigValue[1];
        m_ChipSetting_CFG.m_uConfigValue[2] = m_uConfigValue[2];

        m_ChipSetting_CFG.Create(CDialogChipSetting_CFG_M480LD::IDD, &m_TabChipSetting);
    }

    CRect rcClient;
    m_TabChipSetting.GetClientRect(rcClient);
    m_TabChipSetting.AdjustRect(FALSE, rcClient);

    CDialog *pChipSetting[] =
    {
        &m_ChipSetting_CFG,
        //&m_ChipSetting_XOM
    };

    m_TabChipSetting.SetCurSel(m_nSel);

    nItem = 0;

    for (i = 0; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); ++i)
    {
        if (m_uShowFlag & (1 << i))
        {
            pChipSetting[i]->MoveWindow(rcClient);

            if (m_nSel == nItem)
                pChipSetting[i]->ShowWindow(TRUE);
            else
                pChipSetting[i]->ShowWindow(FALSE);

            nItem++;
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
    CDialog *pChipSetting[] =
    {
        &m_ChipSetting_CFG,
    };

    m_nSel = m_TabChipSetting.GetCurSel();

    int i, nItem = 0;

    for (i = 0; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); ++i)
    {
        if (m_uShowFlag & (1 << i))
        {
            if (m_nSel == nItem)
                pChipSetting[i]->ShowWindow(TRUE);
            else
                pChipSetting[i]->ShowWindow(FALSE);
        }

        nItem++;
    }

    *pResult = 0;
}

void CDialogChipSetting_M480LD::OnOk()
{
    // TODO: Add extra validation here

    this->SetFocus();

    if (m_uShowFlag & 0x01)
    {
        m_uConfigValue[0] = m_ChipSetting_CFG.m_uConfigValue[0];
        m_uConfigValue[1] = m_ChipSetting_CFG.m_uConfigValue[1];
        m_uConfigValue[2] = m_ChipSetting_CFG.m_uConfigValue[2];
    }

    CDialog::OnOK();
}

CString CDialogChipSetting_M480LD::GetConfigWarning()
{
    CString str;

    //  if (!(bSecureLock || bAllLock))
    str += _T("   ") + _I(IDS_DISABLE_SECURITY_LOCK);

    return str;
}

