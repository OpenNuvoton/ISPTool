// DialogChipSetting_M2351.cpp : implementation file
//
#include "stdafx.h"

#include <deque>
#include <string>
#include <utility>
#include "Lang.h"
#include "DialogChipSetting_M2351.h"


// CDialogChipSetting_M2351 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M2351, CDialog)

CDialogChipSetting_M2351::CDialogChipSetting_M2351(unsigned int uProgramMemorySize, unsigned int uFlashPageSize, BOOL bSecureDebug, BOOL bNSecureArea, unsigned int uChipSeries,
                                                   CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_M2351::IDD, pParent)
    , m_nSel(0)
    , m_uShowFlag(0x03)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uFlashPageSize(uFlashPageSize)
    , m_bSecureDebug(bSecureDebug)
    , m_bNSecureArea(bNSecureArea)
    , m_uChipSeries(uChipSeries)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_M2351)
    // NOTE: the ClassWizard will add member initialization here
    //}}AFX_DATA_INIT
}

CDialogChipSetting_M2351::~CDialogChipSetting_M2351()
{
}

void CDialogChipSetting_M2351::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}


BEGIN_MESSAGE_MAP(CDialogChipSetting_M2351, CDialog)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_M2351::OnTcnSelchangeTabChipsetting)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_M2351::OnOk)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()


BOOL CDialogChipSetting_M2351::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here

    m_ChipSetting_CFG.m_uChipType           = NUC_CHIP_TYPE_M2351;

    m_ChipSetting_CFG.m_uConfigValue[0]     = m_uConfigValue[0];
    m_ChipSetting_CFG.m_uConfigValue[1]     = m_uConfigValue[1];
    m_ChipSetting_CFG.m_uConfigValue[2]     = m_uConfigValue[2];
    m_ChipSetting_CFG.m_uConfigValue[3]     = m_uConfigValue[3];

    m_ChipSetting_SECURE.m_bSecureDebug     = m_bSecureDebug;
    m_ChipSetting_SECURE.m_bNSecureArea     = m_bNSecureArea;
    m_ChipSetting_SECURE.m_uProgramMemorySize   = m_uProgramMemorySize;
    m_ChipSetting_SECURE.m_uNSecure_Addr    = m_uNSecure_Addr;

    m_ChipSetting_SECURE.m_bNSAddr_Write    = (m_bNSAddr_Write) ? TRUE : FALSE;
    m_ChipSetting_SECURE.m_bSecure_Lock     = (m_bSecure_Lock) ? TRUE : FALSE;
    m_ChipSetting_SECURE.m_bAll_Lock        = (m_bAll_Lock) ? TRUE : FALSE;

    int nItem = 0;

    if (m_uShowFlag & 0x01)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("Configuration"));
        m_ChipSetting_CFG.Create(CDialogChipSetting_CFG_M2351::IDD, &m_TabChipSetting);
    }

    if (m_uShowFlag & 0x02)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("Secure Setting"));
        m_ChipSetting_SECURE.Create(CDialogChipSetting_SECURE_M2351::IDD, &m_TabChipSetting);
    }

    CRect rcTmpTab, rcTmp;

    m_TabChipSetting.GetWindowRect(&rcTmpTab);

    if (m_bSecureDebug)
        m_ChipSetting_CFG.GetWindowRect(&rcTmp);
    else
        m_ChipSetting_SECURE.GetWindowRect(&rcTmp);

    LONG lDiff = rcTmpTab.bottom - rcTmp.bottom;

    if (lDiff > 25)
    {
        lDiff -= 25;

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

    CDialog *pChipSetting[] =
    {
        &m_ChipSetting_CFG,
        &m_ChipSetting_SECURE
    };

    m_TabChipSetting.SetCurSel(m_nSel);

    nItem = 0;

    for (int i = 0; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); i++)
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

void CDialogChipSetting_M2351::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSetting[] =
    {
        &m_ChipSetting_CFG,
        &m_ChipSetting_SECURE
    };

    m_nSel = m_TabChipSetting.GetCurSel();

    int nItem = 0;

    for (int i = 0; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); i++)
    {
        if (m_uShowFlag & (1 << i))
        {
            if (m_nSel == nItem)
                pChipSetting[i]->ShowWindow(TRUE);
            else
                pChipSetting[i]->ShowWindow(FALSE);

            nItem++;
        }
    }

    *pResult = 0;
}

void CDialogChipSetting_M2351::OnOk()
{
    // TODO: Add extra validation here

    this->SetFocus();

    m_uConfigValue[0] = m_ChipSetting_CFG.m_uConfigValue[0];
    m_uConfigValue[1] = m_ChipSetting_CFG.m_uConfigValue[1];
    m_uConfigValue[2] = m_ChipSetting_CFG.m_uConfigValue[2];
    m_uConfigValue[3] = m_ChipSetting_CFG.m_uConfigValue[3];

    m_bNSAddr_Write = (m_ChipSetting_SECURE.m_bNSAddr_Write) ? true : false;
    m_bSecure_Lock  = (m_ChipSetting_SECURE.m_bSecure_Lock) ? true : false;
    m_bAll_Lock     = (m_ChipSetting_SECURE.m_bAll_Lock) ? true : false;
    m_uNSecure_Addr =  m_ChipSetting_SECURE.m_uNSecure_Addr;

    CDialog::OnOK();
}

