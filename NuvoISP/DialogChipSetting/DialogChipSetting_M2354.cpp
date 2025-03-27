// DialogChipSetting_M2354.cpp : implementation file
//
#include "stdafx.h"

#include <deque>
#include <string>
#include <utility>
#include "Lang.h"
#include "DialogChipSetting_M2354.h"


// CDialogChipSetting_M2354 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M2354, CDialog)

CDialogChipSetting_M2354::CDialogChipSetting_M2354(unsigned int uProgramMemorySize, unsigned int uFlashPageSize, BOOL bSecureDebug, unsigned int uChipSeries, CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_M2354::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uFlashPageSize(uFlashPageSize)
    , m_bSecureDebug(bSecureDebug)
    , m_uChipSeries(uChipSeries)
    , m_nSel(0)
    , m_uShowFlag(0x3)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_M2354)
    // NOTE: the ClassWizard will add member initialization here
    //}}AFX_DATA_INIT
}

CDialogChipSetting_M2354::~CDialogChipSetting_M2354()
{

}

void CDialogChipSetting_M2354::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}


BEGIN_MESSAGE_MAP(CDialogChipSetting_M2354, CDialog)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_M2354::OnTcnSelchangeTabChipsetting)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_M2354::OnOk)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()


BOOL CDialogChipSetting_M2354::OnInitDialog()
{
    CDialog::OnInitDialog();

    m_pChipSetting_CFG = new CDialogChipSetting_CFG_M2351();
    m_pChipSetting_NSCBA = new CDialogChipSetting_NSCBA();

    m_pChipSetting_CFG->m_uChipType             = NUC_CHIP_TYPE_M2354;

    m_pChipSetting_CFG->m_uConfigValue[0]       = m_uConfigValue[0];
    m_pChipSetting_CFG->m_uConfigValue[1]       = m_uConfigValue[1];
    m_pChipSetting_CFG->m_uConfigValue[2]       = m_uConfigValue[2];
    m_pChipSetting_CFG->m_uConfigValue[3]       = m_uConfigValue[3];

    m_pChipSetting_NSCBA->m_bSecureDebug        = m_bSecureDebug;
    m_pChipSetting_NSCBA->m_bCanWrite           = m_bNSCBA_CanWrite;
    m_pChipSetting_NSCBA->m_uFlashBaseAddr      = NUMICRO_FLASH_APROM_ADDR;
    m_pChipSetting_NSCBA->m_uProgramMemorySize  = m_uProgramMemorySize;
    m_pChipSetting_NSCBA->m_uFlashPageSize      = m_uFlashPageSize;

    m_pChipSetting_NSCBA->m_uNSAddr             = m_uNSCBA_NSAddr;
    m_pChipSetting_NSCBA->m_bNSAddrWrite        = (m_bNSCBA_Write) ? TRUE : FALSE;
    m_pChipSetting_NSCBA->m_bMirrorEnable       = (m_bNSCBA_MirrorEnable) ? TRUE : FALSE;

    int i, nItem = 0;

    if (m_uShowFlag & 0x01)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("Configuration"));

        m_pChipSetting_CFG->Create(CDialogChipSetting_CFG_M2351::IDD, &m_TabChipSetting);
    }

    if (m_uShowFlag & 0x02)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("NSCBA"));
        m_pChipSetting_NSCBA->Create(CDialogChipSetting_NSCBA::IDD, &m_TabChipSetting);
    }

    CRect rcTmpTab, rcTmp;

    m_TabChipSetting.GetWindowRect(&rcTmpTab);

    if (m_bSecureDebug)
        m_pChipSetting_CFG->GetWindowRect(&rcTmp);
    else
        m_pChipSetting_NSCBA->GetWindowRect(&rcTmp);

    LONG lDiff = rcTmpTab.bottom - rcTmp.bottom;

    if (lDiff > 125)
    {
        lDiff -= 125;

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
        m_pChipSetting_CFG,
        m_pChipSetting_NSCBA,
    };

    m_TabChipSetting.SetCurSel(m_nSel);

    nItem = 0;

    for (i = 0; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); i++)
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

void CDialogChipSetting_M2354::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSetting[] =
    {
        m_pChipSetting_CFG,
        m_pChipSetting_NSCBA,
    };

    m_nSel = m_TabChipSetting.GetCurSel();

    int i, nItem = 0;

    for (i = 0; i < sizeof(pChipSetting) / sizeof(pChipSetting[0]); i++)
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

void CDialogChipSetting_M2354::OnOk()
{
    this->SetFocus();

    m_uConfigValue[0] = m_pChipSetting_CFG->m_uConfigValue[0];
    m_uConfigValue[1] = m_pChipSetting_CFG->m_uConfigValue[1];
    m_uConfigValue[2] = m_pChipSetting_CFG->m_uConfigValue[2];
    m_uConfigValue[3] = m_pChipSetting_CFG->m_uConfigValue[3];

    m_bNSCBA_Write          = (m_pChipSetting_NSCBA->m_bNSAddrWrite && m_bSecureDebug) ? true : false;
    m_bNSCBA_MirrorEnable   = (m_pChipSetting_NSCBA->m_bMirrorEnable) ? true : false;
    m_uNSCBA_NSAddr         =  m_pChipSetting_NSCBA->m_uNSAddr;

    CDialog::OnOK();
}