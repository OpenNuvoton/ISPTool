// DialogChipSetting_M2A23.cpp : implementation file
//
#include "stdafx.h"

#include <deque>
#include <string>
#include <utility>
#include "Lang.h"
#include "DialogChipSetting_M2A23.h"

// CDialogChipSetting_M2A23 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M2A23, CDialog)

CDialogChipSetting_M2A23::CDialogChipSetting_M2A23(unsigned int uProgramMemorySize, unsigned int uFlashPageSize, unsigned int uPID, unsigned int uDID, unsigned int uChipSeries,
                                                   CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_M2A23::IDD, pParent)
    , m_nSel(0)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uFlashPageSize(uFlashPageSize)
    , m_uPID(uPID)
    , m_uDID(uDID)
    , m_uChipSeries(uChipSeries)
    , m_uShowFlag(0x3)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_M2A23)
    // NOTE: the ClassWizard will add member initialization here
    m_uConfigValue[0]   = 0xFFFFFFFF;
    m_uConfigValue[1]   = 0xFFFFFFFF;
    m_uConfigValue[2]   = 0xFFFF5A5A;
    m_uConfigValue[3]   = 0xFFFFFFFF;
    m_uConfigValue[4]   = 0xFFFFFFFF;
    m_uConfigValue[5]   = 0xFFFFFFFF;
    m_uConfigValue[6]   = 0xFFFFFFFF;
    m_uConfigValue[7]   = 0xFFFFFFFF;
    m_uConfigValue[8]   = 0xFFFFFFFF;
    m_uConfigValue[9]   = 0xFFFFFFFF;
    m_uConfigValue[10]  = 0xFFFFFFFF;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_M2A23::~CDialogChipSetting_M2A23()
{
    if (m_uShowFlag & 0x01)
    {
        delete m_pChipSetting_CFG;
    }

    if (m_uShowFlag & 0x02)
    {
        delete m_pChipSetting_APWPROT;
    }
}

void CDialogChipSetting_M2A23::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}


BEGIN_MESSAGE_MAP(CDialogChipSetting_M2A23, CDialog)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_M2A23::OnTcnSelchangeTabChipsetting)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_M2A23::OnOk)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()


BOOL CDialogChipSetting_M2A23::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here

    int nItem = 0;

    if (m_uShowFlag & 0x01)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("CONFIG 0-2"));

        m_pChipSetting_CFG = new CDialogChipSetting_CFG_M2A23();

        m_pChipSetting_CFG->m_uProgramMemorySize    = m_uProgramMemorySize;
        m_pChipSetting_CFG->m_uFlashPageSize        = m_uFlashPageSize;
        m_pChipSetting_CFG->m_uConfigValue[0]       = m_uConfigValue[0];
        m_pChipSetting_CFG->m_uConfigValue[1]       = m_uConfigValue[1];
        m_pChipSetting_CFG->m_uConfigValue[2]       = m_uConfigValue[2];

        m_pChipSetting_CFG->Create(CDialogChipSetting_CFG_M2A23::IDD, &m_TabChipSetting);
    }

    if (m_uShowFlag & 0x02)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("CONFIG 8-10"));

        m_pChipSetting_APWPROT = new CDialogChipSetting_APWPROT2();

        m_pChipSetting_APWPROT->m_uAPROMAddr        = NUMICRO_FLASH_APROM_ADDR;
        m_pChipSetting_APWPROT->m_uAPROMSize        = m_uProgramMemorySize;
        m_pChipSetting_APWPROT->m_uRegionSize       = 0x2000;
        m_pChipSetting_APWPROT->m_uConfigValue[0]   = m_uConfigValue[8];
        m_pChipSetting_APWPROT->m_uConfigValue[1]   = m_uConfigValue[9];
        m_pChipSetting_APWPROT->m_uConfigValue[2]   = m_uConfigValue[10];

        m_pChipSetting_APWPROT->Create(CDialogChipSetting_APWPROT2::IDD, &m_TabChipSetting);
    }

    CRect rcClient;
    m_TabChipSetting.GetClientRect(rcClient);
    m_TabChipSetting.AdjustRect(FALSE, rcClient);

    CDialog *pChipSetting[] =
    {
        m_pChipSetting_CFG,
        m_pChipSetting_APWPROT,
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

void CDialogChipSetting_M2A23::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSetting[] =
    {
        m_pChipSetting_CFG,
        m_pChipSetting_APWPROT,
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

void CDialogChipSetting_M2A23::OnOk()
{
    // TODO: Add extra validation here

    this->SetFocus();

    if (m_uShowFlag & 0x01)
    {
        m_uConfigValue[0]   = m_pChipSetting_CFG->m_uConfigValue[0];
        m_uConfigValue[1]   = m_pChipSetting_CFG->m_uConfigValue[1];
        m_uConfigValue[2]   = m_pChipSetting_CFG->m_uConfigValue[2];
    }

    if (m_uShowFlag & 0x02)
    {
        m_uConfigValue[8]   = m_pChipSetting_APWPROT->m_uConfigValue[0] | m_pChipSetting_APWPROT->m_uConfigValue_c[0];
        m_uConfigValue[9]   = m_pChipSetting_APWPROT->m_uConfigValue[1] | m_pChipSetting_APWPROT->m_uConfigValue_c[1];
        m_uConfigValue[10]  = m_pChipSetting_APWPROT->m_uConfigValue[2];
    }

    CDialog::OnOK();
}
