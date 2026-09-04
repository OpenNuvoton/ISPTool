// DialogChipSetting_M55M1.cpp : implementation file
//
#include "stdafx.h"
#include "NuDataBase.h"
#include <deque>
#include <string>
#include <utility>
#include "DialogChipSetting_M55M1.h"


// CDialogChipSetting_M55M1 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M55M1, CDialog)

CDialogChipSetting_M55M1::CDialogChipSetting_M55M1(BOOL bSecureDebug, unsigned int uPID, unsigned int uDID, unsigned int uChipSeries, CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_M55M1::IDD, pParent)
    , m_bSecureDebug(bSecureDebug)
    , m_uPID(uPID)
    , m_uDID(uDID)
    , m_uChipSeries(uChipSeries)
    , m_nSel(0)
    , m_uShowFlag(0x7)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_M55M1)
    // NOTE: the ClassWizard will add member initialization here
    m_uConfigValue[0]       = 0xFFFFFFFF;
    m_uConfigValue[1]       = 0xFFFFFFFF;
    m_uConfigValue[2]       = 0xFFFFFFFF;
    m_uConfigValue[3]       = 0xFFFFFFFF;
    m_uConfigValue[4]       = 0xFFFFFFFF;
    m_uConfigValue[5]       = 0xFFFFFFFF;
    m_uConfigValue[6]       = 0xFFFFFFFF;
    m_uConfigValue[7]       = 0xFFFFFFFF;
    m_uConfigValue[8]       = 0xFFFFFFFF;
    m_uConfigValue[9]       = 0xFFFFFFFF;
    m_uConfigValue[10]      = 0xFFFFFFFF;
    m_uConfigValue[11]      = 0xFFFFFFFF;
    m_uConfigValue[12]      = 0xFFFFFFFF;
    m_uConfigValue[13]      = 0xFFFFFFFF;

    m_bNSCBA_Write          = FALSE;
    m_uNSCBA_NSAddr         = 0xFFFFFFFF;
    m_bNSCBA_MirBoundEnable = FALSE;

    m_bSCRLOCK_Enable       = FALSE;
    m_bARLOCK_Enable        = FALSE;

    //}}AFX_DATA_INIT
}

CDialogChipSetting_M55M1::~CDialogChipSetting_M55M1()
{
    if (m_uShowFlag & 0x01)
    {
        delete m_pChipSetting_CFG;
    }

    if (m_uShowFlag & 0x02)
    {
        delete m_pChipSetting_APWPROT;
    }

    if (m_uShowFlag & 0x04)
    {
        delete m_pChipSetting_NSCBA;
    }
}

void CDialogChipSetting_M55M1::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_M55M1, CDialog)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_M55M1::OnOk)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_M55M1::OnTcnSelchangeTabChipsetting)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()


BOOL CDialogChipSetting_M55M1::OnInitDialog()
{
    CDialog::OnInitDialog();

    int i, nItem = 0;
    unsigned int uFlash_PageSize = NUMICRO_FLASH_PAGE_SIZE_8K;
    FLASH_PID_INFO_BASE_T chipInfo = {0};

    GetInfo(m_uPID, &chipInfo);

    if (m_uShowFlag & 0x01)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("CONFIG 0-6"));

        m_pChipSetting_CFG = new CDialogChipSetting_CFG_M55M1();

        m_pChipSetting_CFG->m_uProgramMemorySize    = chipInfo.uProgramMemorySize;
        m_pChipSetting_CFG->m_uFlashPageSize        = uFlash_PageSize;
        m_pChipSetting_CFG->m_uConfigValue[0]       = m_uConfigValue[0];
        m_pChipSetting_CFG->m_uConfigValue[1]       = m_uConfigValue[1];
        m_pChipSetting_CFG->m_uConfigValue[2]       = m_uConfigValue[2];
        m_pChipSetting_CFG->m_uConfigValue[3]       = m_uConfigValue[3];
        m_pChipSetting_CFG->m_uConfigValue[4]       = m_uConfigValue[4];
        m_pChipSetting_CFG->m_uConfigValue[5]       = m_uConfigValue[5];
        m_pChipSetting_CFG->m_uConfigValue[6]       = m_uConfigValue[6];

        m_pChipSetting_CFG->Create(CDialogChipSetting_CFG_M55M1::IDD, &m_TabChipSetting);
    }

    if (m_uShowFlag & 0x02)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("CONFIG 8-10"));

        m_pChipSetting_APWPROT = new CDialogChipSetting_APWPROT();

        m_pChipSetting_APWPROT->m_uAPROMAddr        = NUMICRO_M55_APROM_ADDR;
        m_pChipSetting_APWPROT->m_uAPROMSize        = chipInfo.uProgramMemorySize;
        m_pChipSetting_APWPROT->m_uRegionNum        = 64;
        m_pChipSetting_APWPROT->m_uRegionSize       = M55M1_APWPROT_REGION_SIZE;
        m_pChipSetting_APWPROT->m_uConfigValue[0]   = m_uConfigValue[8];
        m_pChipSetting_APWPROT->m_uConfigValue[1]   = m_uConfigValue[9];
        m_pChipSetting_APWPROT->m_uConfigValue[2]   = m_uConfigValue[10];

        m_pChipSetting_APWPROT->Create(CDialogChipSetting_APWPROT::IDD, &m_TabChipSetting);
    }

    if (m_uShowFlag & 0x04)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("NSCBA"));

        m_pChipSetting_NSCBA = new CDialogChipSetting_NSCBA_LOCK();

        {
            const uint32_t uFNSAddr_min = NUMICRO_M55_APROM_ADDR + uFlash_PageSize;
            const uint32_t uFNSAddr_max = NUMICRO_M55_APROM_ADDR + chipInfo.uProgramMemorySize;

            m_uNSCBA_NSAddr         = m_uConfigValue[12] & 0x00FFFFFF;
            m_bNSCBA_Write          = FALSE;
            m_bNSCBA_MirBoundEnable = (m_uConfigValue[12] & 0x80000000) ? TRUE : FALSE;
            m_bSCRLOCK_Enable       = ((m_uConfigValue[11] & 0xFF) == 0x5A) ? FALSE : TRUE;
            m_bARLOCK_Enable        = ((m_uConfigValue[13] & 0xFF) == 0x5A) ? FALSE : TRUE;

            if (m_uNSCBA_NSAddr < uFNSAddr_min)
                m_uNSCBA_NSAddr = uFNSAddr_min;
            else if (m_uNSCBA_NSAddr > uFNSAddr_max)
                m_uNSCBA_NSAddr = uFNSAddr_max;
        }

        m_pChipSetting_NSCBA->m_bSecureDebug        = m_bSecureDebug;
        m_pChipSetting_NSCBA->m_bSupportLock        = TRUE;
        m_pChipSetting_NSCBA->m_uFlash_BaseAddr     = NUMICRO_M55_APROM_ADDR;
        m_pChipSetting_NSCBA->m_uFlash_Size         = chipInfo.uProgramMemorySize;
        m_pChipSetting_NSCBA->m_uFlash_PageSize     = uFlash_PageSize;
        m_pChipSetting_NSCBA->m_uFNSAddr            = m_uNSCBA_NSAddr;
        m_pChipSetting_NSCBA->m_uFNSAddr_min        = NUMICRO_M55_APROM_ADDR + uFlash_PageSize;
        m_pChipSetting_NSCBA->m_bWrite              = m_bNSCBA_Write;
        m_pChipSetting_NSCBA->m_bMirBoundEnable     = m_bNSCBA_MirBoundEnable;

        m_pChipSetting_NSCBA->m_bSCRLOCK            = m_bSCRLOCK_Enable;
        m_pChipSetting_NSCBA->m_bARLOCK             = m_bARLOCK_Enable;

        m_pChipSetting_NSCBA->m_uConfigValue[0]     = m_uConfigValue[11];
        m_pChipSetting_NSCBA->m_uConfigValue[1]     = m_uConfigValue[12];
        m_pChipSetting_NSCBA->m_uConfigValue[2]     = m_uConfigValue[13];

        m_pChipSetting_NSCBA->Create(CDialogChipSetting_NSCBA_LOCK::IDD, &m_TabChipSetting);
    }

    CRect rcClient;
    m_TabChipSetting.GetClientRect(rcClient);
    m_TabChipSetting.AdjustRect(FALSE, rcClient);

    CDialog *pChipSetting[] =
    {
        m_pChipSetting_CFG,
        m_pChipSetting_APWPROT,
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

void CDialogChipSetting_M55M1::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSetting[] =
    {
        m_pChipSetting_CFG,
        m_pChipSetting_APWPROT,
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

void CDialogChipSetting_M55M1::OnOk()
{
    this->SetFocus();

    if (m_uShowFlag & 0x01)
    {
        m_uConfigValue[0]       = m_pChipSetting_CFG->m_uConfigValue[0];
        m_uConfigValue[1]       = m_pChipSetting_CFG->m_uConfigValue[1];
        m_uConfigValue[2]       = m_pChipSetting_CFG->m_uConfigValue[2];
        m_uConfigValue[3]       = m_pChipSetting_CFG->m_uConfigValue[3];
        m_uConfigValue[4]       = m_pChipSetting_CFG->m_uConfigValue[4];
        m_uConfigValue[5]       = m_pChipSetting_CFG->m_uConfigValue[5];
        m_uConfigValue[6]       = m_pChipSetting_CFG->m_uConfigValue[6];
    }

    if (m_uShowFlag & 0x02)
    {
        m_uConfigValue[8]       = m_pChipSetting_APWPROT->m_uConfigValue[0] | m_pChipSetting_APWPROT->m_uConfigValue_c[0];
        m_uConfigValue[9]       = m_pChipSetting_APWPROT->m_uConfigValue[1] | m_pChipSetting_APWPROT->m_uConfigValue_c[1];
        m_uConfigValue[10]      = m_pChipSetting_APWPROT->m_uConfigValue[2];
    }

    if (m_uShowFlag & 0x04)
    {
        m_bNSCBA_Write          = (m_pChipSetting_NSCBA->m_bWrite && m_bSecureDebug) ? TRUE : FALSE;
        m_bNSCBA_MirBoundEnable =  m_pChipSetting_NSCBA->m_bMirBoundEnable;
        m_uNSCBA_NSAddr         =  m_pChipSetting_NSCBA->m_uFNSAddr;

        m_bSCRLOCK_Enable       = m_pChipSetting_NSCBA->m_bSCRLOCK;
        m_bARLOCK_Enable        = m_pChipSetting_NSCBA->m_bARLOCK;

        {
            if (m_bSCRLOCK_Enable)
                m_uConfigValue[11] = ((m_uConfigValue[11] & 0xFF) == 0x5A) ? 0xFFFFFF00 : m_uConfigValue[11];
            else
                m_uConfigValue[11] = (m_uConfigValue[11] & ~0xFF) | 0x5A;

            if (m_bNSCBA_Write)
                m_uConfigValue[12] = (m_bNSCBA_MirBoundEnable ? 0x80000000 : 0) + (m_uNSCBA_NSAddr & 0x00FFFFFF);

            if (m_bARLOCK_Enable)
                m_uConfigValue[13] = ((m_uConfigValue[13] & 0xFF) == 0x5A) ? 0xFFFFFF00 : m_uConfigValue[13];
            else
                m_uConfigValue[13] = (m_uConfigValue[13] & ~0xFF) | 0x5A;
        }
    }

    CDialog::OnOK();
}
