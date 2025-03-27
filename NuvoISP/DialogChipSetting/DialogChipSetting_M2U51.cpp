// DialogChipSetting_M2U51.cpp : implementation file
//
#include "stdafx.h"

#include <deque>
#include <string>
#include <utility>
#include "Lang.h"
#include "DialogChipSetting_M2U51.h"


// CDialogChipSetting_M2U51 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_M2U51, CDialog)

CDialogChipSetting_M2U51::CDialogChipSetting_M2U51(unsigned int uPID, unsigned int uDID, unsigned int uChipSeries, CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_M2U51::IDD, pParent)
    , m_uPID(uPID)
    , m_uDID(uDID)
    , m_uChipSeries(uChipSeries)
    , m_nSel(0)
    , m_uShowFlag(0)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_M2U51)
    // NOTE: the ClassWizard will add member initialization here
    m_uCONFIG_Value[0]  = 0xFFFFFFFF;
    m_uCONFIG_Value[1]  = 0xFFFFFFFF;
    m_uCONFIG_Value[2]  = 0xFFFFFF5A;
    m_uCONFIG_Value[3]  = 0xFFFFFFFF;
    m_uCONFIG_Value[4]  = 0xFFFFFFFF;
    m_uCONFIG_Value[5]  = 0xFFFFFFFF;
    m_uCONFIG_Value[6]  = 0xFFFFFFFF;
    m_uCONFIG_Value[7]  = 0xFFFFFFFF;
    m_uCONFIG_Value[8]  = 0xFFFFFFFF;
    m_uCONFIG_Value[9]  = 0xFFFFFFFF;
    m_uCONFIG_Value[10] = 0xFFFFFFFF;

    m_uXOM_WriteFlag    = 0;

    m_uXOM_Addr[0]      = 0xFFFFFFFF;
    m_uXOM_Addr[1]      = 0xFFFFFFFF;
    m_uXOM_Count[0]     = 0;
    m_uXOM_Count[1]     = 0;
    m_uXOM_Ctrl[0]      = 0x5A;
    m_uXOM_Ctrl[1]      = 0x5A;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_M2U51::~CDialogChipSetting_M2U51()
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
        delete m_pChipSetting_XOM;
    }
}

void CDialogChipSetting_M2U51::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB_CHIP_SETTING, m_TabChipSetting);
}


BEGIN_MESSAGE_MAP(CDialogChipSetting_M2U51, CDialog)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_CHIP_SETTING, &CDialogChipSetting_M2U51::OnTcnSelchangeTabChipsetting)
    ON_BN_CLICKED(IDOK, &CDialogChipSetting_M2U51::OnOk)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()


BOOL CDialogChipSetting_M2U51::OnInitDialog()
{
    CDialog::OnInitDialog();

    int i, nItem = 0;

    CHIP_INFO_NUMICRO_T chipInfo;

    memset(&chipInfo, 0, sizeof(chipInfo));

    GetChipInfo_NuMicro(m_uPID, m_uDID, 0xFFFFFFFF, 0xFFFFFFFF, false, &chipInfo);

    if (m_uShowFlag & 0x01)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("CONFIG 0-2"));

        //if (m_uChipSeries == IDS_M2U51_SERIES)
        //{
        m_pChipSetting_CFG = new CDialogChipSetting_CFG_M2U51();
        //}

        m_pChipSetting_CFG->m_uProgramMemorySize    = chipInfo.uProgramMemorySize;
        m_pChipSetting_CFG->m_uFlashPageSize        = (1 << chipInfo.uFlash_PageSize);
        m_pChipSetting_CFG->m_uConfigValue[0]       = m_uCONFIG_Value[0];
        m_pChipSetting_CFG->m_uConfigValue[1]       = m_uCONFIG_Value[1];
        m_pChipSetting_CFG->m_uConfigValue[2]       = m_uCONFIG_Value[2];

        m_pChipSetting_CFG->Create(CDialogChipSetting_CFG_M2U51::IDD, &m_TabChipSetting);
    }

    if (m_uShowFlag & 0x02)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("CONFIG 8-10"));

        m_pChipSetting_APWPROT = new CDialogChipSetting_APWPROT();

        m_pChipSetting_APWPROT->m_uAPROMAddr        = chipInfo.uAPROM_Addr;
        m_pChipSetting_APWPROT->m_uAPROMSize        = chipInfo.uProgramMemorySize;
        m_pChipSetting_APWPROT->m_uRegionNum        = chipInfo.uAPWPROT_RegionNum;
        m_pChipSetting_APWPROT->m_uRegionSize       = chipInfo.uAPWPROT_RegionSize;
        m_pChipSetting_APWPROT->m_uConfigValue[0]   = m_uCONFIG_Value[8];
        m_pChipSetting_APWPROT->m_uConfigValue[1]   = m_uCONFIG_Value[9];
        m_pChipSetting_APWPROT->m_uConfigValue[2]   = m_uCONFIG_Value[10];

        m_pChipSetting_APWPROT->Create(CDialogChipSetting_APWPROT::IDD, &m_TabChipSetting);
    }

    if (m_uShowFlag & 0x04)
    {
        m_TabChipSetting.InsertItem(nItem++, _T("XOM"));

        m_pChipSetting_XOM = new CDialogChipSetting_XOM_NPU();

        m_pChipSetting_XOM->m_bSupportNPU           = FALSE;
        m_pChipSetting_XOM->m_uShowFlag             = 0x03;
        m_pChipSetting_XOM->m_uPageSize             = (1 << chipInfo.uFlash_PageSize);
        m_pChipSetting_XOM->m_uMaxCount             = chipInfo.uXOM_MaxCount;

#if 0

        for (i = 0; i < chipInfo.uXOM_Num; i++)
        {
            if (m_uXOM_WriteFlag & (1 << i))
            {
                m_pChipSetting_XOM->m_uAddr[i]      = (m_uXOM_Status[i] & 0xFFFFFF00) >> 8;
                m_pChipSetting_XOM->m_uCount[i]     = (m_uXOM_Status[i] & 0xFF);
                m_pChipSetting_XOM->m_uCtrl[i]      = (m_uXOM_WriteFlag & (1 << (i + 24))) ? 0x50 : 0x12;
                m_pChipSetting_XOM->m_bWrite[i]     = TRUE;
            }
            else
            {
                m_pChipSetting_XOM->m_uAddr[i]      = 0xFFFFFFFF;
                m_pChipSetting_XOM->m_uCount[i]     = 0;
                m_pChipSetting_XOM->m_uCtrl[i]      = 0x5A;
                m_pChipSetting_XOM->m_bWrite[i]     = FALSE;
            }
        }

#else
        m_pChipSetting_XOM->m_uAddr[0]              = m_uXOM_Addr[0];
        m_pChipSetting_XOM->m_uAddr[1]              = m_uXOM_Addr[1];

        m_pChipSetting_XOM->m_uCount[0]             = m_uXOM_Count[0];
        m_pChipSetting_XOM->m_uCount[1]             = m_uXOM_Count[1];

        m_pChipSetting_XOM->m_uCtrl[0]              = m_uXOM_Ctrl[0];
        m_pChipSetting_XOM->m_uCtrl[1]              = m_uXOM_Ctrl[1];

        m_pChipSetting_XOM->m_bWrite[0]             = (m_uXOM_WriteFlag & 0x01) ? TRUE : FALSE;
        m_pChipSetting_XOM->m_bWrite[1]             = (m_uXOM_WriteFlag & 0x02) ? TRUE : FALSE;
#endif

        m_pChipSetting_XOM->m_uBaseAddr.push_back(chipInfo.uXOM_RangeStart);
        m_pChipSetting_XOM->m_uEndAddr.push_back(chipInfo.uXOM_RangeEnd);

        m_pChipSetting_XOM->Create(CDialogChipSetting_XOM_NPU::IDD, &m_TabChipSetting);
    }

    CRect rcClient;
    m_TabChipSetting.GetClientRect(rcClient);
    m_TabChipSetting.AdjustRect(FALSE, rcClient);

    CDialog *pChipSetting[] =
    {
        m_pChipSetting_CFG,
        m_pChipSetting_APWPROT,
        m_pChipSetting_XOM,
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

void CDialogChipSetting_M2U51::OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult)
{
    CDialog *pChipSetting[] =
    {
        m_pChipSetting_CFG,
        m_pChipSetting_APWPROT,
        m_pChipSetting_XOM,
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

void CDialogChipSetting_M2U51::OnOk()
{
    int i;

    this->SetFocus();

    if (m_uShowFlag & 0x01)
    {
        m_uCONFIG_Value[0]      = m_pChipSetting_CFG->m_uConfigValue[0];
        m_uCONFIG_Value[1]      = m_pChipSetting_CFG->m_uConfigValue[1];
        m_uCONFIG_Value[2]      = m_pChipSetting_CFG->m_uConfigValue[2];
        m_uCONFIG_Value[3]      = m_pChipSetting_CFG->m_uConfigValue[3];
        m_uCONFIG_Value[4]      = m_pChipSetting_CFG->m_uConfigValue[4];
        m_uCONFIG_Value[5]      = m_pChipSetting_CFG->m_uConfigValue[5];
        m_uCONFIG_Value[6]      = m_pChipSetting_CFG->m_uConfigValue[6];
    }

    if (m_uShowFlag & 0x02)
    {
        m_uCONFIG_Value[8]      = m_pChipSetting_APWPROT->m_uConfigValue[0] | m_pChipSetting_APWPROT->m_uConfigValue_c[0];
        m_uCONFIG_Value[9]      = m_pChipSetting_APWPROT->m_uConfigValue[1] | m_pChipSetting_APWPROT->m_uConfigValue_c[1];
        m_uCONFIG_Value[10]     = m_pChipSetting_APWPROT->m_uConfigValue[2];
    }

    if (m_uShowFlag & 0x04)
    {
        m_uXOM_WriteFlag = 0;

        for (i = 0; i < 2; i++)
        {
            m_uXOM_Addr[i]  = 0xFFFFFFFF;
            m_uXOM_Count[i] = 0;
            m_uXOM_Ctrl[i]  = 0x5A;

            if (m_pChipSetting_XOM->m_bWrite[i])
            {
                m_uXOM_WriteFlag |= (1 << i);

                m_uXOM_Addr[i]  = m_pChipSetting_XOM->m_uAddr[i];
                m_uXOM_Count[i] = m_pChipSetting_XOM->m_uCount[i];
                m_uXOM_Ctrl[i]  = m_pChipSetting_XOM->m_uCtrl[i];
            }
        }
    }

    CDialog::OnOK();
}
