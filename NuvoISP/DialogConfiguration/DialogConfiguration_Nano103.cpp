// DialogConfiguration_Nano103.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "Lang.h"
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_Nano103.h"
#include <cassert>

#ifdef _DEBUG
    #define new DEBUG_NEW
    #undef THIS_FILE
    static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_Nano103 dialog


CDialogConfiguration_Nano103::CDialogConfiguration_Nano103(unsigned int uProgramMemorySize,
                                                           CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_Nano103::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_Nano103)
    m_nRadioBor = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_sFlashBaseAddress = _T("");
    //}}AFX_DATA_INIT
}


void CDialogConfiguration_Nano103::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_Nano103)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBor);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_CLOCK_STOP_DETECT, m_bClkStopDetect);
    DDX_Check(pDX, IDC_CHECK_MASS_ERASE, m_bMassErase);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDialogConfiguration_Nano103, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_Nano103)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CLOCK_STOP_DETECT, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_MASS_ERASE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_8, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_9, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_A, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_B, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_C, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_D, OnButtonClick)

    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE, OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_Nano103 message handlers

BOOL CDialogConfiguration_Nano103::OnInitDialog()
{
    CDialog::OnInitDialog();

    // TODO: Add extra initialization here
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);

    ConfigToGUI();
    //GUIToConfig();
    UpdateData(FALSE);

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}


void CDialogConfiguration_Nano103::ConfigToGUI()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];

    switch (uConfig0 & NANO103_FLASH_CONFIG_CBORST)
    {
        case NANO103_FLASH_CONFIG_CBORST_31:
            m_nRadioBor = 0;
            break;

        case NANO103_FLASH_CONFIG_CBORST_30:
            m_nRadioBor = 1;
            break;

        case NANO103_FLASH_CONFIG_CBORST_29:
            m_nRadioBor = 2;
            break;

        case NANO103_FLASH_CONFIG_CBORST_28:
            m_nRadioBor = 3;
            break;

        case NANO103_FLASH_CONFIG_CBORST_27:
            m_nRadioBor = 4;
            break;

        case NANO103_FLASH_CONFIG_CBORST_26:
            m_nRadioBor = 5;
            break;

        case NANO103_FLASH_CONFIG_CBORST_25:
            m_nRadioBor = 6;
            break;

        case NANO103_FLASH_CONFIG_CBORST_24:
            m_nRadioBor = 7;
            break;

        case NANO103_FLASH_CONFIG_CBORST_23:
            m_nRadioBor = 8;
            break;

        case NANO103_FLASH_CONFIG_CBORST_22:
            m_nRadioBor = 9;
            break;

        case NANO103_FLASH_CONFIG_CBORST_21:
            m_nRadioBor = 10;
            break;

        case NANO103_FLASH_CONFIG_CBORST_20:
            m_nRadioBor = 11;
            break;

        case NANO103_FLASH_CONFIG_CBORST_19:
            m_nRadioBor = 12;
            break;

        case NANO103_FLASH_CONFIG_CBORST_18:
            m_nRadioBor = 13;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & NANO103_FLASH_CONFIG_CBORST);
    }

    switch (uConfig0 & NANO100_FLASH_CONFIG_CBS)
    {
        case NANO100_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case NANO100_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case NANO100_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case NANO100_FLASH_CONFIG_CBS_AP_LD:
        default:
            m_nRadioBS = 3;
            break;
    }

    m_bDataFlashEnable = ((uConfig0 & NANO100_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutDetect = ((uConfig0 & NANO103_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bClkStopDetect = ((uConfig0 & NANO100_FLASH_CONFIG_CCKSTOP) == 0 ? TRUE : FALSE);
    m_bMassErase = ((uConfig0 & NANO100_FLASH_CONFIG_MERASE) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NANO100_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);

    unsigned int uFlashBaseAddress = uConfig1 & 0xFFFFF;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);

    unsigned int uPageNum = uFlashBaseAddress / NUMICRO_FLASH_PAGE_SIZE_512;
    unsigned int uLimitNum = m_uProgramMemorySize / NUMICRO_FLASH_PAGE_SIZE_512;
    unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * NUMICRO_FLASH_PAGE_SIZE_512) : 0;
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
    m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable ? TRUE : FALSE);
    GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS)->EnableWindow(m_bDataFlashEnable);

    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
}

void CDialogConfiguration_Nano103::GUIToConfig()
{
    //unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig0 = 0xFFFFFFFF;
    unsigned int uConfig1;

    uConfig0 &= ~NANO100_FLASH_CONFIG_CBS;

    switch (m_nRadioBS)
    {
        case 0:
            uConfig0 |= NANO100_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= NANO100_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= NANO100_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= NANO100_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & NANO100_FLASH_CONFIG_CBS);
    }

    uConfig0 &= ~NANO103_FLASH_CONFIG_CBORST;

    switch (m_nRadioBor)
    {
        case 0:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_31;
            break;

        case 1:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_30;
            break;

        case 2:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_29;
            break;

        case 3:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_28;
            break;

        case 4:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_27;
            break;

        case 5:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_26;
            break;

        case 6:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_25;
            break;

        case 7:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_24;
            break;

        case 8:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_23;
            break;

        case 9:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_22;
            break;

        case 10:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_21;
            break;

        case 11:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_20;
            break;

        case 12:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_19;
            break;

        case 13:
            uConfig0 |= NANO103_FLASH_CONFIG_CBORST_18;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & NANO103_FLASH_CONFIG_CBORST);
    }

    if (m_bCheckBrownOutDetect)
        uConfig0 &= ~NANO103_FLASH_CONFIG_CBODEN;
    else
        uConfig0 |= NANO103_FLASH_CONFIG_CBODEN;

    if (m_bDataFlashEnable)
        uConfig0 &= ~NANO100_FLASH_CONFIG_DFEN;
    else
    {
        uConfig0 |= NANO100_FLASH_CONFIG_DFEN;
        m_sFlashBaseAddress = "FFFFFFFF";
    }

    if (m_bClkStopDetect)
        uConfig0 &= ~NANO100_FLASH_CONFIG_CCKSTOP;
    else
        uConfig0 |= NANO100_FLASH_CONFIG_CCKSTOP;

    if (m_bMassErase)
        uConfig0 &= ~NANO100_FLASH_CONFIG_MERASE;
    else
        uConfig0 |= NANO100_FLASH_CONFIG_MERASE;

    if (m_bSecurityLock)
        uConfig0 &= ~NANO100_FLASH_CONFIG_LOCK;
    else
        uConfig0 |= NANO100_FLASH_CONFIG_LOCK;

    m_ConfigValue.m_value[0] = uConfig0;

    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;// | 0xFFF00000;
}

void CDialogConfiguration_Nano103::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);

    GUIToConfig();
    ConfigToGUI();

    UpdateData(FALSE);
}

void CDialogConfiguration_Nano103::OnKillfocusEditFlashBaseAddress()
{
    UpdateData(TRUE);
    CDialogResize::OnKillfocusEditFlashBaseAddress(m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_512);
}

void CDialogConfiguration_Nano103::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);

    OnKillfocusEditFlashBaseAddress();
    GUIToConfig();

    CDialog::OnOK();
}


CString CDialogConfiguration_Nano103::GetConfigWarning(const CAppConfig::Nano100_configs_t &config)
{
    CString str;
    unsigned int uConfig0 = config.m_value[0];

    switch (uConfig0 & NANO100_FLASH_CONFIG_CFOSC)
    {
        case NANO100_FLASH_CONFIG_E12M:
            str += _T("   ") + _I(IDS_SELECT_EXTERNAL_12M_CLOCK);
            break;

        default:
            ;
    }

    BOOL bSecurityLock = ((uConfig0 & NANO100_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);

    if (!bSecurityLock)
        str += _T("   ") + _I(IDS_DISABLE_SECURITY_LOCK);

    return str;
}

void CDialogConfiguration_Nano103::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    CDialogResize::OnDeltaposSpinDataFlashSize(pNMHDR, pResult, m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_512);
}

void CDialogConfiguration_Nano103::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID())
        return;

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}
