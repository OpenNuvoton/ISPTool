// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_M451.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M451 dialog

CDialogConfiguration_M451::CDialogConfiguration_M451(unsigned int uProgramMemorySize,
        CWnd *pParent)
    : CDialogResize(CDialogConfiguration_M451::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_M451)
    m_nRadioClk = -1;
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_bWDTEnable = FALSE;
    m_bWDTPowerDown = FALSE;
    m_nRadioGPG = -1;
    m_nRadioIO = -1;
    m_sFlashBaseAddress = _T("");
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_M451::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_M451)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_CLK_E12M, m_nRadioClk);
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_WDT_ENABLE, m_bWDTEnable);
    DDX_Check(pDX, IDC_CHECK_WDT_POWER_DOWN, m_bWDTPowerDown);
    DDX_Radio(pDX, IDC_RADIO_GPF_GPIO, m_nRadioGPG);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M451, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_M451)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_CLK_E12M, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_CLK_I22M, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_POWER_DOWN, OnCheckClickWDTPD)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_GPIO, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_CRYSTAL, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE, OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M451 message handlers

BOOL CDialogConfiguration_M451::OnInitDialog()
{
    CDialog::OnInitDialog();
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);
    ConfigToGUI(0);
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;
}

void CDialogConfiguration_M451::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];

    switch (uConfig0 & M451_FLASH_CONFIG_CFOSC) {
        case M451_FLASH_CONFIG_E12M:
            m_nRadioClk = 0;
            break;

        case M451_FLASH_CONFIG_I22M:
        default:
            m_nRadioClk = 1;
            break;
    }

    switch (uConfig0 & M451_FLASH_CONFIG_CBOV) {
        case M451_FLASH_CONFIG_CBOV_45:
            m_nRadioBov = 0;
            break;

        case M451_FLASH_CONFIG_CBOV_38:
            m_nRadioBov = 1;
            break;

        case M451_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 2;
            break;

        case M451_FLASH_CONFIG_CBOV_22:
        default:
            m_nRadioBov = 3;
            break;
    }

    switch (uConfig0 & M451_FLASH_CONFIG_CBS2) {
        case M451_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case M451_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case M451_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case M451_FLASH_CONFIG_CBS_AP_LD:
            m_nRadioBS = 3;
            break;

        default:
            m_nRadioBS = 1;
            break;
    }

    m_nRadioGPG = ((uConfig0 & M451_FLASH_CONFIG_CGPFMFP) == 0 ? 0 : 1);
    m_nRadioIO = ((uConfig0 & M451_FLASH_CONFIG_CIOINI) == 0 ? 1 : 0);
    m_bWDTPowerDown = ((uConfig0 & M451_FLASH_CONFIG_CWDTPDEN) == 0 ? TRUE : FALSE);
    m_bWDTEnable = ((uConfig0 & M451_FLASH_CONFIG_CWDTEN) == 0 ? TRUE : FALSE);;

    if (!m_bWDTEnable) {
        m_bWDTPowerDown = FALSE;
    }

    m_bCheckBrownOutDetect = ((uConfig0 & M451_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & M451_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & M451_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & M451_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    unsigned int uFlashBaseAddress = uConfig1 & 0xFFFFF;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    unsigned int uPageNum = uFlashBaseAddress / NUMICRO_FLASH_PAGE_SIZE_2K;
    unsigned int uLimitNum = m_uProgramMemorySize / NUMICRO_FLASH_PAGE_SIZE_2K;
    unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * NUMICRO_FLASH_PAGE_SIZE_2K) : 0;
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
    m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable ? TRUE : FALSE);
    GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS)->EnableWindow(m_bDataFlashEnable);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
}

void CDialogConfiguration_M451::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1;
    uConfig0 &= ~M451_FLASH_CONFIG_CFOSC;

    switch (m_nRadioClk) {
        case 0:
            uConfig0 |= M451_FLASH_CONFIG_E12M;
            break;

        case 1:
            uConfig0 |= M451_FLASH_CONFIG_I22M;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M451_FLASH_CONFIG_CFOSC);
    }

    uConfig0 &= ~M451_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= M451_FLASH_CONFIG_CBOV_45;
            break;

        case 1:
            uConfig0 |= M451_FLASH_CONFIG_CBOV_38;
            break;

        case 2:
            uConfig0 |= M451_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= M451_FLASH_CONFIG_CBOV_22;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M451_FLASH_CONFIG_CBOV);
    }

    uConfig0 &= ~M451_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= M451_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= M451_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= M451_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= M451_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M451_FLASH_CONFIG_CBS2);
    }

    if (m_nRadioGPG == 0) {
        uConfig0 &= ~M451_FLASH_CONFIG_CGPFMFP;
    } else {
        uConfig0 |= M451_FLASH_CONFIG_CGPFMFP;
    }

    if (m_nRadioIO == 0) {
        uConfig0 |= M451_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~M451_FLASH_CONFIG_CIOINI;
    }

    if (m_bWDTPowerDown) {
        uConfig0 &= ~M451_FLASH_CONFIG_CWDTPDEN;
    } else {
        uConfig0 |= M451_FLASH_CONFIG_CWDTPDEN;
    }

    if (m_bWDTEnable) {
        uConfig0 &= ~M451_FLASH_CONFIG_CWDTEN;

        if (!m_bWDTPowerDown) {
            uConfig0 &= ~M451_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 &= ~M451_FLASH_CONFIG_CWDTEN_BIT0;
        }
    } else {
        uConfig0 |= M451_FLASH_CONFIG_CWDTEN;
    }

    if (nEventID == IDC_CHECK_WDT_POWER_DOWN) {
        if (m_bWDTPowerDown) {
            uConfig0 &= ~M451_FLASH_CONFIG_CWDTEN;
            uConfig0 |= M451_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 |= M451_FLASH_CONFIG_CWDTEN_BIT0;
        }
    } else {
        if (!m_bWDTEnable) {
            uConfig0 |= M451_FLASH_CONFIG_CWDTPDEN;
            uConfig0 |= M451_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 |= M451_FLASH_CONFIG_CWDTEN_BIT0;
        }
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~M451_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= M451_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~M451_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= M451_FLASH_CONFIG_CBORST;
    }

    if (m_bDataFlashEnable) {
        uConfig0 &= ~M451_FLASH_CONFIG_DFEN;
    } else {
        uConfig0 |= M451_FLASH_CONFIG_DFEN;
        m_sFlashBaseAddress = "FFFFF";
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~M451_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= M451_FLASH_CONFIG_LOCK;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;// | 0xFFF00000;
}

void CDialogConfiguration_M451::OnGUIEvent(int nEventID)
{
    UpdateData(TRUE);
    GUIToConfig(nEventID);
    ConfigToGUI(nEventID);
    UpdateData(FALSE);
}

void CDialogConfiguration_M451::OnButtonClick()
{
    OnGUIEvent();
}

void CDialogConfiguration_M451::OnCheckClickWDTPD()
{
    OnGUIEvent(IDC_CHECK_WDT_POWER_DOWN);
}

void CDialogConfiguration_M451::OnKillfocusEditFlashBaseAddress()
{
    UpdateData(TRUE);
    CDialogResize::OnKillfocusEditFlashBaseAddress(m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_2K);
}

void CDialogConfiguration_M451::OnOK()
{
    UpdateData(TRUE);
    GUIToConfig(0);
    CDialog::OnOK();
}

void CDialogConfiguration_M451::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
    UpdateData(TRUE);
    TCHAR *pEnd;
    unsigned int uFlashBaseAddress = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    unsigned int uPageNum = uFlashBaseAddress / NUMICRO_FLASH_PAGE_SIZE_2K; //APROM
    unsigned int uLimitNum = m_uProgramMemorySize / NUMICRO_FLASH_PAGE_SIZE_2K;

    if (pNMUpDown->iDelta == 1) {
        uPageNum += 1;
    } else if (pNMUpDown->iDelta == -1 && uPageNum > 0) {
        uPageNum -= 1;
    }

    uFlashBaseAddress = 0 + min(uPageNum, uLimitNum) * NUMICRO_FLASH_PAGE_SIZE_2K;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    m_sConfigValue1.Format(_T("0x%08X"), uFlashBaseAddress);// | 0xFFF00000);
    unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * NUMICRO_FLASH_PAGE_SIZE_2K) : 0;
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
    UpdateData(FALSE);
    *pResult = 0;
}

void CDialogConfiguration_M451::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M4521
/////////////////////////////////////////////////////////////////////////////
CDialogConfiguration_M4521::CDialogConfiguration_M4521(unsigned int uProgramMemorySize, CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M451(uProgramMemorySize, pParent)
{
}

BOOL CDialogConfiguration_M4521::OnInitDialog()
{
    GetDlgItem(IDC_RADIO_CLK_E12M)->SetWindowText(_T("External Crystal Clock (4~24MHz)"));
    GetDlgItem(IDC_RADIO_GPF_CRYSTAL)->SetWindowText(_T("External Crystal Clock (4~24MHz)"));
    return CDialogConfiguration_M451::OnInitDialog();
}
