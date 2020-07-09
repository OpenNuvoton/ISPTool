// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_I94000.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define page_size NUMICRO_FLASH_PAGE_SIZE_4K

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_I94000 dialog

CDialogConfiguration_I94000::CDialogConfiguration_I94000(unsigned int uProgramMemorySize,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_I94000::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_I94000)
    m_nRadioClk = -1;
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_bICELock = FALSE;
    m_bWDTEnable = FALSE;
    m_bWDTPowerDown = FALSE;
    m_nRadioGPG = -1;
    m_nRadioGPA8 = -1;
    m_nRadioIO = -1;
    //uConfig2_Test=-1;
    m_sFlashBaseAddress = _T("");
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_I94000::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_I94000)
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
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK, m_bICELock);
    DDX_Check(pDX, IDC_CHECK_WDT_ENABLE, m_bWDTEnable);
    DDX_Check(pDX, IDC_CHECK_WDT_POWER_DOWN, m_bWDTPowerDown);
    DDX_Radio(pDX, IDC_RADIO_GPF_GPIO, m_nRadioGPG);
    DDX_Radio(pDX, IDC_RADIO_PA8_IOINI, m_nRadioGPA8);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_I94000, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_I94000)
    ON_BN_CLICKED(IDC_RADIO_CLK_E12M, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)

    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_CLK_I22M, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_POWER_DOWN, OnCheckClickWDTPD)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_GPIO, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_CRYSTAL, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_PA8_IOINI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_PA8_LOW, OnButtonClick)
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
// CDialogConfiguration_I94000 message handlers

BOOL CDialogConfiguration_I94000::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);
    ConfigToGUI(0);
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_I94000::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];

    switch (uConfig0 & M480_FLASH_CONFIG_CFOSC) {
        case M480_FLASH_CONFIG_E12M:
            m_nRadioClk = 0;
            break;

        case M480_FLASH_CONFIG_I22M:
        default:
            m_nRadioClk = 1;
            break;
    }

    switch (uConfig0 & M480_FLASH_CONFIG_CBOV) {
        case M480_FLASH_CONFIG_CBOV_30:
            m_nRadioBov = 0;
            break;

        case M480_FLASH_CONFIG_CBOV_28:
            m_nRadioBov = 1;
            break;

        case M480_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 2;
            break;

        case M480_FLASH_CONFIG_CBOV_24:
            m_nRadioBov = 3;
            break;

        case M480_FLASH_CONFIG_CBOV_22:
            m_nRadioBov = 4;
            break;

        case M480_FLASH_CONFIG_CBOV_20:
            m_nRadioBov = 5;
            break;

        case M480_FLASH_CONFIG_CBOV_18:
            m_nRadioBov = 6;
            break;

        case M480_FLASH_CONFIG_CBOV_16:
            m_nRadioBov = 7;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M480_FLASH_CONFIG_CBOV);
    }

    switch (uConfig0 & M480_FLASH_CONFIG_CBS2) {
        case M480_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case M480_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case M480_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case M480_FLASH_CONFIG_CBS_AP_LD:
            m_nRadioBS = 3;
            break;

        default:
            m_nRadioBS = 1;
            break;
    }

    m_nRadioGPG = ((uConfig0 & M480_FLASH_CONFIG_CGPFMFP) == 0 ? 0 : 1);
    m_nRadioGPA8 = ((uConfig0 & I94000_FLASH_CONFIG_CGPA8) == 0 ? 1 : 0);
    m_nRadioIO = ((uConfig0 & M480_FLASH_CONFIG_CIOINI) == 0 ? 1 : 0);
    m_bWDTPowerDown = ((uConfig0 & M480_FLASH_CONFIG_CWDTPDEN) == 0 ? TRUE : FALSE);
    m_bWDTEnable = ((uConfig0 & M480_FLASH_CONFIG_CWDTEN) == 0 ? TRUE : FALSE);

    if (!m_bWDTEnable) {
        m_bWDTPowerDown = FALSE;
    }

    m_bCheckBrownOutDetect = ((uConfig0 & M480_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & M480_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bICELock = ((uConfig0 & I94000_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    unsigned int uFlashBaseAddress = uConfig1 & 0xFFFFF;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    unsigned int uPageNum = uFlashBaseAddress / page_size;
    unsigned int uLimitNum = m_uProgramMemorySize / page_size;
    unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * page_size) : page_size;
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
    m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable ? TRUE : FALSE);
    GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS)->EnableWindow(m_bDataFlashEnable);

    if (m_bDataFlashEnable) {
        uFlashBaseAddress = m_uProgramMemorySize - uDataFlashSize;
        m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
        uConfig1 = uFlashBaseAddress;
    }

    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
}

void CDialogConfiguration_I94000::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1;
    uConfig0 &= ~M480_FLASH_CONFIG_CFOSC;

    switch (m_nRadioClk) {
        case 0:
            uConfig0 |= M480_FLASH_CONFIG_E12M;
            break;

        case 1:
            uConfig0 |= M480_FLASH_CONFIG_I22M;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M480_FLASH_CONFIG_CFOSC);
    }

    uConfig0 &= ~M480_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_30;
            break;

        case 1:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_28;
            break;

        case 2:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_24;
            break;

        case 4:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_22;
            break;

        case 5:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_20;
            break;

        case 6:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_18;
            break;

        case 7:
            uConfig0 |= M480_FLASH_CONFIG_CBOV_16;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M480_FLASH_CONFIG_CBOV);
    }

    uConfig0 &= ~M480_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= M480_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= M480_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= M480_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= M480_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & M480_FLASH_CONFIG_CBS2);
    }

    if (m_nRadioGPG == 0) {
        uConfig0 &= ~M480_FLASH_CONFIG_CGPFMFP;
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CGPFMFP;
    }

    if (m_nRadioGPA8 == 0) {
        uConfig0 |= I94000_FLASH_CONFIG_CGPA8;
    } else {
        uConfig0 &= ~I94000_FLASH_CONFIG_CGPA8;
    }

    if (m_nRadioIO == 0) {
        uConfig0 |= M480_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~M480_FLASH_CONFIG_CIOINI;
    }

    if (m_bWDTPowerDown) {
        uConfig0 &= ~M480_FLASH_CONFIG_CWDTPDEN;
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CWDTPDEN;
    }

    if (m_bWDTEnable) {
        uConfig0 &= ~M480_FLASH_CONFIG_CWDTEN;

        if (!m_bWDTPowerDown) {
            uConfig0 &= ~M480_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 &= ~M480_FLASH_CONFIG_CWDTEN_BIT0;
        }
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CWDTEN;
    }

    if (nEventID == IDC_CHECK_WDT_POWER_DOWN) {
        if (m_bWDTPowerDown) {
            uConfig0 &= ~M480_FLASH_CONFIG_CWDTEN;
            uConfig0 |= M480_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 |= M480_FLASH_CONFIG_CWDTEN_BIT0;
        }
    } else {
        if (!m_bWDTEnable) {
            uConfig0 |= M480_FLASH_CONFIG_CWDTPDEN;
            uConfig0 |= M480_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 |= M480_FLASH_CONFIG_CWDTEN_BIT0;
        }
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~M480_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~M480_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= M480_FLASH_CONFIG_CBORST;
    }

    if (m_bDataFlashEnable) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_DFEN;
    } else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_DFEN;
        m_sFlashBaseAddress = "FFFFF";
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_LOCK;
    }

    if (m_bICELock) {
        uConfig0 &= ~I94000_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |= I94000_FLASH_CONFIG_ICELOCK;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;// | 0xFFF00000;
}

void CDialogConfiguration_I94000::OnGUIEvent(int nEventID)
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig(nEventID);
    ConfigToGUI(nEventID);
    UpdateData(FALSE);
}

void CDialogConfiguration_I94000::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}

void CDialogConfiguration_I94000::OnCheckClickWDTPD()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent(IDC_CHECK_WDT_POWER_DOWN);
}

void CDialogConfiguration_I94000::OnKillfocusEditFlashBaseAddress()
{
    UpdateData(TRUE);
    CDialogResize::OnKillfocusEditFlashBaseAddress(m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_4K);
}

void CDialogConfiguration_I94000::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    OnKillfocusEditFlashBaseAddress();
    GUIToConfig(0);
    CDialog::OnOK();
}

void CDialogConfiguration_I94000::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    CDialogResize::OnDeltaposSpinDataFlashSize(pNMHDR, pResult, m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_4K);
}

void CDialogConfiguration_I94000::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}
