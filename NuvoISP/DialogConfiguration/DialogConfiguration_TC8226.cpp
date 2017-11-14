// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_TC8226.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define page_size NUMICRO_FLASH_PAGE_SIZE_4K

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_TC8226 dialog

CDialogConfiguration_TC8226::CDialogConfiguration_TC8226(unsigned int uProgramMemorySize,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_TC8226::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_TC8226)
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_nRadioSPIM = -1;
    m_nRadioUART = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bCheckBootLoader = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_bSecurityBootLock = FALSE;
    m_bICELock = FALSE;
    m_bWDTEnable = FALSE;
    m_bWDTPowerDown = FALSE;
    m_nRadioGPG = -1;
    m_nRadioIO = -1;
    m_sFlashBaseAddress = _T("");
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_TC8226::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_TC8226)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_BOR_30, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Radio(pDX, IDC_RADIO_SPIM_SEL0, m_nRadioSPIM);
//	DDX_Radio(pDX, IDC_RADIO_UART1_SEL0, m_nRadioUART);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2, m_sConfigValue2);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_3, m_sConfigValue3);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
//	DDX_Check(pDX, IDC_CHECK_BOOT_LOADER, m_bCheckBootLoader);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK, m_bICELock);
    DDX_Check(pDX, IDC_CHECK_WDT_ENABLE, m_bWDTEnable);
    DDX_Check(pDX, IDC_CHECK_WDT_POWER_DOWN, m_bWDTPowerDown);
    DDX_Radio(pDX, IDC_RADIO_GPF_GPIO, m_nRadioGPG);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
//	DDX_Check(pDX, IDC_CHECK_SECURITYBOOT_LOCK, m_bSecurityBootLock);
    DDX_Check(pDX, IDC_SPROM_LOCK_CACHEABLE, m_bSpromLockCacheable);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_TC8226, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_TC8226)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnChangeEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_BOR_30, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOR_28, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOR_26, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOR_24, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOR_22, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOR_20, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOR_18, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOR_16, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BOOT_LOADER, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITYBOOT_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WATCHDOG_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_SPROM_LOCK_CACHEABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_POWER_DOWN, OnCheckClickWDTPD)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_GPIO, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_CRYSTAL, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_SPIM_SEL3, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_UART1_SEL3, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE, OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_TC8226 message handlers

BOOL CDialogConfiguration_TC8226::OnInitDialog()
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

void CDialogConfiguration_TC8226::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];
    unsigned int uConfig2 = m_ConfigValue.m_value[2];
    unsigned int uConfig3 = m_ConfigValue.m_value[3];
    unsigned int uFlashBaseAddress = uConfig1;

    switch (uConfig0 & TC8226_FLASH_CONFIG_CBOV) {
        case TC8226_FLASH_CONFIG_CBOV_30:
            m_nRadioBov = 0;
            break;

        case TC8226_FLASH_CONFIG_CBOV_28:
            m_nRadioBov = 1;
            break;

        case TC8226_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 2;
            break;

        case TC8226_FLASH_CONFIG_CBOV_24:
            m_nRadioBov = 3;
            break;

        case TC8226_FLASH_CONFIG_CBOV_22:
            m_nRadioBov = 4;
            break;

        case TC8226_FLASH_CONFIG_CBOV_20:
            m_nRadioBov = 5;
            break;

        case TC8226_FLASH_CONFIG_CBOV_18:
            m_nRadioBov = 6;
            break;

        case TC8226_FLASH_CONFIG_CBOV_16:
            m_nRadioBov = 7;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & TC8226_FLASH_CONFIG_CBOV);
    }

    switch (uConfig0 & TC8226_FLASH_CONFIG_CBS2) {
        case TC8226_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case TC8226_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case TC8226_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case TC8226_FLASH_CONFIG_CBS_AP_LD:
            m_nRadioBS = 3;
            break;

        default:
            m_nRadioBS = 1;
            break;
    }

    m_nRadioGPG = ((uConfig0 & TC8226_FLASH_CONFIG_CGPFMFP) == 0 ? 0 : 1);
    m_nRadioIO = ((uConfig0 & TC8226_FLASH_CONFIG_CIOINI) == 0 ? 1 : 0);
    m_bWDTPowerDown = ((uConfig0 & TC8226_FLASH_CONFIG_CWDTPDEN) == 0 ? TRUE : FALSE);
    m_bWDTEnable = ((uConfig0 & TC8226_FLASH_CONFIG_CWDTEN) == 0 ? TRUE : FALSE);;

    if (!m_bWDTEnable) {
        m_bWDTPowerDown = FALSE;
    }

    m_bCheckBrownOutDetect = ((uConfig0 & TC8226_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & TC8226_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckBootLoader = FALSE;//((uConfig0 & TC8226_FLASH_CONFIG_BOOTLOADER) == 0 ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & TC8226_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & TC8226_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bICELock = ((uConfig0 & TC8226_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    m_bSpromLockCacheable = ((uConfig0 & TC8226_FLASH_CONFIG_SPLCAEN) == 0 ? FALSE : TRUE);

    if (m_bDataFlashEnable) {
        uFlashBaseAddress = ((uFlashBaseAddress >= page_size) && (uFlashBaseAddress < m_uProgramMemorySize)) ? uFlashBaseAddress : (m_uProgramMemorySize - page_size);
        uFlashBaseAddress = (uFlashBaseAddress & TC8226_FLASH_CONFIG_DFBA) / page_size * page_size;
        uConfig1 = uFlashBaseAddress;
    }

    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable && (uFlashBaseAddress < m_uProgramMemorySize)) ? ((m_uProgramMemorySize - uFlashBaseAddress) / 1024.) : 0.);
    m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
    m_bSecurityBootLock = FALSE;//((uConfig2 & TC8226_FLASH_CONFIG_SBLOCK) == 0x5A5A ? FALSE : TRUE);

    switch (uConfig3 & TC8226_FLASH_CONFIG_SPIM) {
        case TC8226_FLASH_CONFIG_SPIM_SEL0:
            m_nRadioSPIM = 0;
            break;

        case TC8226_FLASH_CONFIG_SPIM_SEL1:
            m_nRadioSPIM = 1;
            break;

        case TC8226_FLASH_CONFIG_SPIM_SEL2:
            m_nRadioSPIM = 2;
            break;

        case TC8226_FLASH_CONFIG_SPIM_SEL3:
            m_nRadioSPIM = 3;
            break;

        default:
            m_nRadioSPIM = 3;
            break;
    }

    //switch(uConfig3 & TC8226_FLASH_CONFIG_UART1)
    //{
    //case TC8226_FLASH_CONFIG_UART1_SEL0:
    //	m_nRadioUART = 0; break;
    //case TC8226_FLASH_CONFIG_UART1_SEL1:
    //	m_nRadioUART = 1; break;
    //case TC8226_FLASH_CONFIG_UART1_SEL2:
    //	m_nRadioUART = 2; break;
    //case TC8226_FLASH_CONFIG_UART1_SEL3:
    //	m_nRadioUART = 3; break;
    //default:
    //	m_nRadioUART = 3; break;
    //}
    m_sConfigValue2.Format(_T("0x%08X"), uConfig2);
    m_sConfigValue3.Format(_T("0x%08X"), uConfig3);
}

void CDialogConfiguration_TC8226::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1;
    unsigned int uConfig2 = m_ConfigValue.m_value[2];
    unsigned int uConfig3 = m_ConfigValue.m_value[3];
    uConfig0 &= ~TC8226_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_30;
            break;

        case 1:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_28;
            break;

        case 2:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_24;
            break;

        case 4:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_22;
            break;

        case 5:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_20;
            break;

        case 6:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_18;
            break;

        case 7:
            uConfig0 |= TC8226_FLASH_CONFIG_CBOV_16;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & TC8226_FLASH_CONFIG_CBOV);
    }

    uConfig0 &= ~TC8226_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= TC8226_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= TC8226_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= TC8226_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= TC8226_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & TC8226_FLASH_CONFIG_CBS2);
    }

    if (m_nRadioGPG == 0) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CGPFMFP;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_CGPFMFP;
    }

    if (m_nRadioIO == 0) {
        uConfig0 |= TC8226_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CIOINI;
    }

    if (m_bWDTPowerDown) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CWDTPDEN;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_CWDTPDEN;
    }

    if (m_bWDTEnable) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CWDTEN;

        if (!m_bWDTPowerDown) {
            uConfig0 &= ~TC8226_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 &= ~TC8226_FLASH_CONFIG_CWDTEN_BIT0;
        }
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_CWDTEN;
    }

    if (nEventID == IDC_CHECK_WDT_POWER_DOWN) {
        if (m_bWDTPowerDown) {
            uConfig0 &= ~TC8226_FLASH_CONFIG_CWDTEN;
            uConfig0 |= TC8226_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 |= TC8226_FLASH_CONFIG_CWDTEN_BIT0;
        }
    } else {
        if (!m_bWDTEnable) {
            uConfig0 |= TC8226_FLASH_CONFIG_CWDTPDEN;
            uConfig0 |= TC8226_FLASH_CONFIG_CWDTEN_BIT1;
            uConfig0 |= TC8226_FLASH_CONFIG_CWDTEN_BIT0;
        }
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_CBORST;
    }

    if (m_bCheckBootLoader) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_BOOTLOADER;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_BOOTLOADER;
    }

    if (m_bDataFlashEnable) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_DFEN;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_DFEN;
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_LOCK;
    }

    if (m_bICELock) {
        uConfig0 &= ~TC8226_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |= TC8226_FLASH_CONFIG_ICELOCK;
    }

    if (m_bSpromLockCacheable) {
        uConfig0 |= TC8226_FLASH_CONFIG_SPLCAEN;
    } else {
        uConfig0 &= ~TC8226_FLASH_CONFIG_SPLCAEN;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;

    if (m_bSecurityBootLock) {
        uConfig2 &= ~TC8226_FLASH_CONFIG_SBLOCK;
    } else {
        uConfig2 &= ~TC8226_FLASH_CONFIG_SBLOCK;
        uConfig2 |= 0x00005A5A;
    }

    uConfig3 &= ~TC8226_FLASH_CONFIG_SPIM;

    switch (m_nRadioSPIM) {
        case 0:
            uConfig3 |= TC8226_FLASH_CONFIG_SPIM_SEL0;
            break;

        case 1:
            uConfig3 |= TC8226_FLASH_CONFIG_SPIM_SEL1;
            break;

        case 2:
            uConfig3 |= TC8226_FLASH_CONFIG_SPIM_SEL2;
            break;

        case 3:
            uConfig3 |= TC8226_FLASH_CONFIG_SPIM_SEL3;
            break;

        default:
            /* Keep old value */
            uConfig3 |= (m_ConfigValue.m_value[3] & TC8226_FLASH_CONFIG_SPIM);
    }

    //uConfig3 &= ~TC8226_FLASH_CONFIG_UART1;
    //switch(m_nRadioUART)
    //{
    //case 0:
    //	uConfig3 |= TC8226_FLASH_CONFIG_UART1_SEL0; break;
    //case 1:
    //	uConfig3 |= TC8226_FLASH_CONFIG_UART1_SEL1; break;
    //case 2:
    //	uConfig3 |= TC8226_FLASH_CONFIG_UART1_SEL2; break;
    //case 3:
    //	uConfig3 |= TC8226_FLASH_CONFIG_UART1_SEL3; break;
    //default:
    //	/* Keep old value */
    //	uConfig3 |= (m_ConfigValue.m_value[3] & TC8226_FLASH_CONFIG_UART1);
    //}
    m_ConfigValue.m_value[2] = uConfig2;
    m_ConfigValue.m_value[3] = uConfig3;
}

void CDialogConfiguration_TC8226::OnGUIEvent(int nEventID)
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig(nEventID);
    ConfigToGUI(nEventID);
    UpdateData(FALSE);
}

void CDialogConfiguration_TC8226::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}

void CDialogConfiguration_TC8226::OnCheckClickWDTPD()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent(IDC_CHECK_WDT_POWER_DOWN);
}

void CDialogConfiguration_TC8226::OnChangeEditFlashBaseAddress()
{
    // TODO: If this is a RICHEDIT control, the control will not
    // send this notification unless you override the CDialog::OnInitDialog()
    // function and call CRichEditCtrl().SetEventMask()
    // with the ENM_CHANGE flag ORed into the mask.
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    TCHAR *pEnd;
    unsigned int uFlashBaseAddress = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);

    if (m_bDataFlashEnable) {
        if (!((uFlashBaseAddress >= page_size) && (uFlashBaseAddress < m_uProgramMemorySize))) {
            uFlashBaseAddress = m_uProgramMemorySize - page_size;
        }

        uFlashBaseAddress &= ~(page_size - 1);
        m_sDataFlashSize.Format(_T("%.2fK"), (uFlashBaseAddress < m_uProgramMemorySize) ? ((m_uProgramMemorySize - uFlashBaseAddress) / 1024.) : 0.);
    }

    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    m_sConfigValue1.Format(_T("0x%08X"), uFlashBaseAddress);
    UpdateData(FALSE);
}

void CDialogConfiguration_TC8226::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    GUIToConfig(0);
    CDialog::OnOK();
}

void CDialogConfiguration_TC8226::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    TCHAR *pEnd;
    unsigned int uFlashBaseAddress = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);

    if (pNMUpDown->iDelta == 1) {
        if ((uFlashBaseAddress + page_size) < m_uProgramMemorySize) {
            uFlashBaseAddress += page_size;
        }
    } else if (pNMUpDown->iDelta == -1) {
        if (!(uFlashBaseAddress <= page_size)) {
            uFlashBaseAddress -= page_size;
        }
    }

    uFlashBaseAddress = (uFlashBaseAddress & TC8226_FLASH_CONFIG_DFBA) / page_size * page_size;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable && (uFlashBaseAddress < m_uProgramMemorySize)) ? ((m_uProgramMemorySize - uFlashBaseAddress) / 1024.) : 0.);
    m_sConfigValue1.Format(_T("0x%08X"), uFlashBaseAddress);
    UpdateData(FALSE);
    *pResult = 0;
}

void CDialogConfiguration_TC8226::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}
