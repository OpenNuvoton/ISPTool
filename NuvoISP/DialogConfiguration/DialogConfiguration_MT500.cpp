// DialogConfiguration_MT500.cpp : 實作檔
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_MT500.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// CDialogConfiguration_MT500 對話方塊

CDialogConfiguration_MT500::CDialogConfiguration_MT500(BOOL bIsDataFlashFixed,
        unsigned int uProgramMemorySize,
        unsigned int uDataFlashSize,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_MT500::IDD, pParent)
    , m_bIsDataFlashFixed(bIsDataFlashFixed)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uDataFlashSize(uDataFlashSize)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_NUC1xx)
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bClockFilterEnable = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_bWDTEnable = FALSE;
    m_bWDTPowerDown = FALSE;
    m_bCHZ_Even0 = TRUE;
    m_bCHZ_Even1 = TRUE;
    m_BCHZ_Odd0 = TRUE;
    m_BCHZ_Odd1 = TRUE;
    m_BCHZ_BPWM = TRUE;
    m_sFlashBaseAddress = _T("");
    //}}AFX_DATA_INIT
}

//CDialogConfiguration_MT500::~CDialogConfiguration_MT500()
//{
//}

void CDialogConfiguration_MT500::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_NUC1xx)
    DDX_Control(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_DataFlashEnable);
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_CLOCK_FILTER_ENABLE, m_bClockFilterEnable);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_WDT_ENABLE, m_bWDTEnable);
    DDX_Check(pDX, IDC_CHECK_WDT_POWER_DOWN, m_bWDTPowerDown);
    DDX_Check(pDX, IDC_CHECK_CHZ_BPWM_Ctrl, m_BCHZ_BPWM);
    DDX_Check(pDX, IDC_CHECK_CHZ_Odd1_Ctrl, m_BCHZ_Odd1);
    DDX_Check(pDX, IDC_CHECK_CHZ_Even1_Ctrl, m_bCHZ_Even1);
    DDX_Check(pDX, IDC_CHECK_CHZ_Odd0_Ctrl, m_BCHZ_Odd0);
    DDX_Check(pDX, IDC_CHECK_CHZ_Even0_Ctrl, m_bCHZ_Even0);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_MT500, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_MT500)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CLOCK_FILTER_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_POWER_DOWN, OnCheckClickWDTPD)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CHZ_BPWM_Ctrl, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CHZ_Odd1_Ctrl, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CHZ_Even1_Ctrl, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CHZ_Odd0_Ctrl, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CHZ_Even0_Ctrl, OnButtonClick)
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

BOOL CDialogConfiguration_MT500::OnInitDialog()
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

void CDialogConfiguration_MT500::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];

    switch (uConfig0 & NUC1XX_FLASH_CONFIG_CBOV) {
        case NUC1XX_FLASH_CONFIG_CBOV_45:
            m_nRadioBov = 0;
            break;

        case NUC1XX_FLASH_CONFIG_CBOV_38:
            m_nRadioBov = 1;
            break;

        case NUC1XX_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 2;
            break;

        case NUC1XX_FLASH_CONFIG_CBOV_22:
        default:
            m_nRadioBov = 3;
            break;
    }

    //m_nRadioBS = ((uConfig0 & NUC1XX_FLASH_CONFIG_CBS) == 0 ? 0 : 1);
    switch (uConfig0 & NUC1XX_FLASH_CONFIG_CBS2) {
        case NUC1XX_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case NUC1XX_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case NUC1XX_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case NUC1XX_FLASH_CONFIG_CBS_AP_LD:
        default:
            m_nRadioBS = 3;
            break;
    }

    m_bWDTPowerDown = ((uConfig0 & NUC1XX_FLASH_CONFIG_CWDTPDEN) == 0 ? TRUE : FALSE);
    m_bWDTEnable = ((uConfig0 & NUC1XX_FLASH_CONFIG_CWDTEN) == 0 ? TRUE : FALSE);;

    if (!m_bWDTEnable) {
        m_bWDTPowerDown = FALSE;
    }

    m_bCheckBrownOutDetect = ((uConfig0 & NUC1XX_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & NUC1XX_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bClockFilterEnable = ((uConfig0 & NUC1XX_FLASH_CONFIG_CKF) == NUC1XX_FLASH_CONFIG_CKF ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_BCHZ_BPWM = ((uConfig0 & MT5XX_FLASH_CONFIG_CHZ_BPWM) == MT5XX_FLASH_CONFIG_CHZ_BPWM ? TRUE : FALSE);
    m_BCHZ_Odd1 = ((uConfig0 & MT5XX_FLASH_CONFIG_CHZ_Odd1) == MT5XX_FLASH_CONFIG_CHZ_Odd1 ? TRUE : FALSE);
    m_bCHZ_Even1 = ((uConfig0 & MT5XX_FLASH_CONFIG_CHZ_Event1) == MT5XX_FLASH_CONFIG_CHZ_Event1 ? TRUE : FALSE);
    m_BCHZ_Odd0 = ((uConfig0 & MT5XX_FLASH_CONFIG_CHZ_Odd0) == MT5XX_FLASH_CONFIG_CHZ_Odd0 ? TRUE : FALSE);
    m_bCHZ_Even0 = ((uConfig0 & MT5XX_FLASH_CONFIG_CHZ_Event0) == MT5XX_FLASH_CONFIG_CHZ_Event0 ? TRUE : FALSE);
    unsigned int uFlashBaseAddress = uConfig1 & 0xFFFFF;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);

    if (m_bIsDataFlashFixed) {
        m_sDataFlashSize.Format(_T("%.2fK"), m_uDataFlashSize / 1024.);
        m_SpinDataFlashSize.EnableWindow(FALSE);
    } else {
        unsigned int uPageNum = uFlashBaseAddress / NUMICRO_FLASH_PAGE_SIZE_512;
        unsigned int uLimitNum = m_uProgramMemorySize / NUMICRO_FLASH_PAGE_SIZE_512;
        unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * NUMICRO_FLASH_PAGE_SIZE_512) : 0;
        m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
        m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable ? TRUE : FALSE);
    }

    GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS)->EnableWindow(m_bDataFlashEnable && (!m_bIsDataFlashFixed));
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
}

void CDialogConfiguration_MT500::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1;
    uConfig0 &= ~NUC1XX_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= NUC1XX_FLASH_CONFIG_CBOV_45;
            break;

        case 1:
            uConfig0 |= NUC1XX_FLASH_CONFIG_CBOV_38;
            break;

        case 2:
            uConfig0 |= NUC1XX_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= NUC1XX_FLASH_CONFIG_CBOV_22;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & NUC1XX_FLASH_CONFIG_CBOV);
    }

    uConfig0 &= ~NUC1XX_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= NUC1XX_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= NUC1XX_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= NUC1XX_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= NUC1XX_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & NUC1XX_FLASH_CONFIG_CBS2);
    }

    if (m_bWDTPowerDown) {
        uConfig0 &= ~NUC1XX_FLASH_CONFIG_CWDTPDEN;
    } else {
        uConfig0 |= NUC1XX_FLASH_CONFIG_CWDTPDEN;
    }

    if (m_bWDTEnable) {
        uConfig0 &= ~NUC1XX_FLASH_CONFIG_CWDTEN;
    } else {
        uConfig0 |= NUC1XX_FLASH_CONFIG_CWDTEN;
    }

    if (nEventID == IDC_CHECK_WDT_POWER_DOWN) {
        if (m_bWDTPowerDown) {
            uConfig0 &= ~NUC1XX_FLASH_CONFIG_CWDTEN;
        }
    } else {
        if (!m_bWDTEnable) {
            uConfig0 |= NUC1XX_FLASH_CONFIG_CWDTPDEN;
        }
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~NUC1XX_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= NUC1XX_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~NUC1XX_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= NUC1XX_FLASH_CONFIG_CBORST;
    }

    if (m_bClockFilterEnable) {
        uConfig0 |= NUC1XX_FLASH_CONFIG_CKF;
    } else {
        uConfig0 &= ~NUC1XX_FLASH_CONFIG_CKF;
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

    if (m_BCHZ_BPWM) {
        uConfig0 |= MT5XX_FLASH_CONFIG_CHZ_BPWM;
    } else {
        uConfig0 &= ~MT5XX_FLASH_CONFIG_CHZ_BPWM;
    }

    if (m_BCHZ_Odd1) {
        uConfig0 |= MT5XX_FLASH_CONFIG_CHZ_Odd1;
    } else {
        uConfig0 &= ~MT5XX_FLASH_CONFIG_CHZ_Odd1;
    }

    if (m_bCHZ_Even1) {
        uConfig0 |= MT5XX_FLASH_CONFIG_CHZ_Event1;
    } else {
        uConfig0 &= ~MT5XX_FLASH_CONFIG_CHZ_Event1;
    }

    if (m_BCHZ_Odd0) {
        uConfig0 |= MT5XX_FLASH_CONFIG_CHZ_Odd0;
    } else {
        uConfig0 &= ~MT5XX_FLASH_CONFIG_CHZ_Odd0;
    }

    if (m_bCHZ_Even0) {
        uConfig0 |= MT5XX_FLASH_CONFIG_CHZ_Event0;
    } else {
        uConfig0 &= ~MT5XX_FLASH_CONFIG_CHZ_Event0;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;// | 0xFFF00000;
}

void CDialogConfiguration_MT500::OnGUIEvent(int nEventID)
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig(nEventID);
    ConfigToGUI(nEventID);
    UpdateData(FALSE);
}

void CDialogConfiguration_MT500::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent();
}

void CDialogConfiguration_MT500::OnCheckClickWDTPD()
{
    // TODO: Add your control notification handler code here
    OnGUIEvent(IDC_CHECK_WDT_POWER_DOWN);
}

void CDialogConfiguration_MT500::OnKillfocusEditFlashBaseAddress()
{
    UpdateData(TRUE);
    CDialogResize::OnKillfocusEditFlashBaseAddress(m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_512);
}

void CDialogConfiguration_MT500::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    OnKillfocusEditFlashBaseAddress();
    GUIToConfig(0);
    CDialog::OnOK();
}

// CDialogConfiguration_MT500 訊息處理常式

void CDialogConfiguration_MT500::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    CDialogResize::OnDeltaposSpinDataFlashSize(pNMHDR, pResult, m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_512);
}

void CDialogConfiguration_MT500::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}