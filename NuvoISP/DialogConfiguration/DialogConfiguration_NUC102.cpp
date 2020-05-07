// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_NUC102.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_NUC102 dialog

CDialogConfiguration_NUC102::CDialogConfiguration_NUC102(CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_NUC102::IDD, pParent)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_NUC102)
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bClockFilterEnable = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_sFlashBaseAddress = _T("");
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_NUC102::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_NUC102)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Radio(pDX, IDC_RADIO_BOV_0, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_CLOCK_FILTER_ENABLE, m_bClockFilterEnable);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_NUC102, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_NUC102)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CLOCK_FILTER_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_NUC102 message handlers

BOOL CDialogConfiguration_NUC102::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    ConfigToGUI();
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_NUC102::ConfigToGUI()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];

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

    m_nRadioBS = ((uConfig0 & NUC1XX_FLASH_CONFIG_CBS) == 0 ? 0 : 1);
    m_bCheckBrownOutDetect = ((uConfig0 & NUC1XX_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & NUC1XX_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bClockFilterEnable = ((uConfig0 & NUC1XX_FLASH_CONFIG_CKF) == NUC1XX_FLASH_CONFIG_CKF ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_sFlashBaseAddress.Format(_T("%X"), m_ConfigValue.m_value[1]);
    m_sConfigValue0.Format(_T("0x%08X"), m_ConfigValue.m_value[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_ConfigValue.m_value[1]);
}

void CDialogConfiguration_NUC102::GUIToConfig()
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

    if (m_nRadioBS == 0) {
        uConfig0 &= ~NUC1XX_FLASH_CONFIG_CBS;
    } else {
        uConfig0 |= NUC1XX_FLASH_CONFIG_CBS;
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
    }

    if (m_bSecurityLock) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
    } else {
        uConfig0 |= NUMICRO_FLASH_CONFIG_LOCK;
    }

    //Removed if(m_bWatchDogEnable)
    uConfig0 |= NUC1XX_FLASH_CONFIG_CWDTEN;
    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;
}

void CDialogConfiguration_NUC102::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig();
    ConfigToGUI();
    UpdateData(FALSE);
}

void CDialogConfiguration_NUC102::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    GUIToConfig();
    CDialog::OnOK();
}

