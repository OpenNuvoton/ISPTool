// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_AU9100.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_NUC1xx dialog

CDialogConfiguration_AU9100::CDialogConfiguration_AU9100(unsigned int uProgramMemorySize,
        unsigned int uLDROM_Size,
        UINT nIDTemplate,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(nIDTemplate, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uLDROM_Size(uLDROM_Size)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_NUC1xx)
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_bLDROM_EN = FALSE;
    m_sFlashBaseAddress = _T("");
    m_uPageSize = AU91XX_FLASH_PAGE_SIZE;
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_AU9100::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_AU9100)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT, m_bCheckBrownOutDetect);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Check(pDX, IDC_CHECK_LDROM_EN, m_bLDROM_EN);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_AU9100, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_AU9100)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_LDROM_EN, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE, OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_AU9100 message handlers

BOOL CDialogConfiguration_AU9100::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);
    ConfigToGUI();
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_AU9100::ConfigToGUI()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];
    m_nRadioBS = ((uConfig0 & AU91XX_FLASH_CONFIG_CBS) == 0 ? 0 : 1);
    m_bCheckBrownOutDetect = ((uConfig0 & AU91XX_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bLDROM_EN = ((uConfig0 & AU91XX_FLASH_CONFIG_LDROM_EN) == 0 ? FALSE : TRUE);
    unsigned int uFlashBaseAddress = uConfig1 & 0xFFFFF;
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    unsigned int uPageNum = uFlashBaseAddress / m_uPageSize;
    unsigned int uLimitNum = (m_bLDROM_EN ? m_uProgramMemorySize : (m_uProgramMemorySize + m_uLDROM_Size)) / m_uPageSize;
    unsigned int uDataFlashSize = (uPageNum < uLimitNum) ? ((uLimitNum - uPageNum) * m_uPageSize) : m_uPageSize;
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable ? uDataFlashSize : 0) / 1024.);
    m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable ? TRUE : FALSE);
    GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS)->EnableWindow(m_bDataFlashEnable);

    if (m_bDataFlashEnable) {
        uFlashBaseAddress = uLimitNum * m_uPageSize - uDataFlashSize;
        m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
        uConfig1 = uFlashBaseAddress;
    }

    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
}

void CDialogConfiguration_AU9100::GUIToConfig()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1;

    if (m_nRadioBS == 0) {
        uConfig0 &= ~AU91XX_FLASH_CONFIG_CBS;
    } else {
        uConfig0 |= AU91XX_FLASH_CONFIG_CBS;
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~AU91XX_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= AU91XX_FLASH_CONFIG_CBODEN;
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

    if (m_bLDROM_EN) {
        uConfig0 |= AU91XX_FLASH_CONFIG_LDROM_EN;
    } else {
        uConfig0 &= ~AU91XX_FLASH_CONFIG_LDROM_EN;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;// | 0xFFF00000;
}

void CDialogConfiguration_AU9100::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig();
    ConfigToGUI();
    UpdateData(FALSE);
}

void CDialogConfiguration_AU9100::OnKillfocusEditFlashBaseAddress()
{
    UpdateData(TRUE);
    CDialogResize::OnKillfocusEditFlashBaseAddress(m_bDataFlashEnable, (m_bLDROM_EN ? m_uProgramMemorySize : (m_uProgramMemorySize + m_uLDROM_Size)), m_uPageSize);
}

void CDialogConfiguration_AU9100::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    OnKillfocusEditFlashBaseAddress();
    GUIToConfig();
    CDialog::OnOK();
}

void CDialogConfiguration_AU9100::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    CDialogResize::OnDeltaposSpinDataFlashSize(pNMHDR, pResult, m_bDataFlashEnable, (m_bLDROM_EN ? m_uProgramMemorySize : (m_uProgramMemorySize + m_uLDROM_Size)), m_uPageSize);
}

void CDialogConfiguration_AU9100::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_I9200
/////////////////////////////////////////////////////////////////////////////
CDialogConfiguration_I9200::CDialogConfiguration_I9200(unsigned int uProgramMemorySize, unsigned int uLDROM_Size, UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_AU9100(uProgramMemorySize, uLDROM_Size, nIDTemplate, pParent)
{
}

void CDialogConfiguration_I9200::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_I9200)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_LVOL_RSTEN, m_bCheckLowVolResetEnable);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_I9200, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_AU9100)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_LVOL_RSTEN, OnButtonClick)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE, OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

BOOL CDialogConfiguration_I9200::OnInitDialog()
{
    m_uPageSize = NUMICRO_FLASH_PAGE_SIZE_512;
    return CDialogConfiguration_AU9100::OnInitDialog();
}

void CDialogConfiguration_I9200::ConfigToGUI()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    m_bCheckLowVolResetEnable = ((uConfig0 & N570_FLASH_CONFIG_CLVR) == 0 ? TRUE : FALSE);
    CDialogConfiguration_AU9100::ConfigToGUI();
}

void CDialogConfiguration_I9200::GUIToConfig()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];

    if (m_bCheckLowVolResetEnable) {
        uConfig0 &= ~N570_FLASH_CONFIG_CLVR;
    } else {
        uConfig0 |= N570_FLASH_CONFIG_CLVR;
    }

    m_ConfigValue.m_value[0] = uConfig0;
    CDialogConfiguration_AU9100::GUIToConfig();
}