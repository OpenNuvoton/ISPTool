// DialogConfiguration.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_Mini51CN.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_Mini51CN dialog

CDialogConfiguration_Mini51CN::CDialogConfiguration_Mini51CN(unsigned int uProgramMemorySize,
        unsigned int uPID,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_Mini51CN::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uPID(uPID)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_Mini51)
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_bClockFilterEnable = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_sFlashBaseAddress = _T("");
    m_bCheckBrownOutReset = FALSE;
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_Mini51CN::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_Mini51CN)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
    DDX_Radio(pDX, IDC_RADIO_BOV_DISABLE, m_nRadioBov);
    DDX_Radio(pDX, IDC_RADIO_BS_LDROM, m_nRadioBS);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Check(pDX, IDC_CHECK_CLOCK_FILTER_ENABLE, m_bClockFilterEnable);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE, m_bDataFlashEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_Mini51CN, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_Mini51CN)
    ON_BN_CLICKED(IDC_RADIO_BOV_DISABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_CLOCK_FILTER_ENABLE, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
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
// CDialogConfiguration_Mini51 message handlers

BOOL CDialogConfiguration_Mini51CN::OnInitDialog()
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

void CDialogConfiguration_Mini51CN::ConfigToGUI()
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];
    unsigned int uFlashBaseAddress = uConfig1 & 0xFFFFF;

    if ((m_uPID & 0xFFFFFF00) == 0x00A05800) {
        switch (uConfig0 & (MINI51_FLASH_CONFIG_CBOD2VEN | MINI51_FLASH_CONFIG_CBOV)) {
            case (MINI51_FLASH_CONFIG_CBOD2VEN | MINI51_FLASH_CONFIG_CBOV):
                m_nRadioBov = 0;
                break;

            case MINI51_FLASH_CONFIG_CBOV_44:
                m_nRadioBov = 1;
                break;

            case (MINI51_FLASH_CONFIG_CBOD2VEN |MINI51_FLASH_CONFIG_CBOV_38):
            case MINI51_FLASH_CONFIG_CBOV_38:
                m_nRadioBov = 2;
                break;

            case (MINI51_FLASH_CONFIG_CBOD2VEN | MINI51_FLASH_CONFIG_CBOV_27):
            case (MINI51_FLASH_CONFIG_CBOD2VEN | MINI51_FLASH_CONFIG_CBOV_27_):
            case MINI51_FLASH_CONFIG_CBOV_27:
                m_nRadioBov = 3;
                break;

            case MINI51_FLASH_CONFIG_CBOV_27_:
                m_nRadioBov = 4;
                break;

            default:
                m_nRadioBov = 0;
                break;
        }
    } else {
        if (uConfig0 & MINI51_FLASH_CONFIG_CBOD2VEN) {
            m_nRadioBov = 0;
        } else {
            switch (uConfig0 & MINI51_FLASH_CONFIG_CBOV) {
                //case MINI51_FLASH_CONFIG_CBOV_DISABLE:
                //	m_nRadioBov = 0; break;
                case MINI51_FLASH_CONFIG_CBOV_44:
                    m_nRadioBov = 1;
                    break;

                case MINI51_FLASH_CONFIG_CBOV_38:
                    m_nRadioBov = 2;
                    break;

                case MINI51_FLASH_CONFIG_CBOV_27:
                    m_nRadioBov = 3;
                    break;

                case MINI51_FLASH_CONFIG_CBOV_27_:
                    m_nRadioBov = 4;
                    break;

                default:
                    m_nRadioBov = 2;
                    break;
            }
        }
    }

    //m_nRadioBS = ((uConfig0 & MINI51_FLASH_CONFIG_CBS) == 0 ? 0 : 1);

    switch (uConfig0 & MINI51_FLASH_CONFIG_CBS2) {
        case MINI51_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case MINI51_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case MINI51_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case MINI51_FLASH_CONFIG_CBS_AP_LD:
        default:
            m_nRadioBS = 3;
            break;
    }

    m_nRadioIO = ((uConfig0 & MINI51_FLASH_CONFIG_CIOINI) == 0 ? 1 : 0);
    m_bClockFilterEnable = ((uConfig0 & MINI51_FLASH_CONFIG_CKF) == MINI51_FLASH_CONFIG_CKF ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & MINI51_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);

    if (m_bDataFlashEnable) {
        uFlashBaseAddress = ((uFlashBaseAddress >= NUMICRO_FLASH_PAGE_SIZE_512) && (uFlashBaseAddress < m_uProgramMemorySize)) ? uFlashBaseAddress : (m_uProgramMemorySize - NUMICRO_FLASH_PAGE_SIZE_512);
        uFlashBaseAddress = (uFlashBaseAddress & MINI51_FLASH_CONFIG_DFBA) / NUMICRO_FLASH_PAGE_SIZE_512 * NUMICRO_FLASH_PAGE_SIZE_512;
        uConfig1 = uFlashBaseAddress;// | 0xFFF00000;
    }

    m_SpinDataFlashSize.EnableWindow(m_bDataFlashEnable);
    GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS)->EnableWindow(m_bDataFlashEnable);
    m_sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    m_sDataFlashSize.Format(_T("%.2fK"), (m_bDataFlashEnable && (uFlashBaseAddress < m_uProgramMemorySize)) ? ((m_uProgramMemorySize - uFlashBaseAddress) / 1024.) : 0.);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
}

void CDialogConfiguration_Mini51CN::GUIToConfig()
{
    //unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig0 = 0xFFFFFFFF;
    unsigned int uConfig1;
    uConfig0 &= ~MINI51_FLASH_CONFIG_CBOV;
    uConfig0 &= ~MINI51_FLASH_CONFIG_CBOD2VEN;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= MINI51_FLASH_CONFIG_CBOV_DISABLE;
            uConfig0 |= MINI51_FLASH_CONFIG_CBOD2VEN;
            break;

        case 1:
            uConfig0 |= MINI51_FLASH_CONFIG_CBOV_44;
            break;

        case 2:
            uConfig0 |= MINI51_FLASH_CONFIG_CBOV_38;
            break;

        case 3:
            uConfig0 |= MINI51_FLASH_CONFIG_CBOV_27;
            break;

        case 4:
            uConfig0 |= MINI51_FLASH_CONFIG_CBOV_27_;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & MINI51_FLASH_CONFIG_CBOV);
    }

    if (m_nRadioIO) {
        uConfig0 &= ~MINI51_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 |= MINI51_FLASH_CONFIG_CIOINI;
    }

    //if(m_nRadioBS == 0)
    //	uConfig0 &= ~MINI51_FLASH_CONFIG_CBS;
    //else
    //	uConfig0 |= MINI51_FLASH_CONFIG_CBS;
    uConfig0 &= ~MINI51_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= MINI51_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= MINI51_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= MINI51_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= MINI51_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & MINI51_FLASH_CONFIG_CBS2);
    }

    if (m_bClockFilterEnable) {
        uConfig0 |= MINI51_FLASH_CONFIG_CKF;
    } else {
        uConfig0 &= ~MINI51_FLASH_CONFIG_CKF;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~MINI51_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= MINI51_FLASH_CONFIG_CBORST;
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

    m_ConfigValue.m_value[0] = uConfig0;
    TCHAR *pEnd;
    uConfig1 = ::_tcstoul(m_sFlashBaseAddress, &pEnd, 16);
    m_ConfigValue.m_value[1] = uConfig1;// | 0xFFF00000;
}

void CDialogConfiguration_Mini51CN::OnButtonClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig();
    ConfigToGUI();
    UpdateData(FALSE);
}

void CDialogConfiguration_Mini51CN::OnKillfocusEditFlashBaseAddress()
{
    UpdateData(TRUE);
    CDialogResize::OnKillfocusEditFlashBaseAddress(m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_512);
}

void CDialogConfiguration_Mini51CN::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    OnKillfocusEditFlashBaseAddress();
    GUIToConfig();
    CDialog::OnOK();
}

void CDialogConfiguration_Mini51CN::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    CDialogResize::OnDeltaposSpinDataFlashSize(pNMHDR, pResult, m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_512);
}

void CDialogConfiguration_Mini51CN::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}
/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_Mini58
/////////////////////////////////////////////////////////////////////////////

CDialogConfiguration_Mini58::CDialogConfiguration_Mini58(unsigned int uProgramMemorySize, unsigned int uPID, CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_Mini51CN(uProgramMemorySize, uPID, pParent)
{
}

