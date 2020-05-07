// DialogConfiguration.cpp : implementation file

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "AppConfig.h"
#include "DialogConfiguration_NUC4xx.h"
#include <cassert>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_NUC4xx dialog
CDialogConfiguration_NUC4xx::CDialogConfiguration_NUC4xx(unsigned int uProgramMemorySize,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_NUC4xx::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_NUC4xx)
    m_nRadioBov = -1;
    m_nRadioBS = -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_sConfigValue2 = _T("");
    m_sConfigValue3 = _T("");
    m_bCheckBrownOutDetect = FALSE;
    m_bCheckBrownOutReset = FALSE;
    m_bDataFlashEnable = FALSE;
    m_bSecurityLock = FALSE;
    m_bWDTEnable = FALSE;
    m_bWDTPowerDown = FALSE;
    m_nRadioGPG = -1;
    m_nRadioGPG32K = -1;
    m_nRadioRMIIEnable = -1;
    m_nRadioIO = -1;
    m_sFlashBaseAddress = _T("");
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_NUC4xx::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_NUC4xx)
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_FlashBaseAddress);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE, m_SpinDataFlashSize);
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
    DDX_Radio(pDX, IDC_RADIO_GPG2_GPIO, m_nRadioGPG32K);
    DDX_Radio(pDX, IDC_RADIO_MII_MODE, m_nRadioRMIIEnable);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI, m_nRadioIO);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS, m_sFlashBaseAddress);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE, m_sDataFlashSize);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_NUC4xx, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_NUC4xx)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnButtonClick)

    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT, OnButtonClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS, OnKillfocusEditFlashBaseAddress)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnButtonClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_POWER_DOWN, OnCheckClickWDTPD)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_GPIO, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPG2_GPIO, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_MII_MODE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_RMII_MODE, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_CRYSTAL, OnButtonClick)
    ON_BN_CLICKED(IDC_RADIO_GPG2_32K, OnButtonClick)
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
// CDialogConfiguration_NUC4xx message handlers

BOOL CDialogConfiguration_NUC4xx::OnInitDialog()
{
    CDialog::OnInitDialog();
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);
    au32Config[0] = m_ConfigValue.m_value[0];
    au32Config[1] = m_ConfigValue.m_value[1];
    au32Config[2] = m_ConfigValue.m_value[2];
    au32Config[3] = m_ConfigValue.m_value[3];
    ConfigToGUI(0);
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;
}

void CDialogConfiguration_NUC4xx::ConfigToGUI(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1 = m_ConfigValue.m_value[1];

    switch (uConfig0 & NUC4XX_FLASH_CONFIG_CBOV) {
        case NUC4XX_FLASH_CONFIG_CBOV_45:
            m_nRadioBov = 0;
            break;

        case NUC4XX_FLASH_CONFIG_CBOV_38:
            m_nRadioBov = 1;
            break;

        case NUC4XX_FLASH_CONFIG_CBOV_26:
            m_nRadioBov = 2;
            break;

        case NUC4XX_FLASH_CONFIG_CBOV_22:
        default:
            m_nRadioBov = 3;
            break;
    }

    switch (uConfig0 & NUC4XX_FLASH_CONFIG_CBS2) {
        case NUC4XX_FLASH_CONFIG_CBS_LD:
            m_nRadioBS = 0;
            break;

        case NUC4XX_FLASH_CONFIG_CBS_AP:
            m_nRadioBS = 1;
            break;

        case NUC4XX_FLASH_CONFIG_CBS_LD_AP:
            m_nRadioBS = 2;
            break;

        case NUC4XX_FLASH_CONFIG_CBS_AP_LD:
        default:
            m_nRadioBS = 3;
            break;
    }

    m_nRadioGPG = ((uConfig0 & NUC4XX_FLASH_CONFIG_CGPFMFP) == 0 ? 0 : 1);
    m_nRadioGPG32K = ((uConfig0 & NUC4XX_FLASH_CONFIG_CFG32K) == 0 ? 0 : 1);
    m_nRadioRMIIEnable = ((uConfig0 & NUC4XX_FLASH_CONFIG_RMII) == 0 ? 0 : 1);
    m_nRadioIO = ((uConfig0 & NUC4XX_FLASH_CONFIG_CIOINI) == 0 ? 1 : 0);
    m_bWDTPowerDown = ((uConfig0 & NUC4XX_FLASH_CONFIG_CWDTPDEN) == 0 ? TRUE : FALSE);
    m_bWDTEnable = ((uConfig0 & NUC4XX_FLASH_CONFIG_CWDTEN) == 0 ? TRUE : FALSE);;

    if (!m_bWDTEnable) {
        m_bWDTPowerDown = FALSE;
    }

    m_bCheckBrownOutDetect = ((uConfig0 & NUC4XX_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckBrownOutReset = ((uConfig0 & NUC4XX_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bDataFlashEnable = ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    m_bSecurityLock = ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
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
    m_sConfigValue2.Format(_T("0x%08X"), m_ConfigValue.m_value[2]);
    m_sConfigValue3.Format(_T("0x%08X"), m_ConfigValue.m_value[3]);
}

void CDialogConfiguration_NUC4xx::GUIToConfig(int nEventID)
{
    unsigned int uConfig0 = m_ConfigValue.m_value[0];
    unsigned int uConfig1;
    uConfig0 &= ~NUC4XX_FLASH_CONFIG_CBOV;

    switch (m_nRadioBov) {
        case 0:
            uConfig0 |= NUC4XX_FLASH_CONFIG_CBOV_45;
            break;

        case 1:
            uConfig0 |= NUC4XX_FLASH_CONFIG_CBOV_38;
            break;

        case 2:
            uConfig0 |= NUC4XX_FLASH_CONFIG_CBOV_26;
            break;

        case 3:
            uConfig0 |= NUC4XX_FLASH_CONFIG_CBOV_22;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & NUC4XX_FLASH_CONFIG_CBOV);
    }

    uConfig0 &= ~NUC4XX_FLASH_CONFIG_CBS2;

    switch (m_nRadioBS) {
        case 0:
            uConfig0 |= NUC4XX_FLASH_CONFIG_CBS_LD;
            break;

        case 1:
            uConfig0 |= NUC4XX_FLASH_CONFIG_CBS_AP;
            break;

        case 2:
            uConfig0 |= NUC4XX_FLASH_CONFIG_CBS_LD_AP;
            break;

        case 3:
            uConfig0 |= NUC4XX_FLASH_CONFIG_CBS_AP_LD;
            break;

        default:
            /* Keep old value */
            uConfig0 |= (m_ConfigValue.m_value[0] & NUC4XX_FLASH_CONFIG_CBS2);
    }

    if (m_nRadioGPG == 0) {
        uConfig0 &= ~NUC4XX_FLASH_CONFIG_CGPFMFP;
    } else {
        uConfig0 |= NUC4XX_FLASH_CONFIG_CGPFMFP;
    }

    if (m_nRadioGPG32K == 0) {
        uConfig0 &= ~NUC4XX_FLASH_CONFIG_CFG32K;
    } else {
        uConfig0 |= NUC4XX_FLASH_CONFIG_CFG32K;
    }

    if (m_nRadioRMIIEnable == 0) {
        uConfig0 &= ~NUC4XX_FLASH_CONFIG_RMII;
    } else {
        uConfig0 |= NUC4XX_FLASH_CONFIG_RMII;
    }

    if (m_nRadioIO == 0) {
        uConfig0 |= NUC4XX_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~NUC4XX_FLASH_CONFIG_CIOINI;
    }

    if (m_bWDTPowerDown) {
        uConfig0 &= ~NUC4XX_FLASH_CONFIG_CWDTPDEN;
    } else {
        uConfig0 |= NUC4XX_FLASH_CONFIG_CWDTPDEN;
    }

    if (m_bWDTEnable) {
        uConfig0 &= ~NUC4XX_FLASH_CONFIG_CWDTEN;
    } else {
        uConfig0 |= NUC4XX_FLASH_CONFIG_CWDTEN;
    }

    if (nEventID == IDC_CHECK_WDT_POWER_DOWN) {
        if (m_bWDTPowerDown) {
            uConfig0 &= ~NUC4XX_FLASH_CONFIG_CWDTEN;
        }
    } else {
        if (!m_bWDTEnable) {
            uConfig0 |= NUC4XX_FLASH_CONFIG_CWDTPDEN;
        }
    }

    if (m_bCheckBrownOutDetect) {
        uConfig0 &= ~NUC4XX_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |= NUC4XX_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckBrownOutReset) {
        uConfig0 &= ~NUC4XX_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |= NUC4XX_FLASH_CONFIG_CBORST;
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
    au32Config[0] = m_ConfigValue.m_value[0];
    au32Config[1] = m_ConfigValue.m_value[1];
    au32Config[2] = m_ConfigValue.m_value[2];
}

void CDialogConfiguration_NUC4xx::OnGUIEvent(int nEventID)
{
    UpdateData(TRUE);
    GUIToConfig(nEventID);
    au32Config[3] = FMC_CRC8(au32Config, 3);
    m_ConfigValue.m_value[3] = au32Config[3];
    ConfigToGUI(nEventID);
    UpdateData(FALSE);
}

void CDialogConfiguration_NUC4xx::OnButtonClick()
{
    OnGUIEvent();
}

void CDialogConfiguration_NUC4xx::OnCheckClickWDTPD()
{
    OnGUIEvent(IDC_CHECK_WDT_POWER_DOWN);
}

void CDialogConfiguration_NUC4xx::OnKillfocusEditFlashBaseAddress()
{
    UpdateData(TRUE);
    CDialogResize::OnKillfocusEditFlashBaseAddress(m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_2K);
}

void CDialogConfiguration_NUC4xx::OnOK()
{
    UpdateData(TRUE);
    OnKillfocusEditFlashBaseAddress();
    GUIToConfig(0);
    au32Config[3] = FMC_CRC8(au32Config, 3);
    m_ConfigValue.m_value[3] = au32Config[3];
    CDialog::OnOK();
}

unsigned int CDialogConfiguration_NUC4xx::FMC_CRC8(unsigned int au32Data[], unsigned int i32Count)
{
    int         i32ByteIdx;
    unsigned char     i, u8Cnt, u8InData;
    unsigned char    au8CRC[4] = { 0xff, 0xff, 0xff, 0xff };

    for (i32ByteIdx = 0; i32ByteIdx < 4; i32ByteIdx++) {
        for (u8Cnt = 0; u8Cnt < i32Count; u8Cnt++) {
            for (i = 0x80; i != 0; i /= 2) {
                if ((au8CRC[i32ByteIdx] & 0x80) != 0) {
                    au8CRC[i32ByteIdx] *= 2;
                    au8CRC[i32ByteIdx] ^= 7;
                } else {
                    au8CRC[i32ByteIdx] *= 2;
                }

                u8InData = (au32Data[u8Cnt] >> (i32ByteIdx * 8)) & 0xff;

                if ((u8InData & i) != 0) {
                    au8CRC[i32ByteIdx] ^= 0x7;
                }
            }
        }
    }

    return (au8CRC[0] | au8CRC[1] << 8 | au8CRC[2] << 16 | au8CRC[3] << 24);
}

void CDialogConfiguration_NUC4xx::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    CDialogResize::OnDeltaposSpinDataFlashSize(pNMHDR, pResult, m_bDataFlashEnable, m_uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_2K);
}

void CDialogConfiguration_NUC4xx::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}