// DialogConfiguration_OT8051.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "DialogConfiguration_OT8051.h"
#include <cassert>

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_OT8051 dialog

CDialogConfiguration_OT8051::CDialogConfiguration_OT8051(unsigned int uPartNo,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_OT8051::IDD, pParent)
    , m_uPartNo(uPartNo)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_OT8051)
    m_nRadio_FSYS	= -1;
    m_nRadio_RPD	= -1;
    m_nRadio_OCDPWM	= -1;
    m_nRadio_CBS	= -1;
    m_nRadio_LDSIZE	= -1;
    m_nRadio_CBOV	= -1;
    m_nRadio_WDTEN	= -1;
    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_sConfigValue2 = _T("");
    m_sConfigValue3 = _T("");
    m_sConfigValue4 = _T("");
    m_bSecurityLock = FALSE;
    m_bOCDEnable = FALSE;
    m_bCheckBrownOutEnable = TRUE;
    m_bCheckBrownOutReset = TRUE;
    m_bCheckBrownOutIAP = TRUE;
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_OT8051::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_OT8051)
    DDX_Radio(pDX, IDC_RADIO_FSYS_HIRC, m_nRadio_FSYS);
    DDX_Radio(pDX, IDC_RADIO_RPD_RESET, m_nRadio_RPD);
    DDX_Radio(pDX, IDC_RADIO_OCDPWM_TRI, m_nRadio_OCDPWM);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM, m_nRadio_CBS);
    DDX_Radio(pDX, IDC_RADIO_LDSIZE_0K, m_nRadio_LDSIZE);
    DDX_Radio(pDX, IDC_RADIO_BOV_7, m_nRadio_CBOV);
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE, m_nRadio_WDTEN);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0, m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1, m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2, m_sConfigValue2);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_3, m_sConfigValue3);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_4, m_sConfigValue4);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK, m_bSecurityLock);
    DDX_Check(pDX, IDC_CHECK_OCD_ENABLE, m_bOCDEnable);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_ENABLE, m_bCheckBrownOutEnable);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_IAP, m_bCheckBrownOutIAP);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET, m_bCheckBrownOutReset);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_OT8051, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_OT8051)
    ON_BN_CLICKED(IDC_RADIO_FSYS_HIRC, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_FSYS_LIRC, OnRadioClick)

    ON_BN_CLICKED(IDC_RADIO_RPD_RESET, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_RPD_INPUT, OnRadioClick)

    ON_BN_CLICKED(IDC_RADIO_OCDPWM_TRI, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_OCDPWM_CONTI, OnRadioClick)

    ON_BN_CLICKED(IDC_RADIO_BS_APROM, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM, OnRadioClick)

    ON_BN_CLICKED(IDC_RADIO_LDSIZE_0K, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_LDSIZE_1K, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_LDSIZE_2K, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_LDSIZE_3K, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_LDSIZE_4K, OnRadioClick)

    ON_BN_CLICKED(IDC_RADIO_BOV_7, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_0, OnRadioClick)

    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_STOP, OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP, OnRadioClick)

    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_OCD_ENABLE, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_ENABLE, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_IAP, OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET, OnCheckClick)

    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_OT8051 message handlers

BOOL CDialogConfiguration_OT8051::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    //UDACCEL pAccel[1];
    //pAccel[0].nInc = 1;
    //pAccel[0].nSec = 0;
    //m_SpinDataFlashSize.SetAccel(1, pAccel);
    unsigned int uSID = m_uPartNo & 0xFF00;

    switch (uSID) {
        case OT8051_SID_N76E885: {
            m_uLevel = 8;
            GetDlgItem(IDC_GROUP_FSYS)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_HIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_LIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_GROUP_RPD)->SetWindowText(_T("P1.2/RST Pin Function"));
            GetDlgItem(IDC_RADIO_RPD_RESET)->SetWindowText(_T("P1.2 as the external reset pin"));
            GetDlgItem(IDC_RADIO_RPD_INPUT)->SetWindowText(_T("P1.2 as the input-only pin"));
            GetDlgItem(IDC_RADIO_BOV_7)->SetWindowText(_T("1.7V"));
            GetDlgItem(IDC_RADIO_BOV_6)->SetWindowText(_T("2.0V"));
            GetDlgItem(IDC_RADIO_BOV_5)->SetWindowText(_T("2.2V"));
            GetDlgItem(IDC_RADIO_BOV_4)->SetWindowText(_T("2.4V"));
            GetDlgItem(IDC_RADIO_BOV_3)->SetWindowText(_T("2.7V"));
            GetDlgItem(IDC_RADIO_BOV_2)->SetWindowText(_T("3.0V"));
            GetDlgItem(IDC_RADIO_BOV_1)->SetWindowText(_T("3.7V"));
            GetDlgItem(IDC_RADIO_BOV_0)->SetWindowText(_T("4.3V"));
            break;
        }

        case OT8051_SID_N76E616: {
            m_uLevel = 4;
            GetDlgItem(IDC_GROUP_FSYS)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_HIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_LIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_GROUP_RPD)->SetWindowText(_T("P3.6/RST Pin Function"));
            GetDlgItem(IDC_RADIO_RPD_RESET)->SetWindowText(_T("P3.6 as the external reset pin"));
            GetDlgItem(IDC_RADIO_RPD_INPUT)->SetWindowText(_T("P3.6 as the input-only pin"));
            GetDlgItem(IDC_GROUP_OCDPWM)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_OCDPWM_TRI)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_OCDPWM_CONTI)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_7)->SetWindowText(_T("2.2V"));
            GetDlgItem(IDC_RADIO_BOV_6)->SetWindowText(_T("2.7V"));
            GetDlgItem(IDC_RADIO_BOV_5)->SetWindowText(_T("3.8V"));
            GetDlgItem(IDC_RADIO_BOV_4)->SetWindowText(_T("4.3V"));
            GetDlgItem(IDC_RADIO_BOV_3)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_2)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_1)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_0)->ShowWindow(SW_HIDE);
            break;
        }

        case OT8051_SID_N76E003:
        case OT8051_SID_MS51_16K:
        case OT8051_SID_MS51_8K:
        case OT8051_SID_MS51_32K: {
            m_uLevel = 4;
            GetDlgItem(IDC_GROUP_FSYS)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_HIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_LIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_GROUP_RPD)->SetWindowText(_T("P2.0/RST Pin Function"));
            GetDlgItem(IDC_RADIO_RPD_RESET)->SetWindowText(_T("P2.0 as the external reset pin"));
            GetDlgItem(IDC_RADIO_RPD_INPUT)->SetWindowText(_T("P2.0 as the input-only pin"));
            GetDlgItem(IDC_RADIO_BOV_7)->SetWindowText(_T("2.2V"));
            GetDlgItem(IDC_RADIO_BOV_6)->SetWindowText(_T("2.7V"));
            GetDlgItem(IDC_RADIO_BOV_5)->SetWindowText(_T("3.7V"));
            GetDlgItem(IDC_RADIO_BOV_4)->SetWindowText(_T("4.4V"));
            GetDlgItem(IDC_RADIO_BOV_3)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_2)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_1)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_0)->ShowWindow(SW_HIDE);
            break;
        }

        case OT8051_SID_N76L151: {
            m_uLevel = 8;
            GetDlgItem(IDC_GROUP_FSYS)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_HIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_LIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_GROUP_RPD)->SetWindowText(_T("P2.1/RST Pin Function"));
            GetDlgItem(IDC_RADIO_RPD_RESET)->SetWindowText(_T("P2.1 as the external reset pin"));
            GetDlgItem(IDC_RADIO_RPD_INPUT)->SetWindowText(_T("P2.1 as the input-only pin"));
            GetDlgItem(IDC_RADIO_BOV_7)->SetWindowText(_T("1.8V"));
            GetDlgItem(IDC_RADIO_BOV_6)->SetWindowText(_T("1.8V"));
            GetDlgItem(IDC_RADIO_BOV_5)->SetWindowText(_T("2.0V"));
            GetDlgItem(IDC_RADIO_BOV_4)->SetWindowText(_T("2.4V"));
            GetDlgItem(IDC_RADIO_BOV_3)->SetWindowText(_T("2.7V"));
            GetDlgItem(IDC_RADIO_BOV_2)->SetWindowText(_T("3.0V"));
            GetDlgItem(IDC_RADIO_BOV_1)->SetWindowText(_T("3.7V"));
            GetDlgItem(IDC_RADIO_BOV_0)->SetWindowText(_T("4.4V"));
            break;
        }

        case OT8051_SID_ML51_16K:
        case OT8051_SID_ML51_32K:
        case OT8051_SID_ML51_64K: {
            m_uLevel = 8;
            GetDlgItem(IDC_GROUP_FSYS)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_HIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_FSYS_LIRC)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_GROUP_RPD)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_RPD_RESET)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_RPD_INPUT)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_7)->SetWindowText(_T("Default 1.8V"));
            GetDlgItem(IDC_RADIO_BOV_6)->SetWindowText(_T("1.8V"));
            GetDlgItem(IDC_RADIO_BOV_5)->SetWindowText(_T("2.0V"));
            GetDlgItem(IDC_RADIO_BOV_4)->SetWindowText(_T("2.4V"));
            GetDlgItem(IDC_RADIO_BOV_3)->SetWindowText(_T("2.7V"));
            GetDlgItem(IDC_RADIO_BOV_2)->SetWindowText(_T("3.0V"));
            GetDlgItem(IDC_RADIO_BOV_1)->SetWindowText(_T("3.7V"));
            GetDlgItem(IDC_RADIO_BOV_0)->SetWindowText(_T("4.4V"));
            break;
        }

        case OT8051_SID_ML56_64K: {
            m_uLevel = 6;
            m_uPartNo |= 0x00100000;
            GetDlgItem(IDC_GROUP_RPD)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_RPD_RESET)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_RPD_INPUT)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_7)->SetWindowText(_T("Default 1.8V"));
            GetDlgItem(IDC_RADIO_BOV_6)->SetWindowText(_T("1.8V"));
            GetDlgItem(IDC_RADIO_BOV_5)->SetWindowText(_T("2.0V"));
            GetDlgItem(IDC_RADIO_BOV_4)->SetWindowText(_T("2.4V"));
            GetDlgItem(IDC_RADIO_BOV_3)->SetWindowText(_T("2.7V"));
            GetDlgItem(IDC_RADIO_BOV_2)->SetWindowText(_T("3.0V"));
            //GetDlgItem(IDC_RADIO_BOV_1)->SetWindowText(_T("3.7V"));
            //GetDlgItem(IDC_RADIO_BOV_0)->SetWindowText(_T("4.4V"));
            GetDlgItem(IDC_RADIO_BOV_1)->ShowWindow(SW_HIDE);
            GetDlgItem(IDC_RADIO_BOV_0)->ShowWindow(SW_HIDE);
            break;
        }
    }

    ConfigToGUI();
    UpdateData(FALSE);
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;	// return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_OT8051::ConfigToGUI()
{
    unsigned char ucConfig0 = m_ucConfigValue[0];
    unsigned char ucConfig1 = m_ucConfigValue[1];
    unsigned char ucConfig2 = m_ucConfigValue[2];
    unsigned char ucConfig3 = m_ucConfigValue[3];
    unsigned char ucConfig4 = m_ucConfigValue[4];
    m_bSecurityLock = ((ucConfig0 & OT8051_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_nRadio_RPD = ((ucConfig0 & OT8051_CONFIG_RPD) == 0 ? 1 : 0);
    m_bOCDEnable = ((ucConfig0 & OT8051_CONFIG_OCDEN) == 0 ? TRUE : FALSE);
    m_nRadio_OCDPWM = ((ucConfig0 & OT8051_CONFIG_OCDPWM) == 0 ? 1 : 0);
    m_nRadio_FSYS = ((ucConfig0 & OT8051_CONFIG_FSYS) == 0 ? 1 : 0);
    m_nRadio_CBS = ((ucConfig0 & OT8051_CONFIG_CBS) == 0 ? 1 : 0);

    switch (ucConfig1 & OT8051_CONFIG_LDSIZE) {
        case OT8051_CONFIG_LDSIZE_0K:
            m_nRadio_LDSIZE = 0;
            break;

        case OT8051_CONFIG_LDSIZE_1K:
            m_nRadio_LDSIZE = 1;
            break;

        case OT8051_CONFIG_LDSIZE_2K:
            m_nRadio_LDSIZE = 2;
            break;

        case OT8051_CONFIG_LDSIZE_3K:
            m_nRadio_LDSIZE = 3;
            break;

        default:
            m_nRadio_LDSIZE = 4;
            break;
    }

    m_bCheckBrownOutReset = ((ucConfig2 & OT8051_CONFIG_CBORST) != 0 ? TRUE : FALSE);
    m_bCheckBrownOutIAP = ((ucConfig2 & OT8051_CONFIG_BOIAP) != 0 ? TRUE : FALSE);

    if (m_uLevel == 8) {
        switch (ucConfig2 & OT8051_CONFIG_CBOV_8_LEVEL) {
            case OT8051_CONFIG_CBOV_7:
                m_nRadio_CBOV = 0;
                break;

            case OT8051_CONFIG_CBOV_6:
                m_nRadio_CBOV = 1;
                break;

            case OT8051_CONFIG_CBOV_5:
                m_nRadio_CBOV = 2;
                break;

            case OT8051_CONFIG_CBOV_4:
                m_nRadio_CBOV = 3;
                break;

            case OT8051_CONFIG_CBOV_3:
                m_nRadio_CBOV = 4;
                break;

            case OT8051_CONFIG_CBOV_2:
                m_nRadio_CBOV = 5;
                break;

            case OT8051_CONFIG_CBOV_1:
                m_nRadio_CBOV = 6;
                break;

            default:
                m_nRadio_CBOV = 7;
                break;
        }
    } else if (m_uLevel == 6) {
        switch (ucConfig2 & OT8051_CONFIG_CBOV_8_LEVEL) {
            case OT8051_CONFIG_CBOV_7:
                m_nRadio_CBOV = 0;
                break;

            case OT8051_CONFIG_CBOV_6:
                m_nRadio_CBOV = 1;
                break;

            case OT8051_CONFIG_CBOV_5:
                m_nRadio_CBOV = 2;
                break;

            case OT8051_CONFIG_CBOV_4:
                m_nRadio_CBOV = 3;
                break;

            case OT8051_CONFIG_CBOV_3:
                m_nRadio_CBOV = 4;
                break;

            case OT8051_CONFIG_CBOV_2:
                m_nRadio_CBOV = 5;
                break;

            default:
                m_nRadio_CBOV = 0;
        }
    } else {
        switch (ucConfig2 & OT8051_CONFIG_CBOV_4_LEVEL) {
            case OT8051_CONFIG_CBOV_3:
                m_nRadio_CBOV = 0;
                break;

            case OT8051_CONFIG_CBOV_2:
                m_nRadio_CBOV = 1;
                break;

            case OT8051_CONFIG_CBOV_1:
                m_nRadio_CBOV = 2;
                break;

            default:
                m_nRadio_CBOV = 3;
                break;
        }
    }

    if (((m_uPartNo & 0x00100000) && !(ucConfig2 & OT8051_CONFIG_CBODEN)) || (!(m_uPartNo & 0x00100000) && (ucConfig2 & OT8051_CONFIG_CBODEN))) {
        m_bCheckBrownOutEnable = TRUE;
    } else {
        m_bCheckBrownOutEnable = FALSE;
    }

    switch (ucConfig4 & OT8051_CONFIG_WDT) {
        case OT8051_CONFIG_WDT_DIS:
            m_nRadio_WDTEN = 0;
            break;

        case OT8051_CONFIG_WDT_STOP:
            m_nRadio_WDTEN = 1;
            break;

        default:
            m_nRadio_WDTEN = 2;
            break;
    }

    m_sConfigValue0.Format(_T("0x%02X"), ucConfig0);
    m_sConfigValue1.Format(_T("0x%02X"), ucConfig1);
    m_sConfigValue2.Format(_T("0x%02X"), ucConfig2);
    m_sConfigValue3.Format(_T("0x%02X"), ucConfig3);
    m_sConfigValue4.Format(_T("0x%02X"), ucConfig4);
}

void CDialogConfiguration_OT8051::GUIToConfig()
{
    unsigned char ucConfig0 = 0xFF;
    unsigned char ucConfig1 = 0xFF;
    unsigned char ucConfig2 = 0xFF;
    unsigned char ucConfig3 = 0xFF;
    unsigned char ucConfig4 = 0xFF;

    /* CONFIG 0 */
    if (m_bSecurityLock) {
        ucConfig0 &= ~OT8051_CONFIG_LOCK;
    }

    if (m_nRadio_RPD) {
        ucConfig0 &= ~OT8051_CONFIG_RPD;
    }

    if (m_bOCDEnable) {
        ucConfig0 &= ~OT8051_CONFIG_OCDEN;
    }

    if (m_nRadio_OCDPWM) {
        ucConfig0 &= ~OT8051_CONFIG_OCDPWM;
    }

    if (m_nRadio_FSYS) {
        ucConfig0 &= ~OT8051_CONFIG_FSYS;
    }

    if (m_nRadio_CBS) {
        ucConfig0 &= ~OT8051_CONFIG_CBS;
    }

    /* CONFIG 1 */
    switch (m_nRadio_LDSIZE) {
        case 0:
            ucConfig1 = OT8051_CONFIG_LDSIZE_0K;
            break;

        case 1:
            ucConfig1 = OT8051_CONFIG_LDSIZE_1K;
            break;

        case 2:
            ucConfig1 = OT8051_CONFIG_LDSIZE_2K;
            break;

        case 3:
            ucConfig1 = OT8051_CONFIG_LDSIZE_3K;
            break;

        default:
            ucConfig1 = OT8051_CONFIG_LDSIZE_4K;
            break;
    }

    ucConfig1 |= 0xF8;

    /* CONFIG 2 */
    if (m_uLevel == 8) {
        switch (m_nRadio_CBOV) {
            case 0:
                ucConfig2 = OT8051_CONFIG_CBOV_7;
                break;

            case 1:
                ucConfig2 = OT8051_CONFIG_CBOV_6;
                break;

            case 2:
                ucConfig2 = OT8051_CONFIG_CBOV_5;
                break;

            case 3:
                ucConfig2 = OT8051_CONFIG_CBOV_4;
                break;

            case 4:
                ucConfig2 = OT8051_CONFIG_CBOV_3;
                break;

            case 5:
                ucConfig2 = OT8051_CONFIG_CBOV_2;
                break;

            case 6:
                ucConfig2 = OT8051_CONFIG_CBOV_1;
                break;

            default:
                ucConfig2 = OT8051_CONFIG_CBOV_0;
                break;
        }

        ucConfig2 |= 0x8F;
    } else if (m_uLevel == 6) {
        switch (m_nRadio_CBOV) {
            case 0:
                ucConfig2 = OT8051_CONFIG_CBOV_7;
                break;

            case 1:
                ucConfig2 = OT8051_CONFIG_CBOV_6;
                break;

            case 2:
                ucConfig2 = OT8051_CONFIG_CBOV_5;
                break;

            case 3:
                ucConfig2 = OT8051_CONFIG_CBOV_4;
                break;

            case 4:
                ucConfig2 = OT8051_CONFIG_CBOV_3;
                break;

            case 5:
                ucConfig2 = OT8051_CONFIG_CBOV_2;
                break;

            default:
                ucConfig2 = OT8051_CONFIG_CBOV_7;
        }

        ucConfig2 |= 0x8F;
    } else {
        switch (m_nRadio_CBOV) {
            case 0:
                ucConfig2 = OT8051_CONFIG_CBOV_3;
                break;

            case 1:
                ucConfig2 = OT8051_CONFIG_CBOV_2;
                break;

            case 2:
                ucConfig2 = OT8051_CONFIG_CBOV_1;
                break;

            default:
                ucConfig2 = OT8051_CONFIG_CBOV_0;
                break;
        }

        ucConfig2 |= 0xCF;
    }

    if (!m_bCheckBrownOutReset) {
        ucConfig2 &= ~OT8051_CONFIG_CBORST;
    }

    if (!m_bCheckBrownOutIAP) {
        ucConfig2 &= ~OT8051_CONFIG_BOIAP;
    }

    if (((m_uPartNo & 0x00100000) && m_bCheckBrownOutEnable) || (!(m_uPartNo & 0x00100000) && !m_bCheckBrownOutEnable)) {
        ucConfig2 &= ~OT8051_CONFIG_CBODEN;
    }

    /* CONFIG 4 */
    switch (m_nRadio_WDTEN) {
        case 0:
            ucConfig4 = OT8051_CONFIG_WDT_DIS;
            break;

        case 1:
            ucConfig4 = OT8051_CONFIG_WDT_STOP;
            break;

        default:
            ucConfig4 = OT8051_CONFIG_WDT_RUN;
            break;
    }

    ucConfig4 |= 0x0F;
    m_ucConfigValue[0] = ucConfig0;
    m_ucConfigValue[1] = ucConfig1;
    m_ucConfigValue[2] = ucConfig2;
    m_ucConfigValue[3] = ucConfig3;
    m_ucConfigValue[4] = ucConfig4;
    m_ucConfigValue[5] = 0xFF;
    m_ucConfigValue[6] = 0xFF;
    m_ucConfigValue[7] = 0xFF;
}

void CDialogConfiguration_OT8051::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig();
    ConfigToGUI();
    UpdateData(FALSE);
}

void CDialogConfiguration_OT8051::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    GUIToConfig();
    ConfigToGUI();
    UpdateData(FALSE);
}

void CDialogConfiguration_OT8051::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    GUIToConfig();
    CDialog::OnOK();
}

void CDialogConfiguration_OT8051::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    //if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID())
    //	return;
    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}

