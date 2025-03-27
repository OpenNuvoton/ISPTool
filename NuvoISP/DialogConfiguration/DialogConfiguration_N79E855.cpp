// DialogConfiguration_N79E855.cpp : implementation file
//

#include "stdafx.h"
#include <deque>
#include <string>
#include <utility>
#include "Lang.h"
#include "ChipDefs.h"
#include "DialogConfiguration_N79E855.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_N79E855 dialog

CDialogConfiguration_N79E855::CDialogConfiguration_N79E855(CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogConfiguration_N79E855::IDD, pParent)
    , m_DataFlashBase(16, 4)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_N79E855)
    m_nRadio_CBS        = -1;
    m_nRadio_CBOV       = -1;
    m_nRadio_OSCFS      = -1;
    m_nRadio_FOSC       = -1;

    m_bCheck_LOCK       = FALSE;
    m_bCheck_DFEN       = FALSE;
    m_bCheck_CBODEN     = FALSE;
    m_bCheck_CBORST     = TRUE;
    m_bCheck_CWDTEN     = FALSE;
    m_bCheck_CKF        = TRUE;

    m_sDataFlashBase    = _T("");
    m_sDataFlashSize    = _T("");
    m_sConfigValue0     = _T("");
    m_sConfigValue1     = _T("");
    m_sConfigValue2     = _T("");
    m_sConfigValue3     = _T("");
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_N79E855::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_N79E855)
    DDX_Radio(pDX, IDC_RADIO_BS_APROM,                  m_nRadio_CBS);
    DDX_Radio(pDX, IDC_RADIO_BOV_1,                     m_nRadio_CBOV);
    DDX_Radio(pDX, IDC_RADIO_OSCFS_22M,                 m_nRadio_OSCFS);
    DDX_Radio(pDX, IDC_RADIO_FOSC_HIRC,                 m_nRadio_FOSC);

    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK,             m_bCheck_LOCK);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE,         m_bCheck_DFEN);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_ENABLE,          m_bCheck_CBODEN);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET,           m_bCheck_CBORST);
    DDX_Check(pDX, IDC_CHECK_WDT_ENABLE,                m_bCheck_CWDTEN);
    DDX_Check(pDX, IDC_CHECK_CLOCK_FILTER_ENABLE,       m_bCheck_CKF);

    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,       m_DataFlashBase);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE,          m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE,          m_SpinDataFlashSize);

    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,          m_sDataFlashBase);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE,             m_sDataFlashSize);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0,            m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1,            m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2,            m_sConfigValue2);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_3,            m_sConfigValue3);
    //}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDialogConfiguration_N79E855, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_N79E855)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM,                   OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM,                   OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_0,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_OSCFS_22M,                  OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_OSCFS_11M,                  OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_FOSC_HIRC,                  OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_FOSC_XTAL_PIN,              OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_FOSC_HXT,                   OnRadioClick)

    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK,              OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_ENABLE,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET,            OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_WDT_ENABLE,                 OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_CLOCK_FILTER_ENABLE,        OnCheckClick)

    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,        OnKillfocusEditDataFlashBase)
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE,   OnDeltaposSpinDataFlashSize)

    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_N79E855 message handlers

BOOL CDialogConfiguration_N79E855::OnInitDialog()
{
    CDialog::OnInitDialog();

    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);

    ConfigToGUI();

    UpdateData(FALSE);

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;    // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_N79E855::ConfigToGUI()
{
    m_ucCONFIG_t[0] = m_ucCONFIG[0];
    m_ucCONFIG_t[1] = m_ucCONFIG[1];
    m_ucCONFIG_t[2] = m_ucCONFIG[2];
    m_ucCONFIG_t[3] = m_ucCONFIG[3];

    /* CONFIG 0 */
    m_nRadio_CBS    = ((m_ucCONFIG_t[0] & OT8051_CONFIG_CBS) != 0) ? 0 : 1;

    m_bCheck_LOCK   = ((m_ucCONFIG_t[0] & OT8051_CONFIG_LOCK) != 0) ? FALSE : TRUE;

    /* CONFIG 1 */
    CFG2GUI_DFEN();

    /* CONFIG 2 */
    m_bCheck_CBODEN = ((m_ucCONFIG_t[2] & N79E855_CONFIG_CBODEN) != 0) ? FALSE : TRUE;
    m_bCheck_CBORST = ((m_ucCONFIG_t[2] & N79E855_CONFIG_CBORST) != 0) ? TRUE : FALSE;

    switch (m_ucCONFIG_t[2] & N79E855_CONFIG_CBOV)
    {
        case N79E855_CONFIG_CBOV_1:
            m_nRadio_CBOV = 0;
            break;

        case N79E855_CONFIG_CBOV_0:
            m_nRadio_CBOV = 1;
            break;

        default:
            m_nRadio_CBOV = 0;
    }

    /* CONFIG 3 */
    m_bCheck_CWDTEN = ((m_ucCONFIG_t[3] & N79E855_CONFIG_CWDTEN) != 0) ? FALSE : TRUE;

    m_bCheck_CKF    = ((m_ucCONFIG_t[3] & N79E855_CONFIG_CKF) != 0) ? TRUE : FALSE;

    switch (m_ucCONFIG_t[3] & N79E855_CONFIG_OSCFS)
    {
        case N79E855_CONFIG_OSCFS_22M:
            m_nRadio_OSCFS = 0;
            break;

        case N79E855_CONFIG_OSCFS_11M:
            m_nRadio_OSCFS = 1;
            break;

        default:
            m_nRadio_OSCFS = 0;
    }

    switch (m_ucCONFIG_t[3] & N79E855_CONFIG_FOSC)
    {
        case N79E855_CONFIG_FOSC_HIRC:
            m_nRadio_FOSC = 0;
            break;

        case N79E855_CONFIG_FOSC_XTAL_PIN:
            m_nRadio_FOSC = 1;
            break;

        default:
            m_nRadio_FOSC = 2;
    }

    m_sConfigValue0.Format(_T("0x%02X"), m_ucCONFIG_t[0]);
    m_sConfigValue1.Format(_T("0x%02X"), m_ucCONFIG_t[1]);
    m_sConfigValue2.Format(_T("0x%02X"), m_ucCONFIG_t[2]);
    m_sConfigValue3.Format(_T("0x%02X"), m_ucCONFIG_t[3]);

    UpdateData(FALSE);
}

void CDialogConfiguration_N79E855::GUIToConfig()
{
    UpdateData(TRUE);

    m_ucCONFIG_t[0] = 0xFF;
    m_ucCONFIG_t[1] = m_ucCONFIG[1];
    m_ucCONFIG_t[2] = 0xFF;
    m_ucCONFIG_t[3] = 0xFF;

    /* CONFIG 0 */
    if (m_nRadio_CBS)
        m_ucCONFIG_t[0] &= ~OT8051_CONFIG_CBS;
    else
        m_ucCONFIG_t[0] |=  OT8051_CONFIG_CBS;

    if (m_bCheck_LOCK)
        m_ucCONFIG_t[0] &= ~OT8051_CONFIG_LOCK;
    else
        m_ucCONFIG_t[0] |=  OT8051_CONFIG_LOCK;

    /* CONFIG 1 */
    GUI2CFG_DFEN();

    /* CONFIG 2 */
    if (m_bCheck_CBODEN)
        m_ucCONFIG_t[2] &= ~N79E855_CONFIG_CBODEN;
    else
        m_ucCONFIG_t[2] |=  N79E855_CONFIG_CBODEN;

    if (m_bCheck_CBORST)
        m_ucCONFIG_t[2] |=  N79E855_CONFIG_CBORST;
    else
        m_ucCONFIG_t[2] &= ~N79E855_CONFIG_CBORST;

    m_ucCONFIG_t[2] &= ~N79E855_CONFIG_CBOV;

    switch (m_nRadio_CBOV)
    {
        case 0:
            m_ucCONFIG_t[2] |= N79E855_CONFIG_CBOV_1;
            break;

        case 1:
            m_ucCONFIG_t[2] |= N79E855_CONFIG_CBOV_0;
            break;

        default:
            m_ucCONFIG_t[2] |= N79E855_CONFIG_CBOV_1;
    }

    /* CONFIG 3 */
    if (m_bCheck_CWDTEN)
        m_ucCONFIG_t[3] &= ~N79E855_CONFIG_CWDTEN;
    else
        m_ucCONFIG_t[3] |=  N79E855_CONFIG_CWDTEN;

    if (m_bCheck_CKF)
        m_ucCONFIG_t[3] |=  N79E855_CONFIG_CKF;
    else
        m_ucCONFIG_t[3] &= ~N79E855_CONFIG_CKF;

    m_ucCONFIG_t[3] &= ~N79E855_CONFIG_OSCFS;

    switch (m_nRadio_OSCFS)
    {
        case 0:
            m_ucCONFIG_t[3] |= N79E855_CONFIG_OSCFS_22M;
            break;

        case 1:
            m_ucCONFIG_t[3] |= N79E855_CONFIG_OSCFS_11M;
            break;

        default:
            m_ucCONFIG_t[3] |= N79E855_CONFIG_OSCFS_22M;
    }

    m_ucCONFIG_t[3] &= ~N79E855_CONFIG_FOSC;

    switch (m_nRadio_FOSC)
    {
        case 0:
            m_ucCONFIG_t[3] |= N79E855_CONFIG_FOSC_HIRC;
            break;

        case 1:
            m_ucCONFIG_t[3] |= N79E855_CONFIG_FOSC_XTAL_PIN;
            break;

        default:
            m_ucCONFIG_t[3] |= N79E855_CONFIG_FOSC_HXT;
    }

    m_ucCONFIG[0] = m_ucCONFIG_t[0];
    m_ucCONFIG[1] = m_ucCONFIG_t[1];
    m_ucCONFIG[2] = m_ucCONFIG_t[2];
    m_ucCONFIG[3] = m_ucCONFIG_t[3];
}

void CDialogConfiguration_N79E855::CFG2GUI_DFEN()
{
    unsigned int uDataFlashBase, uDataFlashSize;

    m_bCheck_DFEN = ((m_ucCONFIG_t[0] & N79E855_CONFIG_DFEN) == 0) ? TRUE : FALSE;

    if (m_bCheck_DFEN)
    {
        m_SpinDataFlashSize.EnableWindow(TRUE);
        m_DataFlashBase.EnableWindow(TRUE);

        uDataFlashBase = (m_ucCONFIG_t[1] << 8) & ~(N79E855_DATAFLASH_DELTA_SIZE - 1);

        uDataFlashSize = N79E855_FLASH_APROM_SIZE - uDataFlashBase;

        if (uDataFlashSize > N79E855_FLASH_APROM_SIZE)
            uDataFlashSize = 0;
    }
    else
    {
        m_SpinDataFlashSize.EnableWindow(FALSE);
        m_DataFlashBase.EnableWindow(FALSE);
        uDataFlashBase = 0xFFFF;
        uDataFlashSize = 0;
    }

    m_sDataFlashBase.Format(_T("%X"), uDataFlashBase);
    m_sDataFlashSize.Format(_T("%.2fK"), uDataFlashSize / 1024.);
}

void CDialogConfiguration_N79E855::GUI2CFG_DFEN()
{
    if (m_bCheck_DFEN)
    {
        m_ucCONFIG_t[0] &= ~N79E855_CONFIG_DFEN;

        if (m_ucCONFIG_t[1] < 1)
            m_ucCONFIG_t[1] = 1;

        if (m_ucCONFIG_t[1] > ((N79E855_FLASH_APROM_SIZE - N79E855_DATAFLASH_DELTA_SIZE) >> 8))
            m_ucCONFIG_t[1] = ((N79E855_FLASH_APROM_SIZE - N79E855_DATAFLASH_DELTA_SIZE) >> 8);
    }
    else
    {
        m_ucCONFIG_t[0] |=  N79E855_CONFIG_DFEN;

        if (m_ucCONFIG_t[1] != 0xFF)
        {
            m_ucCONFIG_t[1]  = 0xFF;
        }
    }
}

void CDialogConfiguration_N79E855::OnRadioClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogConfiguration_N79E855::OnCheckClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogConfiguration_N79E855::OnKillfocusEditDataFlashBase()
{
    UpdateData(TRUE);

    unsigned int uDataFlashBase = ::_tcstoul(m_sDataFlashBase, NULL, 16);

    uDataFlashBase &= ~(N79E855_DATAFLASH_DELTA_SIZE - 1);

    if (uDataFlashBase < N79E855_DATAFLASH_DELTA_SIZE)
        uDataFlashBase = N79E855_DATAFLASH_DELTA_SIZE;

    if (uDataFlashBase > (N79E855_FLASH_APROM_SIZE - N79E855_DATAFLASH_DELTA_SIZE))
        uDataFlashBase = (N79E855_FLASH_APROM_SIZE - N79E855_DATAFLASH_DELTA_SIZE);

    m_ucCONFIG[1] = (uDataFlashBase >> 8);

    ConfigToGUI();
}

void CDialogConfiguration_N79E855::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);

    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);

    unsigned int uDataFlashBase = ::_tcstoul(m_sDataFlashBase, NULL, 16);

    uDataFlashBase &= ~(N79E855_DATAFLASH_DELTA_SIZE - 1);

    if (pNMUpDown->iDelta == 1)
    {
        uDataFlashBase += N79E855_DATAFLASH_DELTA_SIZE;
    }
    else if (pNMUpDown->iDelta == -1)
    {
        uDataFlashBase -= N79E855_DATAFLASH_DELTA_SIZE;
    }

    if (uDataFlashBase < N79E855_DATAFLASH_DELTA_SIZE)
        uDataFlashBase = N79E855_DATAFLASH_DELTA_SIZE;

    if (uDataFlashBase > (N79E855_FLASH_APROM_SIZE - N79E855_DATAFLASH_DELTA_SIZE))
        uDataFlashBase = (N79E855_FLASH_APROM_SIZE - N79E855_DATAFLASH_DELTA_SIZE);

    m_ucCONFIG[1] = (uDataFlashBase >> 8);

    ConfigToGUI();

    *pResult = 0;
}

void CDialogConfiguration_N79E855::OnOK()
{
    UpdateData(TRUE);

    GUIToConfig();

    CDialog::OnOK();
}

void CDialogConfiguration_N79E855::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    if ((pScrollBar != NULL) && (pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()))
        return;

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}
