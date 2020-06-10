// DialogConfiguration_M0A21.cpp : implementation file
//

#include "stdafx.h"
#include <cassert>
#include <deque>
#include <string>
#include <utility>
#include "ChipDefs.h"
#include "NumEdit.h"
#include "DialogConfiguration_M0A21.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M0A21 dialog

CDialogConfiguration_M0A21::CDialogConfiguration_M0A21(unsigned int uProgramMemorySize,
        unsigned int uFlashPageSize,
        CWnd *pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
    , m_uFlashPageSize(uFlashPageSize)
    , m_ALOCK(16, 2)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_M0A21)
    m_nRadioCWDTEN	= -1;
    m_nRadioCFGXT1	= -1;
    m_nRadioCFGRPS	= -1;
    m_nRadioCBOV	= -1;
    m_nRadioCIOINI	= -1;
    m_nRadioRSTEXT	= -1;
    m_nRadioRSTWSEL	= -1;
    m_nRadioCBS		= -1;
    m_bCheckCBORST	= FALSE;
    m_bCheckCBODEN	= FALSE;
    m_bCheckICELOCK	= FALSE;
    m_bCheckLOCK	= FALSE;
    m_bCheckDFEN	= FALSE;
    m_sDataFlashBase = _T("");
    m_sDataFlashSize = _T("");
    m_sConfigValue0	= _T("");
    m_sConfigValue1	= _T("");
    m_sConfigValue2	= _T("");
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_M0A21::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogConfiguration_M0A21)
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE,			m_nRadioCWDTEN);
    DDX_Radio(pDX, IDC_RADIO_GPF_CRYSTAL,			m_nRadioCFGXT1);
    DDX_Radio(pDX, IDC_RADIO_RPD_RESET,				m_nRadioCFGRPS);
    DDX_Radio(pDX, IDC_RADIO_BOV_3,					m_nRadioCBOV);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI,				m_nRadioCIOINI);
    DDX_Radio(pDX, IDC_RADIO_CHIPRESET_TIMEEXT_1,	m_nRadioRSTEXT);
    DDX_Radio(pDX, IDC_RADIO_RST_PIN_WIDTH_1,		m_nRadioRSTWSEL);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM,				m_nRadioCBS);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET,		m_bCheckCBORST);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT,		m_bCheckCBODEN);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK,				m_bCheckICELOCK);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK,			m_bCheckLOCK);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE,		m_bCheckDFEN);
    DDX_Control(pDX, IDC_EDIT_FLASH_ADVANCE_LOCK,	m_ALOCK);
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,	m_DataFlashBase);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE,		m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE,		m_SpinDataFlashSize);
    DDX_Text(pDX, IDC_EDIT_FLASH_ADVANCE_LOCK,		m_sALOCK);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,		m_sDataFlashBase);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE,			m_sDataFlashSize);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0,		m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1,		m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2,		m_sConfigValue2);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogConfiguration_M0A21, CDialog)
    //{{AFX_MSG_MAP(CDialogConfiguration_M0A21)
    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP,			OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_STOP,			OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_CRYSTAL,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_GPF_GPIO,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_RPD_RESET,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_RPD_INPUT,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_0,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_CHIPRESET_TIMEEXT_1,		OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_CHIPRESET_TIMEEXT_0,		OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_RST_PIN_WIDTH_1,			OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_RST_PIN_WIDTH_0,			OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM,				OnRadioClick)

    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK,					OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,			OnCheckClick)

    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_ADVANCE_LOCK,		OnKillfocusEditAdvanceLock)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,		OnKillfocusEditDataFlashBase)
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE,	OnDeltaposSpinDataFlashSize)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M0A21 message handlers

BOOL CDialogConfiguration_M0A21::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    UpdateUI();
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);
    ConfigToGUI();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogConfiguration_M0A21::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogConfiguration_M0A21::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogConfiguration_M0A21::UpdateUI()
{
    // TODO: Add your control notification handler code here
}

void CDialogConfiguration_M0A21::ConfigToGUI()
{
    unsigned int uConfig0 = m_uConfigValue[0];
    unsigned int uConfig1 = m_uConfigValue[1];
    unsigned int uConfig2 = m_uConfigValue[2];

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN:
            if (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTPDEN) {
                m_nRadioCWDTEN = 2;
            } else {
                m_nRadioCWDTEN = 1;
            }

            break;

        default:
            m_nRadioCWDTEN = 1;
    }

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CBOV_4_LEVEL) {
        case NUMICRO_FLASH_CONFIG_CBOV_3:
            m_nRadioCBOV = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_2:
            m_nRadioCBOV = 1;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_1:
            m_nRadioCBOV = 2;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_0:
        default:
            m_nRadioCBOV = 3;
            break;
    }

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CBS_4_MODE) {
        case NUMICRO_FLASH_CONFIG_CBS_AP:
            m_nRadioCBS = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_LD:
            m_nRadioCBS = 1;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_AP_IAP:
            m_nRadioCBS = 2;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_LD_IAP:
        default:
            m_nRadioCBS = 3;
            break;
    }

    m_nRadioCFGXT1	= ((uConfig0 & M0A21_FLASH_CONFIG_CFGXT1) ? 0 : 1);
    m_nRadioCFGRPS	= ((uConfig0 & M0A21_FLASH_CONFIG_CFGRPS) ? 0 : 1);
    m_nRadioCIOINI	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CIOINI) ? 0 : 1);
    m_nRadioRSTEXT	= ((uConfig0 & M0A21_FLASH_CONFIG_RSTEXT) ? 0 : 1);
    m_nRadioRSTWSEL	= ((uConfig0 & M0A21_FLASH_CONFIG_RSTWSEL) ? 0 : 1);
    m_bCheckCBORST	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckCBODEN	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckICELOCK	= ((uConfig0 & M0A21_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    m_bCheckLOCK	= ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bCheckDFEN	= ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    unsigned int uDataFlashBase, uDataFlashSize;

    if (m_bCheckDFEN) {
        m_SpinDataFlashSize.EnableWindow(TRUE);
        m_DataFlashBase.EnableWindow(TRUE);
        uDataFlashBase = uConfig1 & ~(m_uFlashPageSize - 1);
        uDataFlashSize = m_uProgramMemorySize - uDataFlashBase;

        if (uDataFlashSize > m_uProgramMemorySize) {
            uDataFlashSize = 0;
        }
    } else {
        m_SpinDataFlashSize.EnableWindow(FALSE);
        m_DataFlashBase.EnableWindow(FALSE);
        uDataFlashBase = 0xFFFFFFFF;
        uDataFlashSize = 0;
    }

    m_sDataFlashBase.Format(_T("%X"), uConfig1);
    m_sDataFlashSize.Format(_T("%.2fK"), uDataFlashSize / 1024.);
    m_sALOCK.Format(_T("%02X"), uConfig2 & 0xFF);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
    m_sConfigValue2.Format(_T("0x%08X"), uConfig2);
    UpdateData(FALSE);
}

void CDialogConfiguration_M0A21::GUIToConfig()
{
    UpdateData(TRUE);
    unsigned int uConfig0 = 0xFFFFFFFF;
    unsigned int uConfig1 = m_uConfigValue[1];
    unsigned int uConfig2 = m_uConfigValue[2];

    switch (m_nRadioCWDTEN) {
        case 0:
            uConfig0 &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            uConfig0 |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        case 2:
            uConfig0 &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            uConfig0 |= (NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            break;

        default:
            if (((uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE) ||
                    (((uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN) && (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTPDEN))) {
                uConfig0 &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
                uConfig0 |= (NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            }
    }

    uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBOV_4_LEVEL;

    switch (m_nRadioCBOV) {
        case 0:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_3;
            break;

        case 1:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_2;
            break;

        case 2:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_1;
            break;

        case 3:
        default:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_0;
            break;
    }

    uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBS_4_MODE;

    switch (m_nRadioCBS) {
        case 0:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_AP;
            break;

        case 1:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_LD;
            break;

        case 2:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
            break;

        case 3:
        default:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
            break;
    }

    if (m_nRadioCFGXT1 == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_CFGXT1;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_CFGXT1;
    }

    if (m_nRadioCFGRPS == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_CFGRPS;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_CFGRPS;
    }

    if (m_nRadioCIOINI == 0) {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CIOINI;
    }

    if (m_nRadioRSTEXT == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_RSTEXT;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_RSTEXT;
    }

    if (m_nRadioRSTWSEL == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_RSTWSEL;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_RSTWSEL;
    }

    if (m_bCheckCBORST) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CBORST;
    }

    if (m_bCheckCBODEN) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckICELOCK) {
        uConfig0 &= ~M0A21_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |=  M0A21_FLASH_CONFIG_ICELOCK;
    }

    if (m_bCheckDFEN) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_DFEN;
        uConfig1 &= ~(m_uFlashPageSize - 1);

        if (uConfig1 < m_uFlashPageSize) {
            uConfig1 = m_uFlashPageSize;
        }

        if (uConfig1 > (m_uProgramMemorySize - m_uFlashPageSize)) {
            uConfig1 = (m_uProgramMemorySize - m_uFlashPageSize);
        }
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_DFEN;

        if (uConfig1 != 0xFFFFFFFF) {
            uConfig1  = 0xFFFFFFFF;
        }
    }

    bool bEdit = false;

    if (m_bCheckLOCK) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;

        if ((uConfig2 & 0xFF) == 0x5A) {
            uConfig2 = 0xFFFFFF00;
            bEdit = true;
        }
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_LOCK;

        if ((uConfig2 & 0xFF) != 0x5A) {
            uConfig2 = 0xFFFFFF5A;
            bEdit = true;
        }
    }

    if (!bEdit) {
        unsigned int uALOCK = ::_tcstoul(m_sALOCK, NULL, 16);
        uConfig2 = 0xFFFFFF00 | uALOCK;

        if (uALOCK == 0x5A) {
            uConfig0 |=  NUMICRO_FLASH_CONFIG_LOCK;
        } else {
            uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
        }
    }

    m_uConfigValue[0] = uConfig0;
    m_uConfigValue[1] = uConfig1;
    m_uConfigValue[2] = uConfig2;
}

void CDialogConfiguration_M0A21::OnKillfocusEditAdvanceLock()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogConfiguration_M0A21::OnKillfocusEditDataFlashBase()
{
    UpdateData(TRUE);
    unsigned int uDataFlashBase = ::_tcstoul(m_sDataFlashBase, NULL, 16);
    uDataFlashBase &= ~(m_uFlashPageSize - 1);

    if (uDataFlashBase < m_uFlashPageSize) {
        uDataFlashBase = m_uFlashPageSize;
    }

    if (uDataFlashBase > (m_uProgramMemorySize - m_uFlashPageSize)) {
        uDataFlashBase = (m_uProgramMemorySize - m_uFlashPageSize);
    }

    m_uConfigValue[1] = uDataFlashBase;
    ConfigToGUI();
}

void CDialogConfiguration_M0A21::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);
    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
    unsigned int uDataFlashBase = ::_tcstoul(m_sDataFlashBase, NULL, 16);
    uDataFlashBase &= ~(m_uFlashPageSize - 1);

    if (pNMUpDown->iDelta == 1) {
        uDataFlashBase += m_uFlashPageSize;
    } else if (pNMUpDown->iDelta == -1) {
        uDataFlashBase -= m_uFlashPageSize;
    }

    if (uDataFlashBase < m_uFlashPageSize) {
        uDataFlashBase = m_uFlashPageSize;
    }

    if (uDataFlashBase > (m_uProgramMemorySize - m_uFlashPageSize)) {
        uDataFlashBase = (m_uProgramMemorySize - m_uFlashPageSize);
    }

    m_uConfigValue[1] = uDataFlashBase;
    ConfigToGUI();
    *pResult = 0;
}

void CDialogConfiguration_M0A21::OnOK()
{
    // TODO: Add extra validation here
    UpdateData(TRUE);
    GUIToConfig();
    CDialog::OnOK();
}

void CDialogConfiguration_M0A21::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    if (pScrollBar != NULL && pScrollBar->GetDlgCtrlID() == m_SpinDataFlashSize.GetDlgCtrlID()) {
        return;
    }

    CDialogResize::OnVScroll(nSBCode, nPos, pScrollBar);
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M030G dialog

CDialogConfiguration_M030G::CDialogConfiguration_M030G(unsigned int uProgramMemorySize,
        unsigned int uFlashPageSize,
        CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M0A21(uProgramMemorySize, uFlashPageSize, pParent)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_M030G)
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_M030G::UpdateUI()
{
    // TODO: Add your control notification handler code here
    GetDlgItem(IDC_GROUP_RPD)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RPD_RESET)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RPD_INPUT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_GROUP_GPF)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_GPF_CRYSTAL)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_GPF_GPIO)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_GROUP_BROWN_OUT_VOLTAGE)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_BOV_3)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_BOV_2)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_BOV_1)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_BOV_0)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_CHECK_BROWN_OUT_DETECT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_CHECK_BROWN_OUT_RESET)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_WDT_ENABLE_STOP)->ShowWindow(SW_HIDE);
    RECT rcTmp, rcGroupRSTW, rcGroupCFGXT1, rcGroupCBOD;
    LONG lDiff;
    GetDlgItem(IDC_GROUP_RST_PIN_WIDTH)->GetWindowRect(&rcGroupRSTW);
    GetDlgItem(IDC_GROUP_GPF)->GetWindowRect(&rcGroupCFGXT1);
    GetDlgItem(IDC_GROUP_BROWN_OUT_VOLTAGE)->GetWindowRect(&rcGroupCBOD);
    lDiff = rcGroupCFGXT1.bottom - rcGroupRSTW.bottom;
    int i;
    int nIDs[] = {IDC_GROUP_BOOT_SELECT, IDC_RADIO_BS_APROM, IDC_RADIO_BS_LDROM, IDC_RADIO_BS_APROM_LDROM, IDC_RADIO_BS_LDROM_APROM};

    for (i = 0; i < _countof(nIDs); i++) {
        GetDlgItem(nIDs[i])->GetWindowRect(&rcTmp);
        this->ScreenToClient(&rcTmp);
        GetDlgItem(nIDs[i])->SetWindowPos(NULL, rcTmp.left, rcTmp.top - lDiff, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
    }

    lDiff = rcGroupCBOD.bottom - rcGroupCFGXT1.bottom;
    int nID1s[] = { IDOK, IDCANCEL, IDC_GROUP_WDT, IDC_RADIO_WDT_DISABLE, IDC_RADIO_WDT_ENABLE_KEEP, IDC_RADIO_WDT_ENABLE_STOP,
                    IDC_GROUP_DATA_FLASH, IDC_CHECK_DATA_FLASH_ENABLE, IDC_STATIC_FLASH_BASE_ADDRESS, IDC_EDIT_FLASH_BASE_ADDRESS, IDC_STATIC_DATA_FLASH_SIZE, IDC_EDIT_DATA_FLASH_SIZE, IDC_SPIN_DATA_FLASH_SIZE,
                    IDC_GROUP_ADVANCE_LOCK, IDC_CHECK_SECURITY_LOCK, IDC_STATIC_FLASH_ADVANCE_LOCK, IDC_EDIT_FLASH_ADVANCE_LOCK,
                    IDC_GROUP_CONFIG_VALUE, IDC_STATIC_CONFIG_0, IDC_STATIC_CONFIG_VALUE_0, IDC_STATIC_CONFIG_1, IDC_STATIC_CONFIG_VALUE_1, IDC_STATIC_CONFIG_2, IDC_STATIC_CONFIG_VALUE_2
                  };

    for (i = 0; i < _countof(nID1s); i++) {
        GetDlgItem(nID1s[i])->GetWindowRect(&rcTmp);
        this->ScreenToClient(&rcTmp);
        GetDlgItem(nID1s[i])->SetWindowPos(NULL, rcTmp.left, rcTmp.top - lDiff, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
    }

    this->GetWindowRect(&rcTmp);
    SetWindowPos(this, 0, 0, rcTmp.right - rcTmp.left, rcTmp.bottom - rcTmp.top - lDiff, SWP_NOZORDER | SWP_NOMOVE);
}

void CDialogConfiguration_M030G::ConfigToGUI()
{
    unsigned int uConfig0 = m_uConfigValue[0];
    unsigned int uConfig1 = m_uConfigValue[1];
    unsigned int uConfig2 = m_uConfigValue[2];

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        default:
            m_nRadioCWDTEN = 1;
    }

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CBS_4_MODE) {
        case NUMICRO_FLASH_CONFIG_CBS_AP:
            m_nRadioCBS = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_LD:
            m_nRadioCBS = 1;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_AP_IAP:
            m_nRadioCBS = 2;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_LD_IAP:
        default:
            m_nRadioCBS = 3;
            break;
    }

    m_nRadioCIOINI	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CIOINI) ? 0 : 1);
    m_nRadioRSTEXT	= ((uConfig0 & M0A21_FLASH_CONFIG_RSTEXT) ? 0 : 1);
    m_nRadioRSTWSEL	= ((uConfig0 & M0A21_FLASH_CONFIG_RSTWSEL) ? 0 : 1);
    m_bCheckICELOCK	= ((uConfig0 & M0A21_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    m_bCheckLOCK	= ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bCheckDFEN	= ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    unsigned int uDataFlashBase, uDataFlashSize;

    if (m_bCheckDFEN) {
        m_SpinDataFlashSize.EnableWindow(TRUE);
        m_DataFlashBase.EnableWindow(TRUE);
        uDataFlashBase = uConfig1 & ~(m_uFlashPageSize - 1);
        uDataFlashSize = m_uProgramMemorySize - uDataFlashBase;

        if (uDataFlashSize > m_uProgramMemorySize) {
            uDataFlashSize = 0;
        }
    } else {
        m_SpinDataFlashSize.EnableWindow(FALSE);
        m_DataFlashBase.EnableWindow(FALSE);
        uDataFlashSize = 0;
    }

    m_sDataFlashBase.Format(_T("%X"), uConfig1);
    m_sDataFlashSize.Format(_T("%.2fK"), uDataFlashSize / 1024.);
    m_sALOCK.Format(_T("%02X"), uConfig2 & 0xFF);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
    m_sConfigValue2.Format(_T("0x%08X"), uConfig2);
    UpdateData(FALSE);
}

void CDialogConfiguration_M030G::GUIToConfig()
{
    UpdateData(TRUE);
    unsigned int uConfig0 = 0xFFFFFFFF;
    unsigned int uConfig1 = m_uConfigValue[1];
    unsigned int uConfig2 = m_uConfigValue[2];

    switch (m_nRadioCWDTEN) {
        case 0:
            uConfig0 &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            uConfig0 |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        default:
            if ((uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE) {
                uConfig0 &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
                uConfig0 |=  NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE;
            }
    }

    uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBS_4_MODE;

    switch (m_nRadioCBS) {
        case 0:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_AP;
            break;

        case 1:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_LD;
            break;

        case 2:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
            break;

        case 3:
        default:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
            break;
    }

    if (m_nRadioCIOINI == 0) {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CIOINI;
    }

    if (m_nRadioRSTEXT == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_RSTEXT;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_RSTEXT;
    }

    if (m_nRadioRSTWSEL == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_RSTWSEL;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_RSTWSEL;
    }

    if (m_bCheckICELOCK) {
        uConfig0 &= ~M0A21_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |=  M0A21_FLASH_CONFIG_ICELOCK;
    }

    if (m_bCheckDFEN) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_DFEN;
        uConfig1 &= ~(m_uFlashPageSize - 1);

        if (uConfig1 < m_uFlashPageSize) {
            uConfig1 = m_uFlashPageSize;
        }

        if (uConfig1 > (m_uProgramMemorySize - m_uFlashPageSize)) {
            uConfig1 = (m_uProgramMemorySize - m_uFlashPageSize);
        }
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_DFEN;

        if (uConfig1 != 0xFFFFFFFF) {
            uConfig1  = 0xFFFFFFFF;
        }
    }

    bool bEdit = false;

    if (m_bCheckLOCK) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;

        if ((uConfig2 & 0xFF) == 0x5A) {
            uConfig2 = 0xFFFFFF00;
            bEdit = true;
        }
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_LOCK;

        if ((uConfig2 & 0xFF) != 0x5A) {
            uConfig2 = 0xFFFFFF5A;
            bEdit = true;
        }
    }

    if (!bEdit) {
        unsigned int uALOCK = ::_tcstoul(m_sALOCK, NULL, 16);
        uConfig2 = 0xFFFFFF00 | uALOCK;

        if (uALOCK == 0x5A) {
            uConfig0 |=  NUMICRO_FLASH_CONFIG_LOCK;
        } else {
            uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
        }
    }

    m_uConfigValue[0] = uConfig0;
    m_uConfigValue[1] = uConfig1;
    m_uConfigValue[2] = uConfig2;
}

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M031 dialog

CDialogConfiguration_M031::CDialogConfiguration_M031(unsigned int uProgramMemorySize,
        unsigned int uFlashPageSize,
        CWnd *pParent /*=NULL*/)
    : CDialogConfiguration_M0A21(uProgramMemorySize, uFlashPageSize, pParent)
{
    //{{AFX_DATA_INIT(CDialogConfiguration_M031)
    //}}AFX_DATA_INIT
}

void CDialogConfiguration_M031::UpdateUI()
{
    // TODO: Add your control notification handler code here
    GetDlgItem(IDC_GROUP_RPD)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RPD_RESET)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RPD_INPUT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_BOV_3)->SetWindowText(_T("2.5V"));
    GetDlgItem(IDC_RADIO_BOV_2)->SetWindowText(_T("2.0V"));
    GetDlgItem(IDC_RADIO_BOV_1)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_BOV_0)->ShowWindow(SW_HIDE);
}

void CDialogConfiguration_M031::ConfigToGUI()
{
    unsigned int uConfig0 = m_uConfigValue[0];
    unsigned int uConfig1 = m_uConfigValue[1];
    unsigned int uConfig2 = m_uConfigValue[2];

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN:
            if (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTPDEN) {
                m_nRadioCWDTEN = 2;
            } else {
                m_nRadioCWDTEN = 1;
            }

            break;

        default:
            m_nRadioCWDTEN = 1;
    }

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CBOV_2_LEVEL) {
        case NUMICRO_FLASH_CONFIG_CBOV_1:
            m_nRadioCBOV = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_0:
        default:
            m_nRadioCBOV = 1;
            break;
    }

    switch (uConfig0 & NUMICRO_FLASH_CONFIG_CBS_4_MODE) {
        case NUMICRO_FLASH_CONFIG_CBS_AP:
            m_nRadioCBS = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_LD:
            m_nRadioCBS = 1;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_AP_IAP:
            m_nRadioCBS = 2;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_LD_IAP:
        default:
            m_nRadioCBS = 3;
            break;
    }

    m_nRadioCFGXT1	= ((uConfig0 & M0A21_FLASH_CONFIG_CFGXT1) ? 0 : 1);
    m_nRadioCIOINI	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CIOINI) ? 0 : 1);
    m_nRadioRSTEXT	= ((uConfig0 & M0A21_FLASH_CONFIG_RSTEXT) ? 0 : 1);
    m_nRadioRSTWSEL	= ((uConfig0 & M0A21_FLASH_CONFIG_RSTWSEL) ? 0 : 1);
    m_bCheckCBORST	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckCBODEN	= ((uConfig0 & NUMICRO_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
    m_bCheckICELOCK	= ((uConfig0 & M0A21_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    m_bCheckLOCK	= ((uConfig0 & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_bCheckDFEN	= ((uConfig0 & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);
    unsigned int uDataFlashBase, uDataFlashSize;

    if (m_bCheckDFEN) {
        m_SpinDataFlashSize.EnableWindow(TRUE);
        m_DataFlashBase.EnableWindow(TRUE);
        uDataFlashBase = uConfig1 & ~(m_uFlashPageSize - 1);
        uDataFlashSize = m_uProgramMemorySize - uDataFlashBase;

        if (uDataFlashSize > m_uProgramMemorySize) {
            uDataFlashSize = 0;
        }
    } else {
        m_SpinDataFlashSize.EnableWindow(FALSE);
        m_DataFlashBase.EnableWindow(FALSE);
        uDataFlashBase = 0xFFFFFFFF;
        uDataFlashSize = 0;
    }

    m_sDataFlashBase.Format(_T("%X"), uConfig1);
    m_sDataFlashSize.Format(_T("%.2fK"), uDataFlashSize / 1024.);
    m_sALOCK.Format(_T("%02X"), uConfig2 & 0xFF);
    m_sConfigValue0.Format(_T("0x%08X"), uConfig0);
    m_sConfigValue1.Format(_T("0x%08X"), uConfig1);
    m_sConfigValue2.Format(_T("0x%08X"), uConfig2);
    UpdateData(FALSE);
}

void CDialogConfiguration_M031::GUIToConfig()
{
    UpdateData(TRUE);
    unsigned int uConfig0 = 0xFFFFFFFF;
    unsigned int uConfig1 = m_uConfigValue[1];
    unsigned int uConfig2 = m_uConfigValue[2];

    switch (m_nRadioCWDTEN) {
        case 0:
            uConfig0 &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            uConfig0 |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        case 2:
            uConfig0 &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            uConfig0 |= (NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            break;

        default:
            if (((uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE) ||
                    (((uConfig0 & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN) && (uConfig0 & NUMICRO_FLASH_CONFIG_CWDTPDEN))) {
                uConfig0 &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
                uConfig0 |= (NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            }
    }

    uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBOV_2_LEVEL;

    switch (m_nRadioCBOV) {
        case 0:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_1;
            break;

        case 1:
        default:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBOV_0;
            break;
    }

    uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBS_4_MODE;

    switch (m_nRadioCBS) {
        case 0:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_AP;
            break;

        case 1:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_LD;
            break;

        case 2:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
            break;

        case 3:
        default:
            uConfig0 |= NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
            break;
    }

    if (m_nRadioCFGXT1 == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_CFGXT1;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_CFGXT1;
    }

    if (m_nRadioCIOINI == 0) {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CIOINI;
    } else {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CIOINI;
    }

    if (m_nRadioRSTEXT == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_RSTEXT;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_RSTEXT;
    }

    if (m_nRadioRSTWSEL == 0) {
        uConfig0 |=  M0A21_FLASH_CONFIG_RSTWSEL;
    } else {
        uConfig0 &= ~M0A21_FLASH_CONFIG_RSTWSEL;
    }

    if (m_bCheckCBORST) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBORST;
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CBORST;
    }

    if (m_bCheckCBODEN) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_CBODEN;
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_CBODEN;
    }

    if (m_bCheckICELOCK) {
        uConfig0 &= ~M0A21_FLASH_CONFIG_ICELOCK;
    } else {
        uConfig0 |=  M0A21_FLASH_CONFIG_ICELOCK;
    }

    if (m_bCheckDFEN) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_DFEN;
        uConfig1 &= ~(m_uFlashPageSize - 1);

        if (uConfig1 < m_uFlashPageSize) {
            uConfig1 = m_uFlashPageSize;
        }

        if (uConfig1 > (m_uProgramMemorySize - m_uFlashPageSize)) {
            uConfig1 = (m_uProgramMemorySize - m_uFlashPageSize);
        }
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_DFEN;

        if (uConfig1 != 0xFFFFFFFF) {
            uConfig1  = 0xFFFFFFFF;
        }
    }

    bool bEdit = false;

    if (m_bCheckLOCK) {
        uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;

        if ((uConfig2 & 0xFF) == 0x5A) {
            uConfig2 = 0xFFFFFF00;
            bEdit = true;
        }
    } else {
        uConfig0 |=  NUMICRO_FLASH_CONFIG_LOCK;

        if ((uConfig2 & 0xFF) != 0x5A) {
            uConfig2 = 0xFFFFFF5A;
            bEdit = true;
        }
    }

    if (!bEdit) {
        unsigned int uALOCK = ::_tcstoul(m_sALOCK, NULL, 16);
        uConfig2 = 0xFFFFFF00 | uALOCK;

        if (uALOCK == 0x5A) {
            uConfig0 |=  NUMICRO_FLASH_CONFIG_LOCK;
        } else {
            uConfig0 &= ~NUMICRO_FLASH_CONFIG_LOCK;
        }
    }

    m_uConfigValue[0] = uConfig0;
    m_uConfigValue[1] = uConfig1;
    m_uConfigValue[2] = uConfig2;
}
