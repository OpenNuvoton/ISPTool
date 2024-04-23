// DialogChipSetting_CFG_M460.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "FlashInfo.h"
#include "DialogChipSetting_CFG_M2A23.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M2A23, CDialog)

CDialogChipSetting_CFG_M2A23::CDialogChipSetting_CFG_M2A23(CWnd *pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_DataFlashBase(16, 8)
    , m_ALOCK(16, 2)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M460)
    m_nRadioCWDTEN	= -1;
    m_nRadioCBOV	= -1;
    m_nRadioCBS		= -1;
    m_nRadioCIOINI  = -1;
    m_nRadioCRST    = -1;
    m_nRadioRPW     = -1;
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

void CDialogChipSetting_CFG_M2A23::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_CFG_M2L31)
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE,			m_nRadioCWDTEN);
    DDX_Radio(pDX, IDC_RADIO_BOV_3,					m_nRadioCBOV);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM,		        m_nRadioCBS);
    DDX_Radio(pDX, IDC_RADIO_IO_TRI,                m_nRadioCIOINI);
    DDX_Radio(pDX, IDC_RADIO_CHIPRESET_TIMEEXT_1,   m_nRadioCRST);
    DDX_Radio(pDX, IDC_RADIO_RST_PIN_WIDTH_1,       m_nRadioRPW);
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

BEGIN_MESSAGE_MAP(CDialogChipSetting_CFG_M2A23, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_CFG_M2L31)
    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP,			OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_0,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM,				OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM,                   OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM,                   OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_IO_TRI,                     OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_IO_BI,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_CHIPRESET_TIMEEXT_1,        OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_CHIPRESET_TIMEEXT_0,        OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_RST_PIN_WIDTH_1,            OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_RST_PIN_WIDTH_0,            OnRadioClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK,					OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK,				OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,			OnCheckClick)

    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_ADVANCE_LOCK,		OnKillfocusEditAdvanceLock)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,		OnKillfocusEditDataFlashBase)
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE,	OnDeltaposSpinDataFlashSize)

    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 message handlers

BOOL CDialogChipSetting_CFG_M2A23::OnInitDialog()
{
    CDialog::OnInitDialog();
    // TODO: Add extra initialization here
    //CHIP_INFO_NUMICRO_T chipInfo;
    //GetChipInfo_NuMicro(m_uPID, m_uDID, 0xFFFFFFFF, 0xFFFFFFFF, false, &chipInfo);
    //m_uProgramMemorySize = chipInfo.uProgramMemorySize;
    //m_uFlashPageSize	= (1 << chipInfo.uFlash_PageSize);
    UpdateUI();
    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);
    ConfigToGUI();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;	// return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_CFG_M2A23::UpdateUI()
{
    // TODO: Add your control notification handler code here
}

void CDialogChipSetting_CFG_M2A23::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    CFG2GUI_CWDT();
    CFG2GUI_CBOD_4();
    CFG2GUI_ICELOCK();
    CFG2GUI_CBS_4();
    CFG2GUI_CIOINI();
    CFG2GUI_CRST();
    CFG2GUI_RPW();
    CFG2GUI_DFEN();
    CFG2GUI_KSPLOCK();

    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);

    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M2A23::GUIToConfig()
{
    UpdateData(TRUE);
    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    GUI2CFG_CWDT();
    GUI2CFG_CBOD_4();
    GUI2CFG_ICELOCK();
    GUI2CFG_CBS_4();
    GUI2CFG_CIOINI();
    GUI2CFG_CRST();
    GUI2CFG_RPW();
    GUI2CFG_DFEN();
    GUI2CFG_KSPLOCK();   

    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_CWDT()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        default:
            m_nRadioCWDTEN = 1;
    }
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_CBOD_4()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBOV_4_LEVEL) {
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
            m_nRadioCBOV = 3;
            break;

        default:
            m_nRadioCBOV = 0;
    }

    m_bCheckCBORST = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckCBODEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_CBS_4()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBS_4_MODE) {
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
        m_nRadioCBS = 3;
        break;

    default:
        m_nRadioCBS = 0;
    }
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_CIOINI()
{
    m_nRadioCIOINI = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CIOINI) ? 0 : 1);
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_CRST()
{
    m_nRadioCRST = ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_RSTEXT) ? 0 : 1);
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_RPW()
{
    m_nRadioRPW = ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_RSTWSEL) ? 0 : 1);
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_ICELOCK()
{
    m_bCheckICELOCK = ((m_uConfigValue_t[0] & M480_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_DFEN()
{
    unsigned int uDataFlashBase, uDataFlashSize;
    m_bCheckDFEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);

    if (m_bCheckDFEN) {
        m_SpinDataFlashSize.EnableWindow(TRUE);
        m_DataFlashBase.EnableWindow(TRUE);
        uDataFlashBase = m_uConfigValue_t[1] & ~(m_uFlashPageSize - 1);
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

    m_sDataFlashBase.Format(_T("%X"), m_uConfigValue_t[1]);
    m_sDataFlashSize.Format(_T("%.2fK"), uDataFlashSize / 1024.);
}

void CDialogChipSetting_CFG_M2A23::CFG2GUI_KSPLOCK()
{
    m_bCheckLOCK = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_sALOCK.Format(_T("%02X"), m_uConfigValue_t[2] & 0xFF);
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_CWDT()
{
    switch (m_nRadioCWDTEN) {
        case 0:
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        default:
            if (((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE) ||
                    (((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN) && (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTPDEN))) {
                m_uConfigValue_t[0] &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
                m_uConfigValue_t[0] |= (NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            }
    }
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_CBOD_4()
{
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBOV_4_LEVEL;

    switch (m_nRadioCBOV) {
        case 0:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_3;
            break;

        case 1:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_2;
            break;

        case 2:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_1;
            break;

        case 3:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_0;
            break;

        default:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_3;
    }

    if (m_bCheckCBORST) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBORST;
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CBORST;
    }

    if (m_bCheckCBODEN) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBODEN;
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CBODEN;
    }
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_CBS_4()
{
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBS_4_MODE;

    switch (m_nRadioCBS) {
    case 0:
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_AP;
        break;

    case 1:
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_LD;
        break;

    case 2:
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
        break;

    case 3:
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
        break;

    default:
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_AP;
    }
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_CIOINI()
{
    if (m_nRadioCIOINI == 0) {
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CIOINI;
    }
    else {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CIOINI;
    }
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_CRST()
{
    if (m_nRadioCRST == 0) {
        m_uConfigValue_t[0] |= M0A21_FLASH_CONFIG_RSTEXT;
    }
    else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_RSTEXT;
    }
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_RPW()
{
    if (m_nRadioRPW == 0) {
        m_uConfigValue_t[0] |= M0A21_FLASH_CONFIG_RSTWSEL;
    }
    else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_RSTWSEL;
    }
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_ICELOCK()
{
    if (m_bCheckICELOCK) {
        m_uConfigValue_t[0] &= ~M480_FLASH_CONFIG_ICELOCK;
    } else {
        m_uConfigValue_t[0] |=  M480_FLASH_CONFIG_ICELOCK;
    }
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_DFEN()
{
    if (m_bCheckDFEN) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_DFEN;
        m_uConfigValue_t[1] &= ~(m_uFlashPageSize - 1);

        if (m_uConfigValue_t[1] < m_uFlashPageSize) {
            m_uConfigValue_t[1] = m_uFlashPageSize;
        }

        if (m_uConfigValue_t[1] > (m_uProgramMemorySize - m_uFlashPageSize)) {
            m_uConfigValue_t[1] = (m_uProgramMemorySize - m_uFlashPageSize);
        }
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_DFEN;

        if (m_uConfigValue_t[1] != 0xFFFFFFFF) {
            m_uConfigValue_t[1]  = 0xFFFFFFFF;
        }
    }
}

void CDialogChipSetting_CFG_M2A23::GUI2CFG_KSPLOCK()
{
    bool bModify0 = false, bModify1 = false;

    if (m_bCheckLOCK) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;

        if ((m_uConfigValue_t[2] & 0x5A) == 0x5A) {
            m_uConfigValue_t[2] &= ~0xFF;
            bModify0 = true;
        }
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;

        if ((m_uConfigValue_t[2] & 0x5A) != 0x5A) {
            m_uConfigValue_t[2] = (m_uConfigValue_t[2] & ~0xFF) | 0x5A;
            bModify0 = true;
        }
    }

    if (!bModify0) {
        unsigned int uALOCK = ::_tcstoul(m_sALOCK, NULL, 16) & 0x5A;
        m_uConfigValue_t[2] = (m_uConfigValue_t[2] & ~0xFF) | uALOCK;

        if (uALOCK == 0x5A) {
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;

        } else {
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;
        }
    }
}

void CDialogChipSetting_CFG_M2A23::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2A23::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2A23::OnKillfocusEditAdvanceLock()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2A23::OnKillfocusEditKeyStoreLock()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2A23::OnKillfocusEditDataFlashBase()
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

void CDialogChipSetting_CFG_M2A23::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
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

void CDialogChipSetting_CFG_M2A23::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_CFG_M2A23::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}
