// DialogChipSetting_CFG_M0A21.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "DialogChipSetting_CFG_M0A21.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M0A21 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M0A21, CDialog)

CDialogChipSetting_CFG_M0A21::CDialogChipSetting_CFG_M0A21(CWnd *pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_DataFlashBase(16, 8)
    , m_ALOCK(16, 2)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M0A21)
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
    m_sConfigValue3	= _T("");
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M0A21::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_CFG_M0A21)
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

BEGIN_MESSAGE_MAP(CDialogChipSetting_CFG_M0A21, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_CFG_M0A21)
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
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M0A21 message handlers

BOOL CDialogChipSetting_CFG_M0A21::OnInitDialog()
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
    return TRUE;	// return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_CFG_M0A21::UpdateUI()
{
    // TODO: Add your control notification handler code here
}

void CDialogChipSetting_CFG_M0A21::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];
    m_nRadioCFGXT1	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_CFGXT1) ? 0 : 1);
    m_nRadioCFGRPS	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_CFGRPS) ? 0 : 1);
    m_nRadioRSTEXT	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_RSTEXT) ? 0 : 1);
    m_nRadioRSTWSEL	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_RSTWSEL) ? 0 : 1);
    m_bCheckICELOCK	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    CFG2GUI_CWDT();
    CFG2GUI_CBOD_4();
    CFG2GUI_CIOINI();
    CFG2GUI_CBS_4();
    CFG2GUI_DFEN();
    CFG2GUI_ALOCK();
    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M0A21::GUIToConfig()
{
    UpdateData(TRUE);
    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    if (m_nRadioCFGXT1 == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_CFGXT1;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_CFGXT1;
    }

    if (m_nRadioCFGRPS == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_CFGRPS;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_CFGRPS;
    }

    if (m_nRadioRSTEXT == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_RSTEXT;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_RSTEXT;
    }

    if (m_nRadioRSTWSEL == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_RSTWSEL;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_RSTWSEL;
    }

    if (m_bCheckICELOCK) {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_ICELOCK;
    } else {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_ICELOCK;
    }

    GUI2CFG_CWDT();
    GUI2CFG_CBOD_4();
    GUI2CFG_CIOINI();
    GUI2CFG_CBS_4();
    GUI2CFG_DFEN();
    GUI2CFG_ALOCK();
    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
}

void CDialogChipSetting_CFG_M0A21::CFG2GUI_CWDT()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN:
            if (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTPDEN) {
                m_nRadioCWDTEN = 2;
            } else {
                m_nRadioCWDTEN = 1;
            }

            break;

        default:
            m_nRadioCWDTEN = 1;
    }
}

void CDialogChipSetting_CFG_M0A21::CFG2GUI_CBOD_4()
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

void CDialogChipSetting_CFG_M0A21::CFG2GUI_CBOD_2()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBOV_2_LEVEL) {
        case NUMICRO_FLASH_CONFIG_CBOV_1:
            m_nRadioCBOV = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_0:
            m_nRadioCBOV = 1;
            break;

        default:
            m_nRadioCBOV = 0;
    }

    m_bCheckCBORST = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckCBODEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M0A21::CFG2GUI_CIOINI()
{
    m_nRadioCIOINI = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CIOINI) ? 0 : 1);
}

void CDialogChipSetting_CFG_M0A21::CFG2GUI_CBS_4()
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

void CDialogChipSetting_CFG_M0A21::CFG2GUI_DFEN()
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

void CDialogChipSetting_CFG_M0A21::CFG2GUI_LOCK()
{
    m_bCheckLOCK = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M0A21::CFG2GUI_ALOCK()
{
    m_bCheckLOCK = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);
    m_sALOCK.Format(_T("%02X"), m_uConfigValue_t[2] & 0xFF);
}

void CDialogChipSetting_CFG_M0A21::GUI2CFG_CWDT()
{
    switch (m_nRadioCWDTEN) {
        case 0:
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        case 2:
            m_uConfigValue_t[0] &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            m_uConfigValue_t[0] |= (NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            break;

        default:
            if (((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE) ||
                    (((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_BY_LIRCEN) && (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTPDEN))) {
                m_uConfigValue_t[0] &= ~(NUMICRO_FLASH_CONFIG_CWDTEN | NUMICRO_FLASH_CONFIG_CWDTPDEN);
                m_uConfigValue_t[0] |= (NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE | NUMICRO_FLASH_CONFIG_CWDTPDEN);
            }
    }
}

void CDialogChipSetting_CFG_M0A21::GUI2CFG_CBOD_4()
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

void CDialogChipSetting_CFG_M0A21::GUI2CFG_CBOD_2()
{
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBOV_2_LEVEL;

    switch (m_nRadioCBOV) {
        case 0:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_1;
            break;

        case 1:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_0;
            break;

        default:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_1;
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

void CDialogChipSetting_CFG_M0A21::GUI2CFG_CIOINI()
{
    if (m_nRadioCIOINI == 0) {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CIOINI;
    } else {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CIOINI;
    }
}

void CDialogChipSetting_CFG_M0A21::GUI2CFG_CBS_4()
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

void CDialogChipSetting_CFG_M0A21::GUI2CFG_DFEN()
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

void CDialogChipSetting_CFG_M0A21::GUI2CFG_LOCK()
{
    if (m_bCheckLOCK) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;
    }
}

void CDialogChipSetting_CFG_M0A21::GUI2CFG_ALOCK()
{
    bool bEdit = false;

    if (m_bCheckLOCK) {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;

        if ((m_uConfigValue_t[2] & 0x5A) == 0x5A) {
            m_uConfigValue_t[2] = 0xFFFFFF00;
            bEdit = true;
        }
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;

        if ((m_uConfigValue_t[2] & 0x5A) != 0x5A) {
            m_uConfigValue_t[2] = 0xFFFFFF5A;
            bEdit = true;
        }
    }

    if (!bEdit) {
        unsigned int uALOCK = ::_tcstoul(m_sALOCK, NULL, 16) & 0x5A;
        m_uConfigValue_t[2] = 0xFFFFFF00 | uALOCK;

        if (uALOCK == 0x5A) {
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;
        } else {
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;
        }
    }
}

void CDialogChipSetting_CFG_M0A21::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M0A21::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M0A21::OnKillfocusEditAdvanceLock()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M0A21::OnKillfocusEditDataFlashBase()
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

void CDialogChipSetting_CFG_M0A21::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
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

void CDialogChipSetting_CFG_M0A21::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_CFG_M0A21::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M030G dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M030G, CDialog)

CDialogChipSetting_CFG_M030G::CDialogChipSetting_CFG_M030G(CWnd *pParent /*=NULL*/)
    : CDialogChipSetting_CFG_M0A21(pParent)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M030G)
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M030G::UpdateUI()
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
    int nID1s[] = { IDC_GROUP_WDT, IDC_RADIO_WDT_DISABLE, IDC_RADIO_WDT_ENABLE_KEEP, IDC_RADIO_WDT_ENABLE_STOP,
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

void CDialogChipSetting_CFG_M030G::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];
    m_nRadioRSTEXT	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_RSTEXT) ? 0 : 1);
    m_nRadioRSTWSEL	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_RSTWSEL) ? 0 : 1);
    m_bCheckICELOCK	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    CFG2GUI_CWDT();
    CFG2GUI_CIOINI();
    CFG2GUI_CBS_4();
    CFG2GUI_DFEN();
    CFG2GUI_ALOCK();
    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M030G::GUIToConfig()
{
    UpdateData(TRUE);
    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    if (m_nRadioRSTEXT == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_RSTEXT;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_RSTEXT;
    }

    if (m_nRadioRSTWSEL == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_RSTWSEL;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_RSTWSEL;
    }

    if (m_bCheckICELOCK) {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_ICELOCK;
    } else {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_ICELOCK;
    }

    GUI2CFG_CWDT();
    GUI2CFG_CIOINI();
    GUI2CFG_CBS_4();
    GUI2CFG_DFEN();
    GUI2CFG_ALOCK();
    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
}

void CDialogChipSetting_CFG_M030G::CFG2GUI_CWDT()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        default:
            m_nRadioCWDTEN = 1;
    }
}

void CDialogChipSetting_CFG_M030G::GUI2CFG_CWDT()
{
    switch (m_nRadioCWDTEN) {
        case 0:
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        default:
            if ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE) {
                m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
                m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE;
            }
    }
}

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M031 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M031, CDialog)

CDialogChipSetting_CFG_M031::CDialogChipSetting_CFG_M031(CWnd *pParent /*=NULL*/)
    : CDialogChipSetting_CFG_M0A21(pParent)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M031)
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M031::UpdateUI()
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

void CDialogChipSetting_CFG_M031::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];
    m_nRadioCFGXT1	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_CFGXT1) ? 0 : 1);
    m_nRadioRSTEXT	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_RSTEXT) ? 0 : 1);
    m_nRadioRSTWSEL	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_RSTWSEL) ? 0 : 1);
    m_bCheckICELOCK	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    CFG2GUI_CWDT();
    CFG2GUI_CBOD_2();
    CFG2GUI_CIOINI();
    CFG2GUI_CBS_4();
    CFG2GUI_DFEN();
    CFG2GUI_ALOCK();
    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M031::GUIToConfig()
{
    UpdateData(TRUE);
    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    if (m_nRadioCFGXT1 == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_CFGXT1;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_CFGXT1;
    }

    if (m_nRadioRSTEXT == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_RSTEXT;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_RSTEXT;
    }

    if (m_nRadioRSTWSEL == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_RSTWSEL;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_RSTWSEL;
    }

    if (m_bCheckICELOCK) {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_ICELOCK;
    } else {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_ICELOCK;
    }

    GUI2CFG_CWDT();
    GUI2CFG_CBOD_2();
    GUI2CFG_CIOINI();
    GUI2CFG_CBS_4();
    GUI2CFG_DFEN();
    GUI2CFG_ALOCK();
    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
}

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M451 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M451, CDialog)

CDialogChipSetting_CFG_M451::CDialogChipSetting_CFG_M451(CWnd *pParent /*=NULL*/)
    : CDialogChipSetting_CFG_M0A21(pParent)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M451)
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M451::UpdateUI()
{
    // TODO: Add your control notification handler code here
    GetDlgItem(IDC_GROUP_RST_PIN_WIDTH)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RST_PIN_WIDTH_1)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RST_PIN_WIDTH_0)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_CHECK_ICE_LOCK)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_GROUP_CHIPRESET_TIMEEXT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_CHIPRESET_TIMEEXT_1)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_CHIPRESET_TIMEEXT_0)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_GROUP_RPD)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RPD_RESET)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RPD_INPUT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_BOV_3)->SetWindowText(_T("4.5V"));
    GetDlgItem(IDC_RADIO_BOV_2)->SetWindowText(_T("3.7V"));
    GetDlgItem(IDC_RADIO_BOV_1)->SetWindowText(_T("2.7V"));
    GetDlgItem(IDC_RADIO_BOV_0)->SetWindowText(_T("2.2V"));
    GetDlgItem(IDC_GROUP_ADVANCE_LOCK)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_STATIC_FLASH_ADVANCE_LOCK)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_EDIT_FLASH_ADVANCE_LOCK)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_STATIC_CONFIG_2)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_STATIC_CONFIG_VALUE_2)->ShowWindow(SW_HIDE);
    RECT rcTmp, rcICELOCK, rcCFGXT1;
    GetDlgItem(IDC_CHECK_ICE_LOCK)->GetWindowRect(&rcICELOCK);
    GetDlgItem(IDC_RADIO_GPF_CRYSTAL)->GetWindowRect(&rcCFGXT1);
    LONG lDiff0, lDiff1;
    lDiff0 = rcICELOCK.left - rcCFGXT1.left;
    lDiff1 = rcCFGXT1.bottom - rcICELOCK.bottom;
    GetDlgItem(IDC_GROUP_GPF)->GetWindowRect(&rcTmp);
    this->ScreenToClient(&rcTmp);
    GetDlgItem(IDC_GROUP_GPF)->SetWindowPos(NULL, rcTmp.left + lDiff0, rcTmp.top - lDiff1, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
    GetDlgItem(IDC_RADIO_GPF_CRYSTAL)->GetWindowRect(&rcTmp);
    this->ScreenToClient(&rcTmp);
    GetDlgItem(IDC_RADIO_GPF_CRYSTAL)->SetWindowPos(NULL, rcTmp.left + lDiff0, rcTmp.top - lDiff1, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
    GetDlgItem(IDC_RADIO_GPF_GPIO)->GetWindowRect(&rcTmp);
    this->ScreenToClient(&rcTmp);
    GetDlgItem(IDC_RADIO_GPF_GPIO)->SetWindowPos(NULL, rcTmp.left + lDiff0, rcTmp.top - lDiff1, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
    RECT rcGroupRSTWSEL, rcGroupCBS;
    GetDlgItem(IDC_GROUP_RST_PIN_WIDTH)->GetWindowRect(&rcGroupRSTWSEL);
    GetDlgItem(IDC_GROUP_BOOT_SELECT)->GetWindowRect(&rcGroupCBS);
    LONG lDiff = rcGroupCBS.bottom - rcGroupRSTWSEL.bottom;
    int i, nIDs[] = {IDC_GROUP_BOOT_SELECT, IDC_RADIO_BS_APROM, IDC_RADIO_BS_LDROM, IDC_RADIO_BS_APROM_LDROM, IDC_RADIO_BS_LDROM_APROM,
                     IDC_GROUP_BROWN_OUT_VOLTAGE, IDC_RADIO_BOV_3, IDC_RADIO_BOV_2, IDC_RADIO_BOV_1, IDC_RADIO_BOV_0, IDC_CHECK_BROWN_OUT_DETECT, IDC_CHECK_BROWN_OUT_RESET,
                     IDC_GROUP_WDT, IDC_RADIO_WDT_DISABLE, IDC_RADIO_WDT_ENABLE_KEEP, IDC_RADIO_WDT_ENABLE_STOP,
                     IDC_GROUP_DATA_FLASH, IDC_CHECK_DATA_FLASH_ENABLE, IDC_STATIC_FLASH_BASE_ADDRESS, IDC_EDIT_FLASH_BASE_ADDRESS, IDC_STATIC_DATA_FLASH_SIZE, IDC_EDIT_DATA_FLASH_SIZE, IDC_SPIN_DATA_FLASH_SIZE,
                     IDC_GROUP_ADVANCE_LOCK, IDC_CHECK_SECURITY_LOCK, IDC_STATIC_FLASH_ADVANCE_LOCK, IDC_EDIT_FLASH_ADVANCE_LOCK,
                     IDC_GROUP_CONFIG_VALUE, IDC_STATIC_CONFIG_0, IDC_STATIC_CONFIG_VALUE_0, IDC_STATIC_CONFIG_1, IDC_STATIC_CONFIG_VALUE_1, IDC_STATIC_CONFIG_2, IDC_STATIC_CONFIG_VALUE_2
                    };

    for (i = 0; i < _countof(nIDs); i++) {
        GetDlgItem(nIDs[i])->GetWindowRect(&rcTmp);
        this->ScreenToClient(&rcTmp);
        GetDlgItem(nIDs[i])->SetWindowPos(NULL, rcTmp.left, rcTmp.top - lDiff, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
    }

    this->GetWindowRect(&rcTmp);
    SetWindowPos(this, 0, 0, rcTmp.right - rcTmp.left, rcTmp.bottom - rcTmp.top - lDiff, SWP_NOZORDER | SWP_NOMOVE);
}

void CDialogChipSetting_CFG_M451::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];
    m_nRadioCFGXT1	= ((m_uConfigValue_t[0] & M0A21_FLASH_CONFIG_CFGXT1) ? 0 : 1);
    CFG2GUI_CWDT();
    CFG2GUI_CBOD_4();
    CFG2GUI_CIOINI();
    CFG2GUI_CBS_4();
    CFG2GUI_LOCK();
    CFG2GUI_DFEN();
    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M451::GUIToConfig()
{
    UpdateData(TRUE);
    m_uConfigValue_t[0] = ~M451_FLASH_CONFIG_XT1TYP;
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    if (m_nRadioCFGXT1 == 0) {
        m_uConfigValue_t[0] |=  M0A21_FLASH_CONFIG_CFGXT1;
    } else {
        m_uConfigValue_t[0] &= ~M0A21_FLASH_CONFIG_CFGXT1;
    }

    GUI2CFG_CWDT();
    GUI2CFG_CBOD_4();
    GUI2CFG_CIOINI();
    GUI2CFG_CBS_4();
    GUI2CFG_LOCK();
    GUI2CFG_DFEN();
    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
}

void CDialogChipSetting_CFG_M451::CFG2GUI_CBOD_4()
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
    m_bCheckCBODEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBODEN0) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M451::GUI2CFG_CBOD_4()
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
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBODEN0;
    } else {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CBODEN0;
    }
}

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M471 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M471, CDialog)

CDialogChipSetting_CFG_M471::CDialogChipSetting_CFG_M471(CWnd *pParent /*=NULL*/)
    : CDialogChipSetting_CFG_M0A21(pParent)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M471)
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M471::UpdateUI()
{
    // TODO: Add your control notification handler code here
    GetDlgItem(IDC_GROUP_RST_PIN_WIDTH)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RST_PIN_WIDTH_1)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RST_PIN_WIDTH_0)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_GROUP_CHIPRESET_TIMEEXT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_CHIPRESET_TIMEEXT_1)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_CHIPRESET_TIMEEXT_0)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_GROUP_RPD)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RPD_RESET)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_RPD_INPUT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_BOV_3)->SetWindowText(_T("4.4V"));
    GetDlgItem(IDC_RADIO_BOV_2)->SetWindowText(_T("3.7V"));
    GetDlgItem(IDC_RADIO_BOV_1)->SetWindowText(_T("3.3V"));
    GetDlgItem(IDC_RADIO_BOV_0)->SetWindowText(_T("2.3V"));
    GetDlgItem(IDC_GROUP_DATA_FLASH)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_CHECK_DATA_FLASH_ENABLE)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_STATIC_DATA_FLASH_SIZE)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_EDIT_DATA_FLASH_SIZE)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_SPIN_DATA_FLASH_SIZE)->ShowWindow(SW_HIDE);
    RECT rcTmp, rcGroupRSTW, rcGroupCFGXT1, rcGroupDFEN, rcGroupLOCK;
    LONG lDiff;
    GetDlgItem(IDC_GROUP_RST_PIN_WIDTH)->GetWindowRect(&rcGroupRSTW);
    GetDlgItem(IDC_GROUP_GPF)->GetWindowRect(&rcGroupCFGXT1);
    lDiff = rcGroupCFGXT1.top - rcGroupRSTW.top;
    int i;
    int nIDs[] = {IDC_GROUP_GPF, IDC_RADIO_GPF_CRYSTAL, IDC_RADIO_GPF_GPIO,
                  IDC_GROUP_BOOT_SELECT, IDC_RADIO_BS_APROM, IDC_RADIO_BS_LDROM, IDC_RADIO_BS_APROM_LDROM, IDC_RADIO_BS_LDROM_APROM,
                  IDC_GROUP_BROWN_OUT_VOLTAGE, IDC_RADIO_BOV_3, IDC_RADIO_BOV_2, IDC_RADIO_BOV_1, IDC_RADIO_BOV_0, IDC_CHECK_BROWN_OUT_DETECT, IDC_CHECK_BROWN_OUT_RESET,
                  IDC_GROUP_WDT, IDC_RADIO_WDT_DISABLE, IDC_RADIO_WDT_ENABLE_KEEP, IDC_RADIO_WDT_ENABLE_STOP,
                  IDC_GROUP_DATA_FLASH, IDC_CHECK_DATA_FLASH_ENABLE, IDC_STATIC_FLASH_BASE_ADDRESS, IDC_EDIT_FLASH_BASE_ADDRESS, IDC_STATIC_DATA_FLASH_SIZE, IDC_EDIT_DATA_FLASH_SIZE, IDC_SPIN_DATA_FLASH_SIZE
                 };

    for (i = 0; i < _countof(nIDs); i++) {
        GetDlgItem(nIDs[i])->GetWindowRect(&rcTmp);
        this->ScreenToClient(&rcTmp);
        GetDlgItem(nIDs[i])->SetWindowPos(NULL, rcTmp.left, rcTmp.top - lDiff, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
    }

    GetDlgItem(IDC_GROUP_DATA_FLASH)->GetWindowRect(&rcGroupDFEN);
    GetDlgItem(IDC_GROUP_ADVANCE_LOCK)->GetWindowRect(&rcGroupLOCK);
    lDiff = rcGroupLOCK.top - rcGroupDFEN.top;
    int nID1s[] = { IDC_GROUP_ADVANCE_LOCK, IDC_CHECK_SECURITY_LOCK, IDC_STATIC_FLASH_ADVANCE_LOCK, IDC_EDIT_FLASH_ADVANCE_LOCK,
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

void CDialogChipSetting_CFG_M471::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];
    m_nRadioCFGXT1	= ((m_uConfigValue_t[0] & M471_FLASH_CONFIG_CFGXT1) ? 0 : 1);
    m_bCheckICELOCK	= ((m_uConfigValue_t[0] & M471_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
    CFG2GUI_CWDT();
    CFG2GUI_CBOD_4();
    CFG2GUI_CIOINI();
    CFG2GUI_CBS_4();
    CFG2GUI_ALOCK();
    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M471::GUIToConfig()
{
    UpdateData(TRUE);
    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[1] = 0xFFFFFFFF;
    m_uConfigValue_t[2] = m_uConfigValue[2];

    if (m_nRadioCFGXT1 == 0) {
        m_uConfigValue_t[0] |=  M471_FLASH_CONFIG_CFGXT1;
    } else {
        m_uConfigValue_t[0] &= ~M471_FLASH_CONFIG_CFGXT1;
    }

    if (m_bCheckICELOCK) {
        m_uConfigValue_t[0] &= ~M471_FLASH_CONFIG_ICELOCK;
    } else {
        m_uConfigValue_t[0] |=  M471_FLASH_CONFIG_ICELOCK;
    }

    GUI2CFG_CWDT();
    GUI2CFG_CBOD_4();
    GUI2CFG_CIOINI();
    GUI2CFG_CBS_4();
    GUI2CFG_ALOCK();
    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
}
