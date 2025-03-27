// DialogChipSetting_CFG_M2U51.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "DialogChipSetting_CFG_M2U51.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M2U51 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M2U51, CDialog)

CDialogChipSetting_CFG_M2U51::CDialogChipSetting_CFG_M2U51(CWnd* pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_DataFlashBase(16, 8)
    , m_ALOCK(16, 2)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M2U51)
    m_nRadioCWDTEN      = -1;
    m_nRadioCWDTCSEL    = -1;
    m_nRadioCBOV        = -1;
    m_nRadioCBS         = -1;

    m_bCheckCBORST      = FALSE;
    m_bCheckCBODEN      = FALSE;
    m_bCheckICELOCK     = FALSE;
    m_bCheckLOCK        = FALSE;
    m_bCheckDFEN        = FALSE;

    m_sDataFlashBase    = _T("");
    m_sDataFlashSize    = _T("");

    m_sConfigValue0     = _T("");
    m_sConfigValue1     = _T("");
    m_sConfigValue2     = _T("");

    m_uProgramMemorySize    = M2U51_MAX_APROM_SIZE;
    m_uFlashPageSize        = NUMICRO_FLASH_PAGE_SIZE_512;

    m_uConfigValue[0]       = 0xFFFFFFFF;
    m_uConfigValue[1]       = 0xFFFFFFFF;
    m_uConfigValue[2]       = 0xFFFFFFFF;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_CFG_M2U51::~CDialogChipSetting_CFG_M2U51()
{
}

void CDialogChipSetting_CFG_M2U51::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_CFG_M2U51)
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE,           m_nRadioCWDTEN);
    DDX_Radio(pDX, IDC_RADIO_CWDTCSEL_NORMAL,       m_nRadioCWDTCSEL);
    DDX_Radio(pDX, IDC_RADIO_BOV_7,                 m_nRadioCBOV);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM_LDROM,        m_nRadioCBS);

    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET,       m_bCheckCBORST);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT,      m_bCheckCBODEN);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK,              m_bCheckICELOCK);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK,         m_bCheckLOCK);
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE,     m_bCheckDFEN);

    DDX_Control(pDX, IDC_EDIT_FLASH_ADVANCE_LOCK,   m_ALOCK);
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,   m_DataFlashBase);
    DDX_Control(pDX, IDC_EDIT_DATA_FLASH_SIZE,      m_DataFlashSize);
    DDX_Control(pDX, IDC_SPIN_DATA_FLASH_SIZE,      m_SpinDataFlashSize);

    DDX_Text(pDX, IDC_EDIT_FLASH_ADVANCE_LOCK,      m_sALOCK);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,      m_sDataFlashBase);
    DDX_Text(pDX, IDC_EDIT_DATA_FLASH_SIZE,         m_sDataFlashSize);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0,        m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1,        m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2,        m_sConfigValue2);
    //}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDialogChipSetting_CFG_M2U51, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_CFG_M2U51)
    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE,                OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP,            OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_STOP,            OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_CWDTCSEL_NORMAL,            OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_CWDTCSEL_HIGH_ACCURACY,     OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_7,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_6,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_5,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_4,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_3,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_2,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_1,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BOV_0,                      OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_APROM_LDROM,             OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BS_LDROM_APROM,             OnRadioClick)

    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET,            OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK,                   OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK,              OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,          OnCheckClick)

    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_ADVANCE_LOCK,        OnKillfocusEditAdvanceLock)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,        OnKillfocusEditDataFlashBase)
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DATA_FLASH_SIZE,   OnDeltaposSpinDataFlashSize)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M2U51 message handlers

BOOL CDialogChipSetting_CFG_M2U51::OnInitDialog()
{
    CDialog::OnInitDialog();

    UpdateUI();

    UDACCEL pAccel[1];
    pAccel[0].nInc = 1;
    pAccel[0].nSec = 0;
    m_SpinDataFlashSize.SetAccel(1, pAccel);

    ConfigToGUI();

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;    // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_CFG_M2U51::UpdateUI()
{
    // TODO: Add your control notification handler code here
}

void CDialogChipSetting_CFG_M2U51::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    /* Watchdog Timer */
    CFG2GUI_CWDT();

    /* Brown Out Detector */
    CFG2GUI_CBOD_8();

    /* ICE Lock */
    CFG2GUI_ICELOCK();

    /* Boot Select */
    CFG2GUI_CBS_2();

    /* Data Flash */
    CFG2GUI_DFEN();

    /* Security Lock */
    CFG2GUI_ALOCK();

    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);

    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M2U51::GUIToConfig()
{
    UpdateData(TRUE);

    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    /* Watchdog Timer */
    GUI2CFG_CWDT();

    /* Brown Out Detector */
    GUI2CFG_CBOD_8();

    /* ICE Lock */
    GUI2CFG_ICELOCK();

    /* Boot Select */
    GUI2CFG_CBS_2();

    /* Data Flash */
    GUI2CFG_DFEN();

    /* Security Lock */
    GUI2CFG_ALOCK();

    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
}

void CDialogChipSetting_CFG_M2U51::CFG2GUI_CWDT()
{
    switch (m_uConfigValue_t[0] & M2U51_FLASH_CONFIG_CWDTEN)
    {
        case M2U51_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        case M2U51_CONFIG_CWDTEN_ACTIVE:
            m_nRadioCWDTEN = 1;
            break;

        case M2U51_CONFIG_CWDTEN_BY_WDTPDIS:
        default:
            m_nRadioCWDTEN = 2;
    }

    switch (m_uConfigValue_t[0] & M2U51_CONFIG_CWDTCSEL)
    {
        case M2U51_CONFIG_CWDTCSEL_NORMAL:
            m_nRadioCWDTCSEL = 0;
            break;

        case M2U51_CONFIG_CWDTCSEL_HIGH_ACCURACY:
            m_nRadioCWDTCSEL = 1;
            break;

        default:
            m_nRadioCWDTCSEL = 0;
    }
}

void CDialogChipSetting_CFG_M2U51::CFG2GUI_CBOD_8()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL)
    {
        case NUMICRO_FLASH_CONFIG_CBOV_7:
            m_nRadioCBOV = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_6:
            m_nRadioCBOV = 1;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_5:
            m_nRadioCBOV = 2;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_4:
            m_nRadioCBOV = 3;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_3:
            m_nRadioCBOV = 4;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_2:
            m_nRadioCBOV = 5;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_1:
            m_nRadioCBOV = 6;
            break;

        case NUMICRO_FLASH_CONFIG_CBOV_0:
            m_nRadioCBOV = 7;
            break;

        default:
            m_nRadioCBOV = 0;
    }

    m_bCheckCBORST = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBORST) == 0 ? TRUE : FALSE);
    m_bCheckCBODEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBODEN) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M2U51::CFG2GUI_ICELOCK()
{
    m_bCheckICELOCK = ((m_uConfigValue_t[0] & NUMICRO_M23_FLASH_CONFIG_ICELOCK) == 0) ? TRUE : FALSE;
}

void CDialogChipSetting_CFG_M2U51::CFG2GUI_CBS_2()
{
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBS_2_MODE)
    {
        case NUMICRO_FLASH_CONFIG_CBS_AP_IAP:
            m_nRadioCBS = 0;
            break;

        case NUMICRO_FLASH_CONFIG_CBS_LD_IAP:
            m_nRadioCBS = 1;
            break;

        default:
            m_nRadioCBS = 0;
    }
}

void CDialogChipSetting_CFG_M2U51::CFG2GUI_DFEN()
{
    unsigned int uDataFlashBase, uDataFlashSize;

    m_bCheckDFEN = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_DFEN) == 0 ? TRUE : FALSE);

    if (m_bCheckDFEN)
    {
        m_SpinDataFlashSize.EnableWindow(TRUE);
        m_DataFlashBase.EnableWindow(TRUE);

        uDataFlashBase = m_uConfigValue_t[1] & ~(m_uFlashPageSize - 1);

        uDataFlashSize = m_uProgramMemorySize - uDataFlashBase;

        if (uDataFlashSize > m_uProgramMemorySize)
            uDataFlashSize = 0;
    }
    else
    {
        m_SpinDataFlashSize.EnableWindow(FALSE);
        m_DataFlashBase.EnableWindow(FALSE);
        uDataFlashBase = 0xFFFFFFFF;
        uDataFlashSize = 0;
    }

    m_sDataFlashBase.Format(_T("%X"), m_uConfigValue_t[1]);
    m_sDataFlashSize.Format(_T("%.2fK"), uDataFlashSize / 1024.);
}

void CDialogChipSetting_CFG_M2U51::CFG2GUI_ALOCK()
{
    m_bCheckLOCK = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_LOCK) == 0 ? TRUE : FALSE);

    m_sALOCK.Format(_T("%02X"), m_uConfigValue_t[2] & 0xFF);
}

void CDialogChipSetting_CFG_M2U51::GUI2CFG_CWDT()
{
    switch (m_nRadioCWDTEN)
    {
        case 0:
            m_uConfigValue_t[0] &= ~M2U51_FLASH_CONFIG_CWDTEN;
            m_uConfigValue_t[0] |=  M2U51_CONFIG_CWDTEN_INACTIVE;
            break;

        case 1:
            m_uConfigValue_t[0] &= ~M2U51_FLASH_CONFIG_CWDTEN;
            m_uConfigValue_t[0] |=  M2U51_CONFIG_CWDTEN_ACTIVE;
            break;

        default:
            if (((m_uConfigValue_t[0] & M2U51_FLASH_CONFIG_CWDTEN) == M2U51_CONFIG_CWDTEN_INACTIVE) ||
            ((m_uConfigValue_t[0] & M2U51_FLASH_CONFIG_CWDTEN) == M2U51_CONFIG_CWDTEN_ACTIVE))
            {
                m_uConfigValue_t[0] &= ~M2U51_FLASH_CONFIG_CWDTEN;
                m_uConfigValue_t[0] |=  M2U51_CONFIG_CWDTEN_BY_WDTPDIS;
            }
    }

    /* Watchdog Timer Clock Source and Type Selection */
    m_uConfigValue_t[0] &= ~M2U51_CONFIG_CWDTCSEL;

    switch (m_nRadioCWDTCSEL)
    {
        case 0:
            m_uConfigValue_t[0] |= M2U51_CONFIG_CWDTCSEL_NORMAL;
            break;

        case 1:
            m_uConfigValue_t[0] |= M2U51_CONFIG_CWDTCSEL_HIGH_ACCURACY;
            break;

        default:
            m_uConfigValue_t[0] |= M2U51_CONFIG_CWDTCSEL_NORMAL;
    }
}

void CDialogChipSetting_CFG_M2U51::GUI2CFG_CBOD_8()
{
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBOV_8_LEVEL;

    switch (m_nRadioCBOV)
    {
        case 0:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_7;
            break;

        case 1:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_6;
            break;

        case 2:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_5;
            break;

        case 3:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_4;
            break;

        case 4:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_3;
            break;

        case 5:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_2;
            break;

        case 6:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_1;
            break;

        case 7:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_0;
            break;

        default:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBOV_7;
    }

    if (m_bCheckCBORST)
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBORST;
    else
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CBORST;

    if (m_bCheckCBODEN)
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBODEN;
    else
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CBODEN;
}

void CDialogChipSetting_CFG_M2U51::GUI2CFG_ICELOCK()
{
    if (m_bCheckICELOCK)
        m_uConfigValue_t[0] &= ~NUMICRO_M23_FLASH_CONFIG_ICELOCK;
    else
        m_uConfigValue_t[0] |=  NUMICRO_M23_FLASH_CONFIG_ICELOCK;
}

void CDialogChipSetting_CFG_M2U51::GUI2CFG_CBS_2()
{
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBS_2_MODE;

    switch (m_nRadioCBS)
    {
        case 0:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
            break;

        case 1:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
            break;

        default:
            m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
    }
}

void CDialogChipSetting_CFG_M2U51::GUI2CFG_DFEN()
{
    if (m_bCheckDFEN)
    {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_DFEN;
        m_uConfigValue_t[1] &= ~(m_uFlashPageSize - 1);

        if (m_uConfigValue_t[1] < m_uFlashPageSize)
            m_uConfigValue_t[1] = m_uFlashPageSize;

        if (m_uConfigValue_t[1] > (m_uProgramMemorySize - m_uFlashPageSize))
            m_uConfigValue_t[1] = (m_uProgramMemorySize - m_uFlashPageSize);
    }
    else
    {
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_DFEN;

        if (m_uConfigValue_t[1] != 0xFFFFFFFF)
        {
            m_uConfigValue_t[1]  = 0xFFFFFFFF;
        }
    }
}

void CDialogChipSetting_CFG_M2U51::GUI2CFG_ALOCK()
{
    BOOL bEdit = FALSE;

    if (m_bCheckLOCK)
    {
        m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;

        if ((m_uConfigValue_t[2] & 0x5A) == 0x5A)
        {
            m_uConfigValue_t[2] = 0xFFFFFF00;
            bEdit = TRUE;
        }
    }
    else
    {
        m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;

        if ((m_uConfigValue_t[2] & 0x5A) != 0x5A)
        {
            m_uConfigValue_t[2] = 0xFFFFFF5A;
            bEdit = TRUE;
        }
    }

    if (!bEdit)
    {
        unsigned int uALOCK = ::_tcstoul(m_sALOCK, NULL, 16) & 0x5A;

        m_uConfigValue_t[2] = 0xFFFFFF00 | uALOCK;

        if (uALOCK == 0x5A)
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_LOCK;
        else
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_LOCK;
    }
}

void CDialogChipSetting_CFG_M2U51::OnRadioClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2U51::OnCheckClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2U51::OnKillfocusEditAdvanceLock()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2U51::OnKillfocusEditDataFlashBase()
{
    UpdateData(TRUE);

    unsigned int uDataFlashBase = ::_tcstoul(m_sDataFlashBase, NULL, 16);

    uDataFlashBase &= ~(m_uFlashPageSize - 1);

    if (uDataFlashBase < m_uFlashPageSize)
        uDataFlashBase = m_uFlashPageSize;

    if (uDataFlashBase > (m_uProgramMemorySize - m_uFlashPageSize))
        uDataFlashBase = (m_uProgramMemorySize - m_uFlashPageSize);

    m_uConfigValue[1] = uDataFlashBase;

    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2U51::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult)
{
    UpdateData(TRUE);

    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);

    unsigned int uDataFlashBase = ::_tcstoul(m_sDataFlashBase, NULL, 16);

    uDataFlashBase &= ~(m_uFlashPageSize - 1);

    if (pNMUpDown->iDelta == 1)
    {
        uDataFlashBase += m_uFlashPageSize;
    }
    else if (pNMUpDown->iDelta == -1)
    {
        uDataFlashBase -= m_uFlashPageSize;
    }

    if (uDataFlashBase < m_uFlashPageSize)
        uDataFlashBase = m_uFlashPageSize;

    if (uDataFlashBase > (m_uProgramMemorySize - m_uFlashPageSize))
        uDataFlashBase = (m_uProgramMemorySize - m_uFlashPageSize);

    m_uConfigValue[1] = uDataFlashBase;

    ConfigToGUI();

    *pResult = 0;
}

void CDialogChipSetting_CFG_M2U51::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_CFG_M2U51::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}
