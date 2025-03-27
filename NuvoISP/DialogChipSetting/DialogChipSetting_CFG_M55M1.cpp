// DialogChipSetting_CFG_M55M1.cpp : implementation file
//
#include "stdafx.h"
#include "ChipDefs.h"
#include "DialogChipSetting_CFG_M55M1.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M55M1 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M55M1, CDialog)

CDialogChipSetting_CFG_M55M1::CDialogChipSetting_CFG_M55M1(CWnd* pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_SC_BaseAddr(16, 8)
    , m_SC_PageCount(10, 3)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M55M1)
    m_nRadioCWDTEN      = -1;
    m_nRadioCBOV        = -1;
    m_nRadioCBS         = -1;
    m_nRadioUARTPSL     = -1;

    m_bCheckCBORST      = FALSE;
    m_bCheckCBODEN      = FALSE;
    m_bCheckICELOCK     = FALSE;
    m_bCheckSCEN        = FALSE;

    m_bCheckISP_UART    = TRUE;
    m_bCheckISP_USB     = FALSE;
    m_bCheckISP_CAN     = TRUE;
    m_bCheckISP_I2C     = TRUE;
    m_bCheckISP_SPI     = TRUE;

    m_sSC_BaseAddr      = _T("");
    m_sSC_PageCount     = _T("");

    m_sConfigValue0     = _T("");
    m_sConfigValue1     = _T("");
    m_sConfigValue2     = _T("");
    m_sConfigValue3     = _T("");
    m_sConfigValue4     = _T("");
    m_sConfigValue5     = _T("");
    m_sConfigValue6     = _T("");

    m_uProgramMemorySize    = M55M1_MAX_APROM_SIZE;
    m_uFlashPageSize        = NUMICRO_FLASH_PAGE_SIZE_8K;

    m_uConfigValue[0]       = 0xFFFFFFFF;
    m_uConfigValue[1]       = 0xFFFFFFFF;
    m_uConfigValue[2]       = 0xFFFFFFFF;
    m_uConfigValue[3]       = 0xFFFFFFFF;
    m_uConfigValue[4]       = 0xFFFFFFFF;
    m_uConfigValue[5]       = 0xFFFFFFFF;
    m_uConfigValue[6]       = 0xFFFFFFFF;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_CFG_M55M1::~CDialogChipSetting_CFG_M55M1()
{
}

void CDialogChipSetting_CFG_M55M1::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_CFG_M55M1)
    DDX_Radio(pDX, IDC_RADIO_WDT_DISABLE,               m_nRadioCWDTEN);
    DDX_Radio(pDX, IDC_RADIO_BOV_7,                     m_nRadioCBOV);
    DDX_Radio(pDX, IDC_RADIO_BS_APROM_LDROM,            m_nRadioCBS);
    DDX_Radio(pDX, IDC_RADIO_BLISP_UART_SEL_3,          m_nRadioUARTPSL);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_RESET,           m_bCheckCBORST);
    DDX_Check(pDX, IDC_CHECK_BROWN_OUT_DETECT,          m_bCheckCBODEN);
    DDX_Check(pDX, IDC_CHECK_ICE_LOCK,                  m_bCheckICELOCK);
    DDX_Check(pDX, IDC_CHECK_SECURE_CONCEAL,            m_bCheckSCEN);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_UART,             m_bCheckISP_UART);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_USB,              m_bCheckISP_USB);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_CAN,              m_bCheckISP_CAN);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_I2C,              m_bCheckISP_I2C);
    DDX_Check(pDX, IDC_CHECK_ISP_MODE_SPI,              m_bCheckISP_SPI);
    DDX_Control(pDX, IDC_EDIT_SECURE_CONCEAL_BASE_ADDR, m_SC_BaseAddr);
    DDX_Control(pDX, IDC_EDIT_SECURE_CONCEAL_PAGE_COUNT, m_SC_PageCount);

    DDX_Text(pDX, IDC_EDIT_SECURE_CONCEAL_BASE_ADDR,    m_sSC_BaseAddr);
    DDX_Text(pDX, IDC_EDIT_SECURE_CONCEAL_PAGE_COUNT,   m_sSC_PageCount);

    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0,            m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1,            m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2,            m_sConfigValue2);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_3,            m_sConfigValue3);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_4,            m_sConfigValue4);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_5,            m_sConfigValue5);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_6,            m_sConfigValue6);
    //}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDialogChipSetting_CFG_M55M1, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_CFG_M55M1)
    ON_BN_CLICKED(IDC_RADIO_WDT_DISABLE,                OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_WDT_ENABLE_KEEP,            OnRadioClick)
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
    ON_BN_CLICKED(IDC_RADIO_BLISP_UART_SEL_3,           OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BLISP_UART_SEL_2,           OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BLISP_UART_SEL_1,           OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_BLISP_UART_SEL_0,           OnRadioClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_RESET,            OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_BROWN_OUT_DETECT,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ICE_LOCK,                   OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURE_CONCEAL,             OnCheckClick)

    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_UART,              OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_USB,               OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_CAN,               OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_I2C,               OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_ISP_MODE_SPI,               OnCheckClick)

    ON_EN_KILLFOCUS(IDC_EDIT_SECURE_CONCEAL_BASE_ADDR,  OnKillfocusEditSCAddrCount)
    ON_EN_KILLFOCUS(IDC_EDIT_SECURE_CONCEAL_PAGE_COUNT, OnKillfocusEditSCAddrCount)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M55M1 message handlers

BOOL CDialogChipSetting_CFG_M55M1::OnInitDialog()
{
    CDialog::OnInitDialog();

    UpdateUI();

    ConfigToGUI();

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;    // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_CFG_M55M1::UpdateUI()
{
    GetDlgItem(IDC_CHECK_ISP_MODE_UART)->SetWindowText(_T("UART0"));
    GetDlgItem(IDC_CHECK_ISP_MODE_USB)->SetWindowText(_T("USB"));
    GetDlgItem(IDC_CHECK_ISP_MODE_CAN)->SetWindowText(_T("CAN0 (PA.4/PA.5)"));
    GetDlgItem(IDC_CHECK_ISP_MODE_I2C)->SetWindowText(_T("I2C0 (PC.0/PC.1)"));
    GetDlgItem(IDC_CHECK_ISP_MODE_SPI)->SetWindowText(_T("SPI0 (PA.0/PA.1/PA.2/PA.3)"));

    GetDlgItem(IDC_RADIO_BLISP_UART_SEL_3)->SetWindowText(_T("PB.13/PB.12"));
    GetDlgItem(IDC_RADIO_BLISP_UART_SEL_2)->SetWindowText(_T("PC.12/PC.11"));
    GetDlgItem(IDC_RADIO_BLISP_UART_SEL_1)->SetWindowText(_T("PA.7/PA.6"));
    GetDlgItem(IDC_RADIO_BLISP_UART_SEL_0)->SetWindowText(_T("PB.9/PB.8"));
}

void CDialogChipSetting_CFG_M55M1::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];
    m_uConfigValue_t[3] = m_uConfigValue[3];
    m_uConfigValue_t[4] = m_uConfigValue[4];
    m_uConfigValue_t[5] = m_uConfigValue[5];
    m_uConfigValue_t[6] = m_uConfigValue[6];

    /* Watchdog */
    CFG2GUI_CWDT();

    /* Brown Out Detector */
    CFG2GUI_CBOD_8();

    /* Boot Select */
    CFG2GUI_CBS_2();

    /* Security Lock */
    CFG2GUI_LOCK();

    m_bCheckISP_UART = ((m_uConfigValue_t[3] & M55M1_FLASH_CONFIG_UARTISPEN) != 0) ? TRUE : FALSE;
    m_bCheckISP_USB = ((m_uConfigValue_t[3] & M55M1_FLASH_CONFIG_USBISPDIS) == 0) ? TRUE : FALSE;
    m_bCheckISP_CAN = ((m_uConfigValue_t[3] & M55M1_FLASH_CONFIG_CANISPEN) != 0) ? TRUE : FALSE;
    m_bCheckISP_I2C = ((m_uConfigValue_t[3] & M55M1_FLASH_CONFIG_I2CISPEN) != 0) ? TRUE : FALSE;
    m_bCheckISP_SPI = ((m_uConfigValue_t[3] & M55M1_FLASH_CONFIG_SPIISPEN) != 0) ? TRUE : FALSE;

    /* UART0 multi-function pin selection */
    switch (m_uConfigValue_t[3] & M55M1_FLASH_CONFIG_UARTPSL)
    {
        case M55M1_FLASH_CONFIG_UARTPSL_SEL0:
            m_nRadioUARTPSL = 3;
            break;

        case M55M1_FLASH_CONFIG_UARTPSL_SEL1:
            m_nRadioUARTPSL = 2;
            break;

        case M55M1_FLASH_CONFIG_UARTPSL_SEL2:
            m_nRadioUARTPSL = 1;
            break;

        case M55M1_FLASH_CONFIG_UARTPSL_SEL3:
        default:
            m_nRadioUARTPSL = 0;
    }

    unsigned int uSC_BaseAddr, uSC_PageCount;

    uSC_BaseAddr    =  m_uConfigValue_t[4];
    uSC_PageCount   =  m_uConfigValue_t[5];
    m_bCheckSCEN    = (m_uConfigValue_t[6] != 0xFFFFFFFF) ? TRUE : FALSE;

    if (m_bCheckSCEN)
    {
        m_SC_BaseAddr.EnableWindow(TRUE);
        m_SC_PageCount.EnableWindow(TRUE);
    }
    else
    {
        m_SC_BaseAddr.EnableWindow(FALSE);
        m_SC_PageCount.EnableWindow(FALSE);

        uSC_BaseAddr  = 0xFFFFFFFF;
        uSC_PageCount = 0;
    }

    m_sSC_BaseAddr.Format(_T("%08X"), uSC_BaseAddr);
    m_sSC_PageCount.Format(_T("%d"), uSC_PageCount);

    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);
    m_sConfigValue3.Format(_T("0x%08X"), m_uConfigValue_t[3]);
    m_sConfigValue4.Format(_T("0x%08X"), m_uConfigValue_t[4]);
    m_sConfigValue5.Format(_T("0x%08X"), m_uConfigValue_t[5]);
    m_sConfigValue6.Format(_T("0x%08X"), m_uConfigValue_t[6]);

    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M55M1::GUIToConfig()
{
    UpdateData(TRUE);

    m_uConfigValue_t[0] = 0xFFFFFFFF;
    m_uConfigValue_t[1] = 0xFFFFFFFF;
    m_uConfigValue_t[2] = 0xFFFFFFFF;
    m_uConfigValue_t[3] = 0xFFFFFFFF;
    m_uConfigValue_t[4] = m_uConfigValue[4];
    m_uConfigValue_t[5] = m_uConfigValue[5];
    m_uConfigValue_t[6] = m_uConfigValue[6];

    /* Watchdog */
    GUI2CFG_CWDT();

    /* Brown Out Detector */
    GUI2CFG_CBOD_8();

    /* Boot Select */
    GUI2CFG_CBS_2();

    /* Security Lock */
    GUI2CFG_LOCK();

    m_uConfigValue_t[3] &= ~M55M1_FLASH_CONFIG_UARTPSL;

    if (m_bCheckISP_UART)
        m_uConfigValue_t[3] |=  M55M1_FLASH_CONFIG_UARTISPEN;
    else
        m_uConfigValue_t[3] &= ~M55M1_FLASH_CONFIG_UARTISPEN;

    if (!m_bCheckISP_USB)
        m_uConfigValue_t[3] |=  M55M1_FLASH_CONFIG_USBISPDIS;
    else
        m_uConfigValue_t[3] &= ~M55M1_FLASH_CONFIG_USBISPDIS;

    if (m_bCheckISP_CAN)
        m_uConfigValue_t[3] |=  M55M1_FLASH_CONFIG_CANISPEN;
    else
        m_uConfigValue_t[3] &= ~M55M1_FLASH_CONFIG_CANISPEN;

    if (m_bCheckISP_I2C)
        m_uConfigValue_t[3] |=  M55M1_FLASH_CONFIG_I2CISPEN;
    else
        m_uConfigValue_t[3] &= ~M55M1_FLASH_CONFIG_I2CISPEN;

    if (m_bCheckISP_SPI)
        m_uConfigValue_t[3] |=  M55M1_FLASH_CONFIG_SPIISPEN;
    else
        m_uConfigValue_t[3] &= ~M55M1_FLASH_CONFIG_SPIISPEN;

    /* UART0 multi-function pin selection */
    switch (m_nRadioUARTPSL)
    {
        case 3:
            m_uConfigValue_t[3] |= M55M1_FLASH_CONFIG_UARTPSL_SEL0;
            break;

        case 2:
            m_uConfigValue_t[3] |= M55M1_FLASH_CONFIG_UARTPSL_SEL1;
            break;

        case 1:
            m_uConfigValue_t[3] |= M55M1_FLASH_CONFIG_UARTPSL_SEL2;
            break;

        case 0:
        default:
            m_uConfigValue_t[3] |= M55M1_FLASH_CONFIG_UARTPSL_SEL3;
    }

    if (m_bCheckSCEN)
    {
        m_uConfigValue_t[4] &= ~(m_uFlashPageSize - 1);

        if (m_uConfigValue_t[4] >= NUMICRO_M55_LDROM_ADDR)
        {
            m_uConfigValue_t[4]  = NUMICRO_M55_LDROM_ADDR;
            m_uConfigValue_t[5]  = 0x00000001;
        }
        else
        {
            if (m_uConfigValue_t[4] > (NUMICRO_M55_APROM_ADDR + m_uProgramMemorySize - m_uFlashPageSize))
                m_uConfigValue_t[4] = (NUMICRO_M55_APROM_ADDR + m_uProgramMemorySize - m_uFlashPageSize);

            if (m_uConfigValue_t[4] < NUMICRO_M55_APROM_ADDR)
                m_uConfigValue_t[4] = NUMICRO_M55_APROM_ADDR;

            if ((m_uConfigValue_t[5] == 0x00000000) || (m_uConfigValue_t[5] == 0xFFFFFFFF))
                m_uConfigValue_t[5] = 0x00000001;

            if ((m_uConfigValue_t[4] + (m_uConfigValue_t[5] * m_uFlashPageSize)) > (NUMICRO_M55_APROM_ADDR + m_uProgramMemorySize))
                m_uConfigValue_t[5] = (NUMICRO_M55_APROM_ADDR + m_uProgramMemorySize - m_uConfigValue_t[4]) / m_uFlashPageSize;
        }

        if (m_uConfigValue_t[6] == 0xFFFFFFFF)
            m_uConfigValue_t[6] =  0x55AA5AA5;
    }
    else
    {
        m_uConfigValue_t[4] = 0xFFFFFFFF;
        m_uConfigValue_t[5] = 0xFFFFFFFF;
        m_uConfigValue_t[6] = 0xFFFFFFFF;
    }

    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
    m_uConfigValue[3] = m_uConfigValue_t[3];
    m_uConfigValue[4] = m_uConfigValue_t[4];
    m_uConfigValue[5] = m_uConfigValue_t[5];
    m_uConfigValue[6] = m_uConfigValue_t[6];
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_CWDT()
{
    /* Watchdog */
    switch (m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN)
    {
        case NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE:
            m_nRadioCWDTEN = 0;
            break;

        default:
            m_nRadioCWDTEN = 1;
    }
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_CBOD_8()
{
    /* Brown Out Voltage */
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

void CDialogChipSetting_CFG_M55M1::CFG2GUI_CBS_2()
{
    /* Boot Select */
    m_nRadioCBS = ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CBS_2_MODE) ? 0 : 1);
}

void CDialogChipSetting_CFG_M55M1::CFG2GUI_LOCK()
{
    /* ICE Lock */
    m_bCheckICELOCK = ((m_uConfigValue_t[0] & NUMICRO_M55_FLASH_CONFIG_ICELOCK) == 0 ? TRUE : FALSE);
}

void CDialogChipSetting_CFG_M55M1::GUI2CFG_CWDT()
{
    /* Watchdog */
    switch (m_nRadioCWDTEN)
    {
        case 0:
            m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
            m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE;
            break;

        default:
            if ((m_uConfigValue_t[0] & NUMICRO_FLASH_CONFIG_CWDTEN) == NUMICRO_FLASH_CONFIG_CWDTEN_INACTIVE)
            {
                m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CWDTEN;
                m_uConfigValue_t[0] |=  NUMICRO_FLASH_CONFIG_CWDTEN_ACTIVE;
            }
    }
}

void CDialogChipSetting_CFG_M55M1::GUI2CFG_CBOD_8()
{
    /* Brown Out Voltage */
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

void CDialogChipSetting_CFG_M55M1::GUI2CFG_CBS_2()
{
    /* Boot Select */
    m_uConfigValue_t[0] &= ~NUMICRO_FLASH_CONFIG_CBS_2_MODE;

    if (m_nRadioCBS == 0)
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_AP_IAP;
    else
        m_uConfigValue_t[0] |= NUMICRO_FLASH_CONFIG_CBS_LD_IAP;
}

void CDialogChipSetting_CFG_M55M1::GUI2CFG_LOCK()
{
    /* ICE Lock */
    if (m_bCheckICELOCK)
        m_uConfigValue_t[0] &= ~NUMICRO_M55_FLASH_CONFIG_ICELOCK;
    else
        m_uConfigValue_t[0] |=  NUMICRO_M55_FLASH_CONFIG_ICELOCK;
}

void CDialogChipSetting_CFG_M55M1::OnRadioClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1::OnCheckClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1::OnKillfocusEditSCAddrCount()
{
    UpdateData(TRUE);

    unsigned int uSC_BaseAddr, uSC_PageCount;

    uSC_BaseAddr  = ::_tcstoul(m_sSC_BaseAddr, NULL, 16);
    uSC_PageCount = ::_tcstoul(m_sSC_PageCount, NULL, 10);

    uSC_BaseAddr &= ~(m_uFlashPageSize - 1);

    if (uSC_BaseAddr >= NUMICRO_M55_LDROM_ADDR)
    {
        uSC_BaseAddr  = NUMICRO_M55_LDROM_ADDR;
        uSC_PageCount = 0x00000001;
    }
    else
    {
        if (uSC_BaseAddr > (NUMICRO_M55_APROM_ADDR + m_uProgramMemorySize - m_uFlashPageSize))
            uSC_BaseAddr = (NUMICRO_M55_APROM_ADDR + m_uProgramMemorySize - m_uFlashPageSize);

        if (uSC_BaseAddr < NUMICRO_M55_APROM_ADDR)
            uSC_BaseAddr = NUMICRO_M55_APROM_ADDR;

        if (uSC_PageCount == 0)
            uSC_PageCount = 1;

        if ((uSC_BaseAddr + (uSC_PageCount * m_uFlashPageSize)) > (NUMICRO_M55_APROM_ADDR + m_uProgramMemorySize))
            uSC_PageCount = (NUMICRO_M55_APROM_ADDR + m_uProgramMemorySize - uSC_BaseAddr) / m_uFlashPageSize;
    }

    m_uConfigValue[4] = uSC_BaseAddr;
    m_uConfigValue[5] = uSC_PageCount;

    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_CFG_M55M1::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}
