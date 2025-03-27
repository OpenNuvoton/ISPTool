// DialogChipSetting_APWPROT.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "Lang.h"
#include "DialogChipSetting_APWPROT.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_APWPROT dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_APWPROT, CDialog)

CDialogChipSetting_APWPROT::CDialogChipSetting_APWPROT(CWnd* pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_uAPROMAddr(NUMICRO_FLASH_APROM_ADDR)
    , m_uAPROMSize(0)
    , m_uRegionNum(64)
    , m_uRegionSize(0x2000)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_APWPROT)
    m_nRadioAPWPLVL = -1;
    m_nRadioAPWPPIN = -1;

    for (int i = 0; i < 64; i++)
    {
        m_bCheckAPPROEN[i] = FALSE;
    }

    m_sConfigValue8 = _T("");
    m_sConfigValue9 = _T("");
    m_sConfigValue10 = _T("");
    //}}AFX_DATA_INIT
}

CDialogChipSetting_APWPROT::~CDialogChipSetting_APWPROT()
{
}

void CDialogChipSetting_APWPROT::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_APWPROT)
    DDX_Radio(pDX, IDC_RADIO_APROM_WPROT_LEVEL0,    m_nRadioAPWPLVL);
    DDX_Radio(pDX, IDC_RADIO_APROM_WPROT_PIN_A,     m_nRadioAPWPPIN);

    for (unsigned int nIndex = 0; nIndex < 64; nIndex++)
        DDX_Check(pDX, IDC_CHECK_APROM_WPROT_0 + nIndex, m_bCheckAPPROEN[nIndex]);

    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_8,        m_sConfigValue8);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_9,        m_sConfigValue9);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_10,       m_sConfigValue10);
    //}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDialogChipSetting_APWPROT, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_APWPROT)
    ON_BN_CLICKED(IDC_BUTTON_APROM_WPROT0_SELECT_ALL,   OnButtonSelectAll0)
    ON_BN_CLICKED(IDC_BUTTON_APROM_WPROT0_CLEAR_ALL,    OnButtonClearAll0)
    ON_BN_CLICKED(IDC_BUTTON_APROM_WPROT1_SELECT_ALL,   OnButtonSelectAll1)
    ON_BN_CLICKED(IDC_BUTTON_APROM_WPROT1_CLEAR_ALL,    OnButtonClearAll1)

    ON_CONTROL_RANGE(BN_CLICKED, IDC_CHECK_APROM_WPROT_0, IDC_CHECK_APROM_WPROT_63,             OnCheckClick)
    ON_CONTROL_RANGE(BN_CLICKED, IDC_RADIO_APROM_WPROT_LEVEL0, IDC_RADIO_APROM_WPROT_LEVEL3,    OnRadioClick)
    ON_CONTROL_RANGE(BN_CLICKED, IDC_RADIO_APROM_WPROT_PIN_A, IDC_RADIO_APROM_WPROT_PIN_D,      OnRadioClick)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_APWPROT message handlers

BOOL CDialogChipSetting_APWPROT::OnInitDialog()
{
    CDialog::OnInitDialog();

    unsigned int i;

    for (i = 0; i < 64; i++)
    {
        CString str;

        str.Format(_I(IDS_WPROT_REGION), i, m_uAPROMAddr + (m_uRegionSize * i), (m_uAPROMAddr + (m_uRegionSize * (i + 1))) - 1);

        GetDlgItem(IDC_CHECK_APROM_WPROT_0 + i)->SetWindowText(str);
    }

    m_uConfigValue_c[0] = 0x00000000;
    m_uConfigValue_c[1] = 0x00000000;

    for (i = (m_uAPROMSize / m_uRegionSize); i < 64; i++)
    {
        GetDlgItem(IDC_CHECK_APROM_WPROT_0 + i)->EnableWindow(FALSE);
        m_uConfigValue_c[(i >> 5) & 0x01] |= (1 << (i & 0x1F));
    }

    if ((m_uAPROMSize / m_uRegionSize) <= 32)
    {
        GetDlgItem(IDC_STATIC_APROM_WPROT1)->EnableWindow(FALSE);
        GetDlgItem(IDC_BUTTON_APROM_WPROT1_SELECT_ALL)->EnableWindow(FALSE);
        GetDlgItem(IDC_BUTTON_APROM_WPROT1_CLEAR_ALL)->EnableWindow(FALSE);
    }

    for (i = m_uRegionNum; i < 64; i++)
    {
        GetDlgItem(IDC_CHECK_APROM_WPROT_0 + i)->ShowWindow(SW_HIDE);
    }

    if (m_uRegionNum <= 32)
    {
        GetDlgItem(IDC_STATIC_APROM_WPROT1)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_BUTTON_APROM_WPROT1_SELECT_ALL)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_BUTTON_APROM_WPROT1_CLEAR_ALL)->ShowWindow(SW_HIDE);
    }

    UpdateUI();

    ConfigToGUI();

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;    // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_APWPROT::UpdateUI()
{
    GetDlgItem(IDC_RADIO_APROM_WPROT_PIN_A)->SetWindowText(_T("GPB.2"));
    GetDlgItem(IDC_RADIO_APROM_WPROT_PIN_B)->SetWindowText(_T("GPA.6"));
    GetDlgItem(IDC_RADIO_APROM_WPROT_PIN_C)->SetWindowText(_T("GPC.14"));
    GetDlgItem(IDC_RADIO_APROM_WPROT_PIN_D)->SetWindowText(_T("GPD.13"));
}

void CDialogChipSetting_APWPROT::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    unsigned int i;

    for (i = 0; i < 64; i++)
    {
        m_bCheckAPPROEN[i] = ((m_uConfigValue_t[(i >> 5) & 0x01] & (1 << (i & 0x1F))) == 0) ? TRUE : FALSE;
    }

    switch (m_uConfigValue_t[2] & 0xFFFF)
    {
        case 0xFFFF:
            m_nRadioAPWPLVL = 0;
            break;

        case 0x335A:
        case 0x335B:
        case 0x335C:
        case 0x335D:
            m_nRadioAPWPLVL = 2;
            break;

        case 0x995A:
        case 0x995B:
        case 0x995C:
        case 0x995D:
            m_nRadioAPWPLVL = 3;
            break;

        default:
            m_nRadioAPWPLVL = 1;
    }

    if ((m_nRadioAPWPLVL == 2) || (m_nRadioAPWPLVL == 3))
    {
        switch (m_uConfigValue_t[2] & 0x0F)
        {
            case 0x0A:
                m_nRadioAPWPPIN = 0;
                break;

            case 0x0B:
                m_nRadioAPWPPIN = 1;
                break;

            case 0x0C:
                m_nRadioAPWPPIN = 2;
                break;

            case 0x0D:
                m_nRadioAPWPPIN = 3;
                break;
        }
    }

    if (m_nRadioAPWPPIN == -1)
        m_nRadioAPWPPIN = 0;

    m_sConfigValue8.Format(_T("0x%08X"),  m_uConfigValue_t[0]);
    m_sConfigValue9.Format(_T("0x%08X"),  m_uConfigValue_t[1]);
    m_sConfigValue10.Format(_T("0x%08X"), m_uConfigValue_t[2]);

    UpdateData(FALSE);
}

void CDialogChipSetting_APWPROT::GUIToConfig()
{
    UpdateData(TRUE);

    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    unsigned int i;

    for (i = 0; i < 64; i++)
    {
        if (m_bCheckAPPROEN[i])
            m_uConfigValue_t[(i >> 5) & 0x01] &= ~(1 << (i & 0x1F));
        else
            m_uConfigValue_t[(i >> 5) & 0x01] |= (1 << (i & 0x1F));
    }

    m_uConfigValue_t[0] |= m_uConfigValue_c[0];
    m_uConfigValue_t[1] |= m_uConfigValue_c[1];

    m_uConfigValue_t[2] &= ~0xFFFF0000;

    switch (m_nRadioAPWPLVL)
    {
        case 0:
            m_uConfigValue_t[2] = 0xFFFF;
            break;

        case 1:
            if ((m_uConfigValue_t[2] == 0xFFFF) ||
            (m_uConfigValue_t[2] == 0x335A) || (m_uConfigValue_t[2] == 0x335B) || (m_uConfigValue_t[2] == 0x335C) || (m_uConfigValue_t[2] == 0x335D) ||
            (m_uConfigValue_t[2] == 0x995A) || (m_uConfigValue_t[2] == 0x995B) || (m_uConfigValue_t[2] == 0x995C) || (m_uConfigValue_t[2] == 0x995D))
                m_uConfigValue_t[2] = 0x005A;

            break;

        case 2:
            switch (m_nRadioAPWPPIN)
            {
                case 0:
                    m_uConfigValue_t[2] = 0x335A;
                    break;

                case 1:
                    m_uConfigValue_t[2] = 0x335B;
                    break;

                case 2:
                    m_uConfigValue_t[2] = 0x335C;
                    break;

                case 3:
                    m_uConfigValue_t[2] = 0x335D;
                    break;
            }

            break;

        case 3:
            switch (m_nRadioAPWPPIN)
            {
                case 0:
                    m_uConfigValue_t[2] = 0x995A;
                    break;

                case 1:
                    m_uConfigValue_t[2] = 0x995B;
                    break;

                case 2:
                    m_uConfigValue_t[2] = 0x995C;
                    break;

                case 3:
                    m_uConfigValue_t[2] = 0x995D;
                    break;
            }

            break;
    }

    m_uConfigValue_t[2] |= 0xFFFF0000;

    m_uConfigValue[0] = m_uConfigValue_t[0];
    m_uConfigValue[1] = m_uConfigValue_t[1];
    m_uConfigValue[2] = m_uConfigValue_t[2];
}

void CDialogChipSetting_APWPROT::OnRadioClick(unsigned int nID)
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_APWPROT::OnCheckClick(unsigned int nID)
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_APWPROT::OnButtonSelectAll0()
{
    if (m_uConfigValue[0] == m_uConfigValue_c[0])
        return;

    m_uConfigValue[0] = m_uConfigValue_c[0];

    ConfigToGUI();
}

void CDialogChipSetting_APWPROT::OnButtonClearAll0()
{
    if (m_uConfigValue[0] == 0xFFFFFFFF)
        return;

    m_uConfigValue[0] = 0xFFFFFFFF;

    ConfigToGUI();
}

void CDialogChipSetting_APWPROT::OnButtonSelectAll1()
{
    if (m_uConfigValue[1] == m_uConfigValue_c[1])
        return;

    m_uConfigValue[1] = m_uConfigValue_c[1];

    ConfigToGUI();
}

void CDialogChipSetting_APWPROT::OnButtonClearAll1()
{
    if (m_uConfigValue[1] == 0xFFFFFFFF)
        return;

    m_uConfigValue[1] = 0xFFFFFFFF;

    ConfigToGUI();
}

void CDialogChipSetting_APWPROT::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_APWPROT::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_APWPROT2 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_APWPROT2, CDialog)

CDialogChipSetting_APWPROT2::CDialogChipSetting_APWPROT2(CWnd* pParent /*=NULL*/)
    : CDialogChipSetting_APWPROT(pParent)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_APWPROT2)
    // NOTE: the ClassWizard will add member initialization here
    //}}AFX_DATA_INIT
}

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_APWPROT2 message handlers

void CDialogChipSetting_APWPROT2::UpdateUI()
{
    int i;

    for (i = 0; i < 32; i++)
    {
        GetDlgItem(IDC_CHECK_APROM_WPROT_32  + i)->ShowWindow(SW_HIDE);
    }

    GetDlgItem(IDC_STATIC_APROM_WPROT1)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_BUTTON_APROM_WPROT1_SELECT_ALL)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_BUTTON_APROM_WPROT1_CLEAR_ALL)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_APROM_WPROT_LEVEL2)->EnableWindow(FALSE);
    GetDlgItem(IDC_RADIO_APROM_WPROT_LEVEL3)->EnableWindow(FALSE);
    GetDlgItem(IDC_GROUP_APROM_WPROT_PIN)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_APROM_WPROT_PIN_A)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_APROM_WPROT_PIN_B)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_APROM_WPROT_PIN_C)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_RADIO_APROM_WPROT_PIN_D)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_STATIC_CONFIG_9)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_STATIC_CONFIG_VALUE_9)->ShowWindow(SW_HIDE);
}
