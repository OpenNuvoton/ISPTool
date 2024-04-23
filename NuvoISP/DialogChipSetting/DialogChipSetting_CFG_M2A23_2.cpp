// DialogChipSetting_CFG_M460.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "FlashInfo.h"
#include "DialogChipSetting_CFG_M2A23_2.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M2A23_2, CDialog)

CDialogChipSetting_CFG_M2A23_2::CDialogChipSetting_CFG_M2A23_2(int edit_8, CWnd *pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_edit_8(edit_8)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M460)

    for (int i = 0; i < 32; i++) {
        m_bCheckSP[i] = FALSE;
    }

    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M2A23_2::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_CFG_M2L31)
    DDX_Radio(pDX, IDC_RADIO_LEVEL_0,               m_nRadioLockLv);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_0,      m_bCheckSP[0]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_1,      m_bCheckSP[1]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_2,      m_bCheckSP[2]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_3,      m_bCheckSP[3]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_4,      m_bCheckSP[4]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_5,      m_bCheckSP[5]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_6,      m_bCheckSP[6]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_7,      m_bCheckSP[7]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_8,      m_bCheckSP[8]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_9,      m_bCheckSP[9]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_10,     m_bCheckSP[10]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_11,     m_bCheckSP[11]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_12,     m_bCheckSP[12]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_13,     m_bCheckSP[13]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_14,     m_bCheckSP[14]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_15,     m_bCheckSP[15]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_16,     m_bCheckSP[16]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_17,     m_bCheckSP[17]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_18,     m_bCheckSP[18]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_19,     m_bCheckSP[19]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_20,     m_bCheckSP[20]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_21,     m_bCheckSP[21]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_22,     m_bCheckSP[22]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_23,     m_bCheckSP[23]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_24,     m_bCheckSP[24]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_25,     m_bCheckSP[25]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_26,     m_bCheckSP[26]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_27,     m_bCheckSP[27]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_28,     m_bCheckSP[28]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_29,     m_bCheckSP[29]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_30,     m_bCheckSP[30]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_31,     m_bCheckSP[31]);    
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0,		m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1,		m_sConfigValue1);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_CFG_M2A23_2, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_CFG_M2L31)
    ON_BN_CLICKED(IDC_RADIO_LEVEL_0,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_LEVEL_1,					OnRadioClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_0,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_1,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_2,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_3,			OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_4,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_5,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_6,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_7,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_8,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_9,           OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_10,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_11,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_12,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_13,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_14,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_15,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_16,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_17,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_18,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_19,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_20,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_21,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_22,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_23,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_24,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_25,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_26,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_27,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_28,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_29,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_30,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_31,          OnCheckClick)

    ON_BN_CLICKED(IDC_BUTTON_SELECT_ALL_0,              OnButtonClickSelcet0)
    ON_BN_CLICKED(IDC_BUTTON_CLEAR_ALL_0,               OnButtonClickClear0)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 message handlers

BOOL CDialogChipSetting_CFG_M2A23_2::OnInitDialog()
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
    ConfigToGUI();
    LockUI();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();
    return TRUE;	// return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_CFG_M2A23_2::UpdateUI()
{
    // TODO: Add your control notification handler code here
}

void CDialogChipSetting_CFG_M2A23_2::LockUI()
{
    GetDlgItem(IDC_RADIO_LEVEL_0)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_RADIO_LEVEL_1)->EnableWindow(m_edit_8);

    for (int i = 0; i < 64; i++) {
        GetDlgItem(IDC_CHECK_SAFETY_PROTECT_0 + i)->EnableWindow(m_edit_8);
    }
    GetDlgItem(IDC_BUTTON_SELECT_ALL_0)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_BUTTON_CLEAR_ALL_0)->EnableWindow(m_edit_8);
}

void CDialogChipSetting_CFG_M2A23_2::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];

    CFG2GUI_CheckSP();
    CFG2GUI_LockLv();

    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M2A23_2::GUIToConfig()
{
    if (m_edit_8) {
        UpdateData(TRUE);
        m_uConfigValue_t[0] = m_uConfigValue[0];
        m_uConfigValue_t[1] = m_uConfigValue[1];

        GUI2CFG_CheckSP();
        GUI2CFG_LockLv();

        m_uConfigValue[0] = m_uConfigValue_t[0];
        m_uConfigValue[1] = m_uConfigValue_t[1];
    }
}

void CDialogChipSetting_CFG_M2A23_2::CFG2GUI_CheckSP()
{
    for (int i = 0; i < 32; i++) {
        m_bCheckSP[i] = ((m_uConfigValue_t[0] & (1 << i)) == 0 ? TRUE : FALSE);
    }
}

void CDialogChipSetting_CFG_M2A23_2::CFG2GUI_LockLv()
{
    switch (m_uConfigValue_t[1] & 0xFFFF) {
        case 0xFFFF:
            m_nRadioLockLv = 0;
            break;
        default:
            m_nRadioLockLv = 1;
    }
}

void CDialogChipSetting_CFG_M2A23_2::GUI2CFG_CheckSP()
{
    for (int i = 0; i < 32; i++) {
        if (m_bCheckSP[i]) {
            m_uConfigValue_t[0] &= ~(1 << i);
        }
        else {
            m_uConfigValue_t[0] |= (1 << i);
        }
    }
}

void CDialogChipSetting_CFG_M2A23_2::GUI2CFG_LockLv()
{
    m_uConfigValue_t[1] &= ~0xFFFF;
    switch (m_nRadioLockLv) {  
        case 0:
            m_uConfigValue_t[1] |= 0xFFFF;
            break;

        case 1:
            m_uConfigValue_t[1] |= 0x005A;
            break; 

        default:
            m_uConfigValue_t[1] |= 0xFFFF;
    }
}

void CDialogChipSetting_CFG_M2A23_2::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2A23_2::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2A23_2::OnButtonClickSelcet0()
{
    // TODO: Add your control notification handler code here
    m_uConfigValue[0] = 0x00000000;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2A23_2::OnButtonClickClear0()
{
    // TODO: Add your control notification handler code here
    m_uConfigValue[0] = 0xFFFFFFFF;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M2A23_2::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_CFG_M2A23_2::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}
