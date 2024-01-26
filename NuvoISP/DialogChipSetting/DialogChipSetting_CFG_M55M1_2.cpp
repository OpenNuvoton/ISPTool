// DialogChipSetting_CFG_M460.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "FlashInfo.h"
#include "DialogChipSetting_CFG_M55M1_2.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_CFG_M55M1_2, CDialog)

CDialogChipSetting_CFG_M55M1_2::CDialogChipSetting_CFG_M55M1_2(int edit_8, CWnd *pParent /*=NULL*/)
    : CDialogResize(IDD, pParent)
    , m_edit_8(edit_8)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_CFG_M460)

    for (int i = 0; i < 64; i++) {
        m_bCheckSP[i] = FALSE;
    }

    m_sConfigValue0 = _T("");
    m_sConfigValue1 = _T("");
    m_sConfigValue2 = _T("");
    //}}AFX_DATA_INIT
}

void CDialogChipSetting_CFG_M55M1_2::DoDataExchange(CDataExchange *pDX)
{
    CDialogResize::DoDataExchange(pDX);
    DDX_Radio(pDX, IDC_RADIO_LEVEL_0,               m_nRadioLockLv);
    DDX_Radio(pDX, IDC_RADIO_PIN_0,                 m_nRadioLockPin);
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
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_32,     m_bCheckSP[32]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_33,     m_bCheckSP[33]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_34,     m_bCheckSP[34]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_35,     m_bCheckSP[35]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_36,     m_bCheckSP[36]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_37,     m_bCheckSP[37]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_38,     m_bCheckSP[38]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_39,     m_bCheckSP[39]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_40,     m_bCheckSP[40]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_41,     m_bCheckSP[41]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_42,     m_bCheckSP[42]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_43,     m_bCheckSP[43]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_44,     m_bCheckSP[44]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_45,     m_bCheckSP[45]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_46,     m_bCheckSP[46]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_47,     m_bCheckSP[47]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_48,     m_bCheckSP[48]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_49,     m_bCheckSP[49]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_50,     m_bCheckSP[50]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_51,     m_bCheckSP[51]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_52,     m_bCheckSP[52]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_53,     m_bCheckSP[53]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_54,     m_bCheckSP[54]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_55,     m_bCheckSP[55]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_56,     m_bCheckSP[56]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_57,     m_bCheckSP[57]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_58,     m_bCheckSP[58]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_59,     m_bCheckSP[59]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_60,     m_bCheckSP[60]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_61,     m_bCheckSP[61]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_62,     m_bCheckSP[62]);
    DDX_Check(pDX, IDC_CHECK_SAFETY_PROTECT_63,     m_bCheckSP[63]);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_0,		m_sConfigValue0);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_1,		m_sConfigValue1);
    DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_2,		m_sConfigValue2);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_CFG_M55M1_2, CDialog)
    ON_BN_CLICKED(IDC_RADIO_LEVEL_0,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_LEVEL_1,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_LEVEL_2,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_LEVEL_3,					OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_PIN_0,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_PIN_1,						OnRadioClick)
    //ON_BN_CLICKED(IDC_RADIO_PIN_2,						OnRadioClick)
    ON_BN_CLICKED(IDC_RADIO_PIN_3,						OnRadioClick)

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
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_32,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_33,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_34,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_35,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_36,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_37,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_38,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_39,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_40,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_41,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_42,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_43,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_44,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_45,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_46,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_47,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_48,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_49,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_50,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_51,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_52,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_53,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_54,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_55,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_56,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_57,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_58,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_59,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_60,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_61,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_62,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SAFETY_PROTECT_63,          OnCheckClick)

    ON_BN_CLICKED(IDC_BUTTON_SELECT_ALL_0,              OnButtonClickSelcet0)
    ON_BN_CLICKED(IDC_BUTTON_CLEAR_ALL_0,               OnButtonClickClear0)
    ON_BN_CLICKED(IDC_BUTTON_SELECT_ALL_1,              OnButtonClickSelcet1)
    ON_BN_CLICKED(IDC_BUTTON_CLEAR_ALL_1,               OnButtonClickClear1)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 message handlers

BOOL CDialogChipSetting_CFG_M55M1_2::OnInitDialog()
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

void CDialogChipSetting_CFG_M55M1_2::UpdateUI()
{
    // TODO: Add your control notification handler code here
}

void CDialogChipSetting_CFG_M55M1_2::LockUI()
{
    GetDlgItem(IDC_RADIO_LEVEL_0)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_RADIO_LEVEL_1)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_RADIO_LEVEL_2)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_RADIO_LEVEL_3)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_RADIO_PIN_0)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_RADIO_PIN_1)->EnableWindow(m_edit_8);
    //GetDlgItem(IDC_RADIO_PIN_2)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_RADIO_PIN_3)->EnableWindow(m_edit_8);
    for (int i = 0; i < 64; i++) {
        GetDlgItem(IDC_CHECK_SAFETY_PROTECT_0 + i)->EnableWindow(m_edit_8);
    }
    GetDlgItem(IDC_BUTTON_SELECT_ALL_0)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_BUTTON_CLEAR_ALL_0)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_BUTTON_SELECT_ALL_1)->EnableWindow(m_edit_8);
    GetDlgItem(IDC_BUTTON_CLEAR_ALL_1)->EnableWindow(m_edit_8);
}

void CDialogChipSetting_CFG_M55M1_2::ConfigToGUI()
{
    m_uConfigValue_t[0] = m_uConfigValue[0];
    m_uConfigValue_t[1] = m_uConfigValue[1];
    m_uConfigValue_t[2] = m_uConfigValue[2];

    CFG2GUI_CheckSP();
    CFG2GUI_LockLv();
    CFG2GUI_LockPin();

    m_sConfigValue0.Format(_T("0x%08X"), m_uConfigValue_t[0]);
    m_sConfigValue1.Format(_T("0x%08X"), m_uConfigValue_t[1]);
    m_sConfigValue2.Format(_T("0x%08X"), m_uConfigValue_t[2]);
    UpdateData(FALSE);
}

void CDialogChipSetting_CFG_M55M1_2::GUIToConfig()
{
    if (m_edit_8) {
        UpdateData(TRUE);
        m_uConfigValue_t[0] = m_uConfigValue[0];
        m_uConfigValue_t[1] = m_uConfigValue[1];
        m_uConfigValue_t[2] = m_uConfigValue[2];

        GUI2CFG_CheckSP();
        GUI2CFG_LockLv();
        GUI2CFG_LockPin();

        m_uConfigValue[0] = m_uConfigValue_t[0];
        m_uConfigValue[1] = m_uConfigValue_t[1];
        m_uConfigValue[2] = m_uConfigValue_t[2];
    }
}

void CDialogChipSetting_CFG_M55M1_2::CFG2GUI_CheckSP()
{
    for (int i = 0; i < 32; i++) {
        m_bCheckSP[i] = ((m_uConfigValue_t[0] & (1 << i)) == 0 ? TRUE : FALSE);
        m_bCheckSP[i+32] = ((m_uConfigValue_t[1] & (1 << i)) == 0 ? TRUE : FALSE);
    }
}

void CDialogChipSetting_CFG_M55M1_2::CFG2GUI_LockLv()
{
    switch (m_uConfigValue_t[2] & 0xFFFF) {
        case 0xFFFF:
            m_nRadioLockLv = 0;
            break;

        case 0x005A:
            m_nRadioLockLv = 1;
            break;

        case 0x335A:
        case 0x335B:
        case 0x335C:
        case 0x335D:
            m_nRadioLockLv = 2;
            break;

        case 0x995A:
        case 0x995B:
        case 0x995C:
        case 0x995D:
            m_nRadioLockLv = 3;
            break;

        default:
            m_nRadioLockLv = 1;
    }
}

void CDialogChipSetting_CFG_M55M1_2::CFG2GUI_LockPin()
{
    switch (m_uConfigValue_t[2] & 0x000F) {
        case 0xF:
        case 0xA:
            m_nRadioLockPin = 0;
            break;

        case 0xB:
            m_nRadioLockPin = 1;
            break;

        case 0xC:
            m_nRadioLockPin = 2;
            break;

        case 0xD:
            m_nRadioLockPin = 3;
            break;

        default:
            m_nRadioLockPin = 0;
            
    }
}

void CDialogChipSetting_CFG_M55M1_2::GUI2CFG_CheckSP()
{
    for (int i = 0; i < 32; i++) {
        if (m_bCheckSP[i]) {
            m_uConfigValue_t[0] &= ~(1 << i);
        }
        else {
            m_uConfigValue_t[0] |= (1 << i);
        }
        if (m_bCheckSP[i + 32]) {
            m_uConfigValue_t[1] &= ~(1 << i);
        }
        else {
            m_uConfigValue_t[1] |= (1 << i);
        }
    }
}

void CDialogChipSetting_CFG_M55M1_2::GUI2CFG_LockLv()
{
    m_uConfigValue_t[2] &= ~0xFFFF;
    switch (m_nRadioLockLv) {  
        case 0:
            m_uConfigValue_t[2] |= 0xFFFF;
            break;

        case 1:
            m_uConfigValue_t[2] |= 0x005A;
            break;

        case 2:
            m_uConfigValue_t[2] |= 0x335A;
            break;

        case 3:
            m_uConfigValue_t[2] |= 0x995A;
            break;

        default:
            m_uConfigValue_t[2] |= 0xFFFF;
    }
}

void CDialogChipSetting_CFG_M55M1_2::GUI2CFG_LockPin()
{
    m_uConfigValue_t[2] &= ~0xF;
    if (m_nRadioLockLv >= 2) {
        switch (m_nRadioLockPin) {
            case 0:
                m_uConfigValue_t[2] |= 0xA;
                break;

            case 1:
                m_uConfigValue_t[2] |= 0xB;
                break;

            case 2:
                m_uConfigValue_t[2] |= 0xC;
                break;

            case 3:
                m_uConfigValue_t[2] |= 0xD;
                break;

            default:
                m_uConfigValue_t[2] |= 0xA;
        }
    }
    else if (m_nRadioLockLv == 1){
        m_uConfigValue_t[2] |= 0xA;
    }
    else {
        m_uConfigValue_t[2] |= 0xF;
    }
}

void CDialogChipSetting_CFG_M55M1_2::OnRadioClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1_2::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1_2::OnButtonClickSelcet0()
{
    // TODO: Add your control notification handler code here
    m_uConfigValue[0] = 0x00000000;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1_2::OnButtonClickSelcet1()
{
    // TODO: Add your control notification handler code here
    m_uConfigValue[1] = 0x00000000;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1_2::OnButtonClickClear0()
{
    // TODO: Add your control notification handler code here
    m_uConfigValue[0] = 0xFFFFFFFF;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1_2::OnButtonClickClear1()
{
    // TODO: Add your control notification handler code here
    m_uConfigValue[1] = 0xFFFFFFFF;
    ConfigToGUI();
}

void CDialogChipSetting_CFG_M55M1_2::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK();
}

void CDialogChipSetting_CFG_M55M1_2::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel();
}
