#if !defined(AFX_DIALOGCHIPSETTING_CFG_M55M1_2_H__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_CFG_M55M1_2_H__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_CFG_M460.h : header file
//

#include "DialogResize.h"
#include "NumEdit.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 dialog

class CDialogChipSetting_CFG_M55M1_2 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M55M1_2)

// Construction
public:
    CDialogChipSetting_CFG_M55M1_2(int edit_8, CWnd *pParent = NULL);		// standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M460)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_CFG_M55M1_2 };

    int				m_nRadioLockLv;
    int				m_nRadioLockPin;
    int				m_edit_8;

    BOOL			m_bCheckSP[64];

    CString			m_sConfigValue0;
    CString			m_sConfigValue1;
    CString			m_sConfigValue2;

    unsigned int	m_uPID;
    unsigned int	m_uDID;
    unsigned int	m_uConfigValue[3];
    unsigned int	m_uConfigValue_t[3];
    //}}AFX_DATA

    unsigned int	m_uProgramMemorySize;
    unsigned int	m_uFlashPageSize;
protected:

    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_CFG_M0A21)
    BOOL OnInitDialog();
    void DoDataExchange(CDataExchange *pDX);	// DDX/DDV support

    virtual void UpdateUI();
    virtual void LockUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    void CFG2GUI_CheckSP();
    void CFG2GUI_LockLv();
    void CFG2GUI_LockPin();

    void GUI2CFG_CheckSP();
    void GUI2CFG_LockLv();
    void GUI2CFG_LockPin();

    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();
    afx_msg void OnButtonClickSelcet0();
    afx_msg void OnButtonClickSelcet1();
    afx_msg void OnButtonClickClear0();
    afx_msg void OnButtonClickClear1();
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_CFG_M460_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
