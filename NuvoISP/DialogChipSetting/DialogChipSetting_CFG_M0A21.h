#if !defined(AFX_DIALOGCHIPSETTING_CFG_M0A21_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_CFG_M0A21_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_CFG_M0A21.h : header file
//

#include "DialogResize.h"
#include "NumEdit.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M0A21 dialog

class CDialogChipSetting_CFG_M0A21 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M0A21)

// Construction
public:
    CDialogChipSetting_CFG_M0A21(CWnd *pParent = NULL);		// standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M0A21)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_CFG_M0A21 };

    int				m_nRadioCWDTEN;
    int				m_nRadioCFGXT1;
    int				m_nRadioCFGRPS;
    int				m_nRadioCBOV;
    int				m_nRadioCIOINI;
    int				m_nRadioRSTEXT;
    int				m_nRadioRSTWSEL;
    int				m_nRadioCBS;

    BOOL			m_bCheckCBORST;
    BOOL			m_bCheckCBODEN;
    BOOL			m_bCheckICELOCK;
    BOOL			m_bCheckLOCK;
    BOOL			m_bCheckDFEN;

    CNumEdit		m_DataFlashBase;
    CEdit			m_DataFlashSize;
    CSpinButtonCtrl	m_SpinDataFlashSize;

    CNumEdit		m_ALOCK;
    CString			m_sALOCK;

    CString			m_sDataFlashBase;
    CString			m_sDataFlashSize;

    CString			m_sConfigValue0;
    CString			m_sConfigValue1;
    CString			m_sConfigValue2;
    CString			m_sConfigValue3;

    unsigned int	m_uProgramMemorySize;
    unsigned int	m_uFlashPageSize;
    unsigned int	m_uConfigValue[4];
    unsigned int	m_uConfigValue_t[4];
    //}}AFX_DATA

protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_CFG_M0A21)
    BOOL OnInitDialog();
    void DoDataExchange(CDataExchange *pDX);	// DDX/DDV support

    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    void CFG2GUI_CWDT();
    void CFG2GUI_CBOD_4();
    void CFG2GUI_CBOD_2();
    void CFG2GUI_CIOINI();
    void CFG2GUI_CBS_4();
    void CFG2GUI_DFEN();
    void CFG2GUI_LOCK();
    void CFG2GUI_ALOCK();

    void GUI2CFG_CWDT();
    void GUI2CFG_CBOD_4();
    void GUI2CFG_CBOD_2();
    void GUI2CFG_CIOINI();
    void GUI2CFG_CBS_4();
    void GUI2CFG_DFEN();
    void GUI2CFG_LOCK();
    void GUI2CFG_ALOCK();

    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditAdvanceLock();
    afx_msg void OnKillfocusEditDataFlashBase();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M030G dialog

class CDialogChipSetting_CFG_M030G : public CDialogChipSetting_CFG_M0A21
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M030G)

// Construction
public:
    CDialogChipSetting_CFG_M030G(CWnd *pParent = NULL);		// standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M030G)
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    void CFG2GUI_CWDT();
    void GUI2CFG_CWDT();
};

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M031 dialog

class CDialogChipSetting_CFG_M031 : public CDialogChipSetting_CFG_M0A21
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M031)

// Construction
public:
    CDialogChipSetting_CFG_M031(CWnd *pParent = NULL);		// standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M031)
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();
};

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M451 dialog

class CDialogChipSetting_CFG_M451 : public CDialogChipSetting_CFG_M0A21
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M451)

// Construction
public:
    CDialogChipSetting_CFG_M451(CWnd *pParent = NULL);		// standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M451)
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    void CFG2GUI_CBOD_4();
    void GUI2CFG_CBOD_4();
};

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M471 dialog

class CDialogChipSetting_CFG_M471 : public CDialogChipSetting_CFG_M0A21
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M471)

// Construction
public:
    CDialogChipSetting_CFG_M471(CWnd *pParent = NULL);		// standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M471)
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_CFG_M0A21_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
