#if !defined(AFX_DIALOGCONFIGURATION_M2003_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_M2003_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//
#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M480 dialog

class CDialogConfiguration_M2003 : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_M2003(unsigned int uProgramMemorySize = 32 * 1024,
        unsigned int uPID = 0,
        UINT nIDTemplate = IDD_DIALOG_CONFIGURATION_M2003,
        CWnd* pParent = NULL);   // standard constructor

    CAppConfig::M480_configs_t m_ConfigValue;

    // Dialog Data
        //{{AFX_DATA(CDialogConfiguration_M480)
    //	enum { IDD = IDD_DIALOG_CONFIGURATION_M480 };
    int		m_nRadioBov;
    int		m_nRadioBS;
    int		m_nRadioSPIM;
    int		m_nRadioUART;
    int     m_nRadioWDT;
    CString	m_sConfigValue0;
    CString	m_sConfigValue2;
    BOOL	m_bCheckBrownOutDetect;
    BOOL	m_bCheckBrownOutReset;
    BOOL	m_bSecurityLock;
    CNumEdit		m_ALOCK;
    CString			m_sALOCK;
    BOOL	m_bICELock;
    int		m_nRadioIO;

    unsigned int	m_uProgramMemorySize;
    unsigned int	m_uPID;
    //}}AFX_DATA

// Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_M480)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

// Implementation
protected:
    virtual void ConfigToGUI(int nEventID);
    virtual void GUIToConfig(int nEventID);
    void OnGUIEvent(int nEventID = 0);

    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_M480)
    virtual BOOL OnInitDialog();
    afx_msg void OnButtonClick();
    virtual afx_msg void OnKillfocusEditAdvanceLock();
    virtual void OnOK();
    //virtual afx_msg void OnDeltaposSpinDataFlashSize(NMHDR* pNMHDR, LRESULT* pResult);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_M2003_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
