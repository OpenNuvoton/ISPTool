#if !defined(AFX_DIALOGCONFIGURATION_NANO100_H__E0CD7A09_C5DC_481E_A871_03B422A615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_NANO100_H__E0CD7A09_C5DC_481E_A871_03B422A615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_Nano100 dialog

class CDialogConfiguration_Nano100 : public CDialogResize
{
// Construction
public:
    CDialogConfiguration_Nano100(unsigned int uProgramMemorySize = 123 * 1024,
                                 CWnd *pParent = NULL);   // standard constructor

    CAppConfig::Nano100_configs_t m_ConfigValue;

// Dialog Data
    //{{AFX_DATA(CDialogConfiguration_Nano100)
    enum { IDD = IDD_DIALOG_CONFIGURATION_NANO100 };
    CNumEdit	m_FlashBaseAddress;
    CEdit	m_DataFlashSize;
    int		m_nRadioBor;
    int		m_nRadioBS;
    CString	m_sConfigValue0;
    CString	m_sConfigValue1;
    CString	m_sFlashBaseAddress;
    CString	m_sDataFlashSize;
    BOOL	m_bClockFilterEnable;
    BOOL	m_bDataFlashEnable;
    BOOL	m_bSecurityLock;
    BOOL	m_bWDTEnable;
    CSpinButtonCtrl	m_SpinDataFlashSize;

    unsigned int	m_uProgramMemorySize;
    //}}AFX_DATA

// Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_Nano100)
protected:
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

// Implementation
protected:
    void ConfigToGUI();
    void GUIToConfig();

    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_Nano100)
    virtual BOOL OnInitDialog();
    afx_msg void OnButtonClick();
    afx_msg void OnKillfocusEditFlashBaseAddress();
    virtual void OnOK();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_Nano100AN dialog

class CDialogConfiguration_Nano100AN : public CDialogConfiguration_Nano100
{
// Construction
public:
    CDialogConfiguration_Nano100AN(unsigned int uProgramMemorySize = 64 * 1024,
                                   CWnd *pParent = NULL);   // standard constructor

protected:
    virtual BOOL OnInitDialog();
};

#endif // !defined(AFX_DIALOGCONFIGURATION_NANO100_H__E0CD7A09_C5DC_481E_A871_03B422A615F8__INCLUDED_)
