#if !defined(AFX_DIALOGCONFIGURATION_M05X_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_M05X_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M05x dialog

class CDialogConfiguration_M05x : public CDialogResize
{
// Construction
public:
    CDialogConfiguration_M05x(UINT nIDTemplate = IDD_DIALOG_CONFIGURATION_M051, CWnd *pParent = NULL);   // standard constructor

    CAppConfig::M05x_configs_t m_ConfigValue;

// Dialog Data
    //{{AFX_DATA(CDialogConfiguration_M05x)
    enum { IDD = IDD_DIALOG_CONFIGURATION_M051 };
    int		m_nRadioBov;
    int		m_nRadioBS;
    CString	m_sConfigValue0;
    BOOL	m_bCheckBrownOutDetect;
    BOOL	m_bCheckBrownOutReset;
    BOOL	m_bClockFilterEnable;
    BOOL	m_bSecurityLock;
    //}}AFX_DATA

// Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_M05x)
protected:
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

// Implementation
protected:
    virtual void ConfigToGUI(int nEventID = 0);
    virtual void GUIToConfig(int nEventID = 0);

    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_M05x)
    virtual BOOL OnInitDialog();
    afx_msg void OnButtonClick();
    virtual void OnOK();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

class CDialogConfiguration_M05XAN : public CDialogConfiguration_M05x
{
public:
    CDialogConfiguration_M05XAN(CWnd *pParent = NULL);
};

class CDialogConfiguration_M05XBN : public CDialogConfiguration_M05x
{
public:
    CDialogConfiguration_M05XBN(CWnd *pParent = NULL);
protected:
    virtual BOOL OnInitDialog();
};

class CDialogConfiguration_M05XDN : public CDialogConfiguration_M05x
{
public:
    CDialogConfiguration_M05XDN(UINT nIDTemplate = IDD_DIALOG_CONFIGURATION_M051CN, CWnd *pParent = NULL);
    int		m_nRadioIO;
    BOOL	m_bWDTEnable;
    BOOL	m_bWDTPowerDown;
protected:
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support
    virtual void ConfigToGUI(int nEventID = 0);
    virtual void GUIToConfig(int nEventID = 0);
    afx_msg void OnCheckClickWDTPD();
    DECLARE_MESSAGE_MAP()
};

class CDialogConfiguration_M058SAN : public CDialogConfiguration_M05XDN
{
public:
    CDialogConfiguration_M058SAN(CWnd *pParent = NULL);
    int		m_nRadioGP7;
protected:
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support
    virtual void ConfigToGUI(int nEventID = 0);
    virtual void GUIToConfig(int nEventID = 0);
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
