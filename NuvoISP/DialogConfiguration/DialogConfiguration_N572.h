#if !defined(AFX_DIALOGCONFIGURATION_N572_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_N572_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_N572 dialog

class CDialogConfiguration_N572 : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_N572(CWnd* pParent = NULL);   // standard constructor

    static CString GetConfigWarning(const CAppConfig::N572_configs_t &config);

    CAppConfig::N572_configs_t m_ConfigValue;

    // Dialog Data
    //{{AFX_DATA(CDialogConfiguration_N572)
    enum { IDD = IDD_DIALOG_CONFIGURATION_N572 };
    CNumEdit    m_FlashInfo4;
    CNumEdit    m_FlashInfo2;
    CNumEdit    m_FlashInfo3;
    CNumEdit    m_FlashInfo1;
    CString m_sConfigValue0;
    BOOL    m_bClockFilterEnable;
    BOOL    m_bSecurityLock;
    BOOL    m_bWatchDogEnable;
    BOOL    m_bCVDEN;
    int     m_nRadioClk;
    int     m_nRadioCVDTV;
    CString m_sFlashInfo1;
    CString m_sFlashInfo2;
    CString m_sFlashInfo3;
    CString m_sFlashInfo4;
    //}}AFX_DATA


    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_N572)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

    // Implementation
protected:
    void ConfigToGUI();
    void GUIToConfig();

    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_N572)
    virtual BOOL OnInitDialog();
    virtual void OnOK();
    afx_msg void OnButtonClick();
    afx_msg void OnChangeEditFlashInfo1();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
