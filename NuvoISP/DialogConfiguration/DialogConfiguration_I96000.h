#if !defined(AFX_DIALOGCONFIGURATION_I96000_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_I96000_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_I96000 dialog

class CDialogConfiguration_I96000 : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_I96000(CWnd* pParent = NULL);   // standard constructor

    static CString GetConfigWarning(const CAppConfig::I96000_configs_t &config);

    CAppConfig::I96000_configs_t m_ConfigValue;

    // Dialog Data
    //{{AFX_DATA(CDialogConfiguration_I96000)
    enum { IDD = IDD_DIALOG_CONFIGURATION_I96000 };
    CString m_sConfigValue0;
    CString m_sConfigValue1;
    CString m_sConfigValue2;
    BOOL    m_bCheckBrownOutDetect;
    BOOL    m_bCheckBrownOutReset;
    BOOL    m_bSecurityLock;
    BOOL    m_bICELock;
    BOOL    m_bCheckJTAGBootFail;
    BOOL    m_bCheckClearRstSts;
    BOOL    m_bCheckJTAGBootSuccess;
    BOOL    m_bPDWakeupPin; //PD
    BOOL    m_bPCWakeupPin; //PC
    BOOL    m_bPBWakeupPin; //PB
    BOOL    m_bPAWakeupPin; //PA
    int     m_nRadioBov;
    int     m_nRadioIO;
    int     m_nRadioStrapSel;
    int     m_nRadioSPIMPinSel;
    int     m_nRadioAppLoadFail;
    //Combobox
    int     m_nComboboxRetryCount;
    int     m_nComboboxPDWakeupPinSel;
    int     m_nComboboxPCWakeupPinSel;
    int     m_nComboboxPBWakeupPinSel;
    int     m_nComboboxPAWakeupPinSel;
    //}}AFX_DATA


    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_I96000)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

    // Implementation
protected:
    void ConfigToGUI(int nEventID);
    void GUIToConfig(int nEventID);
    void OnGUIEvent(int nEventID = 0);

    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_I96000)
    virtual BOOL OnInitDialog();
    afx_msg void OnButtonClick();
    virtual void OnOK();
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
