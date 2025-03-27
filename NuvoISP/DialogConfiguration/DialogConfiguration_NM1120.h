#if !defined(AFX_DIALOGCONFIGURATION_NM1120_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_NM1120_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_NM1120 dialog

class CDialogConfiguration_NM1120 : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_NM1120(unsigned int uProgramMemorySize = 29.5 * 1024,
                                CWnd* pParent = NULL);   // standard constructor

    static CString GetConfigWarning(const CAppConfig::NM1120_configs_t &config);

    CAppConfig::NM1120_configs_t m_ConfigValue;

    // Dialog Data
    //{{AFX_DATA(CDialogConfiguration_NM1120)
    enum { IDD = IDD_DIALOG_CONFIGURATION_NM1120 };
    CNumEdit    m_FlashBaseAddress;
    CEdit   m_DataFlashSize;
    int     m_nRadioClk;
    int     m_nRadioBor;
    int     m_nRadioBov;
    int     m_nRadioBS;
    CString m_sConfigValue0;
    CString m_sConfigValue1;
    CString m_sFlashBaseAddress;
    CString m_sDataFlashSize;
    BOOL    m_bDataFlashEnable;
    BOOL    m_bSecurityLock;
    BOOL    m_bRadioHIRC;
    BOOL    m_bCheckBrownOutEnable;
    BOOL    m_bCheckBrownOutReset;
    int     m_nRadioIO;
    int     m_nGPA0rini;
    int     m_nGPA1rini;
    int     m_nGPA2rini;
    int     m_nGPA3rini;
    int     m_nGPA4rini;
    int     m_nGPA5rini;
    CSpinButtonCtrl m_SpinDataFlashSize;
    unsigned int    m_uProgramMemorySize;
    //}}AFX_DATA


    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_NM1120)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

    // Implementation
protected:
    void ConfigToGUI();
    void GUIToConfig();


    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_NM1120)
    virtual BOOL OnInitDialog();
    afx_msg void OnButtonClick();
    afx_msg void OnKillfocusEditFlashBaseAddress();
    virtual void OnOK();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
