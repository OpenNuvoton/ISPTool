#if !defined(AFX_DIALOGCONFIGURATION_N570_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_N570_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_NUC1xx dialog

class CDialogConfiguration_N570 : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_N570(unsigned int uProgramMemorySize = 64 * 1024,
                              CWnd* pParent = NULL);   // standard constructor

    static CString GetConfigWarning(const CAppConfig::AU91xx_configs_t &config);

    CAppConfig::AU91xx_configs_t m_ConfigValue;

    // Dialog Data
    //{{AFX_DATA(CDialogConfiguration_N570)
    enum { IDD = IDD_DIALOG_CONFIGURATION_N570 };
    CNumEdit    m_FlashBaseAddress;
    CEdit   m_DataFlashSize;
    int     m_nRadioBov;
    int     m_nRadioBS;
    CString m_sConfigValue0;
    CString m_sConfigValue1;
    CString m_sFlashBaseAddress;
    CString m_sDataFlashSize;
    BOOL    m_bCheckLowVolResetEnable;
    BOOL    m_bCheckBrownOutHysteresis;
    BOOL    m_bCheckBrownOutEnable;
    BOOL    m_bCheckBrownOutReset;
    BOOL    m_bDataFlashEnable;
    BOOL    m_bSecurityLock;
    CSpinButtonCtrl m_SpinDataFlashSize;

    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uPageSize;
    //}}AFX_DATA


    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_N570)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

    // Implementation
protected:
    void ConfigToGUI();
    void GUIToConfig();


    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_NUC1xx)
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
