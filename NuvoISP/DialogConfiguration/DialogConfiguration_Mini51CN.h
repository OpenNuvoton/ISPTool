#if !defined(AFX_DIALOGCONFIGURATION_MINI51CN_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_MINI51CN_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_Mini51CN dialog

class CDialogConfiguration_Mini51CN : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_Mini51CN(unsigned int uProgramMemorySize = 16 * 1024,
                                  unsigned int uPID = 0,
                                  CWnd* pParent = NULL);   // standard constructor

    static CString GetConfigWarning(const CAppConfig::Mini51_configs_t &config);

    CAppConfig::Mini51_configs_t m_ConfigValue;

    // Dialog Data
    //{{AFX_DATA(CDialogConfiguration_Mini51CN)
    enum { IDD = IDD_DIALOG_CONFIGURATION_MINI51CN };
    CNumEdit    m_FlashBaseAddress;
    CEdit   m_DataFlashSize;
    int     m_nRadioBov;
    int     m_nRadioBS;
    CString m_sConfigValue0;
    CString m_sConfigValue1;
    CString m_sFlashBaseAddress;
    CString m_sDataFlashSize;
    BOOL    m_bClockFilterEnable;
    BOOL    m_bDataFlashEnable;
    BOOL    m_bSecurityLock;
    BOOL    m_bCheckBrownOutReset;
    int     m_nRadioIO;
    CSpinButtonCtrl m_SpinDataFlashSize;

    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uPID;
    //}}AFX_DATA


    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_Mini51CN)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

    // Implementation
protected:
    void ConfigToGUI();
    void GUIToConfig();

    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_Mini51CN)
    virtual BOOL OnInitDialog();
    afx_msg void OnButtonClick();
    afx_msg void OnKillfocusEditFlashBaseAddress();
    virtual void OnOK();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

class CDialogConfiguration_Mini58 : public CDialogConfiguration_Mini51CN
{
public:
    CDialogConfiguration_Mini58(unsigned int uProgramMemorySize = 32 * 1024, unsigned int uPID = 0x00A05800, CWnd* pParent = NULL);   // standard constructor
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
