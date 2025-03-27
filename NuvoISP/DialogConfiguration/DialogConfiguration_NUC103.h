#if !defined(AFX_DIALOGCONFIGURATION_NUC103BN_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_NUC103BN_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_NUC103 dialog

class CDialogConfiguration_NUC103 : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_NUC103(UINT nIDTemplate = IDD_DIALOG_CONFIGURATION_NUC103BN,
                                unsigned int uProgramMemorySize = 64 * 1024,
                                unsigned int uDataFlashSize = 4 * 1024,
                                CWnd* pParent = NULL);   // standard constructor

    static CString GetConfigWarning(const unsigned int config[2]);

    CAppConfig::NUC1xx_configs_t m_ConfigValue;

    // Dialog Data
    //{{AFX_DATA(CDialogConfiguration_NUC103)
    CNumEdit    m_FlashBaseAddress;
    CEdit   m_DataFlashSize;
    int     m_nRadioBov;
    int     m_nRadioBS;
    CString m_sConfigValue0;
    CString m_sConfigValue1;
    CString m_sFlashBaseAddress;
    CString m_sDataFlashSize;
    BOOL    m_bCheckBrownOutDetect;
    BOOL    m_bCheckBrownOutReset;
    BOOL    m_bClockFilterEnable;
    BOOL    m_bDataFlashVarSizeEnable;
    BOOL    m_bDataFlashEnable;
    BOOL    m_bSecurityLock;
    BOOL    m_bWDTEnable;
    BOOL    m_bWDTPowerDown;
    int     m_nRadioGPF;
    int     m_nRadioIO;
    CSpinButtonCtrl m_SpinDataFlashSize;

    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uDataFlashSize;
    //}}AFX_DATA


    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_NUC103)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

    // Implementation
protected:
    void ConfigToGUI(int nEventID);
    virtual void GUIToConfig(int nEventID);
    void OnGUIEvent(int nEventID = 0);


    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_NUC103)
    virtual BOOL OnInitDialog();
    afx_msg void OnButtonClick();
    afx_msg void OnCheckClickWDTPD();
    afx_msg void OnKillfocusEditFlashBaseAddress();
    virtual void OnOK();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

class CDialogConfiguration_NUC123AN : public CDialogConfiguration_NUC103
{
public:
    CDialogConfiguration_NUC123AN(unsigned int uProgramMemorySize = 64 * 1024,
                                  unsigned int uDataFlashSize = 4 * 1024,
                                  CWnd* pParent = NULL);   // standard constructor
};

class CDialogConfiguration_NUC123AE : public CDialogConfiguration_NUC103
{
public:
    CDialogConfiguration_NUC123AE(unsigned int uProgramMemorySize = 64 * 1024,
                                  unsigned int uDataFlashSize = 4 * 1024,
                                  CWnd* pParent = NULL);   // standard constructor
};

class CDialogConfiguration_NUC131 : public CDialogConfiguration_NUC103
{
public:
    CDialogConfiguration_NUC131(unsigned int uProgramMemorySize = 64 * 1024,
                                unsigned int uDataFlashSize = 4 * 1024,
                                CWnd* pParent = NULL);   // standard constructor
protected:
    virtual void GUIToConfig(int nEventID);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
