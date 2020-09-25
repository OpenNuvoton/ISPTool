#if !defined(AFX_DIALOGCONFIGURATION_M480_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_M480_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M480 dialog

class CDialogConfiguration_M480 : public CDialogResize
{
// Construction
public:
    CDialogConfiguration_M480(unsigned int uProgramMemorySize = 512 * 1024,
                              unsigned int uPID = 0,
                              UINT nIDTemplate = IDD_DIALOG_CONFIGURATION_M480,
                              CWnd *pParent = NULL);   // standard constructor

    CAppConfig::M480_configs_t m_ConfigValue;

// Dialog Data
    //{{AFX_DATA(CDialogConfiguration_M480)
//	enum { IDD = IDD_DIALOG_CONFIGURATION_M480 };
    CNumEdit	m_FlashBaseAddress;
    CEdit	m_DataFlashSize;
    int		m_nRadioBov;
    int		m_nRadioBS;
    int		m_nRadioSPIM;
    int		m_nRadioUART;
    CString	m_sConfigValue0;
    CString	m_sConfigValue1;
    CString	m_sConfigValue2;
    CString	m_sConfigValue3;
    CString	m_sFlashBaseAddress;
    CString	m_sDataFlashSize;
    BOOL	m_bCheckBrownOutDetect;
    BOOL	m_bCheckBrownOutReset;
    BOOL	m_bCheckBootLoader;
    BOOL	m_bDataFlashEnable;
    BOOL	m_bSecurityLock;
    BOOL	m_bSecurityBootLock;
    BOOL	m_bICELock;
    BOOL	m_bWDTEnable;
    BOOL	m_bWDTPowerDown;
    BOOL	m_bSpromLockCacheable;
    int		m_nRadioGPG;
    int		m_nRadioIO;
    CSpinButtonCtrl	m_SpinDataFlashSize;

    unsigned int	m_uProgramMemorySize;
    unsigned int	m_uPID;
    //}}AFX_DATA

// Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_M480)
protected:
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support
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
    afx_msg void OnCheckClickWDTPD();
    virtual afx_msg void OnKillfocusEditFlashBaseAddress();
    virtual void OnOK();
    virtual afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M480LD dialog

class CDialogChipSetting_CFG_M480LD : public CDialogConfiguration_M480
{
// Construction
public:
    CDialogChipSetting_CFG_M480LD(unsigned int uProgramMemorySize = 256 * 1024,
                                  UINT nIDTemplate = IDD_DIALOG_CONFIGURATION_M480LD,
                                  CWnd *pParent = NULL);   // standard constructor

    enum { IDD = IDD_DIALOG_CONFIGURATION_M480LD };
    unsigned int m_uConfigValue[4];

protected:
    virtual void DoDataExchange(CDataExchange *pDX);   // DDX/DDV support
    void ConfigToGUI(int nEventID);
    void GUIToConfig(int nEventID);
    afx_msg void OnKillfocusEditFlashBaseAddress();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOK();
    afx_msg void OnCancel();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
