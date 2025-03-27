#if !defined(AFX_DIALOGCONFIGURATION_OT8051_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_OT8051_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration_OT8051.h : header file
//

#include "DialogResize.h"
#include "ChipDefs.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_OT8051 dialog

class CDialogConfiguration_OT8051 : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_OT8051(unsigned int uPartNo = 0x3650,
                                CWnd* pParent = NULL);

    // Dialog Data
    //{{AFX_DATA(CDialogConfiguration_OT8051)
    enum { IDD = IDD_DIALOG_CONFIGURATION_OT8051 };

    int             m_nRadio_CBS;
    int             m_nRadio_FSYS;
    int             m_nRadio_OCDPWM;
    int             m_nRadio_RPD;
    int             m_nRadio_IODEFAULT;
    int             m_nRadio_LDSIZE;
    int             m_nRadio_CBOV;
    int             m_nRadio_SYSCLKDIV;
    int             m_nRadio_WDTEN;

    BOOL            m_bADCInterruptEnable;
    BOOL            m_bOCDEnable;
    BOOL            m_bSecurityLock;
    BOOL            m_bCheckBrownOutEnable;
    BOOL            m_bCheckBrownOutIAP;
    BOOL            m_bCheckBrownOutReset;

    CString         m_sConfigValue0;
    CString         m_sConfigValue1;
    CString         m_sConfigValue2;
    CString         m_sConfigValue3;
    CString         m_sConfigValue4;

    unsigned int    m_uPartNo;
    unsigned int    m_uLevel;

    bool            m_bFSYS;
    bool            m_bOCDPWM;
    bool            m_bRPD;
    bool            m_bIODEFAULT;
    bool            m_bSYSCLKDIV;
    bool            m_bADCINTEN;

    unsigned char   m_ucConfigValue[OT8051_FLASH_CONFIG_SIZE];
    //}}AFX_DATA

    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_OT8051)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

    // Implementation
protected:
    void ConfigToGUI();
    void GUIToConfig();

    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_OT8051)
    virtual BOOL OnInitDialog();
    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();
    virtual void OnOK();
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_OT8051_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
