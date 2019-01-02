#if !defined(AFX_DIALOGCHIPSETTING_CFG_M2351_H__BF199825_A718_4D54_9236_F52EAC800A92__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_CFG_M2351_H__BF199825_A718_4D54_9236_F52EAC800A92__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration_M2351.h : header file
//
#include "DialogResize.h"
#include "Resource.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M2351 dialog

class CDialogConfiguration_M2351 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogConfiguration_M2351)

// Construction
public:
    CDialogConfiguration_M2351(CWnd *pParent = NULL);		// standard constructor
    virtual ~CDialogConfiguration_M2351();

// Dialog Data
    enum { IDD = IDD_DIALOG_CONFIGURATION_M2351 };

    int		m_nRadioBov;
    int		m_nRadioBS;
    int		m_nRadioGPG;
    int		m_nRadioIO;
    int		m_nRadioUART;
    int		m_nRadioWDT;

    CString	m_sConfigValue0;
    CString	m_sConfigValue3;

    BOOL	m_bCheckMBS;
    BOOL	m_bCheckBrownOutDetect;
    BOOL	m_bCheckBrownOutReset;
    BOOL	m_bDisableICE;

    unsigned int m_uConfigValue[4];

// Implementation
protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_M2351)
    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange *pDX);		// DDX/DDV support

    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();

    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_CFG_M2351_H__BF199825_A718_4D54_9236_F52EAC800A92__INCLUDED_)
