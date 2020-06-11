#if !defined(AFX_DIALOGCONFIGURATION_M0A21_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_M0A21_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration_M0A21.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M0A21 dialog

class CDialogConfiguration_M0A21 : public CDialogResize
{
// Construction
public:
    CDialogConfiguration_M0A21(unsigned int uProgramMemorySize,
                               unsigned int uPageSize,
                               CWnd *pParent = NULL);   // standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogConfiguration_M0A21)
    enum { IDD = IDD_DIALOG_CONFIGURATION_M0A21 };

    int				m_nRadioCWDTEN;
    int				m_nRadioCFGXT1;
    int				m_nRadioCFGRPS;
    int				m_nRadioCBOV;
    int				m_nRadioCIOINI;
    int				m_nRadioRSTEXT;
    int				m_nRadioRSTWSEL;
    int				m_nRadioCBS;

    BOOL			m_bCheckCBORST;
    BOOL			m_bCheckCBODEN;
    BOOL			m_bCheckICELOCK;
    BOOL			m_bCheckLOCK;
    BOOL			m_bCheckDFEN;

    CNumEdit		m_DataFlashBase;
    CEdit			m_DataFlashSize;
    CSpinButtonCtrl	m_SpinDataFlashSize;

    CNumEdit		m_ALOCK;
    CString			m_sALOCK;

    CString			m_sDataFlashBase;
    CString			m_sDataFlashSize;

    CString			m_sConfigValue0;
    CString			m_sConfigValue1;
    CString			m_sConfigValue2;

    unsigned int	m_uProgramMemorySize;
    unsigned int	m_uFlashPageSize;
    unsigned int	m_uConfigValue[3];
    //}}AFX_DATA

protected:
    void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support

    virtual BOOL OnInitDialog();
    virtual void OnOK();

    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditAdvanceLock();
    afx_msg void OnKillfocusEditDataFlashBase();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar);
    DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M030G dialog

class CDialogConfiguration_M030G : public CDialogConfiguration_M0A21
{
// Construction
public:
    CDialogConfiguration_M030G(unsigned int uProgramMemorySize,
                               unsigned int uPageSize,
                               CWnd *pParent = NULL);   // standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogConfiguration_M030G)
    enum { IDD = IDD_DIALOG_CONFIGURATION_M0A21 };
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();
};

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M031 dialog

class CDialogConfiguration_M031 : public CDialogConfiguration_M0A21
{
// Construction
public:
    CDialogConfiguration_M031(unsigned int uProgramMemorySize,
                              unsigned int uPageSize,
                              CWnd *pParent = NULL);    // standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogConfiguration_M031)
    enum { IDD = IDD_DIALOG_CONFIGURATION_M0A21 };
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();
};

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M451 dialog

class CDialogConfiguration_M451 : public CDialogConfiguration_M0A21
{
// Construction
public:
    CDialogConfiguration_M451(unsigned int uProgramMemorySize,
                              unsigned int uPageSize,
                              CWnd *pParent = NULL);    // standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogConfiguration_M031)
    enum { IDD = IDD_DIALOG_CONFIGURATION_M0A21 };
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_M0A21_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)