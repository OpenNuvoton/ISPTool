#if !defined(AFX_DIALOGCONFIGURATION_M251_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_M251_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include <deque>
#include <string>
#include "AppConfig.h"
#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M251 dialog

class CDialogConfiguration_M251 : public CDialogResize
{
// Construction
public:
    CDialogConfiguration_M251(unsigned int uProgramMemorySize = 192 * 1024,
                              UINT nIDTemplate = CDialogConfiguration_M251::IDD,
                              CWnd *pParent = NULL);   // standard constructor

    CAppConfig::M251_configs_t m_ConfigValue;

    //{{AFX_DATA(CDialogConfiguration_M251)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_CFG_M251 };
    int		m_nRadioBov;
    int		m_nRadioBS;
    CString	m_sConfigValue0;
    BOOL	m_bCheckBrownOutDetect;
    BOOL	m_bCheckBrownOutReset;
    BOOL	m_bSecurityLock;
    BOOL	m_bICELock;
    BOOL	m_bWDTEnable;
    BOOL	m_bWDTPowerDown;
    int		m_nRadioIO;
    CSpinButtonCtrl	m_SpinDataFlashSize;

    unsigned int m_uProgramMemorySize;
    virtual void ConfigToGUI(int nEventID = 0);
    virtual void GUIToConfig(int nEventID = 0);

protected:
    virtual void DoDataExchange(CDataExchange *pDX);
    void OnGUIEvent(int nEventID = 0);

    virtual BOOL OnInitDialog();
    afx_msg void OnButtonClick();
    afx_msg void OnCheckClickWDTPD();
    virtual void OnOK();
    virtual void OnCancel();
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar);
    DECLARE_MESSAGE_MAP()
};

class CDialogConfiguration_M258 : public CDialogConfiguration_M251
{
public:
    CDialogConfiguration_M258(unsigned int uProgramMemorySize = 128 * 1024,
                              UINT nIDTemplate = CDialogConfiguration_M258::IDD,
                              CWnd *pParent = NULL);

    enum { IDD = IDD_DIALOG_CHIP_SETTING_CFG_M258 };
    int		m_nRadioBootClkSel;
    virtual void ConfigToGUI(int nEventID = 0);
    virtual void GUIToConfig(int nEventID = 0);

protected:
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_M251_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
