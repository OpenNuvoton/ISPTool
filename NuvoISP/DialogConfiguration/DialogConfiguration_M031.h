#if !defined(AFX_DIALOGCONFIGURATION_M031_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_M031_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//

#include "DialogResize.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_M031 dialog

class CDialogConfiguration_M031 : public CDialogResize
{
// Construction
public:
    CDialogConfiguration_M031(unsigned int uProgramMemorySize = 128 * 1024,
                              unsigned int uPageSize = NUMICRO_FLASH_PAGE_SIZE_512,
                              CWnd *pParent = NULL);   // standard constructor

    CAppConfig::Mini51_configs_t m_ConfigValue;

// Dialog Data
    //{{AFX_DATA(CDialogConfiguration_M031)
    enum { IDD = IDD_DIALOG_CONFIGURATION_M031 };
    CNumEdit	m_FlashBaseAddress;
    CEdit	m_DataFlashSize;
    int		m_nRadioLVR;
    int		m_nRadioXT1;
    int		m_nRadioBov;
    int		m_nRadioIO;
    int		m_nRadioRstExt;
    int		m_nRadioRstWSel;
    int		m_nRadioBS;
    CString	m_sConfigValue0;
    CString	m_sConfigValue1;
    CString	m_sFlashBaseAddress;
    CString	m_sDataFlashSize;
    BOOL	m_bCheckBrownOutDetect;
    BOOL	m_bCheckBrownOutReset;
    BOOL	m_bDisableICE;
    BOOL	m_bDataFlashEnable;
    BOOL	m_bSecurityLock;
    BOOL	m_bWDTEnable;
    BOOL	m_bWDTPowerDown;
    CSpinButtonCtrl	m_SpinDataFlashSize;

    unsigned int	m_uProgramMemorySize;
    unsigned int	m_uPageSize;
    //}}AFX_DATA

protected:
    void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support
    void ConfigToGUI(int nEventID = 0);
    void GUIToConfig(int nEventID = 0);
    void OnGUIEvent(int nEventID = 0);
    virtual BOOL OnInitDialog();
    virtual void OnOK();

    afx_msg void OnButtonClick();
    afx_msg void OnCheckClickWDTPD();
    afx_msg void OnKillfocusEditFlashBaseAddress();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar);
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_M031_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)