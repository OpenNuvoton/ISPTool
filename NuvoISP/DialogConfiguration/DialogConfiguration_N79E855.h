#if !defined(AFX_DIALOGCONFIGURATION_N79E855_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCONFIGURATION_N79E855_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogConfiguration_N79E855.h : header file
//

#include "DialogResize.h"
#include "NumEdit.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogConfiguration_N79E855 dialog

class CDialogConfiguration_N79E855 : public CDialogResize
{
    // Construction
public:
    CDialogConfiguration_N79E855(CWnd* pParent = NULL);

    // Dialog Data
    //{{AFX_DATA(CDialogConfiguration_N79E855)
    enum { IDD = IDD_DIALOG_CONFIGURATION_N79E855 };

    int             m_nRadio_CBS;
    int             m_nRadio_CBOV;
    int             m_nRadio_OSCFS;
    int             m_nRadio_FOSC;

    BOOL            m_bCheck_LOCK;
    BOOL            m_bCheck_DFEN;
    BOOL            m_bCheck_CBODEN;
    BOOL            m_bCheck_CBORST;
    BOOL            m_bCheck_CWDTEN;
    BOOL            m_bCheck_CKF;

    CNumEdit        m_DataFlashBase;
    CEdit           m_DataFlashSize;
    CSpinButtonCtrl m_SpinDataFlashSize;

    CString         m_sDataFlashBase;
    CString         m_sDataFlashSize;

    CString         m_sConfigValue0;
    CString         m_sConfigValue1;
    CString         m_sConfigValue2;
    CString         m_sConfigValue3;

    unsigned char   m_ucCONFIG[N79E855_FLASH_CONFIG_SIZE];
    unsigned char   m_ucCONFIG_t[N79E855_FLASH_CONFIG_SIZE];

    //}}AFX_DATA

    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogConfiguration_N79E855)
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

    // Implementation
protected:
    void ConfigToGUI();
    void GUIToConfig();
    void CFG2GUI_DFEN();
    void GUI2CFG_DFEN();

    // Generated message map functions
    //{{AFX_MSG(CDialogConfiguration_N79E855)
    virtual BOOL OnInitDialog();
    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditDataFlashBase();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    virtual void OnOK();
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCONFIGURATION_N79E855_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
