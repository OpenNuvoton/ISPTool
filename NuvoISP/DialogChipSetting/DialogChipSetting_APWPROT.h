#if !defined(AFX_DIALOGCHIPSETTING_APWPROT_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_APWPROT_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_APWPROT.h : header file
//
#include "DialogResize.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_APWPROT dialog

class CDialogChipSetting_APWPROT : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_APWPROT)

    // Construction
public:
    CDialogChipSetting_APWPROT(CWnd* pParent = NULL);       // standard constructor
    virtual ~CDialogChipSetting_APWPROT();

    // Dialog Data
    //{{AFX_DATA(CDialogChipSetting_APWPROT)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_APWPROT };

    int             m_nRadioAPWPLVL;
    int             m_nRadioAPWPPIN;

    BOOL            m_bCheckAPPROEN[64];

    CString         m_sConfigValue8;
    CString         m_sConfigValue9;
    CString         m_sConfigValue10;

    unsigned int    m_uAPROMAddr;
    unsigned int    m_uAPROMSize;
    unsigned int    m_uRegionNum;
    unsigned int    m_uRegionSize;
    unsigned int    m_uConfigValue[3];
    unsigned int    m_uConfigValue_t[3];
    unsigned int    m_uConfigValue_c[2];
    //}}AFX_DATA

protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_APWPROT)
    BOOL OnInitDialog();
    void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    afx_msg void OnButtonSelectAll0();
    afx_msg void OnButtonClearAll0();
    afx_msg void OnButtonSelectAll1();
    afx_msg void OnButtonClearAll1();
    afx_msg void OnRadioClick(unsigned int nID);
    afx_msg void OnCheckClick(unsigned int nID);
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_APWPROT2 dialog

class CDialogChipSetting_APWPROT2 : public CDialogChipSetting_APWPROT
{
    DECLARE_DYNAMIC(CDialogChipSetting_APWPROT2)

    // Construction
public:
    CDialogChipSetting_APWPROT2(CWnd* pParent = NULL);      // standard constructor

    // Dialog Data
    //{{AFX_DATA(CDialogChipSetting_APWPROT2)
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_APWPROT_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
