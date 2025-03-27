#if !defined(AFX_DIALOGCUSTOMFORMULA_OT8051_H__581CBDAF_9EED_4D78_B550_A72F4F14E98F__INCLUDED_)
#define AFX_DIALOGCUSTOMFORMULA_OT8051_H__581CBDAF_9EED_4D78_B550_A72F4F14E98F__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogCustomFormula_OT8051.h : header file
//
#include "DialogResize.h"
#include "NumEdit.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogCustomFormula_OT8051 dialog

class CDialogCustomFormula_OT8051 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogCustomFormula_OT8051)

public:
    CDialogCustomFormula_OT8051(unsigned int uFlashSize = 0x10000, CWnd *pParent = NULL);   // standard constructor

    ~CDialogCustomFormula_OT8051();

    void ConfigToGUI();
    void GUIToConfig();

    // Dialog Data
    enum { IDD = IDD_DIALOG_CUSTOM_FORMULA };

    unsigned char   m_ucUID[12];
    unsigned char   m_ucOperator[3];
    unsigned char   m_ucValue[3];
    unsigned int    m_uWriteFlag;
    unsigned int    m_uAddr[12];

    unsigned int    m_uProgramMemorySize;

    CComboBox       m_combobox_operator[3];

    CNumEdit        m_Formula_Addr[12];
    CNumEdit        m_Formula_Value[3];

    CString         m_sFormula_Value[3];
    CString         m_sFormula_Addr[12];
    CString         m_sFormula_Result[12];

    BOOL            m_bCheckAddr[12];

protected:
    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
    afx_msg void OnChangeComboOperator();
    afx_msg void OnCheckClick(UINT nID);
    afx_msg void OnKillfocusEditValue(UINT nID);
    afx_msg void OnKillfocusEditAddr(UINT nID);
    afx_msg void OnOK();
    afx_msg void OnCancel();
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCUSTOMFORMULA_OT8051_H__581CBDAF_9EED_4D78_B550_A72F4F14E98F__INCLUDED_)

