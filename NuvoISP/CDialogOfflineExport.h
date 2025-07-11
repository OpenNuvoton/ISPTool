
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_CFG_M0A21.h : header file
//

#include "DialogResize.h"
#include <vector>

class CDialogOfflineExport : public CDialogResize
{
	DECLARE_DYNAMIC(CDialogOfflineExport)

    // Construction
public:
    CDialogOfflineExport(CWnd* pParent = NULL);       // standard constructor
    virtual ~CDialogOfflineExport();

    
    // Dialog Data
    //{{AFX_DATA(CDialogChipSetting_APWPROT)
    enum { IDD = IDD_DIALOG_OFFLINE_LUA };

    CEdit       m_EditName;
    CComboBox   m_ComboInterface;
    BOOL        m_LimitCheck;
    CEdit       m_LimitCount;
    int         m_nRadioStorage;
    std::vector<CString> m_Interfaces;

protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_APWPROT)
    BOOL OnInitDialog();
    void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    virtual void OnOK();

    afx_msg void OnRadioSelect();
    afx_msg void OnCheckLimit();
    afx_msg void OnFileNameChange();
    afx_msg void OnLimitCountChange();
    afx_msg void OnSelchangeInterface();

    DECLARE_MESSAGE_MAP()
};

