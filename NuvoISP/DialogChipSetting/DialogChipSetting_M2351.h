#if !defined(AFX_DIALOGCHIPSETTING_M2351_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_M2351_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_M2351.h : header file
//
#include "ChipDefs.h"
#include "DialogChipSetting_CFG_M2351.h"

// DialogChipSetting_M2351 dialog

class CDialogChipSetting_M2351 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_M2351)

public:
    CDialogChipSetting_M2351(unsigned int uChipSeries, CWnd *pParent = NULL);

    virtual ~CDialogChipSetting_M2351();

// Dialog Data
    enum { IDD = IDD_DIALOG_CHIP_SETTING_NUMICRO };

protected:
    unsigned int m_uProgramMemorySize;
    unsigned int m_uFlashPageSize;
    unsigned int m_uChipSeries;

    BOOL m_bSecureDebug;
    BOOL m_bNSecureArea;

    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()
public:
    unsigned int m_uConfigValue[4];

    unsigned int m_uNSecure_Addr;

    bool m_bNSAddr_Write;
    bool m_bSecure_Lock;
    bool m_bAll_Lock;

    unsigned int m_nSel;
    unsigned int m_uShowFlag;

    CTabCtrl m_TabChipSetting;
    CDialogChipSetting_CFG_M2351	m_ChipSetting_CFG;

    afx_msg void OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOk();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_M2351_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)