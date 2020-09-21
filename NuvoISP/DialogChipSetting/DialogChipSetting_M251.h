#if !defined(AFX_DIALOGCHIPSETTING_M251_H__E0CD7A09_C5DC_481E_A871_07C5231B426A9__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_M251_H__E0CD7A09_C5DC_481E_A871_07C5231B426A9__INCLUDED_

//#if _MSC_VER > 1000
#pragma once
//#endif // _MSC_VER > 1000
// DialogConfiguration.h : header file
//
#include "afxcmn.h"

#include "DialogResize.h"
#include "DialogConfiguration_M251.h"

// CDialogChipSetting_M251 dialog

class CDialogChipSetting_M251 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_M251)

public:
    CDialogChipSetting_M251(int uParam = 0, unsigned int uProgramMemorySize = 192 * 1024, UINT nIDTemplate = CDialogChipSetting_M251::IDD, CWnd *pParent = NULL);   // standard constructor
    virtual ~CDialogChipSetting_M251();

// Dialog Data
    enum { IDD = IDD_DIALOG_CHIP_SETTING_M251 };

protected:
    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange *pDX);

    DECLARE_MESSAGE_MAP()
public:
    unsigned int m_uConfigOption_ConfigValue[4];
    unsigned int m_uProgramMemorySize;
    unsigned int m_uParam;

    unsigned int m_nSel;
    CTabCtrl m_TabChipSetting;
    CDialogConfiguration_M251 m_ConfigOptionM251;
    CDialogConfiguration_M258 m_ConfigOptionM258;

    afx_msg void OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOk();
};

#endif // !defined(AFX_DIALOGCHIPSETTING_M251_H__E0CD7A09_C5DC_481E_A871_07C5231B426A9__INCLUDED_)