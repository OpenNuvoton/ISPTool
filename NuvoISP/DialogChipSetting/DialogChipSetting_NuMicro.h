#if !defined(AFX_DIALOGCHIPSETTING_NUMICRO_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_NUMICRO_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_NuMicro.h : header file
//
#include "ChipDefs.h"
#include "DialogChipSetting_CFG_M0A21.h"

// DialogChipSetting_NuMicro dialog

class CDialogChipSetting_NuMicro : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_NuMicro)

public:
    CDialogChipSetting_NuMicro(unsigned int uProgramMemorySize, unsigned int uFlashPageSize, unsigned int uChipSeries, CWnd *pParent = NULL);	// standard constructor

    virtual ~CDialogChipSetting_NuMicro();

// Dialog Data
    enum { IDD = IDD_DIALOG_CHIP_SETTING_NUMICRO };

protected:
    unsigned int m_uProgramMemorySize;
    unsigned int m_uFlashPageSize;
    unsigned int m_uChipSeries;

    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange *pDX);	// DDX/DDV support

    DECLARE_MESSAGE_MAP()
public:
    unsigned int	m_uConfigValue[4];

    unsigned int	m_nSel;
    unsigned int	m_uShowFlag;

    CTabCtrl m_TabChipSetting;
    CDialogChipSetting_CFG_M0A21	*m_pChipSetting_CFG;

    afx_msg void OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOk();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_NUMICRO_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)