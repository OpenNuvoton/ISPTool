#if !defined(AFX_DIALOGCHIPSETTING_M480LD_H__E0CD7A09_C5DC_481E_A871_03B422E97826__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_M480LD_H__E0CD7A09_C5DC_481E_A871_03B422E97826__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_M480LD.h : header file
//
#include "DialogResize.h"
#include "DialogConfiguration_M480.h"

// DialogChipSetting_M480LD dialog

class CDialogChipSetting_M480LD : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_M480LD)

public:
    CDialogChipSetting_M480LD(unsigned int uAPROMSize = 0x40000, CWnd *pParent = NULL);	// standard constructor

    virtual ~CDialogChipSetting_M480LD();

    unsigned int m_uConfigValue[4];

// Dialog Data
    enum { IDD = IDD_DIALOG_CHIP_SETTING_M2351 };

protected:
    unsigned int m_uAPROM_Size;

    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()
public:

    unsigned int m_nSel;
    unsigned int m_nSelOffset;

    CTabCtrl m_TabChipSetting;
    CDialogChipSetting_CFG_M480LD		m_ChipSetting_CFG;

    afx_msg void OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOk();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_M480LD_H__E0CD7A09_C5DC_481E_A871_03B422E97826__INCLUDED_)