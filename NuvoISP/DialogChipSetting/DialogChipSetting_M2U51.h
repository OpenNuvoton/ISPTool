#if !defined(AFX_DIALOGCHIPSETTING_M2U51_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_M2U51_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_M2U51.h : header file
//
#include "ChipDefs.h"
#include "FlashInfo.h"
#include "DialogChipSetting_CFG_M2U51.h"
#include "DialogChipSetting_APWPROT.h"
#include "DialogChipSetting_XOM_NPU.h"


// CDialogChipSetting_M2U51 dialog

class CDialogChipSetting_M2U51 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_M2U51)

public:
    CDialogChipSetting_M2U51(unsigned int uPID, unsigned int uDID, unsigned int uChipSeries, CWnd* pParent = NULL);     // standard constructor

    virtual ~CDialogChipSetting_M2U51();

    // Dialog Data
    enum { IDD = IDD_DIALOG_CHIP_SETTING_NUMICRO2 };

protected:
    unsigned int    m_uPID;
    unsigned int    m_uDID;
    unsigned int    m_uChipSeries;

    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()
public:
    unsigned int    m_uCONFIG_Value[11];

    unsigned int    m_uXOM_WriteFlag;
    unsigned int    m_uXOM_Addr[2];
    unsigned int    m_uXOM_Count[2];
    unsigned int    m_uXOM_Ctrl[2];

    int             m_nSel;
    unsigned int    m_uShowFlag;

    CTabCtrl                        m_TabChipSetting;

    CDialogChipSetting_CFG_M2U51    *m_pChipSetting_CFG;
    CDialogChipSetting_APWPROT      *m_pChipSetting_APWPROT;
    CDialogChipSetting_XOM_NPU      *m_pChipSetting_XOM;

    afx_msg void OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOk();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_M2U51_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)