#if !defined(AFX_DIALOGCHIPSETTING_M55M1_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_M55M1_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_M55M1.h : header file
//
#include "ChipDefs.h"
#include "FlashInfo.h"
#include "DialogChipSetting_CFG_M55M1.h"
#include "DialogChipSetting_APWPROT.h"
#include "DialogChipSetting_SECURE_M2351.h"

// CDialogChipSetting_M55M1 dialog

class CDialogChipSetting_M55M1 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_M55M1)

public:
    CDialogChipSetting_M55M1(BOOL bSecureDebug = TRUE, unsigned int uPID = 0, unsigned int uDID = 0x73DF, unsigned int uChipSeries = IDS_M55M1_SERIES, CWnd* pParent = NULL);   // standard constructor

    virtual ~CDialogChipSetting_M55M1();

    // Dialog Data
    enum { IDD = IDD_DIALOG_CHIP_SETTING_NUMICRO2 };

protected:
    BOOL            m_bSecureDebug;

    unsigned int    m_uPID;
    unsigned int    m_uDID;
    unsigned int    m_uChipSeries;

    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()
public:
    unsigned int    m_uConfigValue[14];

    unsigned int    m_uNSCBA_NSAddr;

    BOOL            m_bNSCBA_Write;
    BOOL            m_bNSCBA_MirBoundEnable;
    BOOL            m_bNSCBA_CanWrite;

    int             m_nSel;
    unsigned int    m_uShowFlag;

    CTabCtrl                        m_TabChipSetting;

    CDialogChipSetting_CFG_M55M1    *m_pChipSetting_CFG;
    CDialogChipSetting_APWPROT      *m_pChipSetting_APWPROT;
    CDialogChipSetting_NSCBA_LOCK   *m_pChipSetting_NSCBA;

    afx_msg void OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOk();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_M55M1_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)