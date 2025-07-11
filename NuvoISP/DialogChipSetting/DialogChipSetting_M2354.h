#if !defined(AFX_DIALOGCHIPSETTING_M2354_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_M2354_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_M2354.h : header file
//
#include "ChipDefs.h"
#include "DialogChipSetting_CFG_M2351.h"
#include "DialogChipSetting_SECURE_M2351.h"

// CDialogChipSetting_M2354 dialog

class CDialogChipSetting_M2354 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_M2354)

public:
    CDialogChipSetting_M2354(unsigned int uProgramMemorySize = M2354_MAX_APROM_SIZE, unsigned int uFlashPageSize = NUMICRO_FLASH_PAGE_SIZE_2K, BOOL bSecureDebug = TRUE,
                             unsigned int uChipSeries = NUC_CHIP_TYPE_M2354, CWnd* pParent = NULL); // standard constructor

    virtual ~CDialogChipSetting_M2354();

    // Dialog Data
    enum { IDD = IDD_DIALOG_CHIP_SETTING_NUMICRO };

protected:
    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uFlashPageSize;
    unsigned int    m_uChipSeries;

    BOOL            m_bSecureDebug;

    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()
public:
    unsigned int    m_uConfigValue[config_amount];

    unsigned int    m_uNSCBA_NSAddr;

    bool            m_bNSCBA_Write;
    bool            m_bNSCBA_MirrorEnable;
    bool            m_bNSCBA_CanWrite;

    unsigned int    m_nSel;
    unsigned int    m_uShowFlag;

    CTabCtrl                        m_TabChipSetting;

    CDialogChipSetting_CFG_M2351    *m_pChipSetting_CFG;
    CDialogChipSetting_NSCBA        *m_pChipSetting_NSCBA;

    afx_msg void OnTcnSelchangeTabChipsetting(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOk();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_M2354_H__E0CD7A09_C5DC_481E_A871_03B422E615F8__INCLUDED_)