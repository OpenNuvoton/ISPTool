#if !defined(AFX_DIALOGCHIPSETTING_CFG_M460_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_CFG_M460_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_CFG_M460.h : header file
//

#include "DialogResize.h"
#include "NumEdit.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 dialog

class CDialogChipSetting_CFG_M460 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M460)

    // Construction
public:
    CDialogChipSetting_CFG_M460(CWnd* pParent = NULL);      // standard constructor
    virtual ~CDialogChipSetting_CFG_M460();

    // Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M460)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_CFG_M460 };

    int             m_nRadioCWDTEN;
    int             m_nRadioCBOV;
    int             m_nRadioCIOINI;
    int             m_nRadioCBS;

    BOOL            m_bCheckCBORST;
    BOOL            m_bCheckCBODEN;
    BOOL            m_bCheckICELOCK;
    BOOL            m_bCheckLOCK;
    BOOL            m_bCheckDFEN;

    BOOL            m_bCheckISP_UART;
    BOOL            m_bCheckISP_USB;
    BOOL            m_bCheckISP_CAN;
    BOOL            m_bCheckISP_I2C;
    BOOL            m_bCheckISP_SPI;

    CNumEdit        m_DataFlashBase;
    CEdit           m_DataFlashSize;
    CSpinButtonCtrl m_SpinDataFlashSize;

    CNumEdit        m_ALOCK;
    CString         m_sALOCK;

    CNumEdit        m_KSPLOCK;
    CString         m_sKSPLOCK;

    CString         m_sDataFlashBase;
    CString         m_sDataFlashSize;

    CString         m_sConfigValue0;
    CString         m_sConfigValue1;
    CString         m_sConfigValue2;
    CString         m_sConfigValue3;

    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uFlashPageSize;
    unsigned int    m_uConfigValue[config_amount];
    unsigned int    m_uConfigValue_t[config_amount];
    //}}AFX_DATA

protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_CFG_M460)
    BOOL OnInitDialog();
    void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    void CFG2GUI_CWDT();
    void CFG2GUI_CBOD_8();
    void CFG2GUI_CIOINI();
    void CFG2GUI_CBS_2();
    void CFG2GUI_ICELOCK();
    void CFG2GUI_DFEN();
    void CFG2GUI_KSPLOCK();

    void GUI2CFG_CWDT();
    void GUI2CFG_CBOD_8();
    void GUI2CFG_CIOINI();
    void GUI2CFG_CBS_2();
    void GUI2CFG_ICELOCK();
    void GUI2CFG_DFEN();
    void GUI2CFG_KSPLOCK();

    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditAdvanceLock();
    afx_msg void OnKillfocusEditKeyStoreLock();
    afx_msg void OnKillfocusEditDataFlashBase();
    afx_msg void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_CFG_M460_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
