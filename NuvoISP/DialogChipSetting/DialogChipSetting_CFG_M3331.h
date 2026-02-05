#if !defined(AFX_DIALOGCHIPSETTING_CFG_M3331_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_CFG_M3331_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_CFG_M3331.h : header file
//
#include "DialogResize.h"
#include "NumEdit.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M3331 dialog

class CDialogChipSetting_CFG_M3331 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M3331)

// Construction
public:
    CDialogChipSetting_CFG_M3331(CWnd* pParent = NULL);        // standard constructor
    virtual ~CDialogChipSetting_CFG_M3331();

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M3331)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_CFG_M3331 };

    int                m_nRadioCWDTEN;
    int                m_nRadioCBOV;
    int                m_nRadioCIOINI;
    int                m_nRadioCBS;

    BOOL            m_bCheckSRAM_ECC;
    BOOL            m_bCheckCBORST;
    BOOL            m_bCheckCBODEN;
    BOOL            m_bCheckICELOCK;
    BOOL            m_bCheckSCEN;

    BOOL            m_bCheckISP_UART;
    BOOL            m_bCheckISP_USB;
    BOOL            m_bCheckISP_CAN;
    BOOL            m_bCheckISP_I2C;
    BOOL            m_bCheckISP_SPI;

    CNumEdit        m_SC_BaseAddr;
    CNumEdit        m_SC_PageCount;

    CString            m_sSC_BaseAddr;
    CString            m_sSC_PageCount;

    CString            m_sConfigValue0;
    CString            m_sConfigValue1;
    CString            m_sConfigValue2;
    CString            m_sConfigValue3;
    CString            m_sConfigValue4;
    CString            m_sConfigValue5;
    CString            m_sConfigValue6;

    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uFlashPageSize;
    unsigned int    m_uLDROM_Addr;
    unsigned int    m_uLDROM_Size;
    unsigned int    m_uConfigValue[7];
    unsigned int    m_uConfigValue_t[7];
    //}}AFX_DATA

protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_CFG_M3331)
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
    void CFG2GUI_BLISP();
    void CFG2GUI_SC_FUNC();

    void GUI2CFG_CWDT();
    void GUI2CFG_CBOD_8();
    void GUI2CFG_CIOINI();
    void GUI2CFG_CBS_2();
    void GUI2CFG_ICELOCK();
    void GUI2CFG_BLISP();
    void GUI2CFG_SC_FUNC();

    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditSCAddrCount();
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M3351 dialog

class CDialogChipSetting_CFG_M3351 : public CDialogChipSetting_CFG_M3331
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M3351)

// Construction
public:
    CDialogChipSetting_CFG_M3351(CWnd* pParent = NULL);        // standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M3351)
    //}}AFX_DATA

protected:
    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    void CFG2GUI_CWDT();
    void CFG2GUI_CBOD();

    void GUI2CFG_CWDT();
    void GUI2CFG_CBOD();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_CFG_M3331_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
