#if !defined(AFX_DIALOGCHIPSETTING_CFG_M55M1_H__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_CFG_M55M1_H__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_CFG_M460.h : header file
//

#include "DialogResize.h"
#include "NumEdit.h"

/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_CFG_M460 dialog

class CDialogChipSetting_CFG_M55M1 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_CFG_M55M1)

// Construction
public:
    CDialogChipSetting_CFG_M55M1(CWnd *pParent = NULL);		// standard constructor

// Dialog Data
    //{{AFX_DATA(CDialogChipSetting_CFG_M460)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_CFG_M55M1 };

    int				m_nRadioCWDTEN;
    int				m_nRadioCBOV;
    int				m_nRadioCBS;
    int				m_nRadioUART;

    BOOL			m_bCheckCBORST;
    BOOL			m_bCheckCBODEN;
    BOOL			m_bCheckICELOCK;
    BOOL			m_bCheckSCEN;
    BOOL	        m_bTamperPowerDown;

    BOOL			m_bCheckNSCRLOCK;
    BOOL			m_bCheckSCRLOCK;
    BOOL			m_bCheckARLOCK;
    BOOL			m_bMirror;

    BOOL			m_bCheckISP_UART;
    BOOL			m_bCheckISP_USB;
    BOOL			m_bCheckISP_CAN;
    BOOL			m_bCheckISP_I2C;
    BOOL			m_bCheckISP_SPI;

    CString		    m_sSecureConcealBase;
    CString		    m_sSecureConcealPageSize;
    CString		    m_sNSCRBase;

    CNumEdit		m_SecureConcealBase;
    CNumEdit		m_SecureConcealPageSize;
    CNumEdit		m_NSCRBase;  

    CString			m_sConfigValue0;
    CString			m_sConfigValue3;

    CString			m_sConfigValue4;
    CString			m_sConfigValue5;
    CString			m_sConfigValue6;

    CString			m_sConfigValue11;
    CString			m_sConfigValue12;
    CString			m_sConfigValue13;

    unsigned int	m_uPID;
    unsigned int	m_uDID;
    unsigned int	m_uConfigValue[14];
    unsigned int	m_uConfigValue_t[14];
    //}}AFX_DATA

    unsigned int	m_uProgramMemorySize;
    unsigned int	m_uFlashPageSize;
protected:

    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_CFG_M0A21)
    BOOL OnInitDialog();
    void DoDataExchange(CDataExchange *pDX);	// DDX/DDV support

    virtual void UpdateUI();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();

    void CFG2GUI_CWDT();
    void CFG2GUI_CBOD_8();
    void CFG2GUI_CBS_2();
    void CFG2GUI_UART1();
    void CFG2GUI_ICELOCK();
    void CFG2GUI_SCEN();
    void CFG2GUI_NSCR();

    void GUI2CFG_CWDT();
    void GUI2CFG_CBOD_8();
    void GUI2CFG_CBS_2();
    void GUI2CFG_UART1();
    void GUI2CFG_ICELOCK();
    void GUI2CFG_SCEN();
    void GUI2CFG_NSCR();

    afx_msg void OnRadioClick();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditSCBase();
    afx_msg void OnKillfocusEditSCSize();
    afx_msg void OnKillfocusEditNSCRBase();
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
public:

};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_CFG_M460_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)