#if !defined(AFX_DIALOGCHIPSETTING_LDWPROT_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_LDWPROT_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_LDWPROT.h : header file
//
#include "DialogResize.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_LDWPROT dialog

class CDialogChipSetting_LDWPROT : public CDialogResize
{
	DECLARE_DYNAMIC(CDialogChipSetting_LDWPROT)

// Construction
public:
	CDialogChipSetting_LDWPROT(CWnd* pParent = NULL);		// standard constructor
	virtual ~CDialogChipSetting_LDWPROT();

// Dialog Data
	//{{AFX_DATA(CDialogChipSetting_LDWPROT)
	enum { IDD = IDD_DIALOG_CHIP_SETTING_LDWPROT };

	int				m_nRadioLDWPLVL;

	BOOL			m_bCheckLDPROEN;
	BOOL			m_bCheckDFPROEN[4];

	CString			m_sConfigValue16;
	CString			m_sConfigValue17;
	CString			m_sConfigValue18;

	unsigned int	m_uLDROM_Addr;
	unsigned int	m_uLDROM_Size;
	unsigned int	m_uLDROM_RegionSize;
	unsigned int	m_uDATAFLASH_Addr;
	unsigned int	m_uDATAFLASH_Size;
	unsigned int	m_uDATAFLASH_RegionSize;
	unsigned int	m_uConfigValue[3];
	unsigned int	m_uConfigValue_t[3];
	//}}AFX_DATA

protected:
	// Generated message map functions
	//{{AFX_MSG(CDialogChipSetting_LDWPROT)
	BOOL OnInitDialog();
	void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

	virtual void UpdateUI();
	virtual void ConfigToGUI();
	virtual void GUIToConfig();

	afx_msg void OnRadioClick();
	afx_msg void OnCheckClick();
	afx_msg void OnOK();
	afx_msg void OnCancel();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_LDWPROT_H__E0CD7A09_C5DC_481E_A871_03B423E615F8__INCLUDED_)
