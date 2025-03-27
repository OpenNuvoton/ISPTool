#if !defined(AFX_DIALOGCHIPSETTING_SECURE_M2351_H__BF199825_A718_4D54_9236_F52EAC800A92__INCLUDED_)
#define AFX_DIALOGCHIPSETTING_SECURE_M2351_H__BF199825_A718_4D54_9236_F52EAC800A92__INCLUDED_

#if _MSC_VER > 1000
    #pragma once
#endif // _MSC_VER > 1000
// DialogChipSetting_SECURE_M2351.h : header file
//
#include "DialogResize.h"
#include "NumEdit.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_SECURE_M2351 dialog

class CDialogChipSetting_SECURE_M2351 : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_SECURE_M2351)

    // Construction
public:
    CDialogChipSetting_SECURE_M2351(CWnd* pParent = NULL);      // standard constructor
    virtual ~CDialogChipSetting_SECURE_M2351();

    // Dialog Data
    //{{AFX_DATA(CDialogChipSetting_SECURE_M2351)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_SECURE_M2351 };
    // NOTE: the ClassWizard will add data members here
    //}}AFX_DATA

    BOOL            m_bNSAddr_Write;
    BOOL            m_bSecure_Lock;
    BOOL            m_bAll_Lock;

    CNumEdit        m_NSecure_Addr;
    CString         m_sNSecure_Addr;

    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uNSecure_Addr;
    unsigned int    m_uNSecure_Addr_restore;
    BOOL            m_bSecureDebug;
    BOOL            m_bNSecureArea;

    // Overrides
    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CDialogChipSetting_SECURE_M2351)
    //}}AFX_VIRTUAL

    // Implementation
protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_SECURE_M2351)
    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    virtual void ConfigToGUI();
    virtual void GUIToConfig();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditNSCBA();
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_NSCBA dialog

class CDialogChipSetting_NSCBA : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_NSCBA)

    // Construction
public:
    CDialogChipSetting_NSCBA(CWnd* pParent = NULL);         // standard constructor
    virtual ~CDialogChipSetting_NSCBA();

    // Dialog Data
    //{{AFX_DATA(CDialogChipSetting_NSCBA)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_NSCBA };
    // NOTE: the ClassWizard will add data members here
    //}}AFX_DATA

    BOOL            m_bSecureDebug;
    BOOL            m_bCanWrite;

    unsigned int    m_uFlashBaseAddr;
    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uFlashPageSize;

    unsigned int    m_uNSAddr;
    BOOL            m_bNSAddrWrite;
    BOOL            m_bMirrorEnable;

    CNumEdit        m_NSAddr;
    CString         m_sNSAddr;

    // Implementation
protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_NSCBA)
    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange* pDX);        // DDX/DDV support
    virtual void ConfigToGUI();
    virtual void GUIToConfig();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditNSCBA();
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_NSCBA_LOCK dialog

class CDialogChipSetting_NSCBA_LOCK : public CDialogResize
{
    DECLARE_DYNAMIC(CDialogChipSetting_NSCBA_LOCK)

    // Construction
public:
    CDialogChipSetting_NSCBA_LOCK(CWnd* pParent = NULL);    // standard constructor
    virtual ~CDialogChipSetting_NSCBA_LOCK();

    // Dialog Data
    //{{AFX_DATA(CDialogChipSetting_NSCBA_LOCK)
    enum { IDD = IDD_DIALOG_CHIP_SETTING_NSCBA_LOCK };
    // NOTE: the ClassWizard will add data members here
    //}}AFX_DATA

    BOOL            m_bSecureDebug;
    BOOL            m_bSupportLock;

    unsigned int    m_uFlashBaseAddr;
    unsigned int    m_uProgramMemorySize;
    unsigned int    m_uFlashPageSize;

    unsigned int    m_uNSAddr;
    unsigned int    m_uNSAddr_min;
    BOOL            m_bWrite;
    BOOL            m_bMirBoundEnable;

    BOOL            m_bSCRLOCK;
    BOOL            m_bARLOCK;

    CNumEdit        m_NSAddr;
    CString         m_sNSAddr;

    // Implementation
protected:
    // Generated message map functions
    //{{AFX_MSG(CDialogChipSetting_NSCBA_LOCK)
    virtual BOOL OnInitDialog();
    virtual void DoDataExchange(CDataExchange* pDX);        // DDX/DDV support
    virtual void UpdateConfigString();
    virtual void ConfigToGUI();
    virtual void GUIToConfig();
    afx_msg void OnCheckClick();
    afx_msg void OnKillfocusEditNSCBA();
    afx_msg void OnOK();
    afx_msg void OnCancel();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIALOGCHIPSETTING_SECURE_M2351_H__BF199825_A718_4D54_9236_F52EAC800A92__INCLUDED_)
