#ifndef INC__ABOUT_H__
#define INC__ABOUT_H__
#include "Resource.h"
#include "afxlinkctrl.h" // CMFCLinkCtrl

/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
    CAboutDlg(const CString &sTitle);

// Dialog Data
    //{{AFX_DATA(CAboutDlg)
    enum { IDD = IDD_ABOUTBOX };
    CMFCLinkCtrl m_LinkNuvoton;
    CMFCLinkCtrl m_LinkGitHub;
    CMFCLinkCtrl m_LinkGITEE;
    CMFCLinkCtrl m_LinkGitLab;
    //}}AFX_DATA

    // ClassWizard generated virtual function overrides
    //{{AFX_VIRTUAL(CAboutDlg)
protected:
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support
    //}}AFX_VIRTUAL

// Implementation
protected:
    CString	m_sUpdateURL;
    CString	m_sTitle;
    CString	m_sDate;
    //{{AFX_MSG(CAboutDlg)
    virtual BOOL OnInitDialog();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};

void OpenConsole();

#endif

