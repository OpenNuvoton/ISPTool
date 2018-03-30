#ifndef INC__ABOUT_H__
#define INC__ABOUT_H__
#include "Resource.h"
#include "HyperLink.h"

/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
    CAboutDlg(const CString &sTitle);

// Dialog Data
    //{{AFX_DATA(CAboutDlg)
    enum { IDD = IDD_ABOUTBOX };
    CHyperLink	m_LinkNuvoton;
    CHyperLink	m_LinkGitHub;
    CHyperLink	m_LinkGITEE;
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
    afx_msg void OnLinkNuvoton();
    afx_msg void OnLinkGitHub();
    afx_msg void OnLinkGITEE();
    //}}AFX_MSG
    DECLARE_MESSAGE_MAP()
};



void OpenConsole();

#endif

