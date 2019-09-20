#include "stdafx.h"
#include "HyperLink.h"
#include <sstream>	// for std::ostringstream

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif
#include "About.h"
/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

CAboutDlg::CAboutDlg(const CString &sTitle)
    :	CDialog(CAboutDlg::IDD)
    ,	m_sUpdateURL(_T("http://www.nuvoton.com/NuMicro/"))
    ,	m_sTitle(sTitle)
{
    std::ostringstream os;
    os << "Build: " << __DATE__ << " @ " << __TIME__;
    std::string cstr = os.str();
    std::wstring wcstr(cstr.begin(), cstr.end());
    m_sDate = wcstr.c_str();
}

void CAboutDlg::DoDataExchange(CDataExchange *pDX)
{
    CDialog::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CAboutDlg)
    DDX_Control(pDX, IDC_LINK_NUVOTON, m_LinkNuvoton);
    DDX_Control(pDX, IDC_LINK_GITHUB, m_LinkGitHub);
    DDX_Control(pDX, IDC_LINK_GITEE, m_LinkGITEE);
    DDX_Control(pDX, IDC_LINK_GITLAB, m_LinkGitLab);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
    //{{AFX_MSG_MAP(CAboutDlg)
    ON_BN_CLICKED(IDC_LINK_NUVOTON, OnLinkNuvoton)
    ON_BN_CLICKED(IDC_LINK_GITHUB, OnLinkGitHub)
    ON_BN_CLICKED(IDC_LINK_GITEE, OnLinkGITEE)
    ON_BN_CLICKED(IDC_LINK_GITLAB, OnLinkGitLab)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

void CAboutDlg::OnLinkNuvoton()
{
    m_LinkNuvoton.VisitURL();
}

BOOL CAboutDlg::OnInitDialog()
{
    CDialog::OnInitDialog();
    CString sTitle = _T("About ") + m_sTitle;
    CString sText;
    sText.Format(_T("%s\n\n%s"), m_sTitle, m_sDate);
    SetWindowText(sTitle);
    SetDlgItemText(IDC_STATIC_MESSAGE, sText);
    m_LinkNuvoton.SetWindowText(m_sUpdateURL);
    m_LinkNuvoton.SetAutoSize(TRUE);
    m_LinkNuvoton.SetURL(m_sUpdateURL);
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CAboutDlg::OnLinkGitHub()
{
    m_LinkGitHub.VisitURL();
}

void CAboutDlg::OnLinkGITEE()
{
    m_LinkGITEE.VisitURL();
}

void CAboutDlg::OnLinkGitLab()
{
    m_LinkGitLab.VisitURL();
}
