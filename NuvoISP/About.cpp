#include "stdafx.h"
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
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

BOOL CAboutDlg::OnInitDialog()
{
    CDialog::OnInitDialog();
    CString sTitle = _T("About ") + m_sTitle;
    CString sText;
    sText.Format(_T("%s\n\n%s"), m_sTitle, m_sDate);
    SetWindowText(sTitle);
    SetDlgItemText(IDC_STATIC_MESSAGE, sText);
    m_LinkNuvoton.SetURL(_T("https://www.nuvoton.com/"));
    m_LinkGitHub.SetURL(_T("https://github.com/OpenNuvoton/ISPTool/"));
    m_LinkGITEE.SetURL(_T("https://gitee.com/OpenNuvoton/ISPTool/"));
    m_LinkGitLab.SetURL(_T("https://gitlab.com/OpenNuvoton/nuvoton-tools/ISPTool/"));
    m_LinkNuvoton.SizeToContent();
    m_LinkGitHub.SizeToContent();
    m_LinkGITEE.SizeToContent();
    m_LinkGitLab.SizeToContent();
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}
