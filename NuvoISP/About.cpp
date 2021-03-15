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
    DDX_Control(pDX, IDC_LINK_NUVOTON, m_LinkUrl[0]);
    DDX_Control(pDX, IDC_LINK_GITHUB, m_LinkUrl[1]);
    DDX_Control(pDX, IDC_LINK_GITEE, m_LinkUrl[2]);
    DDX_Control(pDX, IDC_LINK_GITLAB, m_LinkUrl[3]);
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
    const TCHAR *_sURL[4] = {
        _T("https://www.nuvoton.com/"),
        _T("https://github.com/OpenNuvoton/ISPTool/"),
        _T("https://gitee.com/OpenNuvoton/ISPTool/"),
        _T("https://gitlab.com/OpenNuvoton/nuvoton-tools/ISPTool/")
    };

    for (int i = 0; i < 4; i++) {
        m_LinkUrl[i].SetWindowText(_sURL[i]);
        m_LinkUrl[i].SetURL(_sURL[i]);
        m_LinkUrl[i].SizeToContent();
    }

    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}
