
// ISPTool.cpp : 定義應用程式的類別行為。
//

#include "stdafx.h"
#include "NuvoISPTool.h"
#include "DlgNuvoISP.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

BEGIN_MESSAGE_MAP(CISPToolApp, CWinApp)
    ON_COMMAND(ID_HELP, &CWinApp::OnHelp)
END_MESSAGE_MAP()

CISPToolApp::CISPToolApp()
{
}

CISPToolApp theApp;

BOOL CISPToolApp::InitInstance()
{
    CWinApp::InitInstance();
    AfxInitRichEdit();
    SetRegistryKey(_T("NuvotonISP"));
    CNuvoISPDlg MainDlg;
    MainDlg.DoModal();
    return FALSE;
}
