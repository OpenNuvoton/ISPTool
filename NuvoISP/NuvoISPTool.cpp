
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


class CMyCommandLineInfo
    :	public CCommandLineInfo, public CISPProc
{
protected:
    void (CMyCommandLineInfo::*fnParseArg)(const TCHAR *pszParam, BOOL bLast);
    size_t szArgIndex;
public:
    bool m_bBatch;

    CMyCommandLineInfo()
        : fnParseArg(NULL)
        , szArgIndex(0)
        , CISPProc(NULL)
        , m_bBatch(false)
    {
        m_uAPROM_Addr   = 0;
        m_uAPROM_Size   = 0xFFFFFFFF;
        m_uAPROM_Offset = 0;
        Set_ThreadAction(&CISPProc::Thread_Idle);
    }

    virtual void ParseParam(const TCHAR *pszParam, BOOL bFlag, BOOL bLast)
    {
        if (bFlag) {
            szArgIndex = 0;
            fnParseArg = NULL;

            if (_tcscmp(pszParam, _T("aprom")) == 0) {
                m_bProgram_APROM = TRUE;
                szArgIndex = 0; // 0: APROM
                fnParseArg = &CMyCommandLineInfo::ParseArg_2files;
            } else if (_tcscmp(pszParam, _T("interface")) == 0) {
                fnParseArg = &CMyCommandLineInfo::ParseArg_Interface;
            } else if (_tcscmp(pszParam, _T("run")) == 0) {
                m_bRunAPROM = TRUE;
            } else if (_tcscmp(pszParam, _T("batch")) == 0) {
                m_bRunAPROM = TRUE;
                m_bBatch = true;
            } else {
                printf("Unknown Error @ ParseParam.\n");

                while (1);
            }
        } else if (fnParseArg != NULL) {
            (this->*fnParseArg)(pszParam, bLast);
        } else {
            printf("Unknown Error @ ParseParam.\n");

            while (1);
        }
    }

    virtual void ParseArg_2files(const TCHAR *pszParam, BOOL bLast)
    {
        if (UpdateBinFile(szArgIndex, pszParam)) {
            printf("Load File OK.\n");
        } else {
            printf("Load File NG.\n");

            while (1);
        }
    }

    virtual void ParseArg_Interface(const TCHAR *pszParam, BOOL bLast)
    {
        if (_tcscmp(pszParam, _T("HID")) == 0) {
            SetInterface(1, _T("")); // 1: HID
        } else {
            SetInterface(2, pszParam); // 2: UART & specific COM port
        }
    }
};


BOOL CISPToolApp::InitInstance()
{
    CWinApp::InitInstance();
    AfxInitRichEdit();
    {
        // https://docs.microsoft.com/en-us/windows/win32/api/shellapi/nf-shellapi-commandlinetoargvw
        int nArgs = 1;
        {
            LPWSTR *szArglist;
            szArglist = CommandLineToArgvW(GetCommandLineW(), &nArgs);
            // Free memory allocated for CommandLineToArgvW arguments.
            LocalFree(szArglist);
        }

        if (nArgs == 1) {
            goto _UI_MODE;
        }

        AttachConsole(ATTACH_PARENT_PROCESS);
        freopen("CONIN$", "r+t", stdin);
        freopen("CONOUT$", "w+t", stdout);
        setbuf(stdout, NULL);
        /* Parse command line */
        CMyCommandLineInfo rCmdInfo;
        ParseCommandLine(rCmdInfo);

        do {
            void (CISPProc::*fnThreadProcStatus)() = &CISPProc::Thread_Pause;
            void (CISPProc::*fnThreadProcStatus_backup)() = &CISPProc::Thread_Pause;
            rCmdInfo.Set_ThreadAction(&CISPProc::Thread_CheckUSBConnect);
            _tprintf(_T("\n --------------------------------"));

            while (fnThreadProcStatus != &CISPProc::Thread_Idle) {
                if (fnThreadProcStatus == fnThreadProcStatus_backup) { // status not changed, check again
                    fnThreadProcStatus = rCmdInfo.m_fnThreadProcStatus;
                    continue;
                } else { // status changed
                    fnThreadProcStatus_backup = fnThreadProcStatus;

                    if (fnThreadProcStatus == &CISPProc::Thread_CheckUSBConnect) {
                        _tprintf(_T("\n CheckUSBConnect."));
                    } else if (fnThreadProcStatus == &CISPProc::Thread_CheckDeviceConnect) {
                        _tprintf(_T("\n CheckDeviceConnect."));
                    } else if (fnThreadProcStatus == &CISPProc::Thread_ProgramFlash) {
                        _tprintf(_T("\n ProgramFlash."));
                    } else {
                        _tprintf(_T("\n UnKnown Stage."));
                    }
                }
            }

            if (rCmdInfo.m_eProcSts == EPS_OK) {
                printf("\nProgram OK.");
            } else {
                printf("\nProgram NG.");
            }

            if (rCmdInfo.m_bBatch) {
                continue;
            } else {
                break;
            }
        } while (rCmdInfo.m_bBatch);

        _tprintf(_T("\n Bye. \n\n"));
        fflush(stdout);
        return FALSE;
    }
_UI_MODE:
    SetRegistryKey(_T("NuvotonISP"));
    CNuvoISPDlg MainDlg;
    MainDlg.DoModal();
    return FALSE;
}
