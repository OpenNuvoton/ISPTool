
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
    unsigned int m_uInterface;

    CMyCommandLineInfo()
        : fnParseArg(NULL)
        , szArgIndex(0)
        , CISPProc(NULL)
        , m_bBatch(false)
        , m_uInterface(0)
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
            } else if (_tcscmp(pszParam, _T("nvm")) == 0) {
                m_bProgram_NVM = TRUE;
                szArgIndex = 1; // 1: Data Flash
                fnParseArg = &CMyCommandLineInfo::ParseArg_2files;
            } else if (_tcscmp(pszParam, _T("spi")) == 0) {
                m_bProgram_SPI = TRUE;
                szArgIndex = 2; // 2: SPI Flash
                fnParseArg = &CMyCommandLineInfo::ParseArg_2files;
            } else if (_tcscmp(pszParam, _T("interface")) == 0) {
                szArgIndex = 0;
                fnParseArg = &CMyCommandLineInfo::ParseArg_Interface;
            } else if (_tcscmp(pszParam, _T("run")) == 0) {
                m_bRunAPROM = TRUE;
            } else if (_tcscmp(pszParam, _T("batch")) == 0) {
                m_bRunAPROM = TRUE;
                m_bBatch = true;
            } else if (_tcscmp(pszParam, _T("erase")) == 0) {
                m_bErase = TRUE;
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
        if (szArgIndex == 0) {
            if (_tcscmp(pszParam, _T("HID")) == 0) {
                m_uInterface = INTF_HID;
                SetInterface(m_uInterface, _T(""), _T(""), _T(""));
            } else if (_tcscmp(pszParam, _T("UART")) == 0) {
                m_uInterface = INTF_UART;
            }
        } else if (szArgIndex == 1) {
            if (m_uInterface == INTF_UART) {
                SetInterface(m_uInterface, pszParam, _T(""), _T(""));
            }
        }

        ++szArgIndex;
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

        if (!AttachConsole(ATTACH_PARENT_PROCESS)) {
            if (!AllocConsole()) {
                MessageBox(GetConsoleWindow(), _T("Failed to create debug console."), NULL, MB_OKCANCEL | MB_ICONEXCLAMATION);
            }
        }

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
                    } else if (fnThreadProcStatus == &CISPProc::Thread_CheckDisconnect) {
                        // In some minor error case (file size check error), isptool did not go into disconnect status.
                        break;
                    } else {
                        _tprintf(_T("\n UnKnown Stage."));
                        break;
                    }
                }
            }

            printf("\n ");

            switch (rCmdInfo.m_eProcSts) {
                case EPS_ERR_OPENPORT:
                    printf("Open Port Error");
                    break;

                case EPS_ERR_CONNECT:
                    printf("CMD_CONNECT Error");
                    break;

                case EPS_ERR_ERASE:
                    printf("Erase failed");
                    break;

                case EPS_ERR_CONFIG:
                    printf("Update CONFIG failed");
                    break;

                case EPS_ERR_APROM:
                    printf("Update APROM failed");
                    break;

                case EPS_ERR_NVM:
                    printf("Update Dataflash failed");
                    break;

                case EPS_ERR_SPI:
                    printf("Update SPI Flash failed");
                    break;

                case EPS_ERR_SIZE:
                    printf("Error: File Size > Flash Size");
                    break;

                case EPS_PROG_DONE:
                case EPS_OK:
                    printf("Programming flash, OK!");
                    break;

                default:
                    printf(("Unknown Status %d"), rCmdInfo.m_eProcSts);
                    break;
            }

            //if (rCmdInfo.m_eProcSts == EPS_OK) {
            //    printf("\n Program OK.");
            //} else {
            //    printf("\n Program NG. %d", rCmdInfo.m_eProcSts);
            //    break;
            //}

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
