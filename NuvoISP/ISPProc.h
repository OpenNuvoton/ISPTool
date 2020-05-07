// ICPToolDlg.h : header file
//

#if !defined(_ISP_PROC_H_)
#define _ISP_PROC_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "ISPLdCMD.h"

#include "fileinfo.h"

#if (SUPPORT_SPIFLASH)
#define NUM_VIEW 3
#else
#define NUM_VIEW 2
#endif

#define MSG_USER_EVENT				(WM_APP+1)
#define MSG_UPDATE_ERASE_STATUS		3
#define MSG_UPDATE_WRITE_STATUS		4
#define MSG_UPDATE_CONNECT_STATUS	7
#define CONNECT_STATUS_NONE			0
#define CONNECT_STATUS_USB			1
#define CONNECT_STATUS_CONNECTING	2
#define CONNECT_STATUS_OK			3

/////////////////////////////////////////////////////////////////////////////
// CISPProc dialog
enum EProcSts {
    EPS_OK = 0,
    // Thread_CheckUSBConnect
    EPS_ERR_OPENPORT = 1,
    EPS_ERR_CONNECT = 2,
    // Thread_ProgramFlash
    EPS_ERR_ERASE = 8,
    EPS_ERR_CONFIG = 3,
    EPS_ERR_APROM = 4,
    EPS_ERR_NVM = 5,
    EPS_ERR_SIZE = 6,
    EPS_PROG_DONE = 7,
    EPS_ERR_SPI = 9,
};	// m_eProcSts

class CISPProc
{
// Construction
public:
    CISPProc(HWND *pWnd);	// standard constructor
    virtual ~CISPProc();
    fileinfo m_sFileInfo[NUM_VIEW];
    bool UpdateBinFile(int idx, CString fileName)
    {
        if (idx < NUM_VIEW) {
            return UpdateFileInfo(fileName, &m_sFileInfo[idx]);
        } else {
            return false;
        }
    }

    void Set_ThreadAction(void (CISPProc::*fnThreadProcStatus)());
    void (CISPProc::*m_fnThreadProcStatus)();

protected:
    HWND *MainHWND;
    /* State machine */
    void Call_ThreadAction(void (CISPProc::*fnThreadProcStatus)());
    void (CISPProc::*m_fnThreadProcStatus_backup)();
public:
    HANDLE m_hThreadMutex;
    void LockGUI();
    void UnlockGUI();

    CWinThread *m_pAssistThread;
    DWORD AssistThread(LPVOID pArg);
    void Thread_Pause();
    void Thread_Idle();
    void Thread_CheckUSBConnect();
    void Thread_CheckDeviceConnect();
    void Thread_CheckDisconnect();
    void Thread_ProgramFlash();

    unsigned char m_ucFW_VER;
    unsigned int m_ulDeviceID;
    unsigned int m_CONFIG[4];
    unsigned int m_CONFIG_User[4];

    unsigned int m_uAPROM_Addr;
    unsigned int m_uAPROM_Size;
    unsigned int m_uNVM_Addr;
    unsigned int m_uNVM_Size;

    unsigned int m_uAPROM_Offset;

    bool UpdateSizeInfo(unsigned int uID, unsigned int uConfig0, unsigned int uConfig1);

    // Programming Option is binding with UI
    BOOL	m_bProgram_APROM;
    BOOL	m_bProgram_NVM;
    BOOL	m_bProgram_Config;
    BOOL	m_bErase;
    BOOL	m_bRunAPROM;
#if (SUPPORT_SPIFLASH)
    BOOL	m_bSupport_SPI;
    BOOL	m_bProgram_SPI;
    BOOL	m_bErase_SPI;
#endif
    // ISPLdCMD2 supports different protocol for CAN interface
    ISPLdCMD2	m_ISPLdDev;
    void SetInterface(unsigned int it, CString str)
    {
        m_ISPLdDev.SetInterface(it, str);
    };

    EProcSts m_eProcSts;

    unsigned int m_uProgTime;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(_ISP_PROC_H_)
