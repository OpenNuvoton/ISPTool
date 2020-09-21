#include "stdafx.h"
#include "Resource.h"
#include "DialogConfiguration.h" // Resource ID
#include "DlgNuvoISP.h"
#include "About.h"

#include <dbt.h>

#include "NuDataBase.h"
#include <sstream>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

inline std::string size_str(unsigned int size)
{
    char buf[128];

    if (size == 0) {
        _snprintf_s(buf, sizeof(buf), _TRUNCATE, "0K", size);
    } else if (size <= 1) {
        _snprintf_s(buf, sizeof(buf), _TRUNCATE, "%d byte", size);
    } else if (size < 1024) {
        _snprintf_s(buf, sizeof(buf), _TRUNCATE, "%d bytes", size);
    } else if (((size / 1024.) + 0.005) < 1024) {
        double f = (size / 1024.) + 0.005;
        unsigned int i = (unsigned int)f;
        unsigned int j = (unsigned int)((f - i) * 100.);

        if (j == 0) {
            _snprintf_s(buf, sizeof(buf), _TRUNCATE, "%dK", i);
        } else {
            _snprintf_s(buf, sizeof(buf), _TRUNCATE, "%d.%02dK", i, j);
        }
    } else {
        double f = (size / 1024. / 1024.) + 0.005;
        unsigned int i = (unsigned int)f;
        unsigned int j = (unsigned int)((f - i) * 100.);

        if (j == 0) {
            _snprintf_s(buf, sizeof(buf), _TRUNCATE, "%dM", i);
        } else {
            _snprintf_s(buf, sizeof(buf), _TRUNCATE, "%d.%02dM", i, j);
        }
    }

    return buf;
}

/////////////////////////////////////////////////////////////////////////////
// CNuvoISPDlg dialog

CNuvoISPDlg::CNuvoISPDlg(UINT Template,
                         CWnd *pParent /*=NULL*/)
    : CDialogMain(Template, pParent)
    , CISPProc(&m_hWnd)
{
    m_bConnect = false;
    int i = 0, j = 0;

    for (i = 0; i < NUM_VIEW; i++) {
        pViewer[i] = NULL;
    }

    WINCTRLID buddy[] = {
        {IDC_BUTTON_APROM, IDC_EDIT_FILEPATH_APROM, IDC_STATIC_FILEINFO_APROM},
        {IDC_BUTTON_NVM, IDC_EDIT_FILEPATH_NVM, IDC_STATIC_FILEINFO_NVM},
#if (SUPPORT_SPIFLASH)
        {IDC_BUTTON_SPI, IDC_EDIT_FILEPATH_SPI, IDC_STATIC_FILEINFO_SPI},
#endif
    };
    memcpy(&m_CtrlID, buddy, sizeof(m_CtrlID));
    m_bShowSPI = -1; // not initialized
}

CNuvoISPDlg::~CNuvoISPDlg()
{
    for (int i = 0; i < NUM_VIEW; i++) {
        if (pViewer[i] != NULL) {
            pViewer[i]->DestroyWindow();
        }

        delete pViewer[i];
        pViewer[i] = NULL;
    }

    UnregisterNotification();
}

void CNuvoISPDlg::DoDataExchange(CDataExchange *pDX)
{
    CDialogMain::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CNuvoISPDlg)
    DDX_Control(pDX, IDC_TAB_DATA, m_TabData);
    DDX_Control(pDX, IDC_BUTTON_CONNECT, m_ButtonConnect);
    DDX_Text(pDX, IDC_STATIC_CONNECT, m_sConnect);
    DDX_Check(pDX, IDC_CHECK_APROM, m_bProgram_APROM);
    DDX_Check(pDX, IDC_CHECK_NVM, m_bProgram_NVM);
#if (SUPPORT_SPIFLASH)
    DDX_Check(pDX, IDC_CHECK_SPI, m_bProgram_SPI);
    DDX_Check(pDX, IDC_CHECK_ERASE_SPI, m_bErase_SPI);
#endif
    DDX_Check(pDX, IDC_CHECK_CONFIG, m_bProgram_Config);
    DDX_Check(pDX, IDC_CHECK_ERASE, m_bErase);
    DDX_Check(pDX, IDC_CHECK_RUN_APROM, m_bRunAPROM);
    DDX_Control(pDX, IDC_TAB_DATA, m_TabData);
    DDX_Control(pDX, IDC_PROGRESS, m_Progress);
    DDX_Text(pDX, IDC_STATIC_STATUS, m_sStatus);
    DDX_Control(pDX, IDC_COMBO_COM_PORT, m_SelComPort);
    DDX_Control(pDX, IDC_COMBO_INTERFACE, m_SelInterface);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CNuvoISPDlg, CDialog)
    //{{AFX_MSG_MAP(CNuvoISPDlg)
    ON_WM_SYSCOMMAND()
    ON_BN_CLICKED(IDOK, OnOK)
    ON_BN_CLICKED(IDC_BUTTON_CONNECT, OnButtonConnect)
    ON_BN_CLICKED(IDC_BUTTON_APROM, OnButtonLoadFile)
    ON_BN_CLICKED(IDC_BUTTON_NVM, OnButtonLoadFile)
#if (SUPPORT_SPIFLASH)
    ON_BN_CLICKED(IDC_BUTTON_SPI, OnButtonLoadFile)
#endif
    ON_BN_CLICKED(IDC_BUTTON_START, OnButtonStart)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB_DATA, OnSelchangeTabData)
    ON_BN_CLICKED(IDC_BUTTON_CONFIG, OnButtonConfig)
    //}}AFX_MSG_MAP
    ON_WM_DROPFILES()
    ON_WM_CTLCOLOR()
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    ON_WM_PAINT()
    ON_CBN_SELCHANGE(IDC_COMBO_INTERFACE, OnSelchangeInterface)
    ON_CBN_SELCHANGE(IDC_COMBO_COM_PORT, OnComboChange)
    //ON_WM_DEVICECHANGE()
    ON_MESSAGE(WM_DEVICECHANGE, OnDeviceChange)
    ON_EN_KILLFOCUS(IDC_EDIT_APROM_BASE_ADDRESS, OnKillfocusEditAPRomOffset)
END_MESSAGE_MAP()

BOOL CNuvoISPDlg::OnInitDialog()
{
    CDialogMain::OnInitDialog();
    // IDM_ABOUTBOX must be in the system command range.
    ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
    ASSERT(IDM_ABOUTBOX < 0xF000);
    CMenu *pSysMenu = GetSystemMenu(FALSE);

    if (pSysMenu != NULL) {
        CString strAboutMenu;
        strAboutMenu = _T("About Nuvoton NuMicro ISP Programming Tool(&A)");

        if (!strAboutMenu.IsEmpty()) {
            pSysMenu->AppendMenu(MF_SEPARATOR);
            pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
        }
    }

    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
    // Set the icon for this dialog.  The framework does this automatically
    //  when the application's main window is not a dialog
    SetIcon(m_hIcon, TRUE);			// Set big icon
    SetIcon(m_hIcon, FALSE);		// Set small icon
    m_sConnect = _T("Disconnected");
    UpdateData(FALSE);
    // Title
    SetWindowText(_T("Nuvoton NuMicro ISP Programming Tool 4.03"));

    // Set data view area
    // Btn Text --> Tab Text
    for (int i = 0; i < NUM_VIEW; i++) {
        CString strTab;
        GetDlgItemText(m_CtrlID[i].btn, strTab);
        m_TabData.InsertItem(i, strTab);
    }

    CRect rcClient;
    m_TabData.GetClientRect(rcClient);
    m_TabData.AdjustRect(FALSE, rcClient);

    for (int i = 0; i < NUM_VIEW; i++) {
        CDialogHex *pDlg = new CDialogHex;
        BOOL result = pDlg->Create(CDialogHex::IDD, &m_TabData);
        int error = ::GetLastError();
        pViewer[i] = pDlg;
        pViewer[i]->MoveWindow(rcClient);
        pViewer[i]->ShowWindow(SW_HIDE);
    }

    pViewer[0]->ShowWindow(SW_SHOW);
    m_Progress.SetRange(0, 100);
    EnableDlgItem(IDC_BUTTON_START, m_bConnect);
    InitComboBox();
    SetDlgItemText(IDC_EDIT_FLASH_BASE_ADDRESS, _T("100000"));
    Set_ThreadAction(&CISPProc::Thread_Idle);
    RegisterNotification();
    ShowSPIOptions(FALSE);
    // ShowSPIOptions(TRUE);
    return TRUE;	// return TRUE  unless you set the focus to a control
}

void CNuvoISPDlg::OnButtonBinFile(int idx, TCHAR *szPath)
{
    CString FileName = _T("");
    // Backup current directory
    TCHAR szCurDir[MAX_PATH];

    if (GetCurrentDirectory(sizeof(szCurDir) / sizeof(szCurDir[0]), szCurDir) == 0) {
        szCurDir[0] = (TCHAR)'\0';
    }

    // Open file dialog
    CFileDialog dialog(TRUE, NULL, NULL,
                       OFN_EXPLORER | OFN_ENABLESIZING | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY,
                       _T("Binary Files (*.bin)|*.bin|All Files|*.*||"),
                       this);

    if (szPath != NULL) {
        FileName = szPath;
    } else if (dialog.DoModal() == IDOK) {
        FileName = dialog.GetPathName();
    }

    // Restore current directory
    if (szCurDir[0] != (TCHAR)'\0') {
        SetCurrentDirectory(szCurDir);
    }

    if (FileName != _T("")) {
        int sz = getFilesize(FileName.GetBuffer(0));

        if (UpdateFileInfo(FileName, &(m_sFileInfo[idx]))) {
            // File Path
            SetDlgItemText(m_CtrlID[idx].path, FileName);

            // Size & Check Sum Info
            if (m_CtrlID[idx].sizecksum) {
                CString str, strInfo;

                if (sz >= 1024 * 1024 * 10)
                    str.Format(_T("size: %.1fM Bytes"),
                               (float)(sz / 1024.0 / 1024.0));
                else if (sz >= 1024 * 10)
                    str.Format(_T("size: %.1fK Bytes"),
                               (float)(sz / 1024.0));
                else
                    str.Format(_T("size: %d Bytes"),
                               sz);

                strInfo.Format(_T("%s, checksum: %04x"), str, m_sFileInfo[idx].usCheckSum);
                SetDlgItemText(m_CtrlID[idx].sizecksum, strInfo);
                pViewer[idx]->SetHexData(&(m_sFileInfo[idx].vbuf));
                m_TabData.SetCurSel(idx);
                OnSelchangeTabData(NULL, NULL);
            }
        }
    }
}

void CNuvoISPDlg::OnButtonLoadFile()
{
    const MSG *msg = GetCurrentMessage();
    int BtnID = LOWORD(msg->wParam);

    for (int idx = 0; idx < NUM_VIEW; idx++) {
        if (BtnID == m_CtrlID[idx].btn) {
            OnButtonBinFile(idx);
            return;
        }
    }
}

void CNuvoISPDlg::OnButtonConnect()
{
    if (m_fnThreadProcStatus == &CISPProc::Thread_Idle
            || m_fnThreadProcStatus == &CISPProc::Thread_Pause) {
        /* Connect */
        m_SelComPort.GetWindowText(m_ComNum);
        SetInterface(m_SelInterface.GetCurSel() + 1, m_ComNum);
        EnableInterface(false);
        Set_ThreadAction(&CISPProc::Thread_CheckUSBConnect);
    } else if (m_fnThreadProcStatus != NULL) {
        /* Disconnect */
        Set_ThreadAction(&CISPProc::Thread_Idle);
    }
}

HBRUSH CNuvoISPDlg::OnCtlColor(CDC *pDC, CWnd *pWnd, UINT nCtlColor)
{
    HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);

    // TODO:  在此變更 DC 的任何屬性
    switch (pWnd->GetDlgCtrlID()) {
        case IDC_STATIC_CONNECT:
            if (m_bConnect) {
                pDC->SetTextColor(RGB(0, 128, 0));	//DarkGreen
            } else {
                pDC->SetTextColor(RGB(255, 0, 0));	//Red
            }

            break;

        case IDC_STATIC_CONFIG_VALUE_0:
            if (!m_bConnect) {
                break;
            }

            if (m_CONFIG_User[0] == m_CONFIG[0]) {
                pDC->SetTextColor(RGB(0, 128, 0));	//DarkGreen
            } else {
                pDC->SetTextColor(RGB(255, 0, 0));	//Red
            }

            break;

        case IDC_STATIC_CONFIG_VALUE_1:
            if (!m_bConnect) {
                break;
            }

            if (m_CONFIG_User[1] == m_CONFIG[1]) {
                pDC->SetTextColor(RGB(0, 128, 0));	//DarkGreen
            } else {
                pDC->SetTextColor(RGB(255, 0, 0));	//Red
            }

            break;

        case IDC_STATIC_CONFIG_VALUE_2:
            if (!m_bConnect) {
                break;
            }

            if (m_CONFIG_User[2] == m_CONFIG[2]) {
                pDC->SetTextColor(RGB(0, 128, 0));	//DarkGreen
            } else {
                pDC->SetTextColor(RGB(255, 0, 0));	//Red
            }

            break;

        case IDC_STATIC_CONFIG_VALUE_3:
            if (!m_bConnect) {
                break;
            }

            if (m_CONFIG_User[3] == m_CONFIG[3]) {
                pDC->SetTextColor(RGB(0, 128, 0));	//DarkGreen
            } else {
                pDC->SetTextColor(RGB(255, 0, 0));	//Red
            }

            break;

        default:
            break;
    }

    // TODO:  如果預設值並非想要的，則傳回不同的筆刷
    return hbr;
}

LRESULT CNuvoISPDlg::WindowProc(UINT message, WPARAM wParam, LPARAM lParam)
{
    if (message == MSG_USER_EVENT) {
        if (wParam == MSG_UPDATE_CONNECT_STATUS) {
            CString sMessage;
            UpdateData(true);
            m_sStatus = _T("");

            switch (lParam) {
                case CONNECT_STATUS_NONE:
                    EnableInterface(true);
                    m_sConnect		= _T("Disconnected");

                    switch (m_eProcSts) {
                        case EPS_OK:
                            m_sConnect = _T("Disconnected");
                            break;

                        case EPS_ERR_OPENPORT:
                            m_sConnect = _T("Open Port Error");
                            break;

                        case EPS_ERR_CONNECT:
                            m_sConnect = _T("CMD_CONNECT Error");
                            break;

                        default:
                            break;
                    }

                    ShowChipInfo_OffLine();
                    break;

                case CONNECT_STATUS_USB:
                    m_sConnect		= _T("Waiting for device connection");
                    m_ButtonConnect.SetWindowText(_T("Stop check"));
                    break;

                case CONNECT_STATUS_CONNECTING:
                    m_sConnect		= _T("Getting Chip Information ...");
                    m_ButtonConnect.SetWindowText(_T("Stop check"));
                    break;

                case CONNECT_STATUS_OK:
                    m_sConnect		= _T("Connected");
                    EnableProgramOption(TRUE);
                    ShowChipInfo_OnLine();

                    switch (m_eProcSts) {
                        case EPS_ERR_ERASE:
                            AfxMessageBox(_T("Erase failed"));
                            break;

                        case EPS_ERR_CONFIG:
                            //m_sStatus = _T("CONFIG NG");//"Disconnected"
                            AfxMessageBox(_T("Update CONFIG failed"));
                            break;

                        case EPS_ERR_APROM:
                            //m_sStatus = _T("APROM NG");
                            AfxMessageBox(_T("Update APROM failed"));
                            break;

                        case EPS_ERR_NVM:
                            //m_sStatus = _T("Data Flash NG");
                            AfxMessageBox(_T("Update Dataflash failed"));
                            break;

                        case EPS_ERR_SPI:
                            AfxMessageBox(_T("Update SPI Flash failed"));
                            break;

                        case EPS_ERR_SIZE:
                            AfxMessageBox(_T("File Size > Flash Size"));
                            break;

                        case EPS_PROG_DONE:
                            sMessage.Format(_T("Programming flash, OK! (%d secs)"), m_uProgTime);
                            MessageBox(sMessage);
                            break;

                        default:
                            m_sStatus = _T("");
                            break;
                    }

                    m_ButtonConnect.SetWindowText(_T("Disconnected"));
                    break;

                default:
                    break;
            }

            m_bConnect = (lParam == CONNECT_STATUS_OK);
            EnableDlgItem(IDC_BUTTON_START, m_bConnect);
            UpdateData(false);
        } else if (wParam == MSG_UPDATE_ERASE_STATUS) {
            CString sText;
            sText.Format(_T("Erase %d%%"), (int)lParam);
            m_sStatus = sText;
            m_Progress.SetPos((int)lParam);
            UpdateData(FALSE);
        } else if (wParam == MSG_UPDATE_WRITE_STATUS) {
            CString sText;
            sText.Format(_T("Program %d%%"), (int)lParam);
            m_sStatus = sText;
            m_Progress.SetPos((int)lParam);
            UpdateData(FALSE);
        }
    }

    return CDialogMain::WindowProc(message, wParam, lParam);
}

void CNuvoISPDlg::OnButtonStart()
{
    // TODO: Add your control notification handler code here
    UpdateData(TRUE);
    /* Try to reload file if necessary */
#if (SUPPORT_SPIFLASH)

    if (m_bProgram_APROM || m_bProgram_NVM || m_bProgram_Config || m_bErase || m_bRunAPROM) {
        // Check Standart ISP Options
    } else if (m_bSupport_SPI && (m_bProgram_SPI || m_bErase_SPI)) {
        // Check Extend ISP Options for SPI Flash
    } else {
        MessageBox(_T("You did not select any operation."), NULL, MB_ICONSTOP);
        return;
    }

#else

    /* Check program operation */
    if (!(m_bProgram_APROM || m_bProgram_NVM || m_bProgram_Config || m_bErase || m_bRunAPROM)) {
        MessageBox(_T("You did not select any operation."), NULL, MB_ICONSTOP);
        return;
    }

#endif
    /* WYLIWYP : What You Lock Is What You Program*/
    /* Lock ALL */
    EnableProgramOption(FALSE);
    /* Check thread status */
    CString strErr = _T("");
    LockGUI();

    if (m_fnThreadProcStatus == &CISPProc::Thread_CheckDisconnect)
        // if(m_fnThreadProcStatus == &CNuvoISPDlg::Thread_Idle)
    {
        /* Check write size */
        if (strErr.IsEmpty() && m_bProgram_APROM) {
            if (m_sFileInfo[0].st_size == 0) {
                strErr = _T("Can not load APROM file for programming!");
            }
        }

        if (strErr.IsEmpty() && m_bProgram_NVM) {
            if (m_sFileInfo[1].st_size == 0) {
                strErr = _T("Can not load data flash file for programming!");
            }
        }

#if (SUPPORT_SPIFLASH)

        if (strErr.IsEmpty() && m_bProgram_SPI && m_bSupport_SPI) {
            if (m_sFileInfo[2].st_size == 0) {
                strErr = _T("Can not load SPI flash file for programming!");
            }
        }

#endif
        // In case user press "Enter" after typing offset, need to call OnKillfocusEditAPRomOffset manually
        OnKillfocusEditAPRomOffset();
        UpdateAddrOffset();

        if (strErr.IsEmpty()) {
            Set_ThreadAction(&CISPProc::Thread_ProgramFlash);
        }
    } else {
        strErr = _T("Please wait for ISP operation.");
    }

    if (!strErr.IsEmpty()) {
        MessageBox(strErr, NULL, MB_ICONSTOP);
        EnableProgramOption(TRUE);
    }

    UnlockGUI();
}

void CNuvoISPDlg::OnSelchangeTabData(NMHDR *pNMHDR, LRESULT *pResult)
{
    // TODO: Add your control notification handler code here
    int nSelect = m_TabData.GetCurSel();

    for (int i = 0; i < sizeof(pViewer) / sizeof(pViewer[0]); ++i) {
        if (i != nSelect) {
            pViewer[i]->ShowWindow(SW_HIDE);
        }
    }

    pViewer[nSelect]->ShowWindow(SW_SHOW);

    if (pResult != NULL) {
        *pResult = 0;
    }
}

void CNuvoISPDlg::OnDropFiles(HDROP hDropInfo)
{
    // TODO: Add your message handler code here and/or call default
    TCHAR szPath[MAX_PATH];
    POINT point;
    CRect rect;

    //Returns a count of the files dropped
    while ((DragQueryFile(hDropInfo, 0xFFFFFFFF, NULL, 0) > 0)
            && (m_fnThreadProcStatus != &CISPProc::Thread_ProgramFlash)) {
        //Retrieves the position of the mouse pointer
        DragQueryPoint(hDropInfo, &point);
        //Retrieves the name of dropped file
        DragQueryFile(hDropInfo, 0, szPath, _countof(szPath));

        if (1) {
            for (int idx = 0; idx < NUM_VIEW; idx++) {
                GetDlgItem(m_CtrlID[idx].path)->GetWindowRect(&rect);
                ScreenToClient(&rect);

                if (PtInRect(&rect, point)) {
                    OnButtonBinFile(idx, szPath);
                    break;
                }
            }
        }

        break;
    }

    DragFinish(hDropInfo);
    CDialog::OnDropFiles(hDropInfo);
}

void CNuvoISPDlg::OnButtonConfig()
{
#ifdef _DEBUG

    // offline test mode
    // user can select test chip from popup menu
    if (m_fnThreadProcStatus == &CISPProc::Thread_Idle
            || m_fnThreadProcStatus == &CISPProc::Thread_Pause) {
        DemoConfigDlg();
        return;
    }

#endif

    // online mode - must connect to real chip. gsChipCfgInfo must be valid
    if (ConfigDlgSel(m_CONFIG_User, sizeof(m_CONFIG_User))) {
        CString strTmp = _T("");
        strTmp.Format(_T("0x%08X"), m_CONFIG_User[0]);
        SetDlgItemText(IDC_STATIC_CONFIG_VALUE_0, strTmp);
        strTmp.Format(_T("0x%08X"), m_CONFIG_User[1]);
        SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, strTmp);
        strTmp.Format(_T("0x%08X"), m_CONFIG_User[2]);
        SetDlgItemText(IDC_STATIC_CONFIG_VALUE_2, strTmp);
        strTmp.Format(_T("0x%08X"), m_CONFIG_User[3]);
        SetDlgItemText(IDC_STATIC_CONFIG_VALUE_3, strTmp);
    }
}

void CNuvoISPDlg::EnableProgramOption(BOOL bEnable)
{
    /* WYLIWYP : What You Lock Is What You Program*/
    EnableDlgItem(IDC_CHECK_APROM, bEnable);
    EnableDlgItem(IDC_BUTTON_APROM, bEnable);
    EnableDlgItem(IDC_CHECK_NVM, bEnable);
    EnableDlgItem(IDC_BUTTON_NVM, bEnable);
    EnableDlgItem(IDC_CHECK_CONFIG, bEnable);
    EnableDlgItem(IDC_BUTTON_CONFIG, bEnable);
    EnableDlgItem(IDC_CHECK_ERASE, bEnable);
    EnableDlgItem(IDC_CHECK_RUN_APROM, bEnable);
    EnableDlgItem(IDC_BUTTON_START, bEnable);
#if (SUPPORT_SPIFLASH)
    EnableDlgItem(IDC_BUTTON_SPI, bEnable);
    EnableDlgItem(IDC_CHECK_SPI, bEnable);
    EnableDlgItem(IDC_CHECK_ERASE_SPI, bEnable);
#endif
}

void CNuvoISPDlg::OnPaint()
{
    if (IsIconic()) {
        CPaintDC dc(this); // device context for painting
        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
        // Center icon in client rectangle
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;
        // Draw the icon
        dc.DrawIcon(x, y, m_hIcon);
    } else {
        //顯示Logo
        //CDialog::OnPaint();
        CPaintDC   dc(this);   //Device context
        CDC   dcMem;
        dcMem.CreateCompatibleDC(&dc);
        CBitmap   bmpBackground;
        bmpBackground.LoadBitmap(IDB_BITMAP_NUVOTON);
        BITMAP   bitmap;
        bmpBackground.GetBitmap(&bitmap);
        CBitmap   *pbmpOld = dcMem.SelectObject(&bmpBackground);
        CRect MainRect;
        GetClientRect(&MainRect);
        //把dcMem拷貝到dc的相應位置
        //dc.StretchBlt(0,0,MainRect.Width(),bitmap.bmHeight,&dcMem,0,0,
        dc.StretchBlt(0, 0, bitmap.bmWidth, bitmap.bmHeight, &dcMem, 0, 0,
                      bitmap.bmWidth, bitmap.bmHeight, SRCCOPY); //bitmap.bmHeight = 44
        dcMem.SelectObject(pbmpOld);
    }
}

void CNuvoISPDlg::ShowChipInfo_OffLine(void)
{
    m_uAPROM_Offset = 0;
    SetDlgItemText(IDC_EDIT_APROM_BASE_ADDRESS, _T("0"));
    m_ButtonConnect.SetWindowText(_T("Connect"));
    SetDlgItemText(IDC_EDIT_PARTNO, _T(""));
    SetDlgItemText(IDC_STATIC_PARTNO, _T(""));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_0, _T(""));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, _T(""));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_2, _T(""));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_3, _T(""));
    ChangeBtnText(0, _T("APROM"));
    ChangeBtnText(1, _T("Data Flash"));
    ShowDlgItem(IDC_CHECK_CONFIG, 1);
    ShowDlgItem(IDC_CHECK_ERASE, 1);
    EnableDlgItem(IDC_BUTTON_CONFIG, 1);	// For Debug CONFIG dialog
    ShowDlgItem(IDC_STATIC_APOFFSET, 0);
    ShowDlgItem(IDC_EDIT_APROM_BASE_ADDRESS, 0);
    ShowDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS, 0);
    ShowDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS, 0);
    SetDlgItemText(IDC_STATIC_CONFIG_0, _T("Config 0,1:"));
    ShowDlgItem(IDC_STATIC_CONFIG_VALUE_1, 1);
    ShowDlgItem(IDC_STATIC_CONFIG_VALUE_2, 0);
    ShowDlgItem(IDC_STATIC_CONFIG_VALUE_3, 0);
    EnableProgramOption(TRUE);
    Invalidate(1);
}

// NUC505: No CONFIG, only SPI Flash, Don't need to query flash size
void CNuvoISPDlg::ShowChipInfo_NUC505(void)
{
    SetDlgItemText(IDC_EDIT_PARTNO, _T("NUC505"));
    ChangeBtnText(0,  _T("Code"));
    ChangeBtnText(1,  _T("Data"));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_0, _T("NA"));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, _T("NA"));
    m_bProgram_Config = 0;
    m_bErase = 0;
    EnableDlgItem(IDC_CHECK_CONFIG, 0);
    EnableDlgItem(IDC_CHECK_ERASE, 0);
    EnableDlgItem(IDC_BUTTON_CONFIG, 0);
    ShowDlgItem(IDC_STATIC_APOFFSET, 1);
    ShowDlgItem(IDC_EDIT_APROM_BASE_ADDRESS, 1);
    EnableDlgItem(IDC_EDIT_APROM_BASE_ADDRESS, 0);
    ShowDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS, 1);
    ShowDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS, 1);
    CString info;
    info.Format(_T("RAM:128K, SPI Flash:2M\nFW Ver: 0x%X"), int(m_ucFW_VER));
    SetDlgItemText(IDC_STATIC_PARTNO, info);
    UpdateAddrOffset();
    Invalidate(1);
}

void CNuvoISPDlg::ShowChipInfo_M2351(void)
{
    SetDlgItemText(IDC_STATIC_CONFIG_0, _T("Config 0-3:"));
    ShowDlgItem(IDC_STATIC_CONFIG_VALUE_2, 1);
    ShowDlgItem(IDC_STATIC_CONFIG_VALUE_3, 1);
    // ChangeBtnText(1, _T("APROM_NS"));
    m_uAPROM_Size = gsChipCfgInfo.uProgramMemorySize;
    ShowDlgItem(IDC_STATIC_APOFFSET, 1);
    ShowDlgItem(IDC_EDIT_APROM_BASE_ADDRESS, 1);
    EnableDlgItem(IDC_EDIT_APROM_BASE_ADDRESS, 1);
    m_uNVM_Size = 0;
    m_bProgram_NVM = 0;
    EnableDlgItem(IDC_BUTTON_NVM, (m_uNVM_Size != 0));
    EnableDlgItem(IDC_CHECK_NVM, (m_uNVM_Size != 0));
    std::ostringstream os;
    os << "APROM: " << size_str(gsChipCfgInfo.uProgramMemorySize);
    std::string cstr = os.str();
    std::wstring wcstr(cstr.begin(), cstr.end());
    CString str = wcstr.c_str();
    CString tips;
    tips.Format(_T("Information of target chip,\n\n%s"), str);
    CString info;
    info.Format(_T("%s\nFW Ver: 0x%X"), wcstr.c_str(), int(m_ucFW_VER));
    SetDlgItemText(IDC_STATIC_PARTNO, info);
    Invalidate(1);
}

void CNuvoISPDlg::ShowChipInfo_OnLine()
{
    ShowSPIOptions(m_bSupport_SPI);
    bool bSizeValid = UpdateSizeInfo(m_ulDeviceID, m_CONFIG[0], m_CONFIG[1]);

    // There is no specific part number for NUC505
    if (0x00550505 == m_ulDeviceID) {
        ShowChipInfo_NUC505();
        return;
    }

    // Update Part Number & CONFIG0 ~ CONFIG3 for all series
    CString strTmp = _T("");
    strTmp = gsChipCfgInfo.szPartNumber;
    SetDlgItemText(IDC_EDIT_PARTNO, strTmp);
    strTmp.Format(_T("0x%08X"), m_CONFIG_User[0]);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_0, strTmp);
    strTmp.Format(_T("0x%08X"), m_CONFIG_User[1]);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, strTmp);
    strTmp.Format(_T("0x%08X"), m_CONFIG_User[2]);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_2, strTmp);
    strTmp.Format(_T("0x%08X"), m_CONFIG_User[3]);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_3, strTmp);

    // Erase All command is not suppoted for CAN interface
    if (m_SelInterface.GetCurSel() == 5) { // CAN interface
        m_bErase = 0;
        EnableDlgItem(IDC_CHECK_ERASE, 0);
    }

    if ((gsChipCfgInfo.uSeriesCode == IDD_DIALOG_CONFIGURATION_M2351)
            || (gsChipCfgInfo.uSeriesCode == NUC_CHIP_TYPE_M2354)) {
        ShowChipInfo_M2351();
        return;
    }

    if (gsChipCfgInfo.uSeriesCode == IDD_DIALOG_CONFIGURATION_M480) {
        SetDlgItemText(IDC_STATIC_CONFIG_0, _T("Config 0-3:"));
        ShowDlgItem(IDC_STATIC_CONFIG_VALUE_2, 1);
        ShowDlgItem(IDC_STATIC_CONFIG_VALUE_3, 1);
    } else if (gsChipCfgInfo.uSeriesCode == IDD_DIALOG_CONFIGURATION_M480LD) { // M480LD, M479
        SetDlgItemText(IDC_STATIC_CONFIG_0, _T("Config 0-2:"));
        ShowDlgItem(IDC_STATIC_CONFIG_VALUE_2, 1);
    } else if (gsChipCfgInfo.uSeriesCode == NUC_CHIP_TYPE_GENERAL_1T) {
        m_bProgram_NVM = 0;
        EnableDlgItem(IDC_CHECK_NVM, 0);
        EnableDlgItem(IDC_BUTTON_NVM, 0);
    }

    if (bSizeValid) {
        std::ostringstream os;
        os << "APROM: " << size_str(m_uAPROM_Size) << ","
           " Data: " << size_str(m_uNVM_Size);
        std::string cstr = os.str();
        std::wstring wcstr(cstr.begin(), cstr.end());
        CString str = wcstr.c_str();
        CString tips;
        tips.Format(_T("Information of target chip,\n\n%s"), str);
        CString info;
        info.Format(_T("%s\nFW Ver: 0x%X"), wcstr.c_str(), int(m_ucFW_VER));
        SetDlgItemText(IDC_STATIC_PARTNO, info);
    } else {
        CString tips;
        tips.Format(_T("PDID: 0x%X, FW Ver: 0x%X"), m_ulDeviceID, int(m_ucFW_VER));
        SetDlgItemText(IDC_STATIC_PARTNO, tips);
    }

    Invalidate();
}

void CNuvoISPDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
    if ((nID & 0xFFF0) == IDM_ABOUTBOX) {
        CString sTitle;
        GetWindowText(sTitle);
        CAboutDlg dlgAbout(sTitle);
        dlgAbout.DoModal();
    } else {
        CDialog::OnSysCommand(nID, lParam);
    }
}

// This function is used by NUC505 only.
// User can specify the spi flash address.
void CNuvoISPDlg::UpdateAddrOffset()
{
    m_uAPROM_Addr = 0;
    CString strAddr;
    GetDlgItemText(IDC_EDIT_FLASH_BASE_ADDRESS, strAddr);
    unsigned int uAddr = ::_tcstoul(strAddr, 0, 16);

    if (0x00550505 == m_ulDeviceID) {
        m_uAPROM_Addr = 0x4000;
        m_uAPROM_Size = 0x200000 - m_uAPROM_Addr;

        if (1) {
            m_uNVM_Addr = ::_tcstoul(strAddr, 0, 16);
            m_uNVM_Addr &= ~0xFF;

            if (uAddr < m_uAPROM_Addr) {
                uAddr = 0x100000;
            } else if (uAddr > 0x200000) {
                uAddr = 0x100000;
            }

            m_uNVM_Addr = uAddr;
            m_uNVM_Size = 0x200000 - m_uNVM_Addr;
        }

        SetDlgItemText(IDC_EDIT_APROM_BASE_ADDRESS, _T("4000"));
        strAddr.Format(_T("%06X"), uAddr);
        SetDlgItemText(IDC_EDIT_FLASH_BASE_ADDRESS, strAddr);
    } else {
        return;
    }
}

void CNuvoISPDlg::RegisterNotification()
{
    DEV_BROADCAST_DEVICEINTERFACE devIF = {0};
    // Globally Unique Identifier (GUID) for HID class devices.  Windows uses GUIDs to identify things.
    GUID hidGuid = {0x4d1e55b2, 0xf16f, 0x11cf, 0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30};
    devIF.dbcc_size = sizeof(devIF);
    devIF.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    devIF.dbcc_classguid = hidGuid;
    m_hNotifyDevNode = RegisterDeviceNotification(GetSafeHwnd(), &devIF, DEVICE_NOTIFY_WINDOW_HANDLE);
}

void CNuvoISPDlg::UnregisterNotification()
{
    if (m_hNotifyDevNode) {
        UnregisterDeviceNotification(m_hNotifyDevNode);
        m_hNotifyDevNode = NULL;
    }
}

LRESULT CNuvoISPDlg::OnDeviceChange(WPARAM  nEventType, LPARAM  dwData)
{
    PDEV_BROADCAST_DEVICEINTERFACE pdbi = (PDEV_BROADCAST_DEVICEINTERFACE)dwData;
    CString DevPathName = pdbi->dbcc_name;

    switch (nEventType) {
        case DBT_DEVICEARRIVAL:
            // A device has been inserted and is now available.
            //…
            break;

        case DBT_DEVICEREMOVECOMPLETE:

            // Device has been removed.
            //…
            if (pdbi->dbcc_devicetype == DBT_DEVTYP_DEVICEINTERFACE) {
                if (DevPathName.CompareNoCase(m_ISPLdDev.m_strDevPathName) == 0) {
                    Set_ThreadAction(&CISPProc::Thread_Idle);
                }
            }

            break;

        default:
            break;
    }

    return TRUE;
}

void CNuvoISPDlg::ChangeBtnText(int nBtn, LPTSTR pszText)
{
    if (0 == nBtn) {
        SetDlgItemText(IDC_BUTTON_APROM, pszText);
        SetDlgItemText(IDC_CHECK_APROM, pszText);
    } else if (1 == nBtn) {
        SetDlgItemText(IDC_BUTTON_NVM, pszText);
        SetDlgItemText(IDC_CHECK_NVM, pszText);
    } else {
        return;
    }

    TC_ITEM ti;
    ti.mask = TCIF_TEXT;
    ti.pszText = pszText;
    VERIFY(m_TabData.SetItem(nBtn, &ti));
}

void CNuvoISPDlg::OnKillfocusEditAPRomOffset()
{
    if ((gsChipCfgInfo.uSeriesCode != IDD_DIALOG_CONFIGURATION_M2351)
            && (gsChipCfgInfo.uSeriesCode != NUC_CHIP_TYPE_M2354)) {
        return;
    }

    CString strAddr;
    GetDlgItemText(IDC_EDIT_APROM_BASE_ADDRESS, strAddr);
    m_uAPROM_Offset = ::_tcstoul(strAddr, 0, 16);
    m_uAPROM_Offset &= 0xFF800; // 2K page alignment

    if (m_uAPROM_Offset >= m_uAPROM_Size) {
        m_uAPROM_Offset = 0;
    }

    strAddr.Format(_T("%06X"), m_uAPROM_Offset);
    SetDlgItemText(IDC_EDIT_APROM_BASE_ADDRESS, strAddr);
    TRACE(_T("OnKillfocusEditAPRomOffset\n"));
}


void CNuvoISPDlg::ShowSPIOptions(BOOL bShow)
{
    if (m_bShowSPI == bShow) {
        return;
    } else {
        m_bShowSPI = bShow;
    }

    CWnd *pWnd = NULL;
    CRect rect1, rect2;
    int offset;
    // Group - Load File
    ShowDlgItem(IDC_BUTTON_SPI, bShow);
    ShowDlgItem(IDC_STATIC_FILENAME_SPI, bShow);
    ShowDlgItem(IDC_EDIT_FILEPATH_SPI, bShow);
    ShowDlgItem(IDC_STATIC_FILEINFO_SPI, bShow);
    pWnd = GetDlgItem(IDC_GROUP_FLASH_FILE);
    pWnd->GetWindowRect(&rect1);
    ScreenToClient(&rect1);

    if (bShow) {
        GetDlgItem(IDC_STATIC_FILEINFO_SPI)->GetWindowRect(&rect2);
    } else {
        GetDlgItem(IDC_STATIC_FILEINFO_NVM)->GetWindowRect(&rect2);
    }

    ScreenToClient(&rect2);
    rect1.bottom = rect2.bottom + 10;
    pWnd->MoveWindow(rect1.left,
                     rect1.top,
                     rect1.Width(),
                     rect1.Height());
    // Group - Config Bits
    UINT32 nIds_CB[] = {
        IDC_GROUP_CONFIG,
        IDC_BUTTON_CONFIG,
        IDC_STATIC_CONFIG_0,
        IDC_STATIC_CONFIG_VALUE_0,
        IDC_STATIC_CONFIG_VALUE_1,
        IDC_STATIC_CONFIG_VALUE_2,
        IDC_STATIC_CONFIG_VALUE_3,
    };
    GetDlgItem(IDC_GROUP_FLASH_FILE)->GetWindowRect(&rect1);
    ScreenToClient(&rect1);
    GetDlgItem(IDC_GROUP_CONFIG)->GetWindowRect(&rect2);
    ScreenToClient(&rect2);
    offset = rect2.top - rect1.top - rect1.Height() - 5;

    for (int i = 0; i < 7; i++) {
        CRect rect;
        CWnd *pWnd = GetDlgItem(nIds_CB[i]);
        pWnd->GetWindowRect(&rect);
        ScreenToClient(&rect);
        pWnd->MoveWindow(rect.left,
                         rect.top - offset,
                         rect.Width(),
                         rect.Height());
    }

    // Group - File Data
    if (bShow) {
        if (m_TabData.GetItemCount() != NUM_VIEW) {
            CString strTab;
            GetDlgItemText(m_CtrlID[2].btn, strTab);
            m_TabData.InsertItem(2, strTab);
        }
    } else {
        if (m_TabData.GetItemCount() == NUM_VIEW) {
            m_TabData.DeleteItem(2);
        }

        pViewer[2]->ShowWindow(SW_HIDE);
    }

    UINT32 nIds_FD[] = {
        IDC_GROUP_FILE_DATA,
        IDC_TAB_DATA,
    };
    GetDlgItem(IDC_GROUP_CONFIG)->GetWindowRect(&rect1);
    ScreenToClient(&rect1);
    GetDlgItem(IDC_GROUP_FILE_DATA)->GetWindowRect(&rect2);
    ScreenToClient(&rect2);
    offset = rect2.top - rect1.top - rect1.Height() - 5;

    for (int i = 0; i < 2; i++) {
        CRect rect;
        CWnd *pWnd = GetDlgItem(nIds_FD[i]);
        pWnd->GetWindowRect(&rect);
        ScreenToClient(&rect);
        pWnd->MoveWindow(rect.left,
                         rect.top - offset,
                         rect.Width(),
                         rect.Height());
    }

    pViewer[2]->ShowWindow(bShow);
    // Group - Programming Options
    ShowDlgItem(IDC_CHECK_SPI, bShow);
    ShowDlgItem(IDC_CHECK_ERASE_SPI, bShow);
    UINT32 nIds_PO[] = {
        IDC_GROUP_PROGRAM,
        IDC_CHECK_APROM,
        IDC_CHECK_NVM,
        IDC_CHECK_SPI,
        IDC_CHECK_CONFIG,
        IDC_CHECK_RUN_APROM,
        IDC_CHECK_ERASE,
        IDC_CHECK_ERASE_SPI,
        IDC_BUTTON_START,
    };
    GetDlgItem(IDC_GROUP_FILE_DATA)->GetWindowRect(&rect1);
    ScreenToClient(&rect1);
    GetDlgItem(IDC_GROUP_PROGRAM)->GetWindowRect(&rect2);
    ScreenToClient(&rect2);
    offset = rect2.top - rect1.top - rect1.Height() - 5;

    for (int i = 0; i < 9; i++) {
        CRect rect;
        CWnd *pWnd = GetDlgItem(nIds_PO[i]);
        pWnd->GetWindowRect(&rect);
        ScreenToClient(&rect);
        pWnd->MoveWindow(rect.left,
                         rect.top - offset,
                         rect.Width(),
                         rect.Height());
    }

    if (bShow) {
        SetDlgItemText(IDC_CHECK_ERASE, _T("Erase All (Exclude SPI)"));
        GetDlgItem(IDC_CHECK_SPI)->GetWindowRect(&rect2);
    } else {
        SetDlgItemText(IDC_CHECK_ERASE, _T("Erase All"));
        GetDlgItem(IDC_CHECK_APROM)->GetWindowRect(&rect2);
    }

    ScreenToClient(&rect2);
    pWnd = GetDlgItem(IDC_GROUP_PROGRAM);
    pWnd->GetWindowRect(&rect1);
    ScreenToClient(&rect1);
    rect1.bottom = rect2.bottom + 10;
    pWnd->MoveWindow(rect1.left,
                     rect1.top,
                     rect1.Width(),
                     rect1.Height());
    // Progress Bar and Status
    UINT32 nIds_PB[] = {
        IDC_PROGRESS,
        IDC_STATIC_STATUS,
    };

    for (int i = 0; i < 2; i++) {
        CRect rect;
        CWnd *pWnd = GetDlgItem(nIds_PB[i]);
        pWnd->GetWindowRect(&rect);
        ScreenToClient(&rect);
        pWnd->MoveWindow(rect.left,
                         rect1.bottom + 5,
                         rect.Width(),
                         rect.Height());
    }

    // Main Window
    GetWindowRect(&m_rect);
    GetDlgItem(IDC_PROGRESS)->GetWindowRect(&rect2);
    m_rect.bottom = rect2.bottom + 10;
    MoveWindow(m_rect.left,
               m_rect.top,
               m_rect.Width(),
               m_rect.Height());
}
