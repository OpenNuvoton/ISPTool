#include "stdafx.h"
#include "Resource.h"
#include "DlgNuvoISP.h"
#include "About.h"

#include "DialogConfiguration_Nano103.h"
#include "DialogConfiguration_Mini51CN.h"

#include "PartNumID.h"
#include "FlashInfo.h"

#include <dbt.h>

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
        {IDC_BUTTON_NVM, IDC_EDIT_FILEPATH_NVM, IDC_STATIC_FILEINFO_NVM}
    };
    memcpy(&m_CtrlID, buddy, sizeof(m_CtrlID));
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
    SetWindowText(_T("Nuvoton NuMicro ISP Programming Tool 2.04"));

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
    InitUILayout();
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
    // TODO: Add your control notification handler code here
    if (m_fnThreadProcStatus == &CISPProc::Thread_Idle
            || m_fnThreadProcStatus == &CISPProc::Thread_Pause) {
        /* Connect */
        m_SelComPort.GetWindowText(m_ComNum);
        SetInterface(m_SelInterface.GetCurSel() + 1, m_ComNum);
        EnableInterface(false);
        Set_ThreadAction(&CISPProc::Thread_CheckUSBConnect);
    } else if (m_fnThreadProcStatus != NULL) {
        /* Disconnect */
        m_ISPLdDev.Close_Port();
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
    // TODO: Add your specialized code here and/or call the base class
    if (message == MSG_USER_EVENT) {
        if (wParam == MSG_UPDATE_CONNECT_STATUS) {
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

                    InitUILayout();
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
                    ShowChipInfo();

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

                        case EPS_ERR_SIZE:
                            AfxMessageBox(_T("File Size > Flash Size"));
                            break;

                        case EPS_PROG_DONE:
                            MessageBox(_T("Programming flash, OK!"));
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

    /* Check program operation */
    if (!(m_bProgram_APROM || m_bProgram_NVM || m_bProgram_Config || m_bErase || m_bRunAPROM)) {
        MessageBox(_T("You did not select any operation."), NULL, MB_ICONSTOP);
        return;
    }

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

    if (m_fnThreadProcStatus == &CISPProc::Thread_Idle
            || m_fnThreadProcStatus == &CISPProc::Thread_Pause) {
        DemoConfigDlg();
        return;
    }

#endif

    if (ConfigSetting(m_ulDeviceID, m_CONFIG_User, sizeof(m_CONFIG_User))) {
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

void CNuvoISPDlg::ShowChipInfo()
{
    if (0x00550505 == m_ulDeviceID) {
        SetDlgItemText(IDC_EDIT_PARTNO, _T("NUC505"));
        SetDlgItemText(IDC_STATIC_CONFIG_VALUE_0, _T("NA"));
        SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, _T("NA"));
        m_bProgram_Config = 0;
        m_bErase = 0;
        ShowDlgItem(IDC_CHECK_CONFIG, 0);
        ShowDlgItem(IDC_CHECK_ERASE, 0);
        EnableDlgItem(IDC_BUTTON_CONFIG, 0);
        ShowDlgItem(IDC_STATIC_APOFFSET, 1);
        ShowDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS, 1);
        ShowDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS, 1);
        std::ostringstream os;
        os << "RAM:128K, SPI Flash:2M";
        std::string cstr = os.str();
        std::wstring wcstr(cstr.begin(), cstr.end());
        CString str = wcstr.c_str();
        CString info;
        info.Format(_T("%s\nFW Ver: 0x%X"), wcstr.c_str(), int(m_ucFW_VER));
        SetDlgItemText(IDC_STATIC_PARTNO, info);
        UpdateAddrOffset();
        return;
    } else if ((0x00002150 == m_ulDeviceID)
               || (0x00002F50 == m_ulDeviceID)
               || (0x00003650 == m_ulDeviceID)) {
        // N76E885, N76E616 and N76E003
        ShowDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS, 1);
        ShowDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS, 1);
        UpdateAddrOffset();
    } else if ((m_ulDeviceID & 0xFFFFF000) == 0x00D48000) {
        SetDlgItemText(IDC_STATIC_CONFIG_0, _T("Config 0-3:"));
        ShowDlgItem(IDC_STATIC_CONFIG_VALUE_2, 1);
        ShowDlgItem(IDC_STATIC_CONFIG_VALUE_3, 1);
    }

    CString strTmp = _T("");
    strTmp = GetPartNumber(m_ulDeviceID).c_str();
    SetDlgItemText(IDC_EDIT_PARTNO, strTmp);
    strTmp.Format(_T("0x%08X"), m_CONFIG_User[0]);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_0, strTmp);
    strTmp.Format(_T("0x%08X"), m_CONFIG_User[1]);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, strTmp);
    strTmp.Format(_T("0x%08X"), m_CONFIG_User[2]);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_2, strTmp);
    strTmp.Format(_T("0x%08X"), m_CONFIG_User[3]);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_3, strTmp);

    if (UpdateSizeInfo(m_ulDeviceID, m_CONFIG[0], m_CONFIG[1],
                       &m_uNVM_Addr, &m_uAPROM_Size, &m_uNVM_Size)) {
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
    } else if ((0x00002150 == m_ulDeviceID)
               || (0x00002F50 == m_ulDeviceID)
               || (0x00003650 == m_ulDeviceID)) {
        if (uAddr >= 0x4800) {
            uAddr = 0x3800;
        }

        m_uNVM_Addr = uAddr;
        m_uNVM_Size = 0x4800 - m_uNVM_Addr;
    }

    strAddr.Format(_T("%06X"), uAddr);
    SetDlgItemText(IDC_EDIT_FLASH_BASE_ADDRESS, strAddr);
}

void CNuvoISPDlg::RegisterNotification()
{
    DEV_BROADCAST_DEVICEINTERFACE devIF = {0};
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid);
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
                    m_ISPLdDev.Close_Port();
                    Set_ThreadAction(&CISPProc::Thread_Idle);
                }
            }

            break;

        default:
            break;
    }

    return TRUE;
}

void CNuvoISPDlg::InitUILayout()
{
    m_ButtonConnect.SetWindowText(_T("Connect"));
    SetDlgItemText(IDC_EDIT_PARTNO, _T(""));
    SetDlgItemText(IDC_STATIC_PARTNO, _T(""));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_0, _T(""));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, _T(""));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_2, _T(""));
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_3, _T(""));
    ShowDlgItem(IDC_CHECK_CONFIG, 1);
    ShowDlgItem(IDC_CHECK_ERASE, 1);
    EnableDlgItem(IDC_BUTTON_CONFIG, 1);
    ShowDlgItem(IDC_STATIC_APOFFSET, 0);
    ShowDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS, 0);
    ShowDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS, 0);
    SetDlgItemText(IDC_STATIC_CONFIG_0, _T("Config 0,1:"));
    ShowDlgItem(IDC_STATIC_CONFIG_VALUE_2, 0);
    ShowDlgItem(IDC_STATIC_CONFIG_VALUE_3, 0);
    EnableProgramOption(TRUE);
}
