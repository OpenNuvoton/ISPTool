#include "stdafx.h"
#include "CDialogOfflineExport.h"
#include <vector>

IMPLEMENT_DYNAMIC(CDialogOfflineExport, CDialog)

CDialogOfflineExport::CDialogOfflineExport(CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogOfflineExport::IDD, pParent)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_APWPROT)
    m_nRadioStorage = -1;
    m_LimitCheck = FALSE;

    m_Interfaces.clear();
    m_Interfaces.push_back(_T("UART"));
    m_Interfaces.push_back(_T("CAN"));
    m_Interfaces.push_back(_T("SPI"));
    m_Interfaces.push_back(_T("I2C"));
    m_Interfaces.push_back(_T("RS485"));
    m_Interfaces.push_back(_T("USBH"));

    //}}AFX_DATA_INIT
}

CDialogOfflineExport::~CDialogOfflineExport()
{
}

void CDialogOfflineExport::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_APWPROT)
    DDX_Radio(pDX, IDC_RADIO1, m_nRadioStorage);
    DDX_Check(pDX, IDC_CHECK_LIMIT, m_LimitCheck);
    DDX_Control(pDX, IDC_EDIT_FILENAME, m_EditName);
    DDX_Control(pDX, IDC_EDIT_LIMIT, m_LimitCount);
    DDX_Control(pDX, IDC_COMBO_INTERFACE, m_ComboInterface);
    //}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDialogOfflineExport, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_APWPROT)
    ON_BN_CLICKED(IDC_RADIO1, OnRadioSelect)
    ON_BN_CLICKED(IDC_RADIO2, OnRadioSelect)
    ON_BN_CLICKED(IDC_RADIO3, OnRadioSelect)
    ON_BN_CLICKED(IDC_CHECK_LIMIT, OnCheckLimit)
    ON_EN_CHANGE(IDC_EDIT_FILENAME, OnFileNameChange)
    ON_EN_CHANGE(IDC_EDIT_LIMIT, OnLimitCountChange)
    ON_CBN_SELCHANGE(IDC_COMBO_INTERFACE, OnSelchangeInterface)
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

BOOL CDialogOfflineExport::OnInitDialog() {
    CDialog::OnInitDialog();
    GetWindowRect(m_rect);
    AdjustDPI();

    for (size_t i = 0; i < m_Interfaces.size(); i++)
    {
        m_ComboInterface.AddString(m_Interfaces[i]);
    }

    m_ComboInterface.SetCurSel(0);
    OnSelchangeInterface();
    OnCheckLimit();
    return TRUE;
}

void CDialogOfflineExport::OnRadioSelect() {
    if (IsDlgButtonChecked(IDC_RADIO1)){
        m_nRadioStorage = 0;
    }
    if (IsDlgButtonChecked(IDC_RADIO2)) {
        m_nRadioStorage = 1;
    }
    if (IsDlgButtonChecked(IDC_RADIO3)) {
        m_nRadioStorage = 2;
    }
    printf("%d", m_nRadioStorage);
}

void CDialogOfflineExport::OnCheckLimit() {
    if (IsDlgButtonChecked(IDC_CHECK_LIMIT)) {
        GetDlgItem(IDC_EDIT_LIMIT)->EnableWindow(TRUE);
    }
    else {
        GetDlgItem(IDC_EDIT_LIMIT)->EnableWindow(FALSE);
    }
}

void CDialogOfflineExport::OnFileNameChange() {

}

void CDialogOfflineExport::OnLimitCountChange() {

}

void CDialogOfflineExport::OnSelchangeInterface() {

}

void CDialogOfflineExport::OnOK() {
    CString sPath = _T("");

    // Backup current directory
    TCHAR szCurDir[MAX_PATH];
    if (GetCurrentDirectory(sizeof(szCurDir) / sizeof(szCurDir[0]), szCurDir) == 0)
        szCurDir[0] = (TCHAR)'\0';

    // Open file dialog
    CFileDialog dialog(FALSE, _T("lua"), NULL,
        OFN_EXPLORER | OFN_ENABLESIZING | OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
        _T("Lua Script Files (*.lua)|*.lua||"),
        this);

    if (dialog.DoModal() == IDOK)
    {
        sPath = dialog.GetPathName();
    }

    // Restore current directory
    if (szCurDir[0] != (TCHAR)'\0')
        SetCurrentDirectory(szCurDir);

    //do something
    CStdioFile  myFile;
    CString sText;
    if (myFile.Open(sPath, CFile::modeCreate | CFile::modeWrite | CFile::typeText))
    {
        CString str;
        sText.Format(_T("local status, err = pcall(function()\n"));
        myFile.WriteString(sText);
        CString strFileName;
        GetDlgItemText(IDC_EDIT_FILENAME, strFileName);
        sText.Format(_T("File_name=\"%d:\\\\%s\"\n"), m_nRadioStorage, (LPCTSTR)strFileName);
        myFile.WriteString(sText);
        sText.Format(_T("temp = string.format(\"strage=0x%%x\", ISP_SHOW_STORAGE())\nprint(temp)\nfilesize = ISP_AP_FILE_SIZE(File_name)\n"));
        myFile.WriteString(sText);
        sText.Format(_T("if (filesize == 0) then\nprint(\"no file exit\")\nreturn\nend\ntemp = string.format(\"APROM SIZE=%%d\", filesize)\n"));
        myFile.WriteString(sText);
        sText.Format(_T("print(temp)\nfilechecksum = ISP_AP_FILE_CHECKSUM(File_name)\ntemp = string.format(\"APROM checksum = 0x %% x\", filechecksum)\n"));
        myFile.WriteString(sText);         
        sText.Format(_T("print(temp)\nISP_INTERFACE_INIT(%d)\nprint(\"sync:\")\nif (ISP_CmdSyncPackno() == 1) then\nprint(\"sync false\")\nreturn\nend\n"), m_ComboInterface.GetCurSel());
        myFile.WriteString(sText);
        sText.Format(_T("print(\"sync pass\")\ndevid=ISP_CmdGetDeviceID()\ntemp = string.format(\"devce id=0x%%x\", devid)\nprint(temp)\n"));
        myFile.WriteString(sText);
        sText.Format(_T("tb = ISP_CmdFWVersion()\ntemp = (tb[1] << 24) | (tb[2] << 16) | (tb[3] << 8) | tb[4]\nprint(string.format(\"fw version = 0x %% x\", temp))\n"));
        myFile.WriteString(sText);
        sText.Format(_T("tb = ISP_CmdGetConfig()\nlocal config0 = (tb[1] << 24) | (tb[2] << 16) | (tb[3] << 8) | tb[4]\nprint(string.format(\"config0 = 0x %% x\", config0))\n"));
        myFile.WriteString(sText);
        sText.Format(_T("config1 = (tb[5] << 24) | (tb[6] << 16) | (tb[7] << 8) | tb[8]\nprint(string.format(\"config1 = 0x %% x\", config1))\nISP_SET_PFILE(File_name)\n"));
        myFile.WriteString(sText);
        if (IsDlgButtonChecked(IDC_CHECK_LIMIT)) {
            if (IsDlgButtonChecked(IDC_CHECK_LIMIT) && GetDlgItemText(IDC_EDIT_LIMIT, str) && !str.IsEmpty()) {
                sText.Format(_T("ISP_SET_LIMIT(%d)\n"), _tstoi(str));
                myFile.WriteString(sText);
            }
        }
        sText.Format(_T("if (ISP_PROGRAM(0, filesize) == 0) then\nprint(\"program pass\")\nend\nISP_INTERFACE_UNINIT()\nend)\n"));
        myFile.WriteString(sText);
        sText.Format(_T("if not status then\nprint(err);\nend\n"));
        myFile.WriteString(sText);
        myFile.Flush();
        myFile.Seek(0, CFile::begin);
        myFile.Close();
    }

    CDialog::OnOK(); // Close the dialog after processing
    return;
}