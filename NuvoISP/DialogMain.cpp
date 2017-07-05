// DialogScrollable.cpp : implementation file
//

#include "stdafx.h"
#include "DialogMain.h"
#include "resource.h"
#include <cstring>

// CDialogScrollable dialog

IMPLEMENT_DYNAMIC(CDialogMain, CDialog)

CDialogMain::CDialogMain(UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialog(nIDTemplate, pParent)
{
    m_bIsInitialized = false;
    m_bShowScrollBar = false;
    m_ScrollBarWidth = 15;
}

CDialogMain::~CDialogMain()
{
}

BEGIN_MESSAGE_MAP(CDialogMain, CDialog)
    //{{AFX_MSG_MAP(CDialogMain)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

void CDialogMain::DoDataExchange(CDataExchange *pDX)
{
    CDialog::DoDataExchange(pDX);
}

void CDialogMain::OnSize(UINT nType, int cx, int cy)
{
    // TODO: Add your message handler code here.
    CDialog::OnSize(nType, cx, cy);
    int nScrollMaxX;
    int nScrollMaxY;
    CRect rect;
    ScrollWindow(m_nScrollPosX, m_nScrollPosY);
    m_nScrollPosX = 0;
    m_nScrollPosY = 0;
    GetWindowRect(rect);

    if (!m_bIsInitialized) {
        nScrollMaxY = 0;
        nScrollMaxX = 0;
    } else {
        if (rect.Height() < m_rect.Height()) {
            nScrollMaxY = m_rect.Height() - rect.Height() + m_ScrollBarWidth;
        } else {
            nScrollMaxY = 0;
        }

        if (rect.Width() < m_rect.Width()) {
            nScrollMaxX = m_rect.Width() - rect.Width() + m_ScrollBarWidth;
        } else {
            nScrollMaxX = 0;
        }

        if (nScrollMaxY != 0 && nScrollMaxX == 0) {
            nScrollMaxX = 10;
        } else if (nScrollMaxY == 0 && nScrollMaxX != 0) {
            nScrollMaxY = 10;
        }
    }

    if (nScrollMaxX == 0 || nScrollMaxY == 0) {
        m_bShowScrollBar = false;
    } else {
        m_bShowScrollBar = true;
    }

    SCROLLINFO si_y;
    si_y.cbSize = sizeof(SCROLLINFO);
    si_y.fMask = SIF_ALL; // SIF_ALL = SIF_PAGE | SIF_RANGE | SIF_POS;
    si_y.nMin = 0;
    si_y.nMax = nScrollMaxY;
    si_y.nPage = si_y.nMax / 10;
    si_y.nPos = 0;
    SetScrollInfo(SB_VERT, &si_y, TRUE);
    SCROLLINFO si_x;
    si_x.cbSize = sizeof(SCROLLINFO);
    si_x.fMask = SIF_ALL; // SIF_ALL = SIF_PAGE | SIF_RANGE | SIF_POS;
    si_x.nMin = 0;
    si_x.nMax = nScrollMaxX;
    si_x.nPage = si_x.nMax / 10;
    si_x.nPos = 0;
    SetScrollInfo(SB_HORZ, &si_x, TRUE);
    // TODO: Add your message handler code here
}

void CDialogMain::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    // TODO: Add your message handler code here and/or call default
    CDialog::OnVScroll(nSBCode, nPos, pScrollBar);
    SCROLLINFO scrollinfo;
    GetScrollInfo(SB_VERT, &scrollinfo);
    int nMaxPos = scrollinfo.nMax;
    int nDelta;

    switch (nSBCode) {
        case SB_LINEDOWN:
            if (m_nScrollPosY >= nMaxPos) {
                return;
            }

            nDelta = min(nMaxPos / 100, nMaxPos - m_nScrollPosY);

            if (nMaxPos / 100 == 0) {
                nDelta = 1;
            }

            break;

        case SB_LINEUP:
            if (m_nScrollPosY <= 0) {
                return;
            }

            nDelta = -min(nMaxPos / 100, m_nScrollPosY);

            if (nMaxPos / 100 == 0) {
                nDelta = -1;
            }

            break;

        case SB_PAGEDOWN:
            if (m_nScrollPosY >= nMaxPos) {
                return;
            }

            nDelta = min(nMaxPos / 10, nMaxPos - m_nScrollPosY);
            break;

        case SB_THUMBPOSITION:
            nDelta = (int)nPos - m_nScrollPosY;
            break;

        case SB_PAGEUP:
            if (m_nScrollPosY <= 0) {
                return;
            }

            nDelta = -min(nMaxPos / 10, m_nScrollPosY);
            break;

        default:
            return;
    }

    m_nScrollPosY += nDelta;
    SetScrollPos(SB_VERT, m_nScrollPosY, TRUE);
    ScrollWindow(0, -nDelta);
    CDialog::OnVScroll(nSBCode, nPos, pScrollBar);
}

void CDialogMain::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    // TODO: Add your message handler code here and/or call default
    CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
    SCROLLINFO scrollinfo;
    GetScrollInfo(SB_HORZ, &scrollinfo);
    int nMaxPos = scrollinfo.nMax;
    int nDelta;

    switch (nSBCode) {
        case SB_LINERIGHT:
            if (m_nScrollPosX >= nMaxPos) {
                return;
            }

            nDelta = min(nMaxPos / 100, nMaxPos - m_nScrollPosX);

            if (nMaxPos / 100 == 0) {
                nDelta = 1;
            }

            break;

        case SB_LINELEFT:
            if (m_nScrollPosX <= 0) {
                return;
            }

            nDelta = -min(nMaxPos / 100, m_nScrollPosX);

            if (nMaxPos / 100 == 0) {
                nDelta = -1;
            }

            break;

        case SB_PAGERIGHT:
            if (m_nScrollPosX >= nMaxPos) {
                return;
            }

            nDelta = min(nMaxPos / 10, nMaxPos - m_nScrollPosX);
            break;

        case SB_THUMBPOSITION:
            nDelta = (int)nPos - m_nScrollPosX;
            break;

        case SB_PAGELEFT:
            if (m_nScrollPosX <= 0) {
                return;
            }

            nDelta = -min(nMaxPos / 10, m_nScrollPosX);
            break;

        default:
            return;
    }

    m_nScrollPosX += nDelta;
    SetScrollPos(SB_HORZ, m_nScrollPosX, TRUE);
    ScrollWindow(-nDelta, 0);
    CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CDialogMain::OnGetMinMaxInfo(MINMAXINFO *lpMMI)
{
    // TODO: Add your message handler code here and/or call default
    CDialog::OnGetMinMaxInfo(lpMMI);

    if (m_bIsInitialized) {
        lpMMI->ptMaxTrackSize.x = m_rect.Width();
        lpMMI->ptMaxTrackSize.y = m_rect.Height();
        lpMMI->ptMinTrackSize.x = 310;
        lpMMI->ptMinTrackSize.y = 310;
    }
}
BOOL CDialogMain::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
    // TODO: Add your message handler code here and/or call default
    CDialog::OnMouseWheel(nFlags, zDelta, pt);

    if (m_bShowScrollBar) {
        SCROLLINFO scrollinfo;
        GetScrollInfo(SB_VERT, &scrollinfo);
        int nMaxPos = scrollinfo.nMax;
        int nScrollPos = scrollinfo.nPos;
        int nDelta;

        if (zDelta < 0) {
            if (m_nScrollPosY >= nMaxPos) {
                return 0;
            }

            nDelta = min(nMaxPos / 10, nMaxPos - m_nScrollPosY);
        } else {
            if (m_nScrollPosY <= 0) {
                return 0;
            }

            nDelta = -min(nMaxPos / 10, m_nScrollPosY);
        }

        m_nScrollPosY += nDelta;
        SetScrollPos(SB_VERT, m_nScrollPosY, TRUE);
        ScrollWindow(0, -nDelta);
    }

    return CDialog::OnMouseWheel(nFlags, zDelta, pt);
}

void CDialogMain::ShowDlgItem(int nID, int nCmdShow)
{
    CWnd *pWnd = GetDlgItem(nID);

    if (pWnd != NULL) {
        pWnd->ShowWindow(nCmdShow);
    }
}

void CDialogMain::EnableDlgItem(int nID, BOOL bEnable)
{
    CWnd *pWnd = GetDlgItem(nID);

    if (pWnd != NULL) {
        pWnd->EnableWindow(bEnable);
    }
}

#ifndef _NO_CONFIGURATION_DLG

#include "PartNumID.h"
#include "FlashInfo.h"

#include "DialogConfiguration_M058.h"
#include "DialogConfiguration_M05x.h"
#include "DialogConfiguration_M05xCN.h"
#include "DialogConfiguration_M451.h"
#include "DialogConfiguration_Mini51.h"
#include "DialogConfiguration_Mini51BN.h"
#include "DialogConfiguration_Mini51CN.h"
#include "DialogConfiguration_MT500.h"
#include "DialogConfiguration_Nano100.h"
#include "DialogConfiguration_Nano100BN.h"
#include "DialogConfiguration_Nano103.h"
#include "DialogConfiguration_Nano112.h"
#include "DialogConfiguration_NM1120.h"
#include "DialogConfiguration_NM1200.h"
#include "DialogConfiguration_NUC102.h"
#include "DialogConfiguration_NUC103.h"
#include "DialogConfiguration_NUC103BN.h"
#include "DialogConfiguration_NUC131.h"
#include "DialogConfiguration_NUC1xx.h"
#include "DialogConfiguration_NUC2xx.h"
#include "DialogConfiguration_NUC4xx.h"
#include "DialogConfiguration_M0564.h"

#include "DialogConfiguration_N76E1T.h"

#include "DialogConfiguration_TC8226.h"

extern CPartNumID *psChipData;

bool CDialogMain::ConfigDlgSel(unsigned int *pConfig, unsigned int size)
{
    bool ret = false;
    CDialog *pConfigDlg = NULL;
    unsigned int *Config;
    BOOL bIsDataFlashFixed = FALSE;
    unsigned int uProgramMemorySize = 0;
    unsigned int uDataFlashSize = 0;

    if (psChipData) {
        if (gsPidInfo.uPID == psChipData->uID) {
            bIsDataFlashFixed = gsPidInfo.uDataFlashSize;
            uProgramMemorySize = gsPidInfo.uProgramMemorySize;
            uDataFlashSize = gsPidInfo.uDataFlashSize;
        }

        switch (psChipData->uProjectCode) {
            case IDD_DIALOG_CONFIGURATION_NUC100:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_NUC1xx(bIsDataFlashFixed, uProgramMemorySize, uDataFlashSize);    // "NUC100BN";
                } else {
                    pConfigDlg = new CDialogConfiguration_NUC1xx;    // "NUC100BN";
                }

                Config = (((CDialogConfiguration_NUC1xx *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NUC102:
                pConfigDlg = new CDialogConfiguration_NUC102;
                Config = (((CDialogConfiguration_NUC102 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NUC103:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_NUC103(uProgramMemorySize, uDataFlashSize);
                } else {
                    pConfigDlg = new CDialogConfiguration_NUC103;
                }

                Config = (((CDialogConfiguration_NUC103 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NUC200:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_NUC2xx(bIsDataFlashFixed, uProgramMemorySize, uDataFlashSize);
                } else {
                    pConfigDlg = new CDialogConfiguration_NUC2xx;
                }

                Config = (((CDialogConfiguration_NUC2xx *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NUC131:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_NUC131(uProgramMemorySize, uDataFlashSize);
                } else {
                    pConfigDlg = new CDialogConfiguration_NUC131;
                }

                Config = (((CDialogConfiguration_NUC131 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NANO100:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Nano100(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Nano100;
                }

                Config = (((CDialogConfiguration_Nano100 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NANO100BN:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Nano100BN(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Nano100BN;
                }

                Config = (((CDialogConfiguration_Nano100BN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NANO112:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Nano112(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Nano112;
                }

                Config = (((CDialogConfiguration_Nano112 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NANO103:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Nano103(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Nano103;
                }

                Config = (((CDialogConfiguration_Nano103 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M051:
                pConfigDlg = new CDialogConfiguration_M05x;
                Config = (((CDialogConfiguration_M05x *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M051CN:
                pConfigDlg = new CDialogConfiguration_M05xCN;
                Config = (((CDialogConfiguration_M05xCN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M058:
                pConfigDlg = new CDialogConfiguration_M058;
                Config = (((CDialogConfiguration_M058 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_MINI51:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Mini51(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Mini51;
                }

                Config = (((CDialogConfiguration_Mini51 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_MINI51BN:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Mini51BN(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Mini51BN;
                }

                Config = (((CDialogConfiguration_Mini51BN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_MINI51CN:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Mini51CN(uProgramMemorySize, psChipData->uID);
                } else if ((psChipData->uID & 0xFFFFFF00) == 0x00A05800) {
                    pConfigDlg = new CDialogConfiguration_Mini51CN(32 * 1024, psChipData->uID);
                } else {
                    pConfigDlg = new CDialogConfiguration_Mini51CN;
                }

                Config = (((CDialogConfiguration_Mini51CN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NM1200:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_NM1200(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_NM1200;
                }

                Config = (((CDialogConfiguration_NM1200 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_MT500:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_MT500(bIsDataFlashFixed, uProgramMemorySize, uDataFlashSize);
                } else {
                    pConfigDlg = new CDialogConfiguration_MT500;
                }

                Config = (((CDialogConfiguration_MT500 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NUC400:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_NUC4xx(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_NUC4xx;
                }

                Config = (((CDialogConfiguration_NUC4xx *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M451:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_M451(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_M451;
                }

                Config = (((CDialogConfiguration_M451 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NM1120:
                pConfigDlg = new CDialogConfiguration_NM1120;
                Config = (((CDialogConfiguration_NM1120 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_N76E1T:
                pConfigDlg = new CDialogConfiguration_N76E1T(psChipData->uID);
                Config = (((CDialogConfiguration_N76E1T *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M0564:	// M0564, NUC121, NUC125, NUC126
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_M0564(psChipData->uID, uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_M0564();
                }

                Config = (((CDialogConfiguration_M0564 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_TC8226:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_TC8226(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_TC8226();
                }

                Config = (((CDialogConfiguration_TC8226 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case 0x505:	// "NUC505";
                printf("NUC505 ");

            default:
                printf("or Unknow Configuration Dialog %X\n", psChipData->uProjectCode);
                return false;
        }
    }

    // Pass User Config to Configuration Dialog
    memcpy(Config, pConfig, size);

    if (pConfigDlg != NULL) {
        if (pConfigDlg->DoModal() == IDOK) {
            // Update User Config from Configuration Dialog
            memcpy(pConfig, Config, size);
            ret = true;
        }

        delete pConfigDlg;
    }

    return ret;
}

/* called by DlgNuvoISP */
bool CDialogMain::ConfigSetting(unsigned int id, unsigned int *pConfig, unsigned int size)
{
    if (QueryDataBase(id)) {
        return ConfigDlgSel(pConfig, size);
    }

    return false;
}

#endif // #ifndef _NO_CONFIGURATION_DLG

BOOL CDialogMain::OnInitDialog()
{
    CDialog::OnInitDialog();
    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
};


UINT CDialogMain::ScanPCCom()
{
    m_SelComPort.ResetContent();
    m_SelComPort.AddString(_T("Scan Port"));
    UINT nComNum = 0;
    HKEY hKEY;
    LONG hResult = ::RegOpenKeyEx(HKEY_LOCAL_MACHINE, _T("HARDWARE\\DEVICEMAP\\SERIALCOMM"), 0, KEY_READ, &hKEY);

    if (hResult != ERROR_SUCCESS) { //如果無法打開hKEY,則中止程式的執行
        //AfxMessageBox("錯誤：無法打開有關註冊表項");
        return  0 ;
    }

    TCHAR strInf[30];
    DWORD type_1 = REG_SZ;
    DWORD cbData_1 = 10;
    DWORD aa = 30, num = 0, a1, a2, a3, a4, a5, a6, a7;
    hResult = ::RegQueryInfoKey(hKEY, strInf, &a7, NULL, &a3, &a1, &a2, &num, &a4, &a5, &a6, NULL);

    if (hResult != ERROR_SUCCESS) { //如果無法打開hKEY,則中止程式的執行
        //AfxMessageBox("錯誤：無法打開有關註冊表項");
        RegCloseKey(hKEY);
        return   0;
    }

    BYTE portName[30];
    CString csr;

    for (DWORD i = 0 ; i < num ; i++) {
        aa = 30 ;
        cbData_1 = 30;
        hResult = ::RegEnumValue(hKEY, i, strInf, &aa, NULL, &type_1, portName, &cbData_1);

        if ((hResult != ERROR_SUCCESS) && (hResult != ERROR_MORE_DATA)) { //如果無法打開hKEY,則中止程式的執行
            //AfxMessageBox("錯誤：無法獲取有關註冊表項");
            continue;
        }

        csr.Format(_T("%s"), portName);
        m_SelComPort.AddString(csr);
        nComNum++;
    }

    RegCloseKey(hKEY);
    m_SelComPort.SetCurSel(0);
    return nComNum;
}

void CDialogMain::InitComboBox()
{
    m_SelInterface.AddString(_T("USB"));
    m_SelInterface.AddString(_T("UART"));
    m_SelInterface.SetCurSel(0);
    OnSelchangeInterface();

    if (ScanPCCom()) {
        m_SelComPort.SetCurSel(0);
    }
}

void CDialogMain::OnSelchangeInterface()
{
    m_SelComPort.EnableWindow(m_SelInterface.GetCurSel() == 1);

    if (m_SelInterface.GetCurSel() == 0) {
        EnableDlgItem(IDC_BUTTON_CONNECT, true);
    } else {
        OnComboChange();
    }
}

void CDialogMain::OnComboChange()
{
    if (m_SelComPort.GetCurSel() == 0) {
        int portcnt = ScanPCCom();
        printf("Num Port = %d\n", portcnt);
    }

    EnableDlgItem(IDC_BUTTON_CONNECT, m_SelComPort.GetCurSel());
}

void CDialogMain::EnableInterface(bool bEnable)
{
    if (bEnable) {
        m_SelInterface.EnableWindow(1);
        OnSelchangeInterface();
    } else {
        m_SelInterface.EnableWindow(0);
        m_SelComPort.EnableWindow(0);
    }
}

UINT DialogTemplate[] = {
    IDD_DIALOG_CONFIGURATION_M051,
    IDD_DIALOG_CONFIGURATION_M051CN,
    IDD_DIALOG_CONFIGURATION_M0564,	// M0564, NUC121, NUC125, NUC126
    IDD_DIALOG_CONFIGURATION_M058,
    IDD_DIALOG_CONFIGURATION_M451,
    IDD_DIALOG_CONFIGURATION_MINI51,
    IDD_DIALOG_CONFIGURATION_MINI51BN,
    IDD_DIALOG_CONFIGURATION_MINI51CN,
    IDD_DIALOG_CONFIGURATION_MT500,
    IDD_DIALOG_CONFIGURATION_N76E1T,
    IDD_DIALOG_CONFIGURATION_NANO100,
    IDD_DIALOG_CONFIGURATION_NANO100BN,
    IDD_DIALOG_CONFIGURATION_NANO103,
    IDD_DIALOG_CONFIGURATION_NANO112,
    IDD_DIALOG_CONFIGURATION_NM1120,
    IDD_DIALOG_CONFIGURATION_NM1200,
    IDD_DIALOG_CONFIGURATION_NUC100,
    IDD_DIALOG_CONFIGURATION_NUC102,
    IDD_DIALOG_CONFIGURATION_NUC103,
    IDD_DIALOG_CONFIGURATION_NUC131,
    IDD_DIALOG_CONFIGURATION_NUC200,
    IDD_DIALOG_CONFIGURATION_NUC400,
    IDD_DIALOG_CONFIGURATION_TC8226,
};

bool CDialogMain::DemoConfigDlg(UINT Template /* = 0 */)
{
    CPartNumID *psBackup = 	psChipData;
    struct CPartNumID  Demo = {"NuMicro", 0x12345678, Template};
    psChipData = &Demo;
    unsigned int CFG[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };

    if (Template == 0) {
        for (int i = 0; i < _countof(DialogTemplate); ++i) {
            Demo.uProjectCode = DialogTemplate[i];
            ConfigDlgSel(CFG, sizeof(CFG));
        }
    } else {
        ConfigDlgSel(CFG, sizeof(CFG));
    }

    psChipData = psBackup;
    return true;
}
