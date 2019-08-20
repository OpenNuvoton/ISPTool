// DialogScrollable.cpp : implementation file
//

#include "stdafx.h"
#include "DialogMain.h"
#include "resource.h"
#include <cstring>

#include "NuDataBase.h"

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
#include "ChipDefs.h"
#include "DialogConfiguration_M05x.h"
#include "DialogConfiguration_M451.h"
#include "DialogConfiguration_Mini51.h"
#include "DialogConfiguration_Mini51CN.h"
#include "DialogConfiguration_MT500.h"
#include "DialogConfiguration_Nano100.h"
#include "DialogConfiguration_Nano103.h"
#include "DialogConfiguration_Nano112.h"
#include "DialogConfiguration_NM1120.h"
#include "DialogConfiguration_NM1200.h"
#include "DialogConfiguration_NUC102.h"
#include "DialogConfiguration_NUC103.h"
#include "DialogConfiguration_NUC1xx.h"
#include "DialogConfiguration_NUC2xx.h"
#include "DialogConfiguration_NUC4xx.h"
#include "DialogConfiguration_M0564.h"
#include "DialogConfiguration_OT8051.h"
#include "DialogConfiguration_M480.h"
#include "DialogConfiguration_M2351.h"
#include "DialogConfiguration_I94000.h"
#include "DialogConfiguration_AU9100.h"
#include "DialogConfiguration_N570.h"
#include "DialogConfiguration_M031.h"
#include "DialogConfiguration_M251.h"

bool CDialogMain::ConfigDlgSel(unsigned int *pConfig, unsigned int size, unsigned int uSeriesCode /* = 0*/)
{
    bool ret = false;
    CDialog *pConfigDlg = NULL;
    unsigned int *Config;
    BOOL bIsDataFlashFixed = FALSE;
    unsigned int uProgramMemorySize = 0;
    unsigned int uDataFlashSize = 0;
    unsigned int uID = 0;

    if (uSeriesCode == 0) {
        uSeriesCode = gsChipCfgInfo.uSeriesCode;
        bIsDataFlashFixed = gsChipCfgInfo.uDataFlashSize;
        uProgramMemorySize = gsChipCfgInfo.uProgramMemorySize;
        uDataFlashSize = gsChipCfgInfo.uDataFlashSize;
        uID = gsChipCfgInfo.uID;
    }

    if (1) {
        switch (uSeriesCode) {
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
                    pConfigDlg = new CDialogConfiguration_NUC123AN(uProgramMemorySize, uDataFlashSize);
                } else {
                    pConfigDlg = new CDialogConfiguration_NUC123AN;
                }

                Config = (((CDialogConfiguration_NUC123AN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NUC103BN:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_NUC123AE(uProgramMemorySize, uDataFlashSize);
                } else {
                    pConfigDlg = new CDialogConfiguration_NUC123AE;
                }

                Config = (((CDialogConfiguration_NUC123AE *)pConfigDlg)->m_ConfigValue.m_value);
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
                    pConfigDlg = new CDialogConfiguration_Nano100AN(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Nano100AN;
                }

                Config = (((CDialogConfiguration_Nano100AN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NANO100BN:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Nano100(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Nano100;
                }

                Config = (((CDialogConfiguration_Nano100 *)pConfigDlg)->m_ConfigValue.m_value);
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

            // M051AN, M051BN
            case IDD_DIALOG_CONFIGURATION_M051:
                pConfigDlg = new CDialogConfiguration_M05XAN();
                Config = (((CDialogConfiguration_M05XAN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M051BN:
                pConfigDlg = new CDialogConfiguration_M05XBN();
                Config = (((CDialogConfiguration_M05XBN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            // M051DN, M051DE, M058SAN
            case IDD_DIALOG_CONFIGURATION_M051CN:
                pConfigDlg = new CDialogConfiguration_M05XDN;
                Config = (((CDialogConfiguration_M05XDN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M058:
                pConfigDlg = new CDialogConfiguration_M058SAN;
                Config = (((CDialogConfiguration_M058SAN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_MINI51:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Mini51(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_Mini51;
                }

                Config = (((CDialogConfiguration_Mini51 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_MINI51CN:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_Mini51CN(uProgramMemorySize, uID);
                } else if ((uID & 0xFFFFFF00) == 0x00A05800) {
                    pConfigDlg = new CDialogConfiguration_Mini51CN(32 * 1024, uID);
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

            case 0x00E45200:
                pConfigDlg = new CDialogConfiguration_M4521;
                Config = (((CDialogConfiguration_M4521 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_NM1120:
                pConfigDlg = new CDialogConfiguration_NM1120;
                Config = (((CDialogConfiguration_NM1120 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            // IDD_DIALOG_CONFIGURATION_N76E1T offline mode
            case 0x00002140:
            case 0x00002F50:
            case 0x00003650:
            case 0x00003E61:
            case 0x06004721:
                uID = uSeriesCode;

            case IDD_DIALOG_CONFIGURATION_N76E1T: // IDD_DIALOG_CONFIGURATION_OT8051
                pConfigDlg = new CDialogConfiguration_OT8051(uID);
                Config = (((CDialogConfiguration_OT8051 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M0564:	// M0564, NUC126
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_M0564(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_M0564();
                }

                Config = (((CDialogConfiguration_M0564 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case 0x00012100:	// NUC121, NUC125
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_NUC121(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_NUC121();
                }

                Config = (((CDialogConfiguration_NUC121 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_TC8226: // IDD_DIALOG_CONFIGURATION_M480
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_M480(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_M480();
                }

                Config = (((CDialogConfiguration_M480 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M480LD:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_CFG_M480LD(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogChipSetting_CFG_M480LD();
                }

                Config = (((CDialogChipSetting_CFG_M480LD *)pConfigDlg)->m_uConfigValue);
                break;

            case IDD_DIALOG_CONFIGURATION_M2351:
                pConfigDlg = new CDialogConfiguration_M2351;
                Config = (((CDialogConfiguration_M2351 *)pConfigDlg)->m_uConfigValue);
                break;

            case ISD_94000_SERIES:
            case IDD_DIALOG_CONFIGURATION_I94000:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_I94000(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_I94000();
                }

                Config = (((CDialogConfiguration_I94000 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case ISD_91200_SERIES:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_I9200(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_I9200();
                }

                Config = (((CDialogConfiguration_I9200 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case ISD_9160_SERIES:
            case ISD_91300_SERIES:
            case NUVOICE_N575_SERIES:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_AU9100(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_AU9100();
                }

                Config = (((CDialogConfiguration_AU9100 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_AU9100:
                pConfigDlg = new CDialogConfiguration_AU9100();
                Config = (((CDialogConfiguration_AU9100 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case ISD_91000_SERIES:
            case IDD_DIALOG_CONFIGURATION_N570:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_N570(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_N570();
                }

                Config = (((CDialogConfiguration_N570 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M031:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_M031(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_M031();
                }

                Config = (((CDialogConfiguration_M031 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M251:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_M251(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogConfiguration_M251();
                }

                Config = (((CDialogConfiguration_M251 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            default:
                printf("or Unknow Configuration Dialog %X\n", uSeriesCode);
                return false;
        }
    }

    if (pConfigDlg != NULL) {
        // Pass User Config to Configuration Dialog
        memcpy(Config, pConfig, size);

        if (pConfigDlg->DoModal() == IDOK) {
            // Update User Config from Configuration Dialog
            memcpy(pConfig, Config, size);
            ret = true;
        }

        delete pConfigDlg;
    }

    return ret;
}

/* called by void CNuvoISPDlg::OnButtonConfig() */
bool CDialogMain::ConfigSetting(unsigned int id, unsigned int *pConfig, unsigned int size)
{
    if (GetChipConfigInfo(id)) {
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
    // Nu-Link2 ISP Bridge interfaces
    m_SelInterface.AddString(_T("HID to SPI"));
    m_SelInterface.AddString(_T("HID to I2C"));
    m_SelInterface.AddString(_T("HID to RS485"));
    m_SelInterface.AddString(_T("HID to CAN"));
    m_SelInterface.SetCurSel(0);
    OnSelchangeInterface();

    if (ScanPCCom()) {
        m_SelComPort.SetCurSel(0);
    }
}

void CDialogMain::OnSelchangeInterface()
{
    m_SelComPort.EnableWindow(m_SelInterface.GetCurSel() == 1);

    if (m_SelInterface.GetCurSel() != 1) {
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
    IDD_DIALOG_CONFIGURATION_M051BN,
    IDD_DIALOG_CONFIGURATION_M051CN,
    IDD_DIALOG_CONFIGURATION_M058,
    0x00012100, // NUC121, NUC125
    IDD_DIALOG_CONFIGURATION_M0564,	// M0564, NUC126
    IDD_DIALOG_CONFIGURATION_M451,
    0x00E45200,
    IDD_DIALOG_CONFIGURATION_MINI51,
    IDD_DIALOG_CONFIGURATION_MINI51CN,
    IDD_DIALOG_CONFIGURATION_MT500,

    // IDD_DIALOG_CONFIGURATION_N76E1T,
    //{"N76E884", 0x00002140, IDD_DIALOG_CONFIGURATION_N76E1T},
    //{"N76E616", 0x00002F50, IDD_DIALOG_CONFIGURATION_N76E1T},
    //{"N76E003", 0x00003650, IDD_DIALOG_CONFIGURATION_N76E1T},
    //{"N76L151", 0x00003E61, IDD_DIALOG_CONFIGURATION_N76E1T},
    //{"ML51BB9AE", 0x06004721, IDD_DIALOG_CONFIGURATION_N76E1T},

    0x00002140,
    0x00002F50,
    0x00003650,
    0x00003E61,
    0x06004721,

    IDD_DIALOG_CONFIGURATION_NANO100,
    IDD_DIALOG_CONFIGURATION_NANO100BN,
    IDD_DIALOG_CONFIGURATION_NANO103,
    IDD_DIALOG_CONFIGURATION_NANO112,
    IDD_DIALOG_CONFIGURATION_NM1120,
    IDD_DIALOG_CONFIGURATION_NM1200,
    IDD_DIALOG_CONFIGURATION_NUC100,
    IDD_DIALOG_CONFIGURATION_NUC102,
    IDD_DIALOG_CONFIGURATION_NUC103,
    IDD_DIALOG_CONFIGURATION_NUC103BN,
    IDD_DIALOG_CONFIGURATION_NUC131,
    IDD_DIALOG_CONFIGURATION_NUC200,
    IDD_DIALOG_CONFIGURATION_NUC400,
    IDD_DIALOG_CONFIGURATION_TC8226,
    IDD_DIALOG_CONFIGURATION_M480LD,
    IDD_DIALOG_CONFIGURATION_M2351,
    IDD_DIALOG_CONFIGURATION_M251,
    IDD_DIALOG_CONFIGURATION_I94000,
    IDD_DIALOG_CONFIGURATION_AU9100,
    IDD_DIALOG_CONFIGURATION_N570,
    ISD_91200_SERIES,
    IDD_DIALOG_CONFIGURATION_M031,
    IDD_DIALOG_CONFIGURATION_M251,
};

bool CDialogMain::DemoConfigDlg(UINT Template /* = 0 */)
{
    unsigned int CFG[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };

    if (Template == 0) {
        CMenu menu;
        menu.CreatePopupMenu();
        // 8051 1T Series
        CMenu *sub8051 = new CMenu;
        sub8051->CreatePopupMenu();
        sub8051->AppendMenu(MF_STRING, 0x00002140, _T("N76E884"));
        sub8051->AppendMenu(MF_STRING, 0x00002F50, _T("N76E616"));
        sub8051->AppendMenu(MF_STRING, 0x00003650, _T("N76E003"));
        sub8051->AppendMenu(MF_STRING, 0x00003E61, _T("N76L151"));
        sub8051->AppendMenu(MF_STRING, 0x00003E61, _T("ML51"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)sub8051->m_hMenu, _T("8051 1T Series"));
        menu.AppendMenu(MF_SEPARATOR);
        // M051 Series
        CMenu *subM051 = new CMenu;
        subM051->CreatePopupMenu();
        subM051->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M051, _T("M051AN Series"));
        subM051->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M051BN, _T("M051BN Series"));
        subM051->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M051CN, _T("M051DN/DE Series"));
        subM051->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M058, _T("M058SAN Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subM051->m_hMenu, _T("M051 Series"));
        menu.AppendMenu(MF_SEPARATOR);
        // Mini Series
        CMenu *subMini = new CMenu;
        subMini->CreatePopupMenu();
        subMini->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_MINI51, _T("Mini51AN Series"));
        subMini->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_MINI51CN, _T("Mini51DE Series"));
        subMini->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NM1200, _T("Mini55 Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMini->m_hMenu, _T("Mini Series"));
        menu.AppendMenu(MF_SEPARATOR);
        // NANO Series
        CMenu *subNano = new CMenu;
        subNano->CreatePopupMenu();
        subNano->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NANO100, _T("NANO100AN Series"));
        subNano->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NANO100BN, _T("NANO100BN Series"));
        subNano->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NANO112, _T("NANO112AN Series"));
        subNano->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NANO103, _T("NANO103 Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subNano->m_hMenu, _T("NANO Series"));
        menu.AppendMenu(MF_SEPARATOR);
        // NUC100 and NUC200 Series
        CMenu *subNUC1xx = new CMenu;
        subNUC1xx->CreatePopupMenu();
        subNUC1xx->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NUC100, _T("NUC100AN/BN/CN Series"));
        subNUC1xx->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NUC200, _T("NUC100DN Series"));
        subNUC1xx->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M0564, _T("NUC121 Series"));
        subNUC1xx->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NUC102, _T("NUC122AN Series"));
        subNUC1xx->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NUC103, _T("NUC123AN Series"));
        subNUC1xx->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NUC103BN, _T("NUC123AE Series"));
        subNUC1xx->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NUC131, _T("NUC131 Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subNUC1xx->m_hMenu, _T("NUC100 Series"));
        menu.AppendMenu(MF_SEPARATOR);
        // M23, M4 Series
        CMenu *subM234 = new CMenu;
        subM234->CreatePopupMenu();
        subM234->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NUC400, _T("NUC4XX Series"));
        subM234->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M451, _T("M451 Series"));
        subM234->AppendMenu(MF_STRING, 0x00E45200, _T("M4521 Series"));
        subM234->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_TC8226, _T("M480 Series"));
        subM234->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M480LD, _T("M480LD Series"));
        subM234->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M2351, _T("M2351 Series"));
        subM234->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M251, _T("M251 Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subM234->m_hMenu, _T("M23 and M4"));
        menu.AppendMenu(MF_SEPARATOR);
        // NuVoice Series
        CMenu *subVoice = new CMenu;
        subVoice->CreatePopupMenu();
        subVoice->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_AU9100, _T("AU9100 Series"));
        subVoice->AppendMenu(MF_STRING, ISD_91200_SERIES, _T("I91200 Series"));
        subVoice->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_I94000, _T("I94000 Series"));
        subVoice->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_N570, _T("I91000 Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subVoice->m_hMenu, _T("NuVoice"));
        menu.AppendMenu(MF_SEPARATOR);
        // Others Series
        CMenu *subOthers = new CMenu;
        subOthers->CreatePopupMenu();
        subOthers->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_MT500, _T("MT500"));
        subOthers->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_NM1120, _T("NM1120"));
        subOthers->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M031, _T("M031"));
        subOthers->AppendMenu(MF_STRING, IDD_DIALOG_CONFIGURATION_M251, _T("M251AE"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subOthers->m_hMenu, _T("Others"));
        menu.AppendMenu(MF_SEPARATOR);
        POINT point;
        GetCursorPos(&point);
        menu.TrackPopupMenu(TPM_LEFTALIGN, point.x, point.y, this);
        menu.DestroyMenu();
        delete sub8051;
        delete subM051;
        delete subMini;
        delete subNano;
        delete subNUC1xx;
        delete subM234;
        delete subVoice;
        delete subOthers;
    }

    return true;
}

LRESULT CDialogMain::WindowProc(UINT message, WPARAM wParam, LPARAM lParam)
{
    // TODO: Add your specialized code here and/or call the base class
    if (message == WM_COMMAND) {
        for (int i = 0; i < _countof(DialogTemplate); ++i) {
            if (DialogTemplate[i] == wParam) {
                unsigned int CFG[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
                ConfigDlgSel(CFG, sizeof(CFG), wParam);
                return 1;
            }
        }
    }

    return CDialog::WindowProc(message, wParam, lParam);
}
