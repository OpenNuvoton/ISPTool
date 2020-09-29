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
#ifdef _DEBUG
    // offline test mode
    // get chip series from database
    int ret = LoadChipSeries();
#endif // #ifdef _DEBUG
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
#include "DialogConfiguration_M05x.h"
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
#include "DialogChipSetting_M251.h"
//
#include "DialogChipSetting_NuMicro.h"
#include "DialogChipSetting_M480LD.h"

bool CDialogMain::ConfigDlgSel(unsigned int *pConfig, unsigned int size, unsigned int uSeriesCode /* = 0*/)
{
    bool ret = false;
    CDialog *pConfigDlg = NULL;
    unsigned int *Config;
    BOOL bIsDataFlashFixed = FALSE;
    unsigned int uProgramMemorySize = 0;
    unsigned int uDataFlashSize = 0;
    unsigned int uID = 0;
    unsigned int uPage_Size = NUMICRO_FLASH_PAGE_SIZE_512;

    if (uSeriesCode == 0) {
        // uSeriesCode = 0; online mode (load chip info. from gsChipCfgInfo)
        uSeriesCode = gsChipCfgInfo.uSeriesCode;
        bIsDataFlashFixed = gsChipCfgInfo.uDataFlashSize;
        uProgramMemorySize = gsChipCfgInfo.uProgramMemorySize;
        uDataFlashSize = gsChipCfgInfo.uDataFlashSize;
        uID = gsChipCfgInfo.uID;
        // (FlashInfo.cpp) Page Size Type: 0x000 (512 Bytes, default), 0x200 (2K), 0x300 (4K)
        uPage_Size = 1 << (((gsChipCfgInfo.uFlashType & 0x0000FF00) >>  8) + 9);
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

            case IDD_DIALOG_CONFIGURATION_NM1120:
                pConfigDlg = new CDialogConfiguration_NM1120;
                Config = (((CDialogConfiguration_NM1120 *)pConfigDlg)->m_ConfigValue.m_value);
                break;
#ifdef _DEBUG

            // case for NUC_CHIP_TYPE_GENERAL_1T offline test mode
            // test cases from g_80511TPartNumIDs in NuDataBase.cpp
            case 0x00002150: // N76E885
            case 0x00002F50: // N76E616
            case 0x00003650: // N76E003
            case 0x00104832: // ML51LC0XX
            case 0x0B004B21: // MS51FB9AE
            case 0x08125744: // ML56SD1AE
                uID = uSeriesCode;
#endif // #ifdef _DEBUG

            case NUC_CHIP_TYPE_GENERAL_1T: {
                pConfigDlg = new CDialogConfiguration_OT8051(uID);
                unsigned char *pucConfigValue = (((CDialogConfiguration_OT8051 *)pConfigDlg)->m_ucConfigValue);
                unsigned int Config0 = pConfig[0];
                unsigned int Config1 = pConfig[1];
                pucConfigValue[0] = (Config0 & 0x000000FF);
                pucConfigValue[1] = (Config0 & 0x0000FF00) >> 8;
                pucConfigValue[2] = (Config0 & 0x00FF0000) >> 16;
                pucConfigValue[3] = (Config0 & 0xFF000000) >> 24;
                pucConfigValue[4] = (Config1 & 0x000000FF);
                pucConfigValue[5] = 0xFF;
                pucConfigValue[6] = 0xFF;
                pucConfigValue[7] = 0xFF;

                if (pConfigDlg->DoModal() == IDOK) {
                    pConfig[0] = (pucConfigValue[3] << 24) + (pucConfigValue[2] << 16) + (pucConfigValue[1] << 8) + pucConfigValue[0];
                    pConfig[1] = 0xFFFFFF00 | pucConfigValue[4];
                    ret = true;
                }

                delete pConfigDlg;
            }

            return ret;
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

            case IDD_DIALOG_CONFIGURATION_M480:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogConfiguration_M480(uProgramMemorySize, uID);
                } else {
                    pConfigDlg = new CDialogConfiguration_M480();
                }

                Config = (((CDialogConfiguration_M480 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case IDD_DIALOG_CONFIGURATION_M480LD:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_M480LD(uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogChipSetting_M480LD();
                }

                Config = (((CDialogChipSetting_M480LD *)pConfigDlg)->m_uConfigValue);
                break;

            case IDD_DIALOG_CONFIGURATION_M2351:
            case NUC_CHIP_TYPE_M2354:
                pConfigDlg = new CDialogConfiguration_M2351;
                Config = (((CDialogConfiguration_M2351 *)pConfigDlg)->m_uConfigValue);

                if (uSeriesCode == NUC_CHIP_TYPE_M2354) {
                    ((CDialogConfiguration_M2351 *)pConfigDlg)->m_uChipType = NUC_CHIP_TYPE_M2354;
                }

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

            case IDD_DIALOG_CHIP_SETTING_CFG_M251:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_M251(0, uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogChipSetting_M251();
                }

                Config = (((CDialogChipSetting_M251 *)pConfigDlg)->m_uConfigOption_ConfigValue);
                break;

            case IDD_DIALOG_CHIP_SETTING_CFG_M258:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_M251(1, uProgramMemorySize);
                } else {
                    pConfigDlg = new CDialogChipSetting_M251(1);
                }

                Config = (((CDialogChipSetting_M251 *)pConfigDlg)->m_uConfigOption_ConfigValue);
                break;

            case NUC_CHIP_TYPE_M031:  // Page Size: 512
            case NUC_CHIP_TYPE_M031G: // Page Size: 2048
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_NuMicro(uProgramMemorySize, uPage_Size, NUC_CHIP_TYPE_M031);
                } else {
                    if (uSeriesCode == NUC_CHIP_TYPE_M031) {
                        uProgramMemorySize = 128 * 1024;
                        uPage_Size = 512;
                    } else {
                        uProgramMemorySize = 512 * 1024;
                        uPage_Size = 2048;
                    }

                    pConfigDlg = new CDialogChipSetting_NuMicro(uProgramMemorySize, uPage_Size, NUC_CHIP_TYPE_M031);
                }

                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            case NUC_CHIP_TYPE_M0A21:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_NuMicro(uProgramMemorySize, uPage_Size, uSeriesCode);
                } else {
                    pConfigDlg = new CDialogChipSetting_NuMicro(16 * 1024, 512, uSeriesCode);
                }

                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            case NUC_CHIP_TYPE_M030G:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_NuMicro(uProgramMemorySize, uPage_Size, uSeriesCode);
                } else {
                    pConfigDlg = new CDialogChipSetting_NuMicro(64 * 1024, 512, uSeriesCode);
                }

                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            case NUC_CHIP_TYPE_M451: // M451, M4521
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_NuMicro(uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_2K, uSeriesCode);
                } else {
                    pConfigDlg = new CDialogChipSetting_NuMicro(256 * 1024, NUMICRO_FLASH_PAGE_SIZE_2K, uSeriesCode);
                }

                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            case NUC_CHIP_TYPE_M471:
                if (uProgramMemorySize) {
                    pConfigDlg = new CDialogChipSetting_NuMicro(uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_2K, uSeriesCode);
                } else {
                    pConfigDlg = new CDialogChipSetting_NuMicro(512 * 1024, NUMICRO_FLASH_PAGE_SIZE_2K, uSeriesCode);
                }

                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
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
    m_SelInterface.AddString(_T("SPI"));
    m_SelInterface.AddString(_T("I2C"));
    m_SelInterface.AddString(_T("RS485"));
    m_SelInterface.AddString(_T("CAN"));
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

#ifdef _DEBUG

#define SERIES_N1XX  	0
#define SERIES_MINI  	1
#define SERIES_NANO  	2
#define SERIES_M05X  	3
#define SERIES_N029  	4
#define SERIES_M4XX  	5
#define SERIES_NONE  	6
#define SERIES_8051  	7
#define SERIES_AUDIO  	8
#define SERIES_NUM  	9

// call by OnButtonConfig
bool CDialogMain::DemoConfigDlg(UINT Template /* = 0 */)
{
    unsigned int CFG[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };

    if (Template == 0) {
        CMenu menu;
        menu.CreatePopupMenu();
        CMenu *subMenu[SERIES_NUM];
        unsigned int i;

        for (i = 0; i < SERIES_NUM; i++) {
            subMenu[i] = new CMenu;
            subMenu[i]->CreatePopupMenu();
        }

        for (i = 0; i < g_NuMicroChipSeries.size(); i++) {
            std::string str(g_NuMicroChipSeries[i].szPartNumber);

            // NuDataBase.cpp: int LoadChipSeries(void)
            if (g_NuMicroChipSeries[i].uID == NUC_CHIP_TYPE_GENERAL_1T) {
                subMenu[SERIES_8051]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
                continue;
            }

            if (str.find("NUC1") != std::string::npos) {
                subMenu[SERIES_N1XX]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            } else if (str.find("MINI") != std::string::npos) {
                subMenu[SERIES_MINI]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            } else if (str.find("Nano") != std::string::npos) {
                subMenu[SERIES_NANO]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            } else if (str.find("M05") != std::string::npos) {
                subMenu[SERIES_M05X]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            } else if (str.find("NUC029") != std::string::npos) {
                subMenu[SERIES_N029]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            } else if (str.find("M4") != std::string::npos) {
                subMenu[SERIES_M4XX]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            } else {
                subMenu[SERIES_NONE]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            }
        }

        for (i = 0; i < g_AudioChipSeries.size(); i++) {
            std::string str(g_AudioChipSeries[i].szPartNumber);
            subMenu[SERIES_AUDIO]->AppendMenu(MF_STRING, g_AudioChipSeries[i].uProjectCode, CString(g_AudioChipSeries[i].szPartNumber));
        }

        i = 0;
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_N1XX]->m_hMenu, _T("NUC1xx Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_MINI]->m_hMenu, _T("MINI Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_NANO]->m_hMenu, _T("NANO Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_M05X]->m_hMenu, _T("M05x Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_N029]->m_hMenu, _T("NUC029 Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_M4XX]->m_hMenu, _T("M4 Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_NONE]->m_hMenu, _T("Others"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_8051]->m_hMenu, _T("8051 1T Series"));
        menu.AppendMenu(MF_STRING | MF_POPUP, (UINT)subMenu[SERIES_AUDIO]->m_hMenu, _T("Audio Series"));

        for (i = 0; i < SERIES_NUM; i++) {
            delete subMenu[i];
        }

        POINT point;
        GetCursorPos(&point);
        menu.TrackPopupMenu(TPM_LEFTALIGN, point.x, point.y, this);
        menu.DestroyMenu();
        //menu.AppendMenu(MF_SEPARATOR);
    }

    return true;
}

#endif // #ifdef _DEBUG

LRESULT CDialogMain::WindowProc(UINT message, WPARAM wParam, LPARAM lParam)
{
#ifdef _DEBUG

    if (message == WM_COMMAND) {
        unsigned int i;

        for (i = 0; i < g_NuMicroChipSeries.size(); ++i) {
            if (g_NuMicroChipSeries[i].uProjectCode == wParam) {
                unsigned int CFG[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
                ConfigDlgSel(CFG, sizeof(CFG), wParam);
                return 1;
            }
        }

        for (i = 0; i < g_AudioChipSeries.size(); ++i) {
            if (g_AudioChipSeries[i].uProjectCode == wParam) {
                unsigned int CFG[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
                ConfigDlgSel(CFG, sizeof(CFG), wParam);
                return 1;
            }
        }
    }

#endif // #ifdef _DEBUG
    return CDialog::WindowProc(message, wParam, lParam);
}
