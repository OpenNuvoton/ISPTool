// DialogScrollable.cpp : implementation file
//

#include "stdafx.h"
#include "DialogMain.h"
#include "resource.h"
#include <cstring>
#include "ISPLdCmd.h"
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

    if (!m_bIsInitialized)
    {
        nScrollMaxY = 0;
        nScrollMaxX = 0;
    }
    else
    {
        if (rect.Height() < m_rect.Height())
        {
            nScrollMaxY = m_rect.Height() - rect.Height() + m_ScrollBarWidth;
        }
        else
        {
            nScrollMaxY = 0;
        }

        if (rect.Width() < m_rect.Width())
        {
            nScrollMaxX = m_rect.Width() - rect.Width() + m_ScrollBarWidth;
        }
        else
        {
            nScrollMaxX = 0;
        }

        if (nScrollMaxY != 0 && nScrollMaxX == 0)
        {
            nScrollMaxX = 10;
        }
        else if (nScrollMaxY == 0 && nScrollMaxX != 0)
        {
            nScrollMaxY = 10;
        }
    }

    if (nScrollMaxX == 0 || nScrollMaxY == 0)
    {
        m_bShowScrollBar = false;
    }
    else
    {
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

    switch (nSBCode)
    {
        case SB_LINEDOWN:
            if (m_nScrollPosY >= nMaxPos)
            {
                return;
            }

            nDelta = min(nMaxPos / 100, nMaxPos - m_nScrollPosY);

            if (nMaxPos / 100 == 0)
            {
                nDelta = 1;
            }

            break;

        case SB_LINEUP:
            if (m_nScrollPosY <= 0)
            {
                return;
            }

            nDelta = -min(nMaxPos / 100, m_nScrollPosY);

            if (nMaxPos / 100 == 0)
            {
                nDelta = -1;
            }

            break;

        case SB_PAGEDOWN:
            if (m_nScrollPosY >= nMaxPos)
            {
                return;
            }

            nDelta = min(nMaxPos / 10, nMaxPos - m_nScrollPosY);
            break;

        case SB_THUMBPOSITION:
            nDelta = (int)nPos - m_nScrollPosY;
            break;

        case SB_PAGEUP:
            if (m_nScrollPosY <= 0)
            {
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

    switch (nSBCode)
    {
        case SB_LINERIGHT:
            if (m_nScrollPosX >= nMaxPos)
            {
                return;
            }

            nDelta = min(nMaxPos / 100, nMaxPos - m_nScrollPosX);

            if (nMaxPos / 100 == 0)
            {
                nDelta = 1;
            }

            break;

        case SB_LINELEFT:
            if (m_nScrollPosX <= 0)
            {
                return;
            }

            nDelta = -min(nMaxPos / 100, m_nScrollPosX);

            if (nMaxPos / 100 == 0)
            {
                nDelta = -1;
            }

            break;

        case SB_PAGERIGHT:
            if (m_nScrollPosX >= nMaxPos)
            {
                return;
            }

            nDelta = min(nMaxPos / 10, nMaxPos - m_nScrollPosX);
            break;

        case SB_THUMBPOSITION:
            nDelta = (int)nPos - m_nScrollPosX;
            break;

        case SB_PAGELEFT:
            if (m_nScrollPosX <= 0)
            {
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

    if (m_bIsInitialized)
    {
        lpMMI->ptMaxTrackSize.x = m_rect.Width();
        lpMMI->ptMaxTrackSize.y = m_rect.Height();
        lpMMI->ptMinTrackSize.x = 310;
        lpMMI->ptMinTrackSize.y = 310;
    }
}

BOOL CDialogMain::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
    CDialog::OnMouseWheel(nFlags, zDelta, pt);

    if (m_bShowScrollBar)
    {
        SCROLLINFO scrollinfo;
        GetScrollInfo(SB_VERT, &scrollinfo);
        int nMaxPos = scrollinfo.nMax;
        int nScrollPos = scrollinfo.nPos;
        int nDelta;

        if (zDelta < 0)
        {
            if (m_nScrollPosY >= nMaxPos)
            {
                return 0;
            }

            nDelta = min(nMaxPos / 10, nMaxPos - m_nScrollPosY);
        }
        else
        {
            if (m_nScrollPosY <= 0)
            {
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

    if (pWnd != NULL)
    {
        pWnd->ShowWindow(nCmdShow);
    }
}

void CDialogMain::EnableDlgItem(int nID, BOOL bEnable)
{
    CWnd *pWnd = GetDlgItem(nID);

    if (pWnd != NULL)
    {
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
#include "DialogConfiguration_I94000.h"
#include "DialogConfiguration_I96000.h"
#include "DialogConfiguration_AU9100.h"
#include "DialogConfiguration_N570.h"
#include "DialogConfiguration_N572.h"
#include "DialogConfiguration_N572F064.h"
#include "DialogConfiguration_N572F072.h"
#include "DialogConfiguration_N574.h"
#include "DialogChipSetting_M251.h"
//
#include "DialogChipSetting_NuMicro.h"
#include "DialogChipSetting_M480LD.h"
#include "DialogChipSetting_M2351.h"
#include "DialogChipSetting_M2354.h"
#include "DialogChipSetting_M460.h"
#include "DialogChipSetting_M2L31.h"
#include "DialogChipSetting_M55M1.h"
#include "DialogChipSetting_M2A23.h"
#include "DialogChipSetting_M3331.h"
#include "DialogChipSetting_M2U51.h"

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

    if (uSeriesCode == 0)
    {
        // uSeriesCode = 0; online mode (load chip info. from gsChipCfgInfo)
        uSeriesCode = gsChipCfgInfo.uSeriesCode;
        bIsDataFlashFixed = gsChipCfgInfo.uDataFlashSize;
        uProgramMemorySize = gsChipCfgInfo.uProgramMemorySize;
        uDataFlashSize = gsChipCfgInfo.uDataFlashSize;
        uID = gsChipCfgInfo.uID;
        // (FlashInfo.cpp) Page Size Type: 0x000 (512 Bytes, default), 0x200 (2K), 0x300 (4K)
        uPage_Size = 1 << (((gsChipCfgInfo.uFlashType & 0x0000FF00) >>  8) + 9);
    }

    if (1)
    {
        switch (uSeriesCode)
        {
            case PROJ_NUC100AN:
            case PROJ_NUC100CN:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NUC1xx(bIsDataFlashFixed, uProgramMemorySize, uDataFlashSize);    // "NUC100BN";
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NUC1xx;    // "NUC100BN";
                }

                Config = (((CDialogConfiguration_NUC1xx *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NUC122AN:
            case PROJ_NUC122DN:
                pConfigDlg = new CDialogConfiguration_NUC102;
                Config = (((CDialogConfiguration_NUC102 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NUC123AN:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NUC123AN(uProgramMemorySize, uDataFlashSize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NUC123AN;
                }

                Config = (((CDialogConfiguration_NUC123AN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NUC123AE:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NUC123AE(uProgramMemorySize, uDataFlashSize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NUC123AE;
                }

                Config = (((CDialogConfiguration_NUC123AE *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NUC100DN:
            case PROJ_NUC200AE:
            case PROJ_NUC2201:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NUC2xx(bIsDataFlashFixed, uProgramMemorySize, uDataFlashSize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NUC2xx;
                }

                Config = (((CDialogConfiguration_NUC2xx *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NUC1311:
            case PROJ_M0518:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NUC131(uProgramMemorySize, uDataFlashSize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NUC131;
                }

                Config = (((CDialogConfiguration_NUC131 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NANO100AN:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_Nano100AN(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_Nano100AN;
                }

                Config = (((CDialogConfiguration_Nano100AN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NANO100BN:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_Nano100(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_Nano100;
                }

                Config = (((CDialogConfiguration_Nano100 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NANO102AN:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_Nano112(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_Nano112;
                }

                Config = (((CDialogConfiguration_Nano112 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NANO103:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_Nano103(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_Nano103;
                }

                Config = (((CDialogConfiguration_Nano103 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            // M051AN, M051BN
            case PROJ_M051AN:
                pConfigDlg = new CDialogConfiguration_M05XAN();
                Config = (((CDialogConfiguration_M05XAN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_M051BN:
                pConfigDlg = new CDialogConfiguration_M05XBN();
                Config = (((CDialogConfiguration_M05XBN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            // M051DN, M051DE, M058SAN
            case PROJ_M051DN:
            case PROJ_M051DE:
                pConfigDlg = new CDialogConfiguration_M05XDN;
                Config = (((CDialogConfiguration_M05XDN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_M058SAN:
                pConfigDlg = new CDialogConfiguration_M058SAN;
                Config = (((CDialogConfiguration_M058SAN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_MINI51AN:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_Mini51(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_Mini51;
                }

                Config = (((CDialogConfiguration_Mini51 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_MINI51DE:
            case PROJ_MINI58:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_Mini51CN(uProgramMemorySize, uID);
                }
                else if ((uID & 0xFFFFFF00) == 0x00A05800)
                {
                    pConfigDlg = new CDialogConfiguration_Mini51CN(32 * 1024, uID);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_Mini51CN;
                }

                Config = (((CDialogConfiguration_Mini51CN *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_MINI55:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NM1200(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NM1200;
                }

                Config = (((CDialogConfiguration_NM1200 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NM1500AE:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_MT500(bIsDataFlashFixed, uProgramMemorySize, uDataFlashSize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_MT500;
                }

                Config = (((CDialogConfiguration_MT500 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NUC400AE:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NUC4xx(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NUC4xx;
                }

                Config = (((CDialogConfiguration_NUC4xx *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_M3331G:
            case PROJ_M3331IG:
                pConfigDlg = new CDialogChipSetting_M3331(uSeriesCode);

                Config = (((CDialogChipSetting_M3331*)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_NM1120:
            case PROJ_NM1230:
            case PROJ_NM1240:
                pConfigDlg = new CDialogConfiguration_NM1120;
                Config = (((CDialogConfiguration_NM1120 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            // case for NUC_CHIP_TYPE_GENERAL_1T offline test mode
            // test cases from g_80511TPartNumIDs in NuDataBase.cpp
            case PROJ_N76E885: // N76E885
            case PROJ_N76E616: // N76E616
            case PROJ_N76E003: // N76E003
            case PROJ_ML51_32K:
            case PROJ_ML51_16K:
            case PROJ_MS51_32K:
            case PROJ_MS51_16K:
            case PROJ_MS51_8K:
            case PROJ_ML56:
            case PROJ_MUG51:
            case PROJ_MG51:
            case PROJ_MG51D:
            {
                uID = uSeriesCode;
                pConfigDlg = new CDialogConfiguration_OT8051(uID);
                unsigned char *pucConfigValue = (((CDialogConfiguration_OT8051*)pConfigDlg)->m_ucConfigValue);
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

                if (pConfigDlg->DoModal() == IDOK)
                {
                    pConfig[0] = (pucConfigValue[3] << 24) + (pucConfigValue[2] << 16) + (pucConfigValue[1] << 8) + pucConfigValue[0];
                    pConfig[1] = 0xFFFFFF00 | pucConfigValue[4];
                    ret = true;
                }

                delete pConfigDlg;

                return ret;
            }
            break;

            case PROJ_M0564:
            case PROJ_NUC1261:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_M0564(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_M0564();
                }

                Config = (((CDialogConfiguration_M0564 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NUC121:   // NUC121, NUC125
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NUC121(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NUC121();
                }

                Config = (((CDialogConfiguration_NUC121 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_NUC1262:
            case PROJ_NUC1263:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NUC1262(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NUC1262();
                }

                Config = (((CDialogConfiguration_NUC1262 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_M480:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_M480(uProgramMemorySize, uID);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_M480();
                }

                Config = (((CDialogConfiguration_M480 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_M480LD:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogChipSetting_M480LD(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogChipSetting_M480LD();
                }

                Config = (((CDialogChipSetting_M480LD *)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M460HD:
            case PROJ_M460LD:
                pConfigDlg = new CDialogChipSetting_M460(uID, uID & 0xFFFF, uSeriesCode);

                Config = (((CDialogChipSetting_M460 *)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M2351:
                pConfigDlg = new CDialogChipSetting_M2351(M2351_MAX_APROM_SIZE, NUMICRO_FLASH_PAGE_SIZE_2K, TRUE, TRUE, NUC_CHIP_TYPE_M2351);
                Config = (((CDialogChipSetting_M2351*)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M2354ES:
            case PROJ_M2354:
                pConfigDlg = new CDialogChipSetting_M2354(M2354_MAX_APROM_SIZE, NUMICRO_FLASH_PAGE_SIZE_2K, TRUE, NUC_CHIP_TYPE_M2354);
                Config = (((CDialogChipSetting_M2354 *)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M2L31_256K:
            case PROJ_M2L31_512K:
                pConfigDlg = new CDialogChipSetting_M2L31(uID, uID & 0xFFFF, uSeriesCode);

                Config = (((CDialogChipSetting_M2L31*)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M2A23:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogChipSetting_M2A23(uProgramMemorySize, NUMICRO_FLASH_PAGE_SIZE_2K, uID, uID & 0xFFFF, uSeriesCode);
                }
                else
                {
                    pConfigDlg = new CDialogChipSetting_M2A23(256 * 1024, NUMICRO_FLASH_PAGE_SIZE_2K, uID, uID & 0xFFFF, uSeriesCode);
                }

                Config = (((CDialogChipSetting_M2A23*)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M2003:
                pConfigDlg = new CDialogChipSetting_NuMicro(uID, uID & 0xFFFF, IDS_M2003_SERIES);
                Config = (((CDialogChipSetting_NuMicro*)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M2U51G: 
            case PROJ_M2U51C:
                pConfigDlg = new CDialogChipSetting_M2U51(uID, uID & 0xFFFF, uSeriesCode);

                Config = (((CDialogChipSetting_M2U51*)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M55M1:
                pConfigDlg = new CDialogChipSetting_M55M1(TRUE, uID, uID & 0xFFFF, uSeriesCode);

                Config = (((CDialogChipSetting_M55M1*)pConfigDlg)->m_uConfigValue);
                break;

            case ISD_94000_SERIES:
            case NPCx_SERIES:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_I94000(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_I94000();
                }

                Config = (((CDialogConfiguration_I94000 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case ISD_96000_SERIES:

                pConfigDlg = new CDialogConfiguration_I96000();

                Config = (((CDialogConfiguration_I96000*)pConfigDlg)->m_ConfigValue.m_value);

            case ISD_91200_SERIES:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_I9200(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_I9200();
                }

                Config = (((CDialogConfiguration_I9200 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case ISD_9160_SERIES:
            case ISD_91300_SERIES:
            case NUVOICE_N575_SERIES:
            case NUVOICE_N574F128_SERIES:
            case NUVOICE_N574F1K5_SERIES:
            case NUVOICE_NSC74128_SERIES:
            case NUVOICE_NSC741K5_SERIES:
            case NUVOICE_NSC128L42_SERIES:
            case NUVOICE_N576_SERIES:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_N574(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_N574();
                }

                Config = (((CDialogConfiguration_N574*)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case ISD_91500_SERIES:               
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_I91500(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_I91500();
                }

                Config = (((CDialogConfiguration_I91500*)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case NSC_SERIES:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_NSC128(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_NSC128();
                }
                Config = (((CDialogConfiguration_NSC128*)pConfigDlg)->m_ConfigValue.m_value);

            case NUVOICE_N569_SERIES:
            case NUVOICE_N569J_SERIES:
                pConfigDlg = new CDialogConfiguration_AU9100();
                Config = (((CDialogConfiguration_AU9100 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case ISD_91000_SERIES:
            case NUVOICE_N570_SERIES:
            case NUVOICE_N570H_SERIES:
            case NUVOICE_N570J_SERIES:
            case NUVOICE_JNK561_SERIES:
            case NUVOICE_JNK561H_SERIES:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogConfiguration_N570(uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogConfiguration_N570();
                }

                Config = (((CDialogConfiguration_N570 *)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case NUVOICE_N571_SERIES:
                pConfigDlg = new CDialogConfiguration_N572();
                Config = (((CDialogConfiguration_N572*)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case NUVOICE_N572F064_SERIES:
                pConfigDlg = new CDialogConfiguration_N572F064();
                Config = (((CDialogConfiguration_N572F064*)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case NUVOICE_N572F072_SERIES:
                pConfigDlg = new CDialogConfiguration_N572F072();
                Config = (((CDialogConfiguration_N572F072*)pConfigDlg)->m_ConfigValue.m_value);
                break;

            case PROJ_M252_C:
            case PROJ_M252_D:
            case PROJ_M252_E:
            case PROJ_M252_G:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogChipSetting_M251(0, uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogChipSetting_M251();
                }

                Config = (((CDialogChipSetting_M251 *)pConfigDlg)->m_uConfigOption_ConfigValue);
                break;

            case PROJ_M258:
            case PROJ_M253:
            case PROJ_M256D:
            case PROJ_M258G:
                if (uProgramMemorySize)
                {
                    pConfigDlg = new CDialogChipSetting_M251(1, uProgramMemorySize);
                }
                else
                {
                    pConfigDlg = new CDialogChipSetting_M251(1);
                }

                Config = (((CDialogChipSetting_M251 *)pConfigDlg)->m_uConfigOption_ConfigValue);
                break;

            case PROJ_M031_128K:
            case PROJ_M031_64K:
            case PROJ_M031_32K:
            case PROJ_M031_16K:
            case PROJ_M032D:           // Page Size: 512
            case PROJ_M031_512K:
            case PROJ_M031_256K:       // Page Size: 2048
                pConfigDlg = new CDialogChipSetting_NuMicro(uID, uID & 0xFFFF, IDS_M031_SERIES);
                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M0A21:
                pConfigDlg = new CDialogChipSetting_NuMicro(uID, uID & 0xFFFF, IDS_M0A21_SERIES);
                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M030G:
            case PROJ_M031GPON:
                pConfigDlg = new CDialogChipSetting_NuMicro(uID, uID & 0xFFFF, IDS_M030G_SERIES);
                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M451HD: // M451, M4521
            case PROJ_M451LD:
            case PROJ_M4521:
                pConfigDlg = new CDialogChipSetting_NuMicro(uID, uID & 0xFFFF, IDS_M451_SERIES);
                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            case PROJ_M471:
                pConfigDlg = new CDialogChipSetting_NuMicro(uID, uID & 0xFFFF, IDS_M471_SERIES);
                Config = (((CDialogChipSetting_NuMicro *)pConfigDlg)->m_uConfigValue);
                break;

            default:
                printf("or Unknow Configuration Dialog %X\n", uSeriesCode);
                return false;
        }
    }

    if (pConfigDlg != NULL)
    {
        // Pass User Config to Configuration Dialog
        memcpy(Config, pConfig, size);

        if (pConfigDlg->DoModal() == IDOK)
        {
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

    if (hResult != ERROR_SUCCESS)   //如果無法打開hKEY,則中止程式的執行
    {
        //AfxMessageBox("錯誤：無法打開有關註冊表項");
        return  0 ;
    }

    TCHAR strInf[30];
    DWORD type_1 = REG_SZ;
    DWORD cbData_1 = 10;
    DWORD aa = 30, num = 0, a1, a2, a3, a4, a5, a6, a7;
    hResult = ::RegQueryInfoKey(hKEY, strInf, &a7, NULL, &a3, &a1, &a2, &num, &a4, &a5, &a6, NULL);

    if (hResult != ERROR_SUCCESS)   //如果無法打開hKEY,則中止程式的執行
    {
        //AfxMessageBox("錯誤：無法打開有關註冊表項");
        RegCloseKey(hKEY);
        return   0;
    }

    BYTE portName[30];
    CString csr;

    for (DWORD i = 0 ; i < num ; i++)
    {
        aa = 30 ;
        cbData_1 = 30;
        hResult = ::RegEnumValue(hKEY, i, strInf, &aa, NULL, &type_1, portName, &cbData_1);

        if ((hResult != ERROR_SUCCESS) && (hResult != ERROR_MORE_DATA))   //如果無法打開hKEY,則中止程式的執行
        {
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

void CDialogMain::InitComboBox(int iSupportNL2)
{
    m_Interfaces.clear();
    m_Interfaces.push_back(std::make_pair(_T("USB"), INTF_HID));
    m_Interfaces.push_back(std::make_pair(_T("UART"), INTF_UART));

    // Nu-Link2 ISP Bridge interfaces
    if (iSupportNL2)
    {
        m_Interfaces.push_back(std::make_pair(_T("SPI"), INTF_SPI));
        m_Interfaces.push_back(std::make_pair(_T("I2C"), INTF_I2C));
        m_Interfaces.push_back(std::make_pair(_T("RS485"), INTF_RS485));
        m_Interfaces.push_back(std::make_pair(_T("CAN"), INTF_CAN));
        m_Interfaces.push_back(std::make_pair(_T("LIN"), INTF_LIN));
    }

    m_Interfaces.push_back(std::make_pair(_T("Wi-Fi"), INTF_WIFI));
    m_Interfaces.push_back(std::make_pair(_T("BLE"), INTF_BLE));

    m_SelInterface.ResetContent();

    for (size_t i = 0; i < m_Interfaces.size(); i++)
    {
        m_SelInterface.AddString(m_Interfaces[i].first);
    }

    m_SelInterface.SetCurSel(0);
    OnSelchangeInterface();

    if (ScanPCCom())
    {
        m_SelComPort.SetCurSel(0);
    }
}

void CDialogMain::OnSelchangeInterface()
{
    m_SelComPort.EnableWindow(m_Interfaces[m_SelInterface.GetCurSel()].second == INTF_UART);

    if (m_Interfaces[m_SelInterface.GetCurSel()].second != INTF_UART)
    {
        EnableDlgItem(IDC_BUTTON_CONNECT, true);
    }
    else
    {
        OnComboChange();
    }

    GetDlgItem(IDC_COMBO_COM_PORT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_EDIT_IPADDRESS)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_EDIT_IPPORT)->ShowWindow(SW_HIDE);
    GetDlgItem(IDC_EDIT_BDNAME)->ShowWindow(SW_HIDE);

    if (m_Interfaces[m_SelInterface.GetCurSel()].second == INTF_WIFI)
    {
        GetDlgItem(IDC_EDIT_IPADDRESS)->ShowWindow(SW_SHOW);
        GetDlgItem(IDC_EDIT_IPPORT)->ShowWindow(SW_SHOW);
    }
    else if (m_Interfaces[m_SelInterface.GetCurSel()].second == INTF_BLE)
    {
        GetDlgItem(IDC_EDIT_BDNAME)->ShowWindow(SW_SHOW);
    }
    else
    {
        GetDlgItem(IDC_COMBO_COM_PORT)->ShowWindow(SW_SHOW);
    }
}

void CDialogMain::OnComboChange()
{
    if (m_SelComPort.GetCurSel() == 0)
    {
        int portcnt = ScanPCCom();
        printf("Num Port = %d\n", portcnt);
    }

    EnableDlgItem(IDC_BUTTON_CONNECT, m_SelComPort.GetCurSel());
}

void CDialogMain::OnIPAddressChange()
{
}

void CDialogMain::OnIPPortChange()
{
    CString sIPPort;
    m_EditIPPort.GetWindowText(sIPPort);
    m_iIPPort = _tstoi(sIPPort);

    if (m_iIPPort < 0)
        m_iIPPort = 0;
    else if (m_iIPPort > 65536)
        m_iIPPort = 65536;

    UpdateData(FALSE);
}

void CDialogMain::EnableInterface(bool bEnable)
{
    if (bEnable)
    {
        m_SelInterface.EnableWindow(TRUE);
        OnSelchangeInterface();
        m_IPAddress.EnableWindow(TRUE);
        m_EditIPPort.EnableWindow(TRUE);
        m_EditBDName.EnableWindow(FALSE);
    }
    else
    {
        m_SelInterface.EnableWindow(FALSE);
        m_SelComPort.EnableWindow(FALSE);
        m_IPAddress.EnableWindow(FALSE);
        m_EditIPPort.EnableWindow(FALSE);
        m_EditBDName.EnableWindow(FALSE);
    }
}

#ifdef _DEBUG

#define SERIES_N1XX     0
#define SERIES_MINI     1
#define SERIES_NANO     2
#define SERIES_M05X     3
#define SERIES_N029     4
#define SERIES_M4XX     5
#define SERIES_NONE     6
#define SERIES_8051     7
#define SERIES_AUDIO    8
#define SERIES_NUM      9

#include <string>
#include <sstream>
#include <iomanip>
template <typename T>
std::string int_to_hex(T val, size_t width = sizeof(T) * 2)
{
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(width) << std::hex << (val | 0);
    return ss.str();
}

// call by OnButtonConfig
bool CDialogMain::DemoConfigDlg(UINT Template /* = 0 */)
{
    unsigned int CFG[19] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                            0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                            0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                            0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                            0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
                           };

    if (Template == 0)
    {
        CMenu menu;
        menu.CreatePopupMenu();
        CMenu *subMenu[SERIES_NUM];
        unsigned int i;

        for (i = 0; i < SERIES_NUM; i++)
        {
            subMenu[i] = new CMenu;
            subMenu[i]->CreatePopupMenu();
        }

        for (i = 0; i < g_NuMicroChipSeries.size(); i++)
        {
            std::string str(g_NuMicroChipSeries[i].szPartNumber);

            // NuDataBase.cpp: int LoadChipSeries(void)
            if ((g_NuMicroChipSeries[i].uProjectCode >= PROJ_N76E885) && (g_NuMicroChipSeries[i].uProjectCode <= PROJ_MG51D))
            {
                subMenu[SERIES_8051]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
                continue;
            }

            if (str.find("NUC1") != std::string::npos)
            {
                subMenu[SERIES_N1XX]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            }
            else if (str.find("MINI") != std::string::npos)
            {
                subMenu[SERIES_MINI]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            }
            else if (str.find("Nano") != std::string::npos)
            {
                subMenu[SERIES_NANO]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            }
            else if (str.find("M05") != std::string::npos)
            {
                subMenu[SERIES_M05X]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            }
            else if (str.find("NUC029") != std::string::npos)
            {
                subMenu[SERIES_N029]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            }
            else if (str.find("M4") != std::string::npos)
            {
                subMenu[SERIES_M4XX]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            }
            else
            {
                subMenu[SERIES_NONE]->AppendMenu(MF_STRING, g_NuMicroChipSeries[i].uProjectCode, CString(g_NuMicroChipSeries[i].szPartNumber));
            }
        }

        for (i = 0; i < g_AudioChipSeries.size(); i++)
        {
            std::string str(g_AudioChipSeries[i].szPartNumber);
            //subMenu[SERIES_AUDIO]->AppendMenu(MF_STRING, g_AudioChipSeries[i].uProjectCode, CString(g_AudioChipSeries[i].szPartNumber));
            subMenu[SERIES_AUDIO]->AppendMenu(MF_STRING, g_AudioChipSeries[i].uProjectCode, CString(int_to_hex(g_AudioChipSeries[i].uProjectCode).c_str()));
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

        for (i = 0; i < SERIES_NUM; i++)
        {
            delete subMenu[i];
        }

        menu.AppendMenu(MF_STRING | MF_POPUP, 0xFFFFFFFF, _T("Test ALL"));
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

    if (message == WM_COMMAND)
    {
        unsigned int i;
        unsigned int CFG[19] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 
                                 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
                               };

        if (wParam == 0xFFFFFFFF)   // Test ALL
        {
            for (i = 0; i < g_NuMicroChipSeries.size(); ++i)
            {
                unsigned int uSeriesCode = g_NuMicroChipSeries[i].uProjectCode;
                ConfigDlgSel(CFG, sizeof(CFG), uSeriesCode);
            }

            return 1;
        }

        for (i = 0; i < g_NuMicroChipSeries.size(); ++i)
        {
            if (g_NuMicroChipSeries[i].uProjectCode == wParam)
            {
                ConfigDlgSel(CFG, sizeof(CFG), wParam);
                return 1;
            }
        }

        for (i = 0; i < g_AudioChipSeries.size(); ++i)
        {
            if (g_AudioChipSeries[i].uProjectCode == wParam)
            {
                ConfigDlgSel(CFG, sizeof(CFG), wParam);
                return 1;
            }
        }
    }

#endif // #ifdef _DEBUG
    return CDialog::WindowProc(message, wParam, lParam);
}
