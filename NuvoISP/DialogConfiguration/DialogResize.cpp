// DialogScrollable.cpp : implementation file
//

#include "stdafx.h"
#include "DialogResize.h"
#include "resource.h"
#include <cstring>

// CDialogScrollable dialog

IMPLEMENT_DYNAMIC(CDialogResize, CDialog)

CDialogResize::CDialogResize(UINT nIDTemplate, CWnd *pParent /*=NULL*/)
    : CDialog(nIDTemplate, pParent)
{
    m_bIsInitialized = false;
    m_bShowScrollBar = false;
    m_ScrollBarWidth = 15;
}

CDialogResize::~CDialogResize()
{
}

BEGIN_MESSAGE_MAP(CDialogResize, CDialog)
    //{{AFX_MSG_MAP(CDialogResize)
    ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()

void CDialogResize::DoDataExchange(CDataExchange *pDX)
{
    CDialog::DoDataExchange(pDX);
}

void CDialogResize::OnSize(UINT nType, int cx, int cy)
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

void CDialogResize::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
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

void CDialogResize::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
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

void CDialogResize::OnGetMinMaxInfo(MINMAXINFO *lpMMI)
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
BOOL CDialogResize::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
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
void CDialogResize::AdjustDPI()
{
    int nScreenHeight;
    nScreenHeight = GetSystemMetrics(SM_CYSCREEN);
    //Get task bar height
    HWND hWnd = ::FindWindow(_T("Shell_TrayWnd"), NULL);
    CRect taskbar_rc;
    ::GetWindowRect(hWnd, &taskbar_rc);
    nScreenHeight = nScreenHeight - taskbar_rc.Height();

    if (nScreenHeight < m_rect.Height()) {
        CRect r1;
        GetDlgItem(IDCANCEL)->GetWindowRect(r1);
        m_rect.bottom = r1.bottom + 30;
        MoveWindow(m_rect.left, m_rect.top, m_rect.Width(), nScreenHeight);
    }
}

void CDialogResize::OnKillfocusEditFlashBaseAddress(BOOL bDataFlashEnable, unsigned int	uProgramMemorySize, unsigned int uPageSize)
{
    ASSERT(GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS) != NULL);
    ASSERT(GetDlgItem(IDC_EDIT_DATA_FLASH_SIZE) != NULL);
    ASSERT(GetDlgItem(IDC_STATIC_CONFIG_VALUE_1) != NULL);

    if (!bDataFlashEnable) {
        return;
    }

    CString sFlashBaseAddress, sDataFlashSize, sConfigValue1;
    GetDlgItemText(IDC_EDIT_FLASH_BASE_ADDRESS, sFlashBaseAddress);
    TCHAR *pEnd;
    unsigned int uFlashBaseAddress = ::_tcstoul(sFlashBaseAddress, &pEnd, 16);

    if (bDataFlashEnable) {
        if (uFlashBaseAddress < uPageSize) {
            uFlashBaseAddress = uPageSize;
        } else if (uFlashBaseAddress >= uProgramMemorySize) {
            uFlashBaseAddress = uProgramMemorySize - uPageSize;
        } else {
            uFlashBaseAddress &= ~(uPageSize - 1);
        }

        sDataFlashSize.Format(_T("%.2fK"), (uFlashBaseAddress < uProgramMemorySize) ? ((uProgramMemorySize - uFlashBaseAddress) / 1024.) : 0.);
        SetDlgItemText(IDC_EDIT_DATA_FLASH_SIZE, sDataFlashSize);
    }

    sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    SetDlgItemText(IDC_EDIT_FLASH_BASE_ADDRESS, sFlashBaseAddress);
    sConfigValue1.Format(_T("0x%08X"), uFlashBaseAddress);// | 0xFFF00000);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, sConfigValue1);
    UpdateData(TRUE); // "MUST" use TRUE, not FALSE
}

void CDialogResize::OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult, BOOL bDataFlashEnable, unsigned int uProgramMemorySize, unsigned int uPageSize)
{
    ASSERT(GetDlgItem(IDC_EDIT_FLASH_BASE_ADDRESS) != NULL);
    ASSERT(GetDlgItem(IDC_EDIT_DATA_FLASH_SIZE) != NULL);
    ASSERT(GetDlgItem(IDC_STATIC_CONFIG_VALUE_1) != NULL);

    if (!bDataFlashEnable) {
        return;
    }

    CString sFlashBaseAddress, sDataFlashSize, sConfigValue1;
    GetDlgItemText(IDC_EDIT_FLASH_BASE_ADDRESS, sFlashBaseAddress);
    TCHAR *pEnd;
    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
    unsigned int uFlashBaseAddress = ::_tcstoul(sFlashBaseAddress, &pEnd, 16);

    if (pNMUpDown->iDelta == 1) {
        if ((uFlashBaseAddress + uPageSize) < uProgramMemorySize) {
            uFlashBaseAddress += uPageSize;
        }
    } else if (pNMUpDown->iDelta == -1) {
        if (!(uFlashBaseAddress <= uPageSize)) {
            uFlashBaseAddress -= uPageSize;
        }
    }

    if (uFlashBaseAddress < uPageSize) {
        uFlashBaseAddress = uPageSize;
    } else if (uFlashBaseAddress >= uProgramMemorySize) {
        uFlashBaseAddress = uProgramMemorySize - uPageSize;
    } else {
        uFlashBaseAddress &= ~(uPageSize - 1);
    }

    //uFlashBaseAddress &= ~(uPageSize - 1); //(uFlashBaseAddress /*& (uProgramMemorySize - uPageSize)*/) / uPageSize * uPageSize;
    sFlashBaseAddress.Format(_T("%X"), uFlashBaseAddress);
    SetDlgItemText(IDC_EDIT_FLASH_BASE_ADDRESS, sFlashBaseAddress);
    sDataFlashSize.Format(_T("%.2fK"), (bDataFlashEnable && (uFlashBaseAddress < uProgramMemorySize)) ? ((uProgramMemorySize - uFlashBaseAddress) / 1024.) : 0.);
    SetDlgItemText(IDC_EDIT_DATA_FLASH_SIZE, sDataFlashSize);
    sConfigValue1.Format(_T("0x%08X"), uFlashBaseAddress);// | 0xFFF00000);
    SetDlgItemText(IDC_STATIC_CONFIG_VALUE_1, sConfigValue1);
    UpdateData(TRUE); // "MUST" use TRUE, not FALSE
//	UpdateData(FALSE);
    *pResult = 0;
}

