#pragma once

#include "resource.h"
#include "DialogConfiguration.h"

class CDialogResize : public CDialog
{
    DECLARE_DYNAMIC(CDialogResize)

public:
    CDialogResize(UINT nIDTemplate, CWnd *pParent = NULL);  // standard constructor
    virtual ~CDialogResize();

// Dialog Data
    int m_nScrollPosY; // to store the current vertical scroll position
    int m_nScrollPosX; // to store the current horizontal scroll position
    int m_ScrollBarWidth;
    BOOL m_bIsInitialized;
    BOOL m_bShowScrollBar;
    CRect m_rect;		//original dialog rect info

protected:
    void AdjustDPI();
    virtual void DoDataExchange(CDataExchange *pDX);    // DDX/DDV support

    afx_msg void OnSize(UINT nType, int cx, int cy);
    afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar);
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar);
    afx_msg void OnGetMinMaxInfo(MINMAXINFO *lpMMI);
    afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
    DECLARE_MESSAGE_MAP()

public:
    void OnKillfocusEditFlashBaseAddress(BOOL bDataFlashEnable, unsigned int uProgramMemorySize, unsigned int uPageSize);
    void OnDeltaposSpinDataFlashSize(NMHDR *pNMHDR, LRESULT *pResult, BOOL bDataFlashEnable, unsigned int uProgramMemorySize, unsigned int uPageSize);
};
