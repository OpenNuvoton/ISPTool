#pragma once


struct CPartNumID;

struct WINCTRLID {
    int         btn;
    int         path;
    int         sizecksum;
};

class CDialogMain : public CDialog
{
	DECLARE_DYNAMIC(CDialogMain)

public:
	CDialogMain(UINT nIDTemplate, CWnd* pParent = NULL ); // standard constructor
	virtual ~CDialogMain();

// Dialog Data
	int m_nScrollPosY; // to store the current vertical scroll position
	int m_nScrollPosX; // to store the current horizontal scroll position
	int m_ScrollBarWidth;
	BOOL m_bIsInitialized;
	BOOL m_bShowScrollBar;
	CRect m_rect;		//original dialog rect info


#ifndef _NO_CONFIGURATION_DLG
	bool ConfigDlgSel(unsigned int *pConfig, unsigned int size);
	bool ConfigSetting(unsigned int id, unsigned int *pConfig, unsigned int size);
	void Test();
#endif

	void ShowDlgItem(int nID, int nCmdShow);
	void EnableDlgItem(int nID, BOOL bEnable);


protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	virtual BOOL OnInitDialog();

	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnGetMinMaxInfo(MINMAXINFO* lpMMI);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	DECLARE_MESSAGE_MAP()
};
