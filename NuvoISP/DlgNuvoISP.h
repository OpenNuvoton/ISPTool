// ICPToolDlg.h : header file
//

#if !defined(_DLG_NUVOISP_H_)
#define _DLG_NUVOISP_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Resource.h"
#include "DialogHex.h"
#include "DialogMain.h"
#include "ISPProc.h"

/////////////////////////////////////////////////////////////////////////////
// CNuvoISPDlg dialog

class CNuvoISPDlg: public CDialogMain, public CISPProc
{
// Construction
public:
#if (SUPPORT_SPIFLASH)
    enum { IDD = IDD_DIALOG_NUVOISP_WITH_SPI};
#else
    enum { IDD = IDD_DIALOG_NUVOISP};
#endif

    BOOL m_bShowSPI;
    void ShowSPIOptions(BOOL bShow);

    CNuvoISPDlg(UINT Template = CNuvoISPDlg::IDD, CWnd *pParent = NULL);	// standard constructor
    virtual ~CNuvoISPDlg();
    CString	m_sConnect;
    CTabCtrl	m_TabData;
    CDialogHex *pViewer[NUM_VIEW];
    CProgressCtrl	m_Progress;

    WINCTRLID m_CtrlID[NUM_VIEW];
    // virtual BOOL PreCreateWindow(CREATESTRUCT& cs)

    // Temp
    BOOL m_bConnect;
    CButton	m_ButtonConnect;
    void OnButtonBinFile(int idx, TCHAR *szPath = NULL);
    CString	m_sStatus;

    void EnableProgramOption(BOOL bEnable);

protected:
    HICON m_hIcon;
    // Generated message map functions
    //{{AFX_MSG(CNuvoISPDlg)
    void OnOK() {};
    //afx_msg void OnClose();
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    //virtual void OnCancel();
    afx_msg void OnButtonConnect();
    afx_msg void OnButtonLoadFile();
    afx_msg void OnButtonStart();
    afx_msg void OnSelchangeTabData(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnDropFiles(HDROP hDropInfo);
    afx_msg void OnButtonConfig();
    afx_msg void OnPaint();
    afx_msg void OnKillfocusEditAPRomOffset();
    //}}AFX_MSG
    virtual void DoDataExchange(CDataExchange *pDX);	// DDX/DDV support
    virtual LRESULT WindowProc(UINT message, WPARAM wParam, LPARAM lParam);
    //afx_msg BOOL OnDeviceChange(UINT nEventType, DWORD dwData);
    afx_msg LRESULT  OnDeviceChange(WPARAM  nEventType, LPARAM  dwData);
    DECLARE_MESSAGE_MAP()

    void ShowChipInfo_OffLine(void);
    void ShowChipInfo_OnLine(void);
    void ShowChipInfo_NUC505(void);
    void ShowChipInfo_M2351(void);
    void ChangeBtnText(int nBtn, LPTSTR pszText);

public:
    afx_msg HBRUSH OnCtlColor(CDC *pDC, CWnd *pWnd, UINT nCtlColor);
    void UpdateAddrOffset();

private:
    void RegisterNotification();
    void UnregisterNotification();
    HDEVNOTIFY m_hNotifyDevNode;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(_DLG_NUVOISP_H_)
