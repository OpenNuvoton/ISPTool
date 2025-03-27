// DialogChipSetting_SECURE_M2351.cpp : implementation file
//
#include "stdafx.h"
#include "ChipDefs.h"
#include "DialogChipSetting_SECURE_M2351.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_SECURE_M2351 dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_SECURE_M2351, CDialog)

CDialogChipSetting_SECURE_M2351::CDialogChipSetting_SECURE_M2351(CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_SECURE_M2351::IDD, pParent)
    , m_NSecure_Addr(16, 8)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_SECURE_M2351)
    // NOTE: the ClassWizard will add member initialization here
    m_bNSAddr_Write = FALSE;
    m_bSecure_Lock  = FALSE;
    m_bAll_Lock     = FALSE;

    m_uNSecure_Addr = M2351_MAX_APROM_SIZE;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_SECURE_M2351::~CDialogChipSetting_SECURE_M2351()
{
}

void CDialogChipSetting_SECURE_M2351::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_SECURE_M2351)
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE,     m_bNSAddr_Write);
    DDX_Check(pDX, IDC_CHECK_SECURITYBOOT_LOCK,     m_bSecure_Lock);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK,         m_bAll_Lock);
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,   m_NSecure_Addr);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,      m_sNSecure_Addr);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_SECURE_M2351, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_SECURE_M2351)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,      OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITYBOOT_LOCK,      OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK,          OnCheckClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,    OnKillfocusEditNSCBA)
    //ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_SECURE_M2351 message handlers

BOOL CDialogChipSetting_SECURE_M2351::OnInitDialog()
{
    CDialog::OnInitDialog();

    if (!m_bSecureDebug)
    {
        m_bNSAddr_Write = TRUE;
        m_bSecure_Lock  = TRUE;
        m_NSecure_Addr.EnableWindow(FALSE);
        ((CButton*)GetDlgItem(IDC_CHECK_DATA_FLASH_ENABLE))->EnableWindow(FALSE);
        ((CButton*)GetDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS))->EnableWindow(FALSE);
        ((CButton*)GetDlgItem(IDC_CHECK_SECURITYBOOT_LOCK))->EnableWindow(FALSE);
    }

    m_uNSecure_Addr_restore = m_uNSecure_Addr;

    if (!m_bNSecureArea)
    {
        ((CButton*)GetDlgItem(IDC_GROUP_NSCBA))->ShowWindow(SW_HIDE);
        ((CButton*)GetDlgItem(IDC_CHECK_DATA_FLASH_ENABLE))->ShowWindow(SW_HIDE);
        ((CButton*)GetDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS))->ShowWindow(SW_HIDE);
        m_NSecure_Addr.ShowWindow(SW_HIDE);

        RECT rcTmp, rcGroupNSCBA, rcGroupLock;
        GetDlgItem(IDC_GROUP_NSCBA)->GetWindowRect(&rcGroupNSCBA);
        GetDlgItem(IDC_GROUP_TWO_LEVEL_LOCK)->GetWindowRect(&rcGroupLock);
        LONG lDiff = rcGroupLock.top - rcGroupNSCBA.top;

        int nIDs[] = {IDC_GROUP_TWO_LEVEL_LOCK, IDC_CHECK_SECURITYBOOT_LOCK, IDC_CHECK_SECURITY_LOCK};

        for (int i = 0; i < _countof(nIDs); i++)
        {
            GetDlgItem(nIDs[i])->GetWindowRect(&rcTmp);
            this->ScreenToClient(&rcTmp);
            GetDlgItem(nIDs[i])->SetWindowPos(NULL, rcTmp.left, rcTmp.top - lDiff, 0, 0, SWP_NOZORDER | SWP_NOSIZE);
        }

        ((CButton*)GetDlgItem(IDC_CHECK_SECURITYBOOT_LOCK))->ShowWindow(SW_HIDE);
    }

    ConfigToGUI();

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;    // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_SECURE_M2351::OnCheckClick()
{
    // TODO: Add your control notification handler code here
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_SECURE_M2351::ConfigToGUI()
{
    if (m_bSecureDebug)
    {
        m_NSecure_Addr.EnableWindow(m_bNSAddr_Write);
    }

    unsigned int uOffset = (m_bSecureDebug) ? 0 : NUMICRO_NS_OFFSET;

    m_sNSecure_Addr.Format(_T("%08X"), m_uNSecure_Addr | uOffset);

    UpdateData(FALSE);
}

void CDialogChipSetting_SECURE_M2351::GUIToConfig()
{
    UpdateData(TRUE);
}

void CDialogChipSetting_SECURE_M2351::OnKillfocusEditNSCBA()
{
    UpdateData(TRUE);

    m_uNSecure_Addr = ::_tcstoul(m_sNSecure_Addr, NULL, 16);

    if (m_uNSecure_Addr < m_uProgramMemorySize)
    {
        m_uNSecure_Addr &= ~(M2351_FLASH_PAGE_SIZE - 1);

        if (m_uNSecure_Addr < 0x4000)
            m_uNSecure_Addr = 0x4000;
    }
    else
    {
        m_uNSecure_Addr = m_uNSecure_Addr_restore;
        m_bNSAddr_Write = FALSE;
        m_NSecure_Addr.EnableWindow(FALSE);
    }

    m_sNSecure_Addr.Format(_T("%08X"), m_uNSecure_Addr);

    UpdateData(FALSE);
}

void CDialogChipSetting_SECURE_M2351::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK ();
}

void CDialogChipSetting_SECURE_M2351::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel ();
}


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_NSCBA dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_NSCBA, CDialog)

CDialogChipSetting_NSCBA::CDialogChipSetting_NSCBA(CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_NSCBA::IDD, pParent)
    , m_NSAddr(16, 8)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_NSCBA)
    // NOTE: the ClassWizard will add member initialization here
    m_bNSAddrWrite  = FALSE;
    m_bMirrorEnable = FALSE;

    m_uNSAddr       = M2354_MAX_APROM_SIZE;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_NSCBA::~CDialogChipSetting_NSCBA()
{
}

void CDialogChipSetting_NSCBA::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_NSCBA)
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE,     m_bNSAddrWrite);
    DDX_Check(pDX, IDC_CHECK_MIRROR_BOUNDARY,       m_bMirrorEnable);
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,   m_NSAddr);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,      m_sNSAddr);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_NSCBA, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_NSCBA)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,      OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_MIRROR_BOUNDARY,        OnCheckClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,    OnKillfocusEditNSCBA)
    //ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_NSCBA message handlers

BOOL CDialogChipSetting_NSCBA::OnInitDialog()
{
    CDialog::OnInitDialog();

    if (!m_bSecureDebug)
    {
        m_bNSAddrWrite = TRUE;
    }

    ConfigToGUI();

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;    // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_NSCBA::OnCheckClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_NSCBA::ConfigToGUI()
{
    m_uNSAddr &= ~(m_uFlashPageSize - 1);

    if (m_bSecureDebug && m_bNSAddrWrite)
    {
        unsigned int uBoundary = (!m_bMirrorEnable) ? m_uProgramMemorySize : m_uProgramMemorySize / 2;

        uBoundary += m_uFlashBaseAddr;

        if (m_uNSAddr > (uBoundary - m_uFlashPageSize))
            m_uNSAddr = (uBoundary - m_uFlashPageSize);

        if (m_uNSAddr < (m_uFlashBaseAddr + m_uFlashPageSize))
            m_uNSAddr = (m_uFlashBaseAddr + m_uFlashPageSize);
    }

    m_NSAddr.EnableWindow(m_bSecureDebug && m_bNSAddrWrite);
    ((CButton*)GetDlgItem(IDC_CHECK_DATA_FLASH_ENABLE))->EnableWindow(m_bSecureDebug && m_bCanWrite);
    ((CButton*)GetDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS))->EnableWindow(m_bSecureDebug && m_bNSAddrWrite);
    ((CButton*)GetDlgItem(IDC_CHECK_MIRROR_BOUNDARY))->EnableWindow(m_bSecureDebug && m_bNSAddrWrite);

    unsigned int uOffset = (m_bSecureDebug) ? 0 : NUMICRO_NS_OFFSET;

    m_sNSAddr.Format(_T("%08X"), m_uNSAddr | uOffset);

    UpdateData(FALSE);
}

void CDialogChipSetting_NSCBA::GUIToConfig()
{
    UpdateData(TRUE);

    m_uNSAddr = ::_tcstoul(m_sNSAddr, NULL, 16);
    m_uNSAddr &= ~NUMICRO_NS_OFFSET;
}

void CDialogChipSetting_NSCBA::OnKillfocusEditNSCBA()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_NSCBA::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK ();
}

void CDialogChipSetting_NSCBA::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel ();
}


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_NSCBA_LOCK dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_NSCBA_LOCK, CDialog)

CDialogChipSetting_NSCBA_LOCK::CDialogChipSetting_NSCBA_LOCK(CWnd* pParent /*=NULL*/)
    : CDialogResize(CDialogChipSetting_NSCBA_LOCK::IDD, pParent)
    , m_NSAddr(16, 8)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_NSCBA)
    // NOTE: the ClassWizard will add member initialization here
    m_bSecureDebug      = TRUE;
    m_bSupportLock      = TRUE;

    m_bWrite            = FALSE;
    m_bMirBoundEnable   = FALSE;

    m_uNSAddr           = 0xFFFFFFFF;

    m_bSCRLOCK          = FALSE;
    m_bARLOCK           = FALSE;
    //}}AFX_DATA_INIT
}

CDialogChipSetting_NSCBA_LOCK::~CDialogChipSetting_NSCBA_LOCK()
{
}

void CDialogChipSetting_NSCBA_LOCK::DoDataExchange(CDataExchange* pDX)
{
    CDialogResize::DoDataExchange(pDX);
    //{{AFX_DATA_MAP(CDialogChipSetting_NSCBA_LOCK)
    DDX_Check(pDX, IDC_CHECK_DATA_FLASH_ENABLE,     m_bWrite);
    DDX_Check(pDX, IDC_CHECK_MIRROR_BOUNDARY,       m_bMirBoundEnable);
    DDX_Check(pDX, IDC_CHECK_SECURITYBOOT_LOCK,     m_bSCRLOCK);
    DDX_Check(pDX, IDC_CHECK_SECURITY_LOCK,         m_bARLOCK);
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,   m_NSAddr);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,      m_sNSAddr);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_NSCBA_LOCK, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_NSCBA)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,      OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_MIRROR_BOUNDARY,        OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITYBOOT_LOCK,      OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK,          OnCheckClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,    OnKillfocusEditNSCBA)
    //ON_WM_SIZE()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_NSCBA_LOCK message handlers

BOOL CDialogChipSetting_NSCBA_LOCK::OnInitDialog()
{
    CDialog::OnInitDialog();

    if (!m_bSecureDebug)
    {
        m_bWrite    = TRUE;
        m_bSCRLOCK  = TRUE;

        GetDlgItem(IDC_CHECK_DATA_FLASH_ENABLE)->EnableWindow(FALSE);
        GetDlgItem(IDC_CHECK_SECURITYBOOT_LOCK)->EnableWindow(FALSE);
    }

    if (!m_bSupportLock)
    {
        GetDlgItem(IDC_GROUP_TWO_LEVEL_LOCK)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_CHECK_SECURITYBOOT_LOCK)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_CHECK_SECURITY_LOCK)->ShowWindow(SW_HIDE);
    }

    {
        GetDlgItem(IDC_GROUP_CONFIG_VALUE)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_11)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_12)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_13)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_VALUE_11)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_VALUE_12)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_VALUE_13)->ShowWindow(SW_HIDE);
    }

    ConfigToGUI();

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;    // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_NSCBA_LOCK::UpdateConfigString()
{
}

void CDialogChipSetting_NSCBA_LOCK::OnCheckClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_NSCBA_LOCK::ConfigToGUI()
{
    m_uNSAddr &= ~(m_uFlashPageSize - 1);

    if (m_bSecureDebug && m_bWrite)
    {
        unsigned int uBoundary = (!m_bMirBoundEnable) ? m_uProgramMemorySize : m_uProgramMemorySize / 2;

        uBoundary += m_uFlashBaseAddr;

        if (m_uNSAddr > (uBoundary - m_uFlashPageSize))
            m_uNSAddr = (uBoundary - m_uFlashPageSize);

        if (m_uNSAddr < m_uNSAddr_min)
            m_uNSAddr = m_uNSAddr_min;
    }

    m_NSAddr.EnableWindow(m_bSecureDebug && m_bWrite);
    ((CButton*)GetDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS))->EnableWindow(m_bSecureDebug && m_bWrite);
    ((CButton*)GetDlgItem(IDC_CHECK_MIRROR_BOUNDARY))->EnableWindow(m_bSecureDebug && m_bWrite);

    unsigned int uOffset = (m_bSecureDebug) ? 0 : NUMICRO_NS_OFFSET;

    m_sNSAddr.Format(_T("%08X"), m_uNSAddr | uOffset);

    UpdateData(FALSE);
}

void CDialogChipSetting_NSCBA_LOCK::GUIToConfig()
{
    UpdateData(TRUE);

    m_uNSAddr = ::_tcstoul(m_sNSAddr, NULL, 16);
    m_uNSAddr &= ~NUMICRO_NS_OFFSET;
}

void CDialogChipSetting_NSCBA_LOCK::OnKillfocusEditNSCBA()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_NSCBA_LOCK::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK ();
}

void CDialogChipSetting_NSCBA_LOCK::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel ();
}
