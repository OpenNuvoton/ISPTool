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
    , m_FNSAddr(16, 8)
{
    //{{AFX_DATA_INIT(CDialogChipSetting_NSCBA)
    // NOTE: the ClassWizard will add member initialization here
    m_bSecureDebug          = TRUE;
    m_bSupportLock          = TRUE;
    m_bSupportDFMC_NSCBA    = FALSE;
    m_bSupportDFMC_EEPROM   = FALSE;

    m_uFlash_BaseAddr       = NUMICRO_FLASH_APROM_ADDR;
    m_uFlash_Size           = 0;
    m_uFlash_PageSize       = 0;

    m_uFNSAddr              = 0xFFFFFFFF;
    m_uFNSAddr_min          = NUMICRO_FLASH_APROM_ADDR + NUMICRO_FLASH_PAGE_SIZE_8K;

    m_bWrite                = FALSE;
    m_bMirBoundEnable       = FALSE;

    m_bSCRLOCK              = FALSE;
    m_bARLOCK               = FALSE;

    m_uDFlash_BaseAddr      = 0xFFFFFFFF;
    m_uDFlash_Size          = 0;
    m_uDFlash_PageSize      = 0;

    m_bEEPROM_EN            = FALSE;
    m_bEEPROM_SEC           = FALSE;

    m_uConfigValue[0]       = 0xFFFFFFFF;
    m_uConfigValue[1]       = 0xFFFFFFFF;
    m_uConfigValue[2]       = 0xFFFFFFFF;
    m_uConfigValue[3]       = 0xFFFFFFFF;
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
    DDX_Check(pDX, IDC_CHECK_DFMC_EEPROM,           m_bEEPROM_EN);
    DDX_Check(pDX, IDC_CHECK_DFMC_EEPROM_SEC,       m_bEEPROM_SEC);
    DDX_Control(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,   m_FNSAddr);
    DDX_Text(pDX, IDC_EDIT_FLASH_BASE_ADDRESS,      m_sFNSAddr);
    DDX_Text(pDX, IDC_EDIT_DFMC_NSCBA,              m_sDFNSAddr);
    //}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CDialogChipSetting_NSCBA_LOCK, CDialog)
    //{{AFX_MSG_MAP(CDialogChipSetting_NSCBA)
    ON_BN_CLICKED(IDC_CHECK_DATA_FLASH_ENABLE,      OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_MIRROR_BOUNDARY,        OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITYBOOT_LOCK,      OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_SECURITY_LOCK,          OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_DFMC_EEPROM,            OnCheckClick)
    ON_BN_CLICKED(IDC_CHECK_DFMC_EEPROM_SEC,        OnCheckClick)
    ON_EN_KILLFOCUS(IDC_EDIT_FLASH_BASE_ADDRESS,    OnKillfocusEditNSCBA)
    ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_DFMC_NSCBA,    OnDeltaposSpinDFNSCBA)
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

    if (!m_bSupportDFMC_NSCBA)
    {
        GetDlgItem(IDC_GROUP_DFMC_CONFIG)->ShowWindow(SW_HIDE);	
        GetDlgItem(IDC_CHECK_DFMC_EEPROM)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_CHECK_DFMC_EEPROM_SEC)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_DFMC_NSCBA)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_EDIT_DFMC_NSCBA)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_SPIN_DFMC_NSCBA)->ShowWindow(SW_HIDE);
    }

    {
        GetDlgItem(IDC_GROUP_CONFIG_VALUE)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_11)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_12)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_13)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_14)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_VALUE_11)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_VALUE_12)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_VALUE_13)->ShowWindow(SW_HIDE);
        GetDlgItem(IDC_STATIC_CONFIG_VALUE_14)->ShowWindow(SW_HIDE);
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

void CDialogChipSetting_NSCBA_LOCK::ConfigToGUI()
{
    m_uFNSAddr &= ~(m_uFlash_PageSize - 1);

    if (m_bSecureDebug && m_bWrite)
    {
        unsigned int uBoundary = (!m_bMirBoundEnable)? m_uFlash_Size : m_uFlash_Size/2;

        uBoundary += m_uFlash_BaseAddr;

        if (m_uFNSAddr > (uBoundary - m_uFlash_PageSize))
            m_uFNSAddr = (uBoundary - m_uFlash_PageSize);

        if (m_uFNSAddr < m_uFNSAddr_min)
            m_uFNSAddr = m_uFNSAddr_min;
    }

    m_FNSAddr.EnableWindow(m_bSecureDebug && m_bWrite);
    ((CButton*)GetDlgItem(IDC_STATIC_FLASH_BASE_ADDRESS))->EnableWindow(m_bSecureDebug && m_bWrite);
    ((CButton*)GetDlgItem(IDC_CHECK_MIRROR_BOUNDARY))->EnableWindow(m_bSecureDebug && m_bWrite);

    unsigned int uOffset = (m_bSecureDebug) ? 0 : NUMICRO_NS_OFFSET;

    m_sFNSAddr.Format(_T("%08X"), m_uFNSAddr | uOffset);

    m_bEEPROM_EN    = ((m_uConfigValue[3] & M3351_FLASH_CONFIG_DFMC_EE) == 0) ? TRUE : FALSE;
    m_bEEPROM_SEC   = ((m_uConfigValue[3] & M3351_FLASH_CONFIG_DFMC_EE_SEC) == 0) ? TRUE : FALSE;

    unsigned int uDFNS_Addr;

    uDFNS_Addr = m_uDFlash_BaseAddr + (((m_uConfigValue[3] & 0x0F) + 1) * m_uDFlash_PageSize);

    m_sDFNSAddr.Format(_T("%X"), uDFNS_Addr);

    UpdateData(FALSE);
}

void CDialogChipSetting_NSCBA_LOCK::GUIToConfig()
{
    UpdateData(TRUE);

    m_uFNSAddr = ::_tcstoul(m_sFNSAddr, NULL, 16);
    m_uFNSAddr &= ~NUMICRO_NS_OFFSET;

    if (m_bEEPROM_SEC)
        m_uConfigValue[3] &= ~M3351_FLASH_CONFIG_DFMC_EE_SEC;
    else
        m_uConfigValue[3] |=  M3351_FLASH_CONFIG_DFMC_EE_SEC;

    if (m_bEEPROM_EN)
        m_uConfigValue[3] &= ~M3351_FLASH_CONFIG_DFMC_EE;
    else
        m_uConfigValue[3] |=  M3351_FLASH_CONFIG_DFMC_EE;

    if (!(m_uConfigValue[3] & M3351_FLASH_CONFIG_DFMC_EE))
    {
        unsigned int uDFlash_Size;

        uDFlash_Size = ((m_uConfigValue[3] & 0x0F) + 1) * m_uDFlash_PageSize;

        if (uDFlash_Size > (m_uDFlash_Size - (m_uDFlash_PageSize * 2)))
            uDFlash_Size = (m_uDFlash_Size - (m_uDFlash_PageSize * 2));

        m_uConfigValue[3] = (m_uConfigValue[3] & ~0x0F) | ((uDFlash_Size / m_uDFlash_PageSize) - 1);
    }
}

void CDialogChipSetting_NSCBA_LOCK::OnCheckClick()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_NSCBA_LOCK::OnKillfocusEditNSCBA()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogChipSetting_NSCBA_LOCK::OnDeltaposSpinDFNSCBA(NMHDR *pNMHDR, LRESULT *pResult)
{
    unsigned int uDFlash_Size = m_uDFlash_Size;

    if (!(m_uConfigValue[3] & M3351_FLASH_CONFIG_DFMC_EE))
    {
        uDFlash_Size -= (m_uDFlash_PageSize * 2);
    }

    unsigned int uDFlashNS_Addr;

    uDFlashNS_Addr = m_uDFlash_BaseAddr + (((m_uConfigValue[3] & 0x0F) + 1) * m_uDFlash_PageSize);

    LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);

    if (pNMUpDown->iDelta == 1)
    {
        uDFlashNS_Addr -= m_uDFlash_PageSize;
    }
    else if (pNMUpDown->iDelta == -1)
    {
        uDFlashNS_Addr += m_uDFlash_PageSize;
    }

    if (uDFlashNS_Addr < (m_uDFlash_BaseAddr + m_uDFlash_PageSize))
        uDFlashNS_Addr = (m_uDFlash_BaseAddr + m_uDFlash_PageSize);

    if (uDFlashNS_Addr > (m_uDFlash_BaseAddr + uDFlash_Size))
        uDFlashNS_Addr = (m_uDFlash_BaseAddr + uDFlash_Size);

    m_uConfigValue[3] = (m_uConfigValue[3] & ~0x0F) | (((uDFlashNS_Addr - m_uDFlash_BaseAddr) / m_uDFlash_PageSize) - 1);

    ConfigToGUI();

    *pResult = 0;
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
