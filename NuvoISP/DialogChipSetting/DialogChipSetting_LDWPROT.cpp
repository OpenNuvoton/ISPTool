// DialogChipSetting_LDWPROT.cpp : implementation file
//

#include "stdafx.h"
#include "ChipDefs.h"
#include "Lang.h"
#include "DialogChipSetting_LDWPROT.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_LDWPROT dialog

IMPLEMENT_DYNAMIC(CDialogChipSetting_LDWPROT, CDialog)

CDialogChipSetting_LDWPROT::CDialogChipSetting_LDWPROT(CWnd* pParent /*=NULL*/)
	: CDialogResize(IDD, pParent)
	, m_uLDROM_Addr(NUMICRO_FLASH_LDROM_ADDR + NUMICRO_SPECIAL_FLASH_OFFSET)
	, m_uLDROM_Size(NUMICRO_FLASH_LDROM_SIZE_8K)
	, m_uLDROM_RegionSize(M3331_LDWPROT_REGION_SIZE)
	, m_uDFLASH_Addr(M3331_FLASH_DFLASH_ADDR)
	, m_uDFLASH_Size(M3331_FLASH_DFLASH_SIZE)
	, m_uDFLASH_RegionSize(M3331_DFWPROT_REGION_SIZE)
{
	//{{AFX_DATA_INIT(CDialogChipSetting_LDWPROT)
	m_nRadioLDWPLVL		= -1;

	m_bCheckLDPROEN[0]	= FALSE;
	m_bCheckLDPROEN[1]	= FALSE;

	m_bCheckDFPROEN[0]	= FALSE;
	m_bCheckDFPROEN[1]	= FALSE;
	m_bCheckDFPROEN[2]	= FALSE;
	m_bCheckDFPROEN[3]	= FALSE;

	m_sConfigValue16	= _T("");
	m_sConfigValue17	= _T("");
	m_sConfigValue18	= _T("");
	//}}AFX_DATA_INIT
}

CDialogChipSetting_LDWPROT::~CDialogChipSetting_LDWPROT()
{
}

void CDialogChipSetting_LDWPROT::DoDataExchange(CDataExchange* pDX)
{
	CDialogResize::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDialogChipSetting_LDWPROT)
	DDX_Radio(pDX, IDC_RADIO_LDROM_WPROT_LEVEL0,	m_nRadioLDWPLVL);

	DDX_Check(pDX, IDC_CHECK_LDROM_WPROT_0,			m_bCheckLDPROEN[0]);
	DDX_Check(pDX, IDC_CHECK_LDROM_WPROT_1,			m_bCheckLDPROEN[1]);
	DDX_Check(pDX, IDC_CHECK_DFLASH_WPROT_0,		m_bCheckDFPROEN[0]);
	DDX_Check(pDX, IDC_CHECK_DFLASH_WPROT_1,		m_bCheckDFPROEN[1]);
	DDX_Check(pDX, IDC_CHECK_DFLASH_WPROT_2,		m_bCheckDFPROEN[2]);
	DDX_Check(pDX, IDC_CHECK_DFLASH_WPROT_3,		m_bCheckDFPROEN[3]);

	DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_16,		m_sConfigValue16);
	DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_17,		m_sConfigValue17);
	DDX_Text(pDX, IDC_STATIC_CONFIG_VALUE_18,		m_sConfigValue18);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDialogChipSetting_LDWPROT, CDialog)
	//{{AFX_MSG_MAP(CDialogChipSetting_LDWPROT)
	ON_BN_CLICKED(IDC_RADIO_LDROM_WPROT_LEVEL0,		OnRadioClick)
	ON_BN_CLICKED(IDC_RADIO_LDROM_WPROT_LEVEL1,		OnRadioClick)

	ON_BN_CLICKED(IDC_CHECK_LDROM_WPROT_0,			OnCheckClick)
	ON_BN_CLICKED(IDC_CHECK_LDROM_WPROT_1,			OnCheckClick)
	ON_BN_CLICKED(IDC_CHECK_DFLASH_WPROT_0,			OnCheckClick)
	ON_BN_CLICKED(IDC_CHECK_DFLASH_WPROT_1,			OnCheckClick)
	ON_BN_CLICKED(IDC_CHECK_DFLASH_WPROT_2,			OnCheckClick)
	ON_BN_CLICKED(IDC_CHECK_DFLASH_WPROT_3,			OnCheckClick)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CDialogChipSetting_LDWPROT message handlers

BOOL CDialogChipSetting_LDWPROT::OnInitDialog() 
{
	CDialog::OnInitDialog();

	unsigned int i;
	CString str;

	// LDROM
	for (i = 0; i < 2; i++)
	{
	str.Format(_I(IDS_WPROT_REGION), i, m_uLDROM_Addr + (m_uLDROM_RegionSize * i), (m_uLDROM_Addr + (m_uLDROM_RegionSize * (i + 1))) - 1);

	GetDlgItem(IDC_CHECK_LDROM_WPROT_0 + i)->SetWindowText(str);
	}

	for (i = (m_uLDROM_Size / m_uLDROM_RegionSize); i < 2; i++)
	{
		GetDlgItem(IDC_CHECK_LDROM_WPROT_0 + i)->ShowWindow(SW_HIDE);
	}

	// Data Flash
	for (i = 0; i < 4; i++)
	{
		str.Format(_I(IDS_WPROT_REGION), i, m_uDFLASH_Addr + (m_uDFLASH_RegionSize * i), (m_uDFLASH_Addr + (m_uDFLASH_RegionSize * (i + 1))) - 1);

		GetDlgItem(IDC_CHECK_DFLASH_WPROT_0 + i)->SetWindowText(str);
	}

	for (i = (m_uDFLASH_Size / m_uDFLASH_RegionSize); i < 4; i++)
	{
		GetDlgItem(IDC_CHECK_DFLASH_WPROT_0 + i)->ShowWindow(SW_HIDE);
	}

	UpdateUI();

	ConfigToGUI();

	m_bIsInitialized = true;
	GetWindowRect(m_rect);
	AdjustDPI();

	return TRUE;	// return TRUE unless you set the focus to a control
					// EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogChipSetting_LDWPROT::UpdateUI()
{
}

void CDialogChipSetting_LDWPROT::ConfigToGUI()
{
	m_uConfigValue_t[0] = m_uConfigValue[0];
	m_uConfigValue_t[1] = m_uConfigValue[1];
	m_uConfigValue_t[2] = m_uConfigValue[2];

	unsigned int i;

	// LDROM
	for (i = 0; i < 2; i++)
	{
		m_bCheckLDPROEN[i] = ((m_uConfigValue_t[0] & (1 << i)) == 0) ? TRUE : FALSE;
	}

	switch(m_uConfigValue_t[1] & 0xFFFF)
	{
		case 0xFFFF:
			m_nRadioLDWPLVL = 0;
			break;
		default:
			m_nRadioLDWPLVL = 1;
	}

	// Data Flash
	for (i = 0; i < 4; i++)
	{
		m_bCheckDFPROEN[i] = ((m_uConfigValue_t[2] & (1 << i)) == 0) ? TRUE : FALSE;
	}

	m_sConfigValue16.Format(_T("0x%08X"), m_uConfigValue_t[0]);
	m_sConfigValue17.Format(_T("0x%08X"), m_uConfigValue_t[1]);
	m_sConfigValue18.Format(_T("0x%08X"), m_uConfigValue_t[2]);

	UpdateData(FALSE);
}

void CDialogChipSetting_LDWPROT::GUIToConfig()
{
	UpdateData(TRUE);

	m_uConfigValue_t[0] = m_uConfigValue[0];
	m_uConfigValue_t[1] = m_uConfigValue[1];
	m_uConfigValue_t[2] = m_uConfigValue[2];

	unsigned int i;

	// LDROM
	for (i = 0; i < 2; i++)
	{
		if (m_bCheckLDPROEN[i])
			m_uConfigValue_t[0] &= ~(1 << i);
	else
			m_uConfigValue_t[0] |=  (1 << i);
	}

	m_uConfigValue_t[1] &= ~0xFFFF0000;

	switch(m_nRadioLDWPLVL)
	{
		case 0:
			m_uConfigValue_t[1] = 0xFFFF;
			break;
		case 1:
		default:
			if (m_uConfigValue_t[1] == 0xFFFF)
				m_uConfigValue_t[1]  = 0x005A;
	}

	m_uConfigValue_t[1] |= 0xFFFF0000;

	// Data Flash
	for (i = 0; i < 4; i++)
	{
		if (m_bCheckDFPROEN[i])
			m_uConfigValue_t[2] &= ~(1 << i);
		else
			m_uConfigValue_t[2] |=  (1 << i);
	}

	m_uConfigValue[0] = m_uConfigValue_t[0];
	m_uConfigValue[1] = m_uConfigValue_t[1];
	m_uConfigValue[2] = m_uConfigValue_t[2];
}

void CDialogChipSetting_LDWPROT::OnRadioClick() 
{
	GUIToConfig();
	ConfigToGUI();
}

void CDialogChipSetting_LDWPROT::OnCheckClick() 
{
	GUIToConfig();
	ConfigToGUI();
}

void CDialogChipSetting_LDWPROT::OnOK()
{
	GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
	//CDialog::OnOK();
}

void CDialogChipSetting_LDWPROT::OnCancel()
{
	GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
	//CDialog::OnCancel();
}
