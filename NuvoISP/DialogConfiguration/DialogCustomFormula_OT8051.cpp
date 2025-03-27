// DialogCustomFormula_OT8051.cpp : implementation file
//
#include "stdafx.h"
#include "DialogCustomFormula_OT8051.h"
#include "Lang.h"


/////////////////////////////////////////////////////////////////////////////
// CDialogCustomFormula_OT8051 dialog

IMPLEMENT_DYNAMIC(CDialogCustomFormula_OT8051, CDialog)

CDialogCustomFormula_OT8051::CDialogCustomFormula_OT8051(unsigned int uProgramMemorySize, CWnd *pParent /*=NULL*/)
    : CDialogResize(CDialogCustomFormula_OT8051::IDD, pParent)
    , m_uProgramMemorySize(uProgramMemorySize)
{
}

CDialogCustomFormula_OT8051::~CDialogCustomFormula_OT8051()
{
}

void CDialogCustomFormula_OT8051::DoDataExchange(CDataExchange* pDX)
{
    CDialog::DoDataExchange(pDX);

    DDX_Control(pDX, IDC_COMBO_FORMULA_OPERATOR_0,  m_combobox_operator[0]);
    DDX_Control(pDX, IDC_COMBO_FORMULA_OPERATOR_1,  m_combobox_operator[1]);
    DDX_Control(pDX, IDC_COMBO_FORMULA_OPERATOR_2,  m_combobox_operator[2]);

    DDX_Control(pDX, IDC_EDIT_FORMULA_VALUE_0,      m_Formula_Value[0]);
    DDX_Control(pDX, IDC_EDIT_FORMULA_VALUE_1,      m_Formula_Value[1]);
    DDX_Control(pDX, IDC_EDIT_FORMULA_VALUE_2,      m_Formula_Value[2]);

    DDX_Control(pDX, IDC_EDIT_ADDRESS_0,            m_Formula_Addr[0]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_1,            m_Formula_Addr[1]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_2,            m_Formula_Addr[2]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_3,            m_Formula_Addr[3]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_4,            m_Formula_Addr[4]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_5,            m_Formula_Addr[5]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_6,            m_Formula_Addr[6]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_7,            m_Formula_Addr[7]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_8,            m_Formula_Addr[8]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_9,            m_Formula_Addr[9]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_A,            m_Formula_Addr[10]);
    DDX_Control(pDX, IDC_EDIT_ADDRESS_B,            m_Formula_Addr[11]);

    DDX_Check(pDX, IDC_CHECK_ADDRESS_0,             m_bCheckAddr[0]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_1,             m_bCheckAddr[1]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_2,             m_bCheckAddr[2]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_3,             m_bCheckAddr[3]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_4,             m_bCheckAddr[4]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_5,             m_bCheckAddr[5]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_6,             m_bCheckAddr[6]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_7,             m_bCheckAddr[7]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_8,             m_bCheckAddr[8]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_9,             m_bCheckAddr[9]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_A,             m_bCheckAddr[10]);
    DDX_Check(pDX, IDC_CHECK_ADDRESS_B,             m_bCheckAddr[11]);

    DDX_Text(pDX, IDC_EDIT_FORMULA_VALUE_0,         m_sFormula_Value[0]);
    DDX_Text(pDX, IDC_EDIT_FORMULA_VALUE_1,         m_sFormula_Value[1]);
    DDX_Text(pDX, IDC_EDIT_FORMULA_VALUE_2,         m_sFormula_Value[2]);

    DDX_Text(pDX, IDC_EDIT_ADDRESS_0,               m_sFormula_Addr[0]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_1,               m_sFormula_Addr[1]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_2,               m_sFormula_Addr[2]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_3,               m_sFormula_Addr[3]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_4,               m_sFormula_Addr[4]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_5,               m_sFormula_Addr[5]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_6,               m_sFormula_Addr[6]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_7,               m_sFormula_Addr[7]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_8,               m_sFormula_Addr[8]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_9,               m_sFormula_Addr[9]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_A,               m_sFormula_Addr[10]);
    DDX_Text(pDX, IDC_EDIT_ADDRESS_B,               m_sFormula_Addr[11]);

    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_0,          m_sFormula_Result[0]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_1,          m_sFormula_Result[1]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_2,          m_sFormula_Result[2]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_3,          m_sFormula_Result[3]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_4,          m_sFormula_Result[4]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_5,          m_sFormula_Result[5]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_6,          m_sFormula_Result[6]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_7,          m_sFormula_Result[7]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_8,          m_sFormula_Result[8]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_9,          m_sFormula_Result[9]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_A,          m_sFormula_Result[10]);
    DDX_Text(pDX, IDC_STATIC_DATA_VALUE_B,          m_sFormula_Result[11]);
}


BEGIN_MESSAGE_MAP(CDialogCustomFormula_OT8051, CDialog)
    //{{AFX_MSG_MAP(CDialogCustomFormula_OT8051)
    ON_CBN_SELCHANGE(IDC_COMBO_FORMULA_OPERATOR_0, OnChangeComboOperator)
    ON_CBN_SELCHANGE(IDC_COMBO_FORMULA_OPERATOR_1, OnChangeComboOperator)
    ON_CBN_SELCHANGE(IDC_COMBO_FORMULA_OPERATOR_2, OnChangeComboOperator)
    ON_CONTROL_RANGE(BN_CLICKED, IDC_CHECK_ADDRESS_0, IDC_CHECK_ADDRESS_B, OnCheckClick)
    ON_CONTROL_RANGE(EN_KILLFOCUS, IDC_EDIT_FORMULA_VALUE_0, IDC_EDIT_FORMULA_VALUE_2, OnKillfocusEditValue)
    ON_CONTROL_RANGE(EN_KILLFOCUS, IDC_EDIT_ADDRESS_0, IDC_EDIT_ADDRESS_B, OnKillfocusEditAddr)
    ON_WM_CTLCOLOR()
    ON_WM_VSCROLL()
    ON_WM_HSCROLL()
    ON_WM_GETMINMAXINFO()
    ON_WM_MOUSEWHEEL()
    //}}AFX_MSG_MAP
END_MESSAGE_MAP()


// CDialogCustomFormula_OT8051 message handlers

BOOL CDialogCustomFormula_OT8051::OnInitDialog()
{
    CDialog::OnInitDialog();

    int i;

    for (i = 0; i < 3; i++)
    {
        m_combobox_operator[i].AddString(_T("+"));
        m_combobox_operator[i].AddString(_T("-"));
        m_combobox_operator[i].AddString(_T("|"));
        m_combobox_operator[i].AddString(_T("&"));
        m_combobox_operator[i].AddString(_T("^"));
    }

    for (i = 0; i < 3; i++)
        m_Formula_Value[i].SetLimitText(2);

    for (i = 0; i < 12; i++)
        m_Formula_Addr[i].SetLimitText(4);

    ConfigToGUI();

    m_bIsInitialized = true;
    GetWindowRect(m_rect);
    AdjustDPI();

    return TRUE;  // return TRUE unless you set the focus to a control
    // EXCEPTION: OCX Property Pages should return FALSE
}

void CDialogCustomFormula_OT8051::OnChangeComboOperator()
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogCustomFormula_OT8051::OnCheckClick(UINT nID)
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogCustomFormula_OT8051::OnKillfocusEditValue(UINT nID)
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogCustomFormula_OT8051::OnKillfocusEditAddr(UINT nID)
{
    GUIToConfig();
    ConfigToGUI();
}

void CDialogCustomFormula_OT8051::GUIToConfig()
{
    UpdateData(TRUE);

    int i, nSelect;

    m_uWriteFlag = 0;

    for (i = 0; i < 3; i++)
    {
        m_ucValue[i] = (unsigned char)::_tcstoul(m_sFormula_Value[i], NULL, 16);

        nSelect = m_combobox_operator[i].GetCurSel();

        m_ucOperator[i] = nSelect;
    }

    for (i = 0; i < 12; i++)
    {
        m_uAddr[i] = ::_tcstoul(m_sFormula_Addr[i], NULL, 16);

        if (m_uAddr[i] >= m_uProgramMemorySize)
            m_bCheckAddr[i] = FALSE;

        if (m_bCheckAddr[i])
            m_uWriteFlag |= (1 << i);
    }
}

void CDialogCustomFormula_OT8051::ConfigToGUI()
{
    int i, j, nSelect;

    for (i = 0; i < 3; i++)
    {
        m_sFormula_Value[i].Format(_T("%02X"), m_ucValue[i]);

        nSelect = m_ucOperator[i];

        if (nSelect > 4)
            nSelect = 0;

        m_combobox_operator[i].SetCurSel(nSelect);
    }

    for (i = 0; i < 12; i++)
    {
        m_sFormula_Addr[i].Format(_T("%04X"), m_uAddr[i]);

        if (m_uWriteFlag & (1 << i))
            m_bCheckAddr[i] = TRUE;
        else
            m_bCheckAddr[i] = FALSE;
    }

    for (i = 0; i < 12; i++)
    {
        unsigned char ucResult = m_ucUID[i];

        for (j = 0; j < 3; j++)
        {
            switch (m_ucOperator[j])
            {
                case 0:
                    ucResult += m_ucValue[j];
                    break;

                case 1:
                    ucResult -= m_ucValue[j];
                    break;

                case 2:
                    ucResult |= m_ucValue[j];
                    break;

                case 3:
                    ucResult &= m_ucValue[j];
                    break;

                case 4:
                default:
                    ucResult ^= m_ucValue[j];
            }
        }

        m_sFormula_Result[i].Format(_T("%02X"), ucResult);
    }

    UpdateData(FALSE);
}

HBRUSH CDialogCustomFormula_OT8051::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
    HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);

    // TODO: Change any attributes of the DC here
    if (pWnd->GetDlgCtrlID() == IDC_STATIC_CUSTOM_SN_MSG)
    {
        // Set the text color to red
        pDC->SetTextColor(RGB(255, 0, 0));
    }

    // TODO: Return a different brush if the default is not desired
    return hbr;
}

void CDialogCustomFormula_OT8051::OnOK()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDOK);
    //CDialog::OnOK ();
}

void CDialogCustomFormula_OT8051::OnCancel()
{
    GetParent()->GetParent()->PostMessage(WM_COMMAND, IDCANCEL);
    //CDialog::OnCancel ();
}
