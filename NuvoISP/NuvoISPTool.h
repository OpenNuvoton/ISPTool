
// ISPTool.h : PROJECT_NAME ���ε{�����D�n���Y��
//

#pragma once

#ifndef __AFXWIN_H__
    #error "�� PCH �]�t���ɮ׫e���]�t 'stdafx.h'"
#endif

#include "resource.h"       // �D�n�Ÿ�


// CISPToolApp:
// �аѾ\��@�����O�� ISPTool.cpp
//

class CISPToolApp : public CWinApp
{
protected:
    HINSTANCE m_hLangResouce;
    void SetLangID(LANGID langID);
public:
    CISPToolApp();

    // �мg
public:
    virtual BOOL InitInstance();

    // �{���X��@

    DECLARE_MESSAGE_MAP()
};

extern CISPToolApp theApp;