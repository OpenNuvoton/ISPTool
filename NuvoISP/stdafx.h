
// stdafx.h : �i�b�����Y�ɤ��]�t�зǪ��t�� Include �ɡA
// �άO�g�`�ϥΫo�ܤ��ܧ�
// �M�ױM�� Include �ɮ�

#pragma once

#define _CRT_SECURE_NO_WARNINGS

#ifndef _SECURE_ATL
    #define _SECURE_ATL 1
#endif

#ifndef VC_EXTRALEAN
    #define VC_EXTRALEAN            // �q Windows ���Y�ư����`�ϥΪ�����
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // ���T�w�q������ CString �غc�禡

// ���� MFC ���ä@�Ǳ`���Υi����ĵ�i�T�����\��
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC �֤߻P�зǤ���
#include <afxext.h>         // MFC �X�R�\��





#ifndef _AFX_NO_OLE_SUPPORT
    #include <afxdtctl.h>           // MFC �䴩�� Internet Explorer 4 �q�α��
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
    #include <afxcmn.h>             // MFC �䴩�� Windows �q�α��
#endif // _AFX_NO_AFXCMN_SUPPORT

//#include <afxcontrolbars.h>     // �\��ϩM����C�� MFC �䴩

#define DBGLVL_PRINTF(...)


// Used by DialogConfiguration_XXXX
#include "AppConfig.h"
#include "NumEdit.h"







