// PartNumID.h: interface for the CPartNum class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PARTNUMID_H__224714FF_DE1A_41FC_81B6_4B998BDC9FE6__INCLUDED_)
#define AFX_PARTNUMID_H__224714FF_DE1A_41FC_81B6_4B998BDC9FE6__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <string>
#include <sstream>

struct CPartNumID {
    char szPartNumber[32];
    unsigned int uID;
    unsigned int uProjectCode;
};

#endif // !defined(AFX_PARTNUMID_H__224714FF_DE1A_41FC_81B6_4B998BDC9FE6__INCLUDED_)
