#include "StdAfx.h"
#include "Resource.h"
#include "Lang.h"

CString LoadStringFromID(UINT uID, TCHAR *szID)
{
    CString str;

    if (str.LoadString(uID) == FALSE)
        return szID;

    return str;
}
