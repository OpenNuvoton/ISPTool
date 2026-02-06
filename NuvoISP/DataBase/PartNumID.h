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
#include "ChipDefs.h"

struct CPartNumID
{
    char szPartNumber[32];
    unsigned int uID;
    unsigned int uProjectCode;
};

struct CPartNumID_51
{
    char szPartNumber[32];
    unsigned int uID;
    unsigned int uProjectCode;
};

class CPartNum
{
public:
    std::string GetPartNumber(unsigned int uID, unsigned int uCoreType = 0, unsigned int *puProjectCode = NULL) const;

    CPartNum();
    virtual ~CPartNum();
};


#define PROJS \
    _E(UNKNOWN)       \
    _E(NUC100AN)      \
    _E(NUC100BN)      \
    _E(M051AN)        \
    _E(NUC100CN)      \
    _E(NUC122AN)      \
    _E(NUC100DN)      \
    _E(NANO100AN)     \
    _E(M051BN)        \
    _E(MINI51AN)      \
    _E(NUC123AN)      \
    _E(NUC400AE)      \
    _E(M058SAN)       \
    _E(NANO102AN)     \
    _E(MINI55)        \
    _E(M451HD)        \
    _E(M0518)         \
    _E(MINI58)        \
    _E(M0564)         \
    _E(NANO103)       \
    _E(NUC121)        \
    _E(NUC2201)       \
    _E(M4521)         \
    _E(M031_32K)      \
    _E(M031_128K)     \
    _E(M031_64K)      \
    _E(M031_16K)      \
    _E(NUC1261)       \
    _E(M031_512K)     \
    _E(M031_256K)     \
    _E(NUC1311)       \
    _E(M032D)         \
    _E(M0A21)         \
    _E(M030G)         \
    _E(M471)          \
    _E(NUC122DN)      \
    _E(NUC200AE)      \
    _E(NANO100BN)     \
    _E(M051DN)        \
    _E(NUC123AE)      \
    _E(NM1500AE)      \
    _E(M051DE)        \
    _E(MINI51DE)      \
    _E(NM1120)        \
    _E(NUC505)        \
    _E(NM1230)        \
    _E(M252_G)        \
    _E(M252_E)        \
    _E(M252_D)        \
    _E(M252_C)        \
    _E(NM1240)        \
    _E(M258)          \
    _E(M480)          \
    _E(M2351)         \
    _E(M480LD)        \
    _E(M2354)         \
    _E(M2354ES)       \
    _E(M253)          \
    _E(M031GPON)      \
    _E(NUC1262)       \
    _E(M451LD)        \
    _E(M460HD)        \
    _E(M256D)         \
    _E(M258G)         \
    _E(M460LD)        \
    _E(NUC1263)       \
    _E(M2L31_512K)    \
    _E(M2L31_256K)    \
    _E(M55M1)         \
    _E(M2003)         \
    _E(M2A23)         \
    _E(M2U51G)        \
    _E(M2U51C)        \
    _E(M3331IG)       \
    _E(M3331G)        \
    _E(M3351)         \
    \
    _E(N76E885)       \
    _E(N76E616)       \
    _E(N76E003)       \
    _E(ML51_32K)      \
    _E(ML51_16K)      \
    _E(MS51_16K)      \
    _E(MS51_8K)       \
    _E(MS51_32K)      \
    _E(ML56)          \
    _E(MUG51)         \
    _E(MG51)          \
    _E(MG51D)


#define _E(Name) PROJ_##Name,
enum ProjsList { PROJS };
#undef _E

#endif // !defined(AFX_PARTNUMID_H__224714FF_DE1A_41FC_81B6_4B998BDC9FE6__INCLUDED_)
