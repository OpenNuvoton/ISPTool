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
    NUC_CHIP_TYPE_E GetChipType(const char *sPartNo) const;

    bool IsSupportedPID_8051(unsigned int uSID) const;

    CPartNum();
    virtual ~CPartNum();
};


#define PROJS \
    _E(UNKNOWN)    /* UNKNOWN */   \
    _E(NUC100AN)   /* "DA8230" */   \
    _E(NUC100BN)   /* "DA8232" */   \
    _E(M051AN)     /* "DA8233" */   \
    _E(NUC100CN)   /* "DA8245" */   \
    _E(NUC122AN)   /* "FA8236" */   \
    _E(NUC100DN)   /* "FA8239" */   \
    _E(NANO100AN)  /* "FA8240" */   \
    _E(M051BN)     /* "FA8243" */   \
    _E(MINI51AN)   /* "FA8244" */   \
    _E(NUC123AN)   /* "FA8248" */   \
    _E(NUC400AE)   /* "FA8249" */   \
    _E(M058SAN)    /* "FA8252" */   \
    _E(NANO102AN)  /* "FA8253" */   \
    _E(MINI55)     /* "FA8255" */   \
    _E(M451HD)     /* "FA8256" */   \
    _E(M0518)      /* "FA8258" */   \
    _E(MINI58)     /* "FA8260" */   \
    _E(M0564)      /* "FA8262" */   \
    _E(NANO103)    /* "FA8263" */   \
    _E(NUC121)     /* "FA8266" */   \
    _E(NUC2201)    /* "FA8267" */   \
    _E(M4521)      /* "FA8268" */   \
    _E(M031_32K)   /* "FA8269" */   \
    _E(M031_128K)  /* "FA8270" */   \
    _E(M031_64K)   /* "FA8271" */   \
    _E(M031_16K)   /* "FA8272" */   \
    _E(NUC1261)    /* "FA8273" */   \
    _E(M031_512K)  /* "FA8274" */   \
    _E(M031_256K)  /* "FA8275" */   \
    _E(NUC1311)    /* "FA8276" */   \
    _E(M032D)      /* "FA8277" */   \
    _E(M0A21)      /* "FA8279" */   \
    _E(M030G)      /* "FA8280" */   \
    _E(M471)       /* "FA8281" */   \
    _E(NUC122DN)   /* "FB8236" */   \
    _E(NUC200AE)   /* "FB8239" */   \
    _E(NANO100BN)  /* "FB8240" */   \
    _E(M051DN)     /* "FB8243" */   \
    _E(NUC123AE)   /* "FB8248" */   \
    _E(NM1500AE)   /* "FC8238" */   \
    _E(M051DE)     /* "FC8243" */   \
    _E(MINI51DE)   /* "FC8244" */   \
    _E(NM1120)     /* "GAG014" */   \
    _E(NUC505)     /* "HAD009" */   \
    _E(NM1230)     /* "HAG025" */   \
    _E(M252_G)     /* "HAG026" */   \
    _E(M252_E)     /* "HAG027" */   \
    _E(M252_D)     /* "HAG028" */   \
    _E(M252_C)     /* "HAG029" */   \
    _E(NM1240)     /* "HAG039" */   \
    _E(M258)       /* "HAG040" */   \
    _E(M480)       /* "LAG018" */   \
    _E(M2351)      /* "LAG021" */   \
    _E(M480LD)     /* "LAG033" */   \
    _E(M2354)      /* "LAG043" */   \
    _E(M2354ES)    /* "TC8250" */   \
    _E(M253)       /* "HAG046" */   \
    _E(M031GPON)   /* "FA8284" */   \
    _E(NUC1262)    /* "FA8283" */   \
    _E(M451LD)     /* "FA8259" */   \
    _E(M460HD)     /* "LAG048" */   \
    _E(M256D)      /* "HAG050" */   \
    _E(M258G)      /* "HAG051" */   \
    _E(M460LD)     /* "LAG052" */   \
    _E(NUC1263)    /* "FA8285" */   \
    _E(M2L31_512K) /* "MAD025" */   \
    _E(M2L31_256K) /* "MAD026" */   \
    _E(M55M1)      /* "MAD000" */   \
    _E(M2003)      /* "KAG062" */   \
    _E(M2A23)      /* "FA8286" */   \
    _E(M2U51G)     /* "KAG066" */   \
    _E(M2U51C)     /* "KAG067" */   \
	_E(M3331IG)    /* "MAG071" */   \
	_E(M3331G)     /* "MAG072" */   \
    \
    _E(N76E885)    /* "FA8251" */   \
    _E(N76E616)    /* "FA8261" */   \
    _E(N76E003)    /* "FA8265" */   \
    _E(ML51_32K)   /* "HAG023" */   \
    _E(ML51_16K)   /* "HAG024" */   \
    _E(MS51_16K)   /* "HAG030" */   \
    _E(MS51_8K)    /* "HAG034" */   \
    _E(MS51_32K)   /* "HAG035" */   \
    _E(ML56)       /* "HAG037" */   \
    _E(MUG51)      /* "HAG049" */   \
    _E(MG51)       /* "KAG056" */   \
    _E(MG51D)      /* "KAG065" */   \
    \
    _E(MA35D1)                      \
    \
    _E(KM1M7AB)                     \
    _E(KM1M4B)                      \
    _E(KM1M7C)                      \
    _E(KM1M0D)                      \
    _E(KM1M0G)


 #define _E(Name) PROJ_##Name,
enum ProjsList { PROJS };
#undef _E

 #define _E(Name) "PROJ_" #Name,
static const char *g_sProjsName[] = { PROJS };
#undef _E

#ifdef __WASM__
    #include <emscripten.h>

    #ifdef __cplusplus
        #define EXTERN_C    extern "C"
    #else
        #define EXTERN_C
    #endif

    EXTERN_C EMSCRIPTEN_KEEPALIVE
    unsigned int CPartNum_GetProjectCount();
    EXTERN_C EMSCRIPTEN_KEEPALIVE
    const char *CPartNum_GetProjectName(int index);

    EXTERN_C EMSCRIPTEN_KEEPALIVE
    const char *CPartNum_GetPartNumber(unsigned int uID, unsigned int uCoreType = 0, unsigned int *puProjectCode = NULL);
    EXTERN_C EMSCRIPTEN_KEEPALIVE
    bool CPartNum_IsSupportedPID_8051(unsigned int uSID);
#else
    unsigned int CPartNum_GetProjectCount();
    const char *CPartNum_GetProjectName(int index);
#endif

#endif // !defined(AFX_PARTNUMID_H__224714FF_DE1A_41FC_81B6_4B998BDC9FE6__INCLUDED_)
