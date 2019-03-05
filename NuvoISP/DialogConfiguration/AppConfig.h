// AppConfig.h: interface for the CAppConfig class.
//
//////////////////////////////////////////////////////////////////////

#ifndef INC__APP_CONFIG_H__
#define INC__APP_CONFIG_H__

class CAppConfig
{
public:
    virtual ~CAppConfig();

    /* Global configurations */
    struct configs_t {
        unsigned int m_value[4];
    };

#define NUC1xx_configs_t configs_t
#define M451_configs_t configs_t
#define M05x_configs_t configs_t
#define M0564_configs_t configs_t
#define Nano100_configs_t configs_t
#define Mini51_configs_t configs_t
#define MT5xx_configs_t configs_t
#define AU91xx_configs_t configs_t
#define NUC029_configs_t configs_t
#define NM1120_configs_t configs_t
#define N76E1T_configs_t configs_t
#define NUC4xx_configs_t configs_t
#define TC8226_configs_t configs_t
#define M251_configs_t configs_t
#define M480_configs_t configs_t

};

#endif // !defined(AFX_APPCONFIG_H__B96EEC5C_5D1E_4040_9CFB_1A234396F753__INCLUDED_)
