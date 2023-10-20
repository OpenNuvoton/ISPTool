#include "stdafx.h"
#include "CTRSP.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <wspiapi.h>
#include <stdlib.h>
#include <stdio.h>
#include <strsafe.h>
#include <string>
#include <cstring>

#pragma comment(lib,"ws2_32.lib")

CTRSP::CTRSP(void)
    : m_conn_socket(INVALID_SOCKET)
    , m_BleIO()
    , m_ServF01(0)
    , m_CharW01(0)
    , m_CharN01(0)
{
    memset(m_aucRxBuf, 0, sizeof(m_aucRxBuf));
    memset((void *)m_auRxCount, 0, sizeof(m_auRxCount));
}

CTRSP::~CTRSP(void)
{
}

BOOL CTRSP::OpenDevice(INTF_E eInterface, ...)
{
    m_Intf = eInterface;

    if (m_Intf == INTF_E_WIFI) {
        va_list valist;
        va_start(valist, eInterface);

        std::string server_name = CStringA(va_arg(valist, CString));
        std::string port = CStringA(va_arg(valist, CString));

        va_end(valist);

        WSADATA wsaData;
        SOCKET conn_socket = INVALID_SOCKET;

        struct addrinfo hints;
        struct addrinfo *addrptr = NULL;

        char hoststr[NI_MAXHOST] = {0};
        char servstr[NI_MAXSERV] = {0};

        int address_family = AF_UNSPEC;
        int socket_type = SOCK_STREAM;
        struct addrinfo *results = NULL;

        int retval;
        int loopflag = 0;
        int maxloop = -1;

        if ((retval = WSAStartup(MAKEWORD(2, 2), &wsaData)) != 0) {
            WSACleanup();
            return FALSE;
        }

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = address_family;
        hints.ai_socktype = socket_type;
        hints.ai_protocol = ((socket_type == SOCK_STREAM) ? IPPROTO_TCP : IPPROTO_UDP);

        retval = getaddrinfo(server_name.c_str(),
                             port.c_str(),
                             &hints,
                             &results);

        if (retval != 0) {
            return FALSE;
        }

        if (results == NULL) {
            return FALSE;
        }

        addrptr = results;

        while (addrptr) {
            conn_socket = socket(addrptr->ai_family,
                                 addrptr->ai_socktype,
                                 addrptr->ai_protocol);

            if (conn_socket == INVALID_SOCKET) {
                return FALSE;
            }

            retval = getnameinfo(addrptr->ai_addr,
                                 (socklen_t)addrptr->ai_addrlen,
                                 hoststr,
                                 NI_MAXHOST,
                                 servstr,
                                 NI_MAXSERV,
                                 NI_NUMERICHOST | NI_NUMERICSERV);

            if (retval != 0) {
                return FALSE;
            }

            retval = connect(conn_socket,
                             addrptr->ai_addr,
                             (int)addrptr->ai_addrlen);

            if (retval == SOCKET_ERROR) {
                closesocket(conn_socket);
                conn_socket = INVALID_SOCKET;

                addrptr = addrptr->ai_next;
            } else {
                break;
            }
        }

        freeaddrinfo(results);
        results = NULL;

        if (conn_socket == INVALID_SOCKET) {
            return FALSE;
        }

        m_conn_socket = conn_socket;

        return TRUE;
    } else if (m_Intf == INTF_E_BLE) {
        va_list valist;
        va_start(valist, eInterface);

        m_ServF01 = va_arg(valist, USHORT);
        m_CharW01 = va_arg(valist, USHORT);
        m_CharN01 = va_arg(valist, USHORT);

        va_end(valist);

        if (m_BleIO.OpenDevice(m_ServF01)) {
            return SetActiveDevice(0);
        }
    }

    return FALSE;
}

void CTRSP::CloseDevice(void)
{
    if (m_Intf == INTF_E_WIFI) {
        if (m_conn_socket != INVALID_SOCKET) {
            shutdown(m_conn_socket, SD_SEND);

            closesocket(m_conn_socket);

            m_conn_socket = INVALID_SOCKET;
        }

        WSACleanup();
    } else if (m_Intf == INTF_E_BLE) {
        m_BleIO.CloseDevice();
    }
}

size_t CTRSP::GetDeviceLength(void) const
{
    if (m_Intf == INTF_E_WIFI) {
        if (m_conn_socket != INVALID_SOCKET) {
            return 1;
        }
    } else if (m_Intf == INTF_E_BLE) {
        return m_BleIO.GetDeviceLength();
    }

    return 0;
}

BOOL CTRSP::SetActiveDevice(size_t szIndex)
{
    if (m_Intf == INTF_E_WIFI) {
        if (m_conn_socket != INVALID_SOCKET && szIndex < 1) {
            return TRUE;
        }
    } else if (m_Intf == INTF_E_BLE) {
        if (szIndex >= _countof(m_aucRxBuf))
            return FALSE;

        if (m_BleIO.SetActiveDevice(szIndex)) {
            return (m_BleIO.SetCharacteristic(m_CharN01)
                    && m_BleIO.SubScribe(TRUE, FALSE)
                    && m_BleIO.RegisterNotify(BLE_NotifyCallback, this));
        }
    }

    return FALSE;
}

size_t CTRSP::GetActiveDevice(void) const
{
    if (m_Intf == INTF_E_WIFI) {
        if (m_conn_socket != INVALID_SOCKET) {
            return 0;
        }
    } else if (m_Intf == INTF_E_BLE) {
        return m_BleIO.GetActiveDevice();
    }

    return 0;
}

std::string CTRSP::GetActiveDeviceName(void) const
{
    if (m_Intf == INTF_E_BLE) {
        return m_BleIO.GetActiveDeviceAddr();
    }

    return "";
}

BOOL CTRSP::Write(const CHAR *pcBuffer, size_t szLen, DWORD *pdwLength)
{
    if (m_Intf == INTF_E_WIFI) {
        if (m_conn_socket != INVALID_SOCKET) {
            int retval = send(m_conn_socket, pcBuffer, szLen, 0);

            if (retval == SOCKET_ERROR) {
                return FALSE;
            }
        }

        return TRUE;
    } else if (m_Intf == INTF_E_BLE) {
        if (m_BleIO.SetCharacteristic(m_CharW01)) {
            return m_BleIO.Write(pcBuffer, szLen, pdwLength);
        }
    }

    return FALSE;
}

BOOL CTRSP::Read(CHAR *pcBuffer, size_t szLen, DWORD *pdwLength, DWORD dwTime)
{
    if (m_Intf == INTF_E_WIFI) {
        if (m_conn_socket != INVALID_SOCKET) {
            int selectReturn;
            fd_set rfd;
            timeval timeout = {0, 0};
            timeout.tv_sec = dwTime / 1000;
            timeout.tv_usec = (dwTime % 1000) * 1000;

            FD_ZERO(&rfd);
            FD_SET(m_conn_socket, &rfd);

            selectReturn = select(m_conn_socket + 1, &rfd, NULL, NULL, &timeout);

            if (selectReturn == -1) {
                return FALSE;
            }

            if (FD_ISSET(m_conn_socket, &rfd)) {
                int retval = recv(m_conn_socket, pcBuffer, szLen, 0);

                if (retval == SOCKET_ERROR) {
                    return FALSE;
                }
            } else {
                return FALSE;
            }

            return TRUE;
        }
    } else if (m_Intf == INTF_E_BLE) {
        size_t szIndex = GetActiveDevice();
        DWORD dwLength = 0;
        DWORD dwStart = GetTickCount();

        while (m_auRxCount[szIndex] < szLen && (GetTickCount() - dwStart) < dwTime);

        std::lock_guard<std::mutex> lock(m_RxMutex);

        if (m_auRxCount[szIndex] > 0) {
            dwLength = m_auRxCount[szIndex];

            if (dwLength > szLen)
                dwLength = szLen;

            memcpy(pcBuffer, m_aucRxBuf[szIndex], dwLength);

            m_auRxCount[szIndex] = 0;

            if (pdwLength != NULL)
                *pdwLength = dwLength;

            return TRUE;
        }
    }

    return FALSE;
}

void CTRSP::ClearReadBuf(void)
{
    if (m_Intf == INTF_E_BLE) {
        size_t szIndex = GetActiveDevice();
        std::lock_guard<std::mutex> lock(m_RxMutex);

        m_auRxCount[szIndex] = 0;
        memset(m_aucRxBuf[szIndex], 0, sizeof(m_aucRxBuf[szIndex]));
    }
}

void BLE_NotifyCallback(UCHAR *pData, ULONG uDataSize, void *pContext)
{
    CTRSP *pTRSP = (CTRSP *)pContext;

    if (pTRSP != NULL) {
        size_t szIndex = pTRSP->GetActiveDevice();
        std::lock_guard<std::mutex> lock(pTRSP->m_RxMutex);
        ULONG uBufSize = sizeof(pTRSP->m_aucRxBuf[szIndex]) - pTRSP->m_auRxCount[szIndex];

        if (uBufSize > uDataSize)
            uBufSize = uDataSize;

        if (uBufSize > 0) {
            memcpy(pTRSP->m_aucRxBuf[szIndex] + pTRSP->m_auRxCount[szIndex], pData, uBufSize);
            pTRSP->m_auRxCount[szIndex] += uBufSize;
        }
    }
}
