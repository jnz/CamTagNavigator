#pragma once

// Network stuff for Windows and Unix
#ifdef _WIN32
    #pragma comment(lib,"ws2_32")
    #include <winsock2.h>
    #define socket_t          SOCKET
    typedef int socklen_t;
#else
    #define socket_t          int
    #define closesocket       close
    #include <fcntl.h>
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <errno.h>
    #include <stdlib.h>
    #include <memory.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <netdb.h>
#endif

class CUDPSender
{
public:
    CUDPSender() {
        s = 0;
        memset(&remoteaddr, 0, sizeof(remoteaddr));
        remoteport = 0;
    }
    ~CUDPSender() {
        if ( s != 0 )
            closesocket(s);
        s = 0;
    }

    int Send(const unsigned char* bytes_to_send, int bytes) const
    {
        const int bytes_transmitted = sendto(s,
                                             (const char*)bytes_to_send,
                                             bytes,
                                             0,
                                             (struct sockaddr *)&remoteaddr,
                                             sizeof(struct sockaddr_in));
        return bytes_transmitted;
    }

    bool SetRemoteAddr(const char* remoteaddr_str, const int port)
    {
        struct hostent *h = gethostbyname(remoteaddr_str);
        if ( h == NULL )
            return false;

        memset(&remoteaddr, 0, sizeof(struct sockaddr_in));
        remoteaddr.sin_family = AF_INET;
        remoteaddr.sin_port = htons(port);
        memcpy((char *) &remoteaddr.sin_addr.s_addr, h->h_addr_list[0], h->h_length);
        remoteport = port;

        if ( s == 0 )
        {
            s = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
            // make socket non-blocking
            int retval;
#ifdef WIN32
            unsigned long _true = 1;
            retval = ioctlsocket( s, FIONBIO, &_true );
#else
            const int flag = fcntl( s, F_GETFL, 0 );
            retval = fcntl( s, F_SETFL, flag|O_NONBLOCK );
#endif
            if ( retval == -1 )
            {
                closesocket(s);
                s = 0;
            }
        }

#ifdef WIN32
        return ( s != INVALID_SOCKET );
#else
        return (s != -1 );
#endif
    }

private:
    socket_t s; /* output UDP socket */
    struct sockaddr_in remoteaddr; /* target address */
    int remoteport; /* remote UDP port */
};
