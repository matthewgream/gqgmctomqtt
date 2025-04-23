
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <netdb.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>

#define BUF_SIZE 1024

bool http_get(const char *host, const char *port, const char *url, char *response, const int response_size) {
    bool result = false;
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, port, &hints, &res) == 0) {
        int sock = -1;
        if ((sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol)) >= 0) {
            if (connect(sock, res->ai_addr, res->ai_addrlen) >= 0) {
                char request[BUF_SIZE];
                sprintf(request, "GET %s HTTP/1.1\r\nHost: %s\r\n\r\n", url, host);
                if (send(sock, request, strlen(request), 0) > 0) {
                    ssize_t n;
                    if ((n = recv(sock, response, response_size - 1, 0)) >= 0) {
                        response[n] = '\0';
                        result = true;
                    }
                }
            }
            close(sock);
        }
        freeaddrinfo(res);
    }
    return result;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
