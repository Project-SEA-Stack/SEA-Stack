/*********************************************************************
 * @file  mock_external_module.cpp
 * @brief Minimal external-force module for IPC unit tests.
 *
 * Speaks the v1 protocol on TCP. Implements a linear damper:
 *   F = -c * v   with c from config.damping (default 50).
 *
 * Usage: mock_external_module --seastack-port <N>
 *********************************************************************/
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "ws2_32.lib")
using socket_t = SOCKET;
#else
#  include <arpa/inet.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#  include <unistd.h>
using socket_t = int;
#endif

namespace {

void CloseSock(socket_t s) {
#ifdef _WIN32
    closesocket(s);
#else
    close(s);
#endif
}

void SendAll(socket_t s, const char* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
#ifdef _WIN32
        const int n = send(s, data + sent, static_cast<int>(len - sent), 0);
#else
        const ssize_t n = send(s, data + sent, len - sent, 0);
#endif
        if (n <= 0) {
            throw std::runtime_error("send failed");
        }
        sent += static_cast<size_t>(n);
    }
}

void RecvAll(socket_t s, char* data, size_t len) {
    size_t got = 0;
    while (got < len) {
#ifdef _WIN32
        const int n = recv(s, data + got, static_cast<int>(len - got), 0);
#else
        const ssize_t n = recv(s, data + got, len - got, 0);
#endif
        if (n <= 0) {
            throw std::runtime_error("recv failed");
        }
        got += static_cast<size_t>(n);
    }
}

void SendFramed(socket_t s, const std::string& payload) {
    const uint32_t len = static_cast<uint32_t>(payload.size());
    unsigned char hdr[4] = {
        static_cast<unsigned char>((len >> 24) & 0xff),
        static_cast<unsigned char>((len >> 16) & 0xff),
        static_cast<unsigned char>((len >> 8) & 0xff),
        static_cast<unsigned char>(len & 0xff),
    };
    SendAll(s, reinterpret_cast<const char*>(hdr), 4);
    SendAll(s, payload.data(), payload.size());
}

std::string RecvFramed(socket_t s) {
    unsigned char hdr[4];
    RecvAll(s, reinterpret_cast<char*>(hdr), 4);
    const uint32_t len = (static_cast<uint32_t>(hdr[0]) << 24) |
                         (static_cast<uint32_t>(hdr[1]) << 16) |
                         (static_cast<uint32_t>(hdr[2]) << 8) |
                         static_cast<uint32_t>(hdr[3]);
    std::string payload(len, '\0');
    if (len > 0) {
        RecvAll(s, &payload[0], len);
    }
    return payload;
}

bool Contains(const std::string& s, const char* needle) {
    return s.find(needle) != std::string::npos;
}

double ExtractDamping(const std::string& json) {
    const std::string key = "\"damping\"";
    const size_t pos = json.find(key);
    if (pos == std::string::npos) {
        return 50.0;
    }
    size_t i = pos + key.size();
    while (i < json.size() && (json[i] == ' ' || json[i] == ':' || json[i] == '\t')) {
        ++i;
    }
    return std::strtod(json.c_str() + i, nullptr);
}

std::vector<double> ExtractInArray(const std::string& json) {
    std::vector<double> out;
    const size_t pos = json.find("\"in\"");
    if (pos == std::string::npos) {
        return out;
    }
    size_t i = json.find('[', pos);
    if (i == std::string::npos) {
        return out;
    }
    ++i;
    while (i < json.size() && json[i] != ']') {
        while (i < json.size() &&
               (json[i] == ' ' || json[i] == ',' || json[i] == '\t')) {
            ++i;
        }
        if (i >= json.size() || json[i] == ']') {
            break;
        }
        char* end = nullptr;
        const double v = std::strtod(json.c_str() + i, &end);
        if (end == json.c_str() + i) {
            break;
        }
        out.push_back(v);
        i = static_cast<size_t>(end - json.c_str());
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    int port = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--seastack-port") == 0 && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        }
    }
    if (port <= 0) {
        std::cerr << "mock_external_module: missing --seastack-port\n";
        return 2;
    }

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        return 3;
    }
#endif

    socket_t sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock
#ifdef _WIN32
        == INVALID_SOCKET
#else
        < 0
#endif
    ) {
        return 4;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
#ifdef _WIN32
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
#else
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
#endif

    if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        CloseSock(sock);
        return 5;
    }

    double damping = 50.0;
    bool running = true;
    while (running) {
        const std::string req = RecvFramed(sock);
        if (Contains(req, "\"op\":\"initialize\"")) {
            damping = ExtractDamping(req);
            SendFramed(sock,
                       "{\"status\":\"ok\",\"protocol\":1,\"name\":\"MockDamper\","
                       "\"version\":\"1.0\",\"n_states\":0}");
        } else if (Contains(req, "\"op\":\"evaluate\"")) {
            const auto in = ExtractInArray(req);
            const double vel = (in.size() >= 2) ? in[1] : 0.0;
            const double force = -damping * vel;
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "{\"status\":\"ok\",\"out\":[%.17g]}", force);
            SendFramed(sock, buf);
        } else if (Contains(req, "\"op\":\"shutdown\"")) {
            SendFramed(sock, "{\"status\":\"ok\"}");
            running = false;
        } else if (Contains(req, "\"op\":\"reset\"") ||
                   Contains(req, "\"op\":\"commit\"") ||
                   Contains(req, "\"op\":\"rollback\"")) {
            SendFramed(sock, "{\"status\":\"ok\"}");
        } else {
            SendFramed(sock,
                       "{\"status\":\"error\",\"message\":\"unknown op\"}");
        }
    }

    CloseSock(sock);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
