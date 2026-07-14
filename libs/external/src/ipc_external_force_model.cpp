#include <seastack/external/ipc_external_force_model.h>
#include <seastack/external/protocol.h>

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "ws2_32.lib")
using socket_t = SOCKET;
constexpr socket_t kInvalidSocket = INVALID_SOCKET;
#else
#  include <arpa/inet.h>
#  include <fcntl.h>
#  include <netinet/in.h>
#  include <signal.h>
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <sys/wait.h>
#  include <unistd.h>
using socket_t = int;
constexpr socket_t kInvalidSocket = -1;
#endif

namespace seastack {
namespace external {
namespace {

class WinsockInit {
  public:
#ifdef _WIN32
    WinsockInit() {
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
    }
    ~WinsockInit() { WSACleanup(); }
#else
    WinsockInit() = default;
#endif
};

WinsockInit& EnsureSockets() {
    static WinsockInit init;
    return init;
}

void CloseSocket(socket_t s) {
    if (s == kInvalidSocket) {
        return;
    }
#ifdef _WIN32
    closesocket(s);
#else
    close(s);
#endif
}

bool SetNonBlocking(socket_t s, bool nonblock) {
#ifdef _WIN32
    u_long mode = nonblock ? 1u : 0u;
    return ioctlsocket(s, FIONBIO, &mode) == 0;
#else
    int flags = fcntl(s, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    if (nonblock) {
        flags |= O_NONBLOCK;
    } else {
        flags &= ~O_NONBLOCK;
    }
    return fcntl(s, F_SETFL, flags) == 0;
#endif
}

bool WaitReadable(socket_t s, int timeout_ms) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s, &rfds);
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
#ifdef _WIN32
    const int rc = select(0, &rfds, nullptr, nullptr, &tv);
#else
    const int rc = select(s + 1, &rfds, nullptr, nullptr, &tv);
#endif
    return rc > 0;
}

bool WaitWritable(socket_t s, int timeout_ms) {
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(s, &wfds);
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
#ifdef _WIN32
    const int rc = select(0, nullptr, &wfds, nullptr, &tv);
#else
    const int rc = select(s + 1, nullptr, &wfds, nullptr, &tv);
#endif
    return rc > 0;
}

void SendAll(socket_t s, const char* data, size_t len, int timeout_ms) {
    size_t sent = 0;
    while (sent < len) {
        if (!WaitWritable(s, timeout_ms)) {
            throw std::runtime_error("IPC send timeout");
        }
#ifdef _WIN32
        const int n = send(s, data + sent, static_cast<int>(len - sent), 0);
#else
        const ssize_t n = send(s, data + sent, len - sent, 0);
#endif
        if (n <= 0) {
            throw std::runtime_error("IPC send failed");
        }
        sent += static_cast<size_t>(n);
    }
}

void RecvAll(socket_t s, char* data, size_t len, int timeout_ms) {
    size_t got = 0;
    while (got < len) {
        if (!WaitReadable(s, timeout_ms)) {
            throw std::runtime_error("IPC recv timeout");
        }
#ifdef _WIN32
        const int n = recv(s, data + got, static_cast<int>(len - got), 0);
#else
        const ssize_t n = recv(s, data + got, len - got, 0);
#endif
        if (n <= 0) {
            throw std::runtime_error("IPC recv failed (peer closed?)");
        }
        got += static_cast<size_t>(n);
    }
}

void SendFramed(socket_t s, const std::string& payload, int timeout_ms) {
    if (payload.size() > 0xffffffffu) {
        throw std::runtime_error("IPC message too large");
    }
    const uint32_t len = static_cast<uint32_t>(payload.size());
    unsigned char hdr[4] = {
        static_cast<unsigned char>((len >> 24) & 0xff),
        static_cast<unsigned char>((len >> 16) & 0xff),
        static_cast<unsigned char>((len >> 8) & 0xff),
        static_cast<unsigned char>(len & 0xff),
    };
    SendAll(s, reinterpret_cast<const char*>(hdr), 4, timeout_ms);
    SendAll(s, payload.data(), payload.size(), timeout_ms);
}

std::string RecvFramed(socket_t s, int timeout_ms) {
    unsigned char hdr[4];
    RecvAll(s, reinterpret_cast<char*>(hdr), 4, timeout_ms);
    const uint32_t len = (static_cast<uint32_t>(hdr[0]) << 24) |
                         (static_cast<uint32_t>(hdr[1]) << 16) |
                         (static_cast<uint32_t>(hdr[2]) << 8) |
                         static_cast<uint32_t>(hdr[3]);
    if (len > 16u * 1024u * 1024u) {
        throw std::runtime_error("IPC message length exceeds 16 MiB");
    }
    std::string payload(len, '\0');
    if (len > 0) {
        RecvAll(s, &payload[0], len, timeout_ms);
    }
    return payload;
}

struct ChildProcess {
#ifdef _WIN32
    PROCESS_INFORMATION pi{};
    bool running = false;
#else
    pid_t pid = -1;
#endif
};

void TerminateChild(ChildProcess& child) {
#ifdef _WIN32
    if (child.running) {
        TerminateProcess(child.pi.hProcess, 1);
        WaitForSingleObject(child.pi.hProcess, 2000);
        CloseHandle(child.pi.hProcess);
        CloseHandle(child.pi.hThread);
        child.running = false;
    }
#else
    if (child.pid > 0) {
        kill(child.pid, SIGTERM);
        int status = 0;
        for (int i = 0; i < 20; ++i) {
            const pid_t r = waitpid(child.pid, &status, WNOHANG);
            if (r == child.pid) {
                child.pid = -1;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        kill(child.pid, SIGKILL);
        waitpid(child.pid, &status, 0);
        child.pid = -1;
    }
#endif
}

ChildProcess SpawnChild(const std::vector<std::string>& command,
                        const std::string& working_directory) {
    if (command.empty()) {
        throw std::runtime_error("IpcExternalForceModel: empty command");
    }
    ChildProcess child;

#ifdef _WIN32
    std::string cmdline;
    for (size_t i = 0; i < command.size(); ++i) {
        if (i > 0) {
            cmdline += ' ';
        }
        const bool need_quotes =
            command[i].find_first_of(" \t\"") != std::string::npos;
        if (need_quotes) {
            cmdline += '"';
            for (char c : command[i]) {
                if (c == '"') {
                    cmdline += '\\';
                }
                cmdline += c;
            }
            cmdline += '"';
        } else {
            cmdline += command[i];
        }
    }
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    std::vector<char> mutable_cmd(cmdline.begin(), cmdline.end());
    mutable_cmd.push_back('\0');
    const char* cwd =
        working_directory.empty() ? nullptr : working_directory.c_str();
    if (!CreateProcessA(nullptr, mutable_cmd.data(), nullptr, nullptr, FALSE,
                        CREATE_NO_WINDOW, nullptr, cwd, &si, &child.pi)) {
        throw std::runtime_error("Failed to spawn external force module process");
    }
    child.running = true;
#else
    std::vector<char*> argv;
    argv.reserve(command.size() + 1);
    for (const auto& a : command) {
        argv.push_back(const_cast<char*>(a.c_str()));
    }
    argv.push_back(nullptr);

    const pid_t pid = fork();
    if (pid < 0) {
        throw std::runtime_error("fork() failed for external force module");
    }
    if (pid == 0) {
        if (!working_directory.empty()) {
            if (chdir(working_directory.c_str()) != 0) {
                _exit(127);
            }
        }
        execvp(argv[0], argv.data());
        _exit(127);
    }
    child.pid = pid;
#endif
    return child;
}

}  // namespace

struct IpcExternalForceModel::Impl {
    socket_t listen_sock = kInvalidSocket;
    socket_t client_sock = kInvalidSocket;
    ChildProcess child{};
    int timeout_ms = 10000;
};

IpcExternalForceModel::IpcExternalForceModel(IpcExternalForceOptions options)
    : impl_(std::make_unique<Impl>()), options_(std::move(options)) {
    EnsureSockets();
    impl_->timeout_ms = options_.timeout_ms;

    impl_->listen_sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (impl_->listen_sock == kInvalidSocket) {
        throw std::runtime_error("Failed to create listen socket");
    }

    int reuse = 1;
#ifdef _WIN32
    setsockopt(impl_->listen_sock, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof(reuse));
#else
    setsockopt(impl_->listen_sock, SOL_SOCKET, SO_REUSEADDR, &reuse,
               sizeof(reuse));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(0);

    if (bind(impl_->listen_sock, reinterpret_cast<sockaddr*>(&addr),
             sizeof(addr)) != 0) {
        CloseSocket(impl_->listen_sock);
        impl_->listen_sock = kInvalidSocket;
        throw std::runtime_error("Failed to bind loopback listen socket");
    }

    sockaddr_in bound{};
#ifdef _WIN32
    int bound_len = sizeof(bound);
#else
    socklen_t bound_len = sizeof(bound);
#endif
    if (getsockname(impl_->listen_sock, reinterpret_cast<sockaddr*>(&bound),
                    &bound_len) != 0) {
        CloseSocket(impl_->listen_sock);
        impl_->listen_sock = kInvalidSocket;
        throw std::runtime_error("getsockname failed");
    }
    listen_port_ = ntohs(bound.sin_port);

    if (listen(impl_->listen_sock, 1) != 0) {
        CloseSocket(impl_->listen_sock);
        impl_->listen_sock = kInvalidSocket;
        throw std::runtime_error("listen() failed");
    }

    if (!options_.command.empty()) {
        std::vector<std::string> cmd = options_.command;
        cmd.push_back("--seastack-port");
        cmd.push_back(std::to_string(listen_port_));
        impl_->child = SpawnChild(cmd, options_.working_directory);
    }
}

IpcExternalForceModel::~IpcExternalForceModel() {
    try {
        Shutdown();
    } catch (...) {
    }
}

void IpcExternalForceModel::SendRequest(const std::string& json) {
    if (impl_->client_sock == kInvalidSocket) {
        throw std::runtime_error("IPC client socket not connected");
    }
    SendFramed(impl_->client_sock, json, impl_->timeout_ms);
}

std::string IpcExternalForceModel::RecvReply() {
    if (impl_->client_sock == kInvalidSocket) {
        throw std::runtime_error("IPC client socket not connected");
    }
    return RecvFramed(impl_->client_sock, impl_->timeout_ms);
}

void IpcExternalForceModel::RequireOk(const std::string& reply,
                                      const char* context) {
    std::string status;
    std::string message;
    if (!protocol::ParseStatusReply(reply, status, message)) {
        throw std::runtime_error(std::string(context) +
                                 ": malformed IPC reply");
    }
    if (status != "ok") {
        throw std::runtime_error(std::string(context) + ": " +
                                 (message.empty() ? status : message));
    }
}

ExternalMeta IpcExternalForceModel::Initialize(const ExternalInit& init) {
    if (shutdown_) {
        throw std::runtime_error("IpcExternalForceModel already shut down");
    }
    if (initialized_) {
        throw std::runtime_error("IpcExternalForceModel already initialized");
    }
    init_ = init;
    last_dt_ = init.dt;

    if (!WaitReadable(impl_->listen_sock, impl_->timeout_ms)) {
        TerminateChild(impl_->child);
        throw std::runtime_error(
            "Timed out waiting for external force module to connect");
    }

    sockaddr_in client_addr{};
#ifdef _WIN32
    int client_len = sizeof(client_addr);
#else
    socklen_t client_len = sizeof(client_addr);
#endif
    impl_->client_sock =
        accept(impl_->listen_sock, reinterpret_cast<sockaddr*>(&client_addr),
               &client_len);
    if (impl_->client_sock == kInvalidSocket) {
        TerminateChild(impl_->child);
        throw std::runtime_error("accept() failed for external force module");
    }
    SetNonBlocking(impl_->client_sock, true);

    // Listener no longer needed.
    CloseSocket(impl_->listen_sock);
    impl_->listen_sock = kInvalidSocket;

    const std::string req = protocol::MakeInitializeRequest(init);
    SendRequest(req);
    const std::string reply = RecvReply();
    RequireOk(reply, "initialize");

    int protocol = 0;
    if (protocol::ExtractIntField(reply, "protocol", protocol) &&
        protocol != 0 && protocol != kProtocolVersion) {
        throw std::runtime_error("External module protocol version mismatch");
    }

    ExternalMeta meta;
    protocol::ParseInitializeReply(reply, meta);
    initialized_ = true;
    return meta;
}

void IpcExternalForceModel::Evaluate(double time,
                                     const std::vector<double>& in,
                                     std::vector<double>& out) {
    if (!initialized_ || shutdown_) {
        throw std::runtime_error("IpcExternalForceModel not initialized");
    }
    if (static_cast<int>(in.size()) != init_.n_inputs) {
        throw std::runtime_error("Evaluate: unexpected input size");
    }
    const std::string req =
        protocol::MakeEvaluateRequest(time, last_dt_, in);
    SendRequest(req);
    const std::string reply = RecvReply();
    RequireOk(reply, "evaluate");
    if (!protocol::ParseEvaluateReply(reply, out)) {
        throw std::runtime_error("evaluate: missing/invalid out array");
    }
    if (static_cast<int>(out.size()) != init_.n_outputs) {
        throw std::runtime_error("evaluate: unexpected output size");
    }
}

void IpcExternalForceModel::Reset() {
    if (!initialized_ || shutdown_) {
        return;
    }
    SendRequest(protocol::MakeSimpleOpRequest("reset"));
    RequireOk(RecvReply(), "reset");
}

void IpcExternalForceModel::Commit() {
    if (!initialized_ || shutdown_) {
        return;
    }
    SendRequest(protocol::MakeSimpleOpRequest("commit"));
    RequireOk(RecvReply(), "commit");
}

void IpcExternalForceModel::Rollback() {
    if (!initialized_ || shutdown_) {
        return;
    }
    SendRequest(protocol::MakeSimpleOpRequest("rollback"));
    RequireOk(RecvReply(), "rollback");
}

void IpcExternalForceModel::Shutdown() {
    if (shutdown_) {
        return;
    }
    shutdown_ = true;
    if (initialized_ && impl_->client_sock != kInvalidSocket) {
        try {
            SendRequest(protocol::MakeSimpleOpRequest("shutdown"));
            // Best-effort reply; ignore failures during teardown.
            try {
                (void)RecvReply();
            } catch (...) {
            }
        } catch (...) {
        }
    }
    CloseSocket(impl_->client_sock);
    impl_->client_sock = kInvalidSocket;
    CloseSocket(impl_->listen_sock);
    impl_->listen_sock = kInvalidSocket;
    TerminateChild(impl_->child);
    initialized_ = false;
}

}  // namespace external
}  // namespace seastack
