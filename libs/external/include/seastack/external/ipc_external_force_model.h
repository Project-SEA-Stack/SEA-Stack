/*********************************************************************
 * @file  ipc_external_force_model.h
 * @brief Out-of-process IExternalForceModel over TCP loopback + JSON.
 *********************************************************************/
#ifndef SEASTACK_EXTERNAL_IPC_EXTERNAL_FORCE_MODEL_H
#define SEASTACK_EXTERNAL_IPC_EXTERNAL_FORCE_MODEL_H

#include <seastack/external/external_force_model.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace seastack {
namespace external {

struct IpcExternalForceOptions {
    /// Child argv. First element is the executable. SEA-Stack appends
    /// `--seastack-port <N>` before launch. Empty means do not spawn
    /// (caller must connect a client to the listening port — test use).
    std::vector<std::string> command;
    /// Socket and handshake timeout [ms].
    int timeout_ms = 10000;
    /// Working directory for the child process (empty = inherit).
    std::string working_directory;
};

/**
 * @brief Spawns (optional) an external process and exchanges protocol messages
 *        over a 127.0.0.1 TCP socket.
 *
 * Ownership: listen socket is created in the constructor; Initialize() accepts
 * the client connection and completes the handshake. Shutdown() closes the
 * socket and joins/terminates the child.
 */
class IpcExternalForceModel : public IExternalForceModel {
  public:
    explicit IpcExternalForceModel(IpcExternalForceOptions options);
    ~IpcExternalForceModel() override;

    IpcExternalForceModel(const IpcExternalForceModel&) = delete;
    IpcExternalForceModel& operator=(const IpcExternalForceModel&) = delete;

    /// Port the host is listening on (valid after construction, before Shutdown).
    int listening_port() const { return listen_port_; }

    ExternalMeta Initialize(const ExternalInit& init) override;
    void Evaluate(double time,
                  const std::vector<double>& in,
                  std::vector<double>& out) override;
    void Reset() override;
    void Commit() override;
    void Rollback() override;
    void Shutdown() override;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    IpcExternalForceOptions options_;
    int listen_port_ = 0;
    bool initialized_ = false;
    bool shutdown_ = false;
    ExternalInit init_{};
    double last_dt_ = 0.0;

    void SendRequest(const std::string& json);
    std::string RecvReply();
    void RequireOk(const std::string& reply, const char* context);
};

}  // namespace external
}  // namespace seastack

#endif  // SEASTACK_EXTERNAL_IPC_EXTERNAL_FORCE_MODEL_H
