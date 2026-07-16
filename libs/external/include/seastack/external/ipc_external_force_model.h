/*********************************************************************
 * @file  ipc_external_force_model.h
 * @brief Out-of-process IExternalForceModel over TCP loopback + JSON.
 *
 * SEA-Stack listens on 127.0.0.1, spawns the user process (Python/MATLAB/…),
 * and exchanges length-prefixed JSON messages. The child connects using
 * `--seastack-port` (appended to `command` before launch).
 *
 * See docs/extending/EXTERNAL_FORCE_MODULES.md for the wire protocol.
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
 * @brief Spawns (optional) an external process and talks JSON over 127.0.0.1.
 *
 * Typical path used by run_seastack:
 *   construct (listen + spawn) -> Initialize (accept + handshake) ->
 *   Evaluate* -> Shutdown (join child).
 *
 * If `command` is empty, no child is spawned (unit tests connect a mock client).
 */
class IpcExternalForceModel : public IExternalForceModel {
  public:
    explicit IpcExternalForceModel(IpcExternalForceOptions options);
    ~IpcExternalForceModel() override;

    IpcExternalForceModel(const IpcExternalForceModel&) = delete;
    IpcExternalForceModel& operator=(const IpcExternalForceModel&) = delete;

    /// Port the host is listening on (valid after construction, before Shutdown).
    int listening_port() const { return listen_port_; }

    /// Accept child connection + send `initialize` / receive module metadata.
    ExternalMeta Initialize(const ExternalInit& init) override;
    /// One evaluate round-trip; `in`/`out` sizes must match the handshake.
    void Evaluate(double time,
                  const std::vector<double>& in,
                  std::vector<double>& out) override;
    /// Forward `reset` to the child (clear module state).
    void Reset() override;
    /// Forward `commit` (optional co-simulation hook).
    void Commit() override;
    /// Forward `rollback` (optional co-simulation hook).
    void Rollback() override;
    /// Forward `shutdown`, close sockets, terminate the child process.
    void Shutdown() override;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    IpcExternalForceOptions options_;
    int listen_port_ = 0;       ///< Ephemeral port chosen in the constructor.
    bool initialized_ = false;  ///< True after a successful Initialize().
    bool shutdown_ = false;     ///< True after Shutdown() (object is spent).
    ExternalInit init_{};       ///< Copy of the last handshake inputs.
    double last_dt_ = 0.0;      ///< Nominal dt from Initialize (sent on evaluate).

    /// Write one length-prefixed JSON request on the client socket.
    void SendRequest(const std::string& json);
    /// Read one length-prefixed JSON reply from the client socket.
    std::string RecvReply();
    /// Require `"status":"ok"` in `reply` or throw with `context`.
    void RequireOk(const std::string& reply, const char* context);
};

}  // namespace external
}  // namespace seastack

#endif  // SEASTACK_EXTERNAL_IPC_EXTERNAL_FORCE_MODEL_H
