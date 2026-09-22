#include "research_export.h"
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <cerrno>
#include <system_error>
extern char **environ;

namespace {
void writeAll(int fd, const void* data, size_t size) {
  auto p = static_cast<const uint8_t*>(data);
  while (size) {
    auto n = ::write(fd, p, size);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) throw std::system_error(errno, std::generic_category(), "research export write");
    p += n; size -= n;
  }
}
}
ResearchExport::ResearchExport(const std::string& python, const std::string& script,
                               const std::string& output, size_t capacity) : capacity_(capacity) {
  if (!capacity_) throw std::invalid_argument("research.queue_capacity must be positive");
  int pipefd[2];
  if (pipe(pipefd)) throw std::system_error(errno, std::generic_category());
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, pipefd[0], STDIN_FILENO);
  posix_spawn_file_actions_addclose(&actions, pipefd[0]);
  posix_spawn_file_actions_addclose(&actions, pipefd[1]);
  const char* argv[] = {python.c_str(), script.c_str(), "--output", output.c_str(), nullptr};
  int status = posix_spawn(&pid_, python.c_str(), &actions, nullptr, const_cast<char**>(argv), environ);
  posix_spawn_file_actions_destroy(&actions);
  ::close(pipefd[0]);
  if (status) { ::close(pipefd[1]); throw std::system_error(status, std::generic_category(), "MCAP writer spawn"); }
  fd_ = pipefd[1];
  thread_ = std::thread(&ResearchExport::work, this);
}
ResearchExport::~ResearchExport() { try { close(); } catch (...) {} }
void ResearchExport::enqueue(Packet packet) {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [&]{ return error_ || closing_ || queue_.size() < capacity_; });
  if (error_) std::rethrow_exception(error_);
  if (closing_) throw std::runtime_error("research export already closed");
  queue_.push_back(std::move(packet));
  condition_.notify_all();
}
void ResearchExport::work() {
  // Convert a failed pipe into an exception without changing process-wide SIGPIPE behavior.
  sigset_t mask; sigemptyset(&mask); sigaddset(&mask, SIGPIPE);
  pthread_sigmask(SIG_BLOCK, &mask, nullptr);
  try {
    while (true) {
      Packet packet;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&]{ return closing_ || !queue_.empty(); });
        if (queue_.empty()) break;
        packet = std::move(queue_.front()); queue_.pop_front(); condition_.notify_all();
      }
      std::string header = packet.metadata.dump();
      uint32_t n = header.size();
      uint8_t prefix[4] = {uint8_t(n), uint8_t(n >> 8), uint8_t(n >> 16), uint8_t(n >> 24)};
      writeAll(fd_, prefix, 4); writeAll(fd_, header.data(), header.size());
      for (const auto& message : packet.messages) writeAll(fd_, message.data(), message.size());
    }
    // Explicit end record: an interrupted or truncated pipe cannot complete an artifact.
    uint32_t end = 0; writeAll(fd_, &end, 4);
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_); error_ = std::current_exception(); condition_.notify_all();
  }
  ::close(fd_); fd_ = -1;
}
void ResearchExport::close() {
  if (closed_) return;
  { std::lock_guard<std::mutex> lock(mutex_); closing_ = true; condition_.notify_all(); }
  thread_.join();
  int status = 0;
  while (waitpid(pid_, &status, 0) < 0) { if (errno != EINTR) throw std::system_error(errno, std::generic_category()); }
  closed_ = true;
  if (error_) std::rethrow_exception(error_);
  if (!WIFEXITED(status) || WEXITSTATUS(status)) throw std::runtime_error("MCAP export writer failed");
}
