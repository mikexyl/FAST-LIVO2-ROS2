#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

// Direct CDR transport to the MCAP writer, independent of DDS subscriptions.
// Both this queue and the OS pipe are bounded. Writer failures fail the run.
class ResearchExport {
public:
  struct Packet {
    nlohmann::json metadata;
    std::vector<std::vector<uint8_t>> messages;
    template<class T> void add(const std::string& topic, const std::string& type, const T& msg) {
      rclcpp::SerializedMessage serialized;
      rclcpp::Serialization<T>().serialize_message(&msg, &serialized);
      const auto& buffer = serialized.get_rcl_serialized_message();
      messages.emplace_back(buffer.buffer, buffer.buffer + buffer.buffer_length);
      metadata["messages"].push_back({{"topic", topic}, {"type", type}, {"size", buffer.buffer_length}});
    }
  };
  ResearchExport(const std::string& python, const std::string& script,
                 const std::string& output, size_t capacity);
  ~ResearchExport();
  void enqueue(Packet packet);
  void close();
private:
  void work();
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Packet> queue_;
  size_t capacity_;
  bool closing_ = false, closed_ = false;
  std::exception_ptr error_;
  int fd_ = -1, pid_ = -1;
  std::thread thread_;
};
