// Transparent RCL publication probe: logs serialized sizes, never changes messages.
// Build against the same ROS distribution as the monitored native processes.
#include <rcl/rcl.h>
#include <rmw/rmw.h>
#include <dlfcn.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <unordered_map>

namespace {
struct Publisher { std::string topic, node; const rosidl_message_type_support_t* type; };
struct State { std::mutex mutex; std::unordered_map<const rcl_publisher_t*, Publisher> pubs; FILE* file=nullptr; };
State& state() { static auto* s=new State; return *s; }
void record(const Publisher& p, size_t bytes, int error=0) {
  auto& s=state(); std::lock_guard<std::mutex> guard(s.mutex);
  if (!s.file) {
    const char* directory=std::getenv("COMM_TRACE_DIR"); if(!directory) return;
    const std::string path=std::string(directory)+"/"+std::to_string(getpid())+".tsv";
    s.file=std::fopen(path.c_str(),"a"); if(!s.file) { std::perror("communication probe"); std::abort(); }
    std::setvbuf(s.file,nullptr,_IOLBF,0);
  }
  auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  std::fprintf(s.file,"%lld\t%s\t%s\t%zu\t%d\n",static_cast<long long>(ns),p.node.c_str(),p.topic.c_str(),bytes,error);
}
bool lookup(const rcl_publisher_t* pub, Publisher& p) {
  auto& s=state(); std::lock_guard<std::mutex> guard(s.mutex);
  auto it=s.pubs.find(pub); if(it==s.pubs.end()) return false; p=it->second; return true;
}
}

extern "C" rcl_ret_t rcl_publisher_init(rcl_publisher_t* pub,const rcl_node_t* node,
    const rosidl_message_type_support_t* type,const char* topic,const rcl_publisher_options_t* options) {
  static auto original=reinterpret_cast<decltype(&rcl_publisher_init)>(dlsym(RTLD_NEXT,"rcl_publisher_init"));
  auto result=original(pub,node,type,topic,options);
  if(result==RCL_RET_OK) {
    std::string resolved=rcl_publisher_get_topic_name(pub);
    if(resolved.find("/cslam/")!=std::string::npos) {
      std::string ns=rcl_node_get_namespace(node); if(ns=="/") ns.clear();
      auto& s=state(); std::lock_guard<std::mutex> guard(s.mutex);
      s.pubs[pub]={resolved,ns+"/"+rcl_node_get_name(node),type};
    }
  }
  return result;
}
extern "C" rcl_ret_t rcl_publisher_fini(rcl_publisher_t* pub,rcl_node_t* node) {
  static auto original=reinterpret_cast<decltype(&rcl_publisher_fini)>(dlsym(RTLD_NEXT,"rcl_publisher_fini"));
  { auto& s=state(); std::lock_guard<std::mutex> guard(s.mutex); s.pubs.erase(pub); }
  return original(pub,node);
}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* pub,const void* msg,rmw_publisher_allocation_t* allocation) {
  static auto original=reinterpret_cast<decltype(&rcl_publish)>(dlsym(RTLD_NEXT,"rcl_publish"));
  auto result=original(pub,msg,allocation);
  Publisher p;
  if(result==RCL_RET_OK && lookup(pub,p)) {
    auto data=rmw_get_zero_initialized_serialized_message(); auto allocator=rcutils_get_default_allocator();
    int error=rmw_serialized_message_init(&data,0,&allocator);
    if(!error) {
      error=rmw_serialize(msg,p.type,&data); record(p,data.buffer_length,error);
      auto cleanup=rmw_serialized_message_fini(&data); if(cleanup) record(p,0,cleanup);
    } else record(p,0,error);
  }
  return result;
}
extern "C" rcl_ret_t rcl_publish_serialized_message(const rcl_publisher_t* pub,
    const rcl_serialized_message_t* msg,rmw_publisher_allocation_t* allocation) {
  static auto original=reinterpret_cast<decltype(&rcl_publish_serialized_message)>(dlsym(RTLD_NEXT,"rcl_publish_serialized_message"));
  auto result=original(pub,msg,allocation); Publisher p;
  if(result==RCL_RET_OK && lookup(pub,p)) record(p,msg->buffer_length);
  return result;
}
