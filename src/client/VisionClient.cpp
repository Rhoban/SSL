#include "VisionClient.h"

#include <chrono>
#include <iostream>
#include "client_config.h"
#include "../ai/debug.h"

namespace rhoban_ssl
{
namespace vision
{
VisionClient::VisionClient(bool simulation, std::string addr, std::string port, std::string sim_port)
  : MulticastClient(addr, port)
{
  if (simulation)
  {
    port = sim_port;
  }
  std::cout << "Vision client (simulation=" << ((simulation) ? "True" : "False") << "): " << addr << ":" << port
            << std::endl;

  init();
}

SSL_WrapperPacket VisionClient::getData()
{
  SSL_WrapperPacket tmp;

  mutex.lock();
  tmp = data_;
  mutex.unlock();

  return tmp;
}

bool VisionClient::process(char* buffer, size_t len)
{
  SSL_WrapperPacket packet;

  if (packet.ParseFromArray(buffer, len))
  {
    data_ = packet;

    return true;
  }
  else
  {
    std::cerr << "Packet error!" << std::endl;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

VisionDataGlobal VisionDataGlobal::singleton_;

VisionDataGlobal::~VisionDataGlobal()
{
  delete arena_;
}

SSL_WrapperPacket* VisionDataGlobal::getNewPacket()
{
  return google::protobuf::Arena::CreateMessage<SSL_WrapperPacket>(arena_);
}

void VisionDataGlobal::reset()
{
  arena_->Reset();
  last_packets_.clear();
}

// internal allocator for arena

#define PROTOBUF_ALLOC_BLOCK 10 * 1024 * 1024

struct BlocksSet
{
  std::vector<void*> blocks;
  bool deleted;
  BlocksSet() : deleted(false)
  {
  }
  ~BlocksSet()
  {
    for (auto p : blocks)
      delete[] static_cast<char*>(p);
    deleted = true;
  }
};

static BlocksSet free_blocks;

static void* arena_allocator(size_t s)
{
  if (free_blocks.blocks.size() == 0)
    return new char[PROTOBUF_ALLOC_BLOCK];
  void* p = free_blocks.blocks.back();
  free_blocks.blocks.pop_back();
  return p;
}

static void arena_deallocator(void* p, size_t)
{
  if (free_blocks.deleted)
    delete[] static_cast<char*>(p);
  else
    free_blocks.blocks.push_back(p);
}

// end of internal allocator for protobuf arena

VisionDataGlobal::VisionDataGlobal()
{
  struct google::protobuf::ArenaOptions options;
  options.start_block_size = PROTOBUF_ALLOC_BLOCK;
  options.max_block_size = PROTOBUF_ALLOC_BLOCK;
  options.block_alloc = arena_allocator;
  options.block_dealloc = arena_deallocator;
  options.initial_block = new char[PROTOBUF_ALLOC_BLOCK];
  options.initial_block_size = PROTOBUF_ALLOC_BLOCK;
  arena_ = new google::protobuf::Arena(options);
}

VisionClientSingleThread::VisionClientSingleThread(std::string addr, std::string port)
  : MulticastClientSingleThread(addr, port)
{
  init();
}

bool VisionClientSingleThread::process(char* buffer, size_t len)
{
  SSL_WrapperPacket* packet = VisionDataGlobal::singleton_.getNewPacket();

  if (packet->ParsePartialFromArray(buffer, len))
  {
    /*if (packet->has_detection())
      std::cout << packet->detection().frame_number() << std::endl;
    else if (packet->has_geometry())
      std::cout << "geometry!" << std::endl;
    else
      std::cout << "UNKNOWN!!!" << std::endl;
      */
    VisionDataGlobal::singleton_.last_packets_.push_back(packet);
    return true;
  }
  else
  {
    fprintf(stderr, "parsing error! %s ", buffer);
    // error in packet so we ignore it, it will be cleaned later by the arena
  }
  return false;
}

VisionProtoBufReset::VisionProtoBufReset(int freq) : counter_(0), freq_(freq)
{
}

bool VisionProtoBufReset::runTask()
{
  counter_ += 1;
  if (counter_ >= freq_)
  {
    VisionDataGlobal::singleton_.reset();  // this reset the arena and memory associated with protobuf objects
    counter_ = 0;
  }
  return true;
}

VisionPacketStat::VisionPacketStat(int freq) : freq_(freq), counter_(0), sum_(0), min_(100), max_(0)
{
}

bool VisionPacketStat::runTask()
{
  if (counter_ == freq_)
  {
    std::cout << "vision packet stat: (min/avg/max)" << min_ << " " << ((double)sum_) / ((double)freq_) << " " << max_
              << std::endl;
    counter_ = 0;
    min_ = 100;
    max_ = 0;
    sum_ = 0;
  }
  else
  {
    int s = VisionDataGlobal::singleton_.last_packets_.size();
    sum_ += s;
    if (s < min_)
      min_ = s;
    if (s > max_)
      max_ = s;
    counter_ += 1;
  }
  return true;
}

}  // namespace vision
}  // namespace rhoban_ssl
