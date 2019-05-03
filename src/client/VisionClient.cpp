#include <chrono>

#include <iostream>
#include "VisionClient.h"
#include "client_config.h"

using namespace rhoban_utils;

namespace rhoban_ssl
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
}

// internal allocator for arena

#define PROTOBUF_ALLOC_BLOCK 1024 * 1024

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
  if (packet->ParseFromArray(buffer, len))
  {
    VisionDataGlobal::singleton_.last_packets_.push_back(packet);
    return true;
  }
  else
  {
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

}  // namespace rhoban_ssl
