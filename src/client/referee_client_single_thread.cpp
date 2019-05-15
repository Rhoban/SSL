#include "referee_client_single_thread.h"

namespace rhoban_ssl
{
RefereeMessages RefereeMessages::singleton_;

RefereeMessages::~RefereeMessages()
{
  delete arena_;
}

Referee *RefereeMessages::getNewPacket()
{
  return google::protobuf::Arena::CreateMessage<Referee>(arena_);
}

void RefereeMessages::reset()
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

RefereeMessages::RefereeMessages()
{
  struct google::protobuf::ArenaOptions options;
  options.start_block_size = PROTOBUF_ALLOC_BLOCK;
  options.max_block_size = PROTOBUF_ALLOC_BLOCK;
  options.block_alloc = arena_allocator;
  options.block_dealloc = arena_deallocator;
  arena_ = new google::protobuf::Arena(options);
}

RefereeClientSingleThread::RefereeClientSingleThread(std::string addr, std::string port)
  : MulticastClientSingleThread(addr, port)
{
  init();
}

bool RefereeClientSingleThread:: process(char* buffer, size_t len)
{
  Referee* packet = RefereeMessages::singleton_.getNewPacket();
  if (packet->ParseFromArray(buffer, len))
  {
    printf("message %d read: %d\n", len, packet->command_counter());
    RefereeMessages::singleton_.last_packets_.push_back(packet);
    return true;
  }
  else
  {
    // error in packet so we ignore it, it will be cleaned later by the arena
  }
  return false;
}

RefereeProtoBufReset::RefereeProtoBufReset(int freq) : counter_(0), freq_(freq)
{
}

bool RefereeProtoBufReset::runTask()
{
  counter_ += 1;
  if (counter_ >= freq_)
  {
    RefereeMessages::singleton_.reset();  // this reset the arena and memory associated with protobuf objects
    counter_ = 0;
  }
  return true;
}
}
