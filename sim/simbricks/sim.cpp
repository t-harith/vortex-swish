/*
 * Copyright 2024 Max Planck Institute for Software Systems, and
 * National University of Singapore
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <util.h>
#include <mem.h>
#include <malloc.h>
#include <utils.h>
#include <VX_config.h>
#include <VX_types.h>
#include "processor.h"

extern "C" {
#include "sim.h"
//#include "../common/reg_defs.h"
}

//#define DEBUG
#ifdef DEBUG
#define dprintf(x...) do { \
    fprintf(stderr, "DEBUG [%lu]: ", main_time); \
    fprintf(stderr, x); \
  } while (0)
#else
#define dprintf(...) do { } while (0)
#endif

//#define TRACE_ENABLED
//#ifdef TRACE_ENABLED
//#include <verilated_vcd_c.h>
//#endif
//#include <Vtop.h>

// uncomment to enable debug prints
//#define DEBUG

uint64_t clock_period = 10000;

static uint64_t next_edge = 0;
static bool next_rising = false;
static int reset_done = 2;

struct MMIOOp {
  uint64_t offset;
  uint64_t val;
  bool write;
  uint64_t opaque;
};

static std::deque<MMIOOp *> mmio_queue;
static bool mmio_submitted = false;

#define RAM_PAGE_SIZE 4096
vortex::RAM               ram(0, RAM_PAGE_SIZE);
vortex::MemoryAllocator   global_mem(ALLOC_BASE_ADDR, GLOBAL_MEM_SIZE - ALLOC_BASE_ADDR, RAM_PAGE_SIZE, CACHE_BLOCK_SIZE);
vortex::Processor         *processor;

static int mem_returncode = 0xbade;
static uint64_t mem_size;
static uint64_t mem_addr;
static uint64_t mem_flags;
static uint64_t mem_cmd;

static uint64_t acl_saddr, acl_size;
static int acl_flags;

//#ifdef TRACE_ENABLED
//static VerilatedVcdC *trace;
//#endif

int InitState(void) {
  char arg0[] = "hw/hw";
  char *vargs[2] = {arg0, NULL};
  const char* program = "printf";
//  Verilated::commandArgs(1, vargs);
//#ifdef TRACE_ENABLED
//  Verilated::traceEverOn(true);
//#endif

	// create processor
  processor = new vortex::Processor();

	// attach memory module
	processor->attach_ram(&ram);

	// setup base DCRs
	const uint64_t startup_addr(STARTUP_ADDR);
	processor->dcr_write(VX_DCR_BASE_STARTUP_ADDR0, startup_addr & 0xffffffff);
#if (XLEN == 64)
    processor->dcr_write(VX_DCR_BASE_STARTUP_ADDR1, startup_addr >> 32);
#endif
	processor->dcr_write(VX_DCR_BASE_MPM_CLASS, 0);

	//// load program
	//{
	//	std::string program_ext(fileExtension(program));
	//	if (program_ext == "bin") {
	//		ram.loadBinImage(program, startup_addr);
	//	} else if (program_ext == "hex") {
	//		ram.loadHexImage(program);
	//	} else {
	//		std::cout << "*** error: only *.bin or *.hex images supported." << std::endl;
	//		return -1;
	//	}
	//}
  //top = new Vtop;

//#ifdef TRACE_ENABLED
//  trace = new VerilatedVcdC;
//  top->trace(trace, 99);
//  trace->open("out/debug.vcd");
//#endif

  //top->rst = 1;
  return 0;
}

void Finalize(void) {
  delete processor;
//#ifdef TRACE_ENABLED
//  trace->close();
//#endif
}

bool MMIOReadInfra(volatile struct SimbricksProtoPcieH2DRead *read) {
  uint64_t val = 0;

  switch(read->offset) {
    case SB_INFRA_PROC_START:
      val = processor->run(); // returns true if device is running.
      break;
    case SB_INFRA_MEM_FREE:
      val = global_mem.free();
      break;
    case SB_INFRA_MEM_USED:
      val = global_mem.allocated();
      break;
    case SB_INFRA_MEM_ADDR:
      val = mem_addr;
      break;
    case SB_INFRA_MEM_SIZE:
      val = mem_size;
      break;
    case SB_INFRA_MEM_FLAGS:
      val = mem_flags;
      break;
    case SB_INFRA_MEM_CMD:
      val = mem_cmd;
      break;
    case SB_INFRA_MEM_RETURNC:
      val = mem_returncode;
      mem_returncode = 0xbaed;
      break;
    case SB_INFRA_ACL_SET:
      break;
    case SB_INFRA_ACL_SADDR:
      val = acl_saddr;
      break;
    case SB_INFRA_ACL_SIZE:
      val = acl_size;
      break;
    case SB_INFRA_ACL_FLAGS:
      val = acl_flags;
      break;
    case SB_INFRA_ACL_EN:
      break;
    default:
      return false;
  }

  assert(read->len == 8);

  // prepare read completion
  volatile union SimbricksProtoPcieD2H *msg = AllocPcieOut();
  volatile struct SimbricksProtoPcieD2HReadcomp *rc = &msg->readcomp;
  rc->req_id = read->req_id; // set req id so host can match resp to a req

  memcpy((void *) rc->data, &val, read->len);

  // send response
  SendPcieOut(msg, SIMBRICKS_PROTO_PCIE_D2H_MSG_READCOMP);

  return true;

}

void MMIORead(volatile struct SimbricksProtoPcieH2DRead *read) {
  dprintf("MMIO Read: BAR %d offset 0x%lx len %d id %lu\n", read->bar,
    read->offset, read->len, read->req_id);

  if (MMIOReadInfra(read))
    return;

  assert(read->len == 8);

  MMIOOp *op = new MMIOOp;
  op->offset = read->offset;
  op->write = false;
  op->opaque = read->req_id;
  mmio_queue.push_back(op);
}

bool MMIOWriteInfra(volatile struct SimbricksProtoPcieH2DWrite *write) {
  uint64_t val = 0;

  assert(write->len == 8);
  assert(write->offset % write->len == 0);
  memcpy(&val, (const void *) write->data, write->len);

  switch(write->offset) {
    case SB_INFRA_PROC_START:
      processor->run();
      break;
    case SB_INFRA_MEM_FREE:
      break;
    case SB_INFRA_MEM_USED:
      break;
    case SB_INFRA_MEM_ADDR:
      mem_addr = val;
      break;
    case SB_INFRA_MEM_SIZE:
      mem_size = val;
      break;
    case SB_INFRA_MEM_FLAGS:
      mem_flags = val;
      break;
    case SB_INFRA_MEM_CMD:
      mem_cmd = val;
      if(val == SB_INFRA_MEM_CMD_ALLOC)
        mem_returncode = global_mem.allocate(mem_size, &mem_addr);
      else if (val == SB_INFRA_MEM_CMD_RESERVE)
        mem_returncode = global_mem.reserve(mem_addr, mem_size);
      else if (val == SB_INFRA_MEM_CMD_RELEASE)
        mem_returncode = global_mem.release(mem_addr);
      break;
    case SB_INFRA_MEM_RETURNC:
      break;
    case SB_INFRA_ACL_SET:
      ram.set_acl(acl_saddr, acl_size, acl_flags);
      break;
    case SB_INFRA_ACL_SADDR:
      acl_saddr = val;
      break;
    case SB_INFRA_ACL_SIZE:
      acl_size = val;
      break;
    case SB_INFRA_ACL_FLAGS:
      acl_flags = val;
      break;
    case SB_INFRA_ACL_EN:
      ram.enable_acl(val);
      break;
    default:
      return false;
  }
  return true;
}

void MMIOWrite(volatile struct SimbricksProtoPcieH2DWrite *write) {
  dprintf("MMIO Write: BAR %d offset 0x%lx len %d id %lu\n", write->bar,
    write->offset, write->len, write->req_id);

  if (MMIOWriteInfra(write))
    return;

  assert(write->len == 8);
  assert(write->offset % write->len == 0);

  MMIOOp *op = new MMIOOp;
  op->offset = write->offset;
  op->write = true;
  op->opaque = 0;
  op->opaque = write->req_id;
  memcpy(&op->val, (const void *) write->data, 8);
  mmio_queue.push_back(op);
}

static void MMIOPoll() {
  if (mmio_submitted) {
    // we have an ongoing mmio op that we submitted to the device and is now
    // done
    MMIOOp *op = mmio_queue.front();
    mmio_queue.pop_front();

    // for writes we don't have to do anything else, but for reads we do
    if (op->write) {
      dprintf("MMIO Write Complete: id %lu\n", op->opaque);
    } else {
      dprintf("MMIO Read Complete: id %lu val %lx\n", op->opaque,
        (uint64_t) 0);
      assert(false);
    }

    delete op;
    mmio_submitted = false;
  }

  if (!mmio_queue.empty()) {
    // we have a next request to submit
    MMIOOp *op = mmio_queue.front();
    if (op->write) {
      dprintf("MMIO Write Submit: id %lu\n", op->opaque);
      processor->dcr_write((uint32_t) op->offset, (uint32_t) op->val);
    } else {
      dprintf("MMIO Read Submit: id %lu\n", op->opaque);
      assert(false);
    }

    mmio_submitted = true;
  }
}


void PollEvent(void) {
  if (main_time < next_edge)
    return;

  if (next_rising) {
    MMIOPoll();
    //DMAPoll();
  }

  next_rising = !next_rising;
  
  processor->tick();
//#ifdef TRACE_ENABLED
//  trace->dump(main_time);
//#endif

  next_edge += clock_period / 2;
}

uint64_t NextEvent(void) {
  return next_edge;
}
