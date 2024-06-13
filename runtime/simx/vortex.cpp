// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <common.h>

#include <util.h>
#include <processor.h>
#include <arch.h>
#include <mem.h>
#include <constants.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <future>
#include <chrono>

using namespace vortex;

#ifdef VM_ENABLE  

#ifndef NDEBUG
#define DBGPRINT(format, ...) do { printf("[VXDRV] " format "", ##__VA_ARGS__); } while (0)
#else
#define DBGPRINT(format, ...) ((void)0)
#endif

#define CHECK_ERR(_expr, _cleanup)              \
    do {                                        \
        auto err = _expr;                       \
        if (err == 0)                           \
            break;                              \
        printf("[VXDRV] Error: '%s' returned %d!\n", #_expr, (int)err); \
        _cleanup                                \
    } while (false)

///////////////////////////////////////////////////////////////////////////////
//
#include <bitset>
#include <unistd.h>

uint64_t bits(uint64_t addr, uint8_t s_idx, uint8_t e_idx)
{
    return (addr >> s_idx) & ((1 << (e_idx - s_idx + 1)) - 1);
}
bool bit(uint64_t addr, uint8_t idx)
{
    return (addr) & (1 << idx);
}
#endif 

class vx_device {
public:
  vx_device()
    : arch_(NUM_THREADS, NUM_WARPS, NUM_CORES)
    , ram_(0, RAM_PAGE_SIZE)
    , processor_(arch_)
    , global_mem_(ALLOC_BASE_ADDR,
                  GLOBAL_MEM_SIZE - ALLOC_BASE_ADDR,
                  RAM_PAGE_SIZE,
                  CACHE_BLOCK_SIZE)
  {
    // attach memory module
    processor_.attach_ram(&ram_);
#ifdef VM_ENABLE  
        //Set
        set_processor_satp(VM_ADDR_MODE);
#endif
  }

  ~vx_device() {
    if (future_.valid()) {
      future_.wait();
    }
  }

  int init() {
    return 0;
  }

  int get_caps(uint32_t caps_id, uint64_t *value) {
    uint64_t _value;
    switch (caps_id) {
    case VX_CAPS_VERSION:
      _value = IMPLEMENTATION_ID;
      break;
    case VX_CAPS_NUM_THREADS:
      _value = NUM_THREADS;
      break;
    case VX_CAPS_NUM_WARPS:
      _value = NUM_WARPS;
      break;
    case VX_CAPS_NUM_CORES:
      _value = NUM_CORES * NUM_CLUSTERS;
      break;
    case VX_CAPS_CACHE_LINE_SIZE:
      _value = CACHE_BLOCK_SIZE;
      break;
    case VX_CAPS_GLOBAL_MEM_SIZE:
      _value = GLOBAL_MEM_SIZE;
      break;
    case VX_CAPS_LOCAL_MEM_SIZE:
      _value = (1 << LMEM_LOG_SIZE);
      break;
    case VX_CAPS_ISA_FLAGS:
      _value = ((uint64_t(MISA_EXT))<<32) | ((log2floor(XLEN)-4) << 30) | MISA_STD;
      break;
    default:
      std::cout << "invalid caps id: " << caps_id << std::endl;
      std::abort();
      return -1;
    }
    *value = _value;
    return 0;
  }

#ifdef VM_ENABLE  
    // VM SUPPORT
    uint64_t map_local_mem(uint64_t size, uint64_t* dev_maddr) 
    {
        bool is_pc = false;
        std::cout << "startup addr: " << std::hex << STARTUP_ADDR << std::endl;
        std::cout << "bit mode: " << std::dec << XLEN << std::endl;
        if (get_mode() == VA_MODE::BARE)
            return 0;

        if (*dev_maddr == STARTUP_ADDR || *dev_maddr == 0x7FFFF000) {
            is_pc = true;
        }

        uint64_t ppn = *dev_maddr >> 12;
        uint64_t init_pAddr = *dev_maddr;
        uint64_t init_vAddr = *dev_maddr + 0xf0000000; // vpn will change, but we want to return the vpn of the beginning of the virtual allocation
        init_vAddr = (init_vAddr >> 12) << 12;
        uint64_t vpn;

        //dev_maddr can be of size greater than a page, but we have to map and update
        //page tables on a page table granularity. So divide the allocation into pages.
        for (ppn = (*dev_maddr) >> 12; ppn < ((*dev_maddr) >> 12) + (size/RAM_PAGE_SIZE) + 1; ppn++)
        {
            //Currently a 1-1 mapping is used, this can be changed here to support different
            //mapping schemes
            vpn = is_pc ? ppn : ppn + 0xf0000;
            //vpn = ppn;

            //If ppn to vpn mapping doesnt exist.
            if (addr_mapping.find(vpn) == addr_mapping.end())
            {
                //Create mapping.
                update_page_table(ppn, vpn);
                addr_mapping[vpn] = ppn;
            }
        }

        std::cout << "mapped virtual addr: " << init_vAddr << " to physical addr: " << init_pAddr << std::endl;
        uint64_t size_bits;
        if (is_pc) {
            std::cout << "not returning virtual address because it is PC or stack" << std::endl;
            std::pair<uint64_t, uint8_t> ptw_access = page_table_walk(init_vAddr - 0xf0000000, &size_bits);
            return 0;
        } else {
            std::pair<uint64_t, uint8_t> ptw_access = page_table_walk(init_vAddr, &size_bits);
        }
        *dev_maddr = init_vAddr; // commit vpn to be returned to host
        return 0;
    }
#endif 

    int mem_alloc(uint64_t size, int flags, uint64_t* dev_addr) {
        uint64_t addr;
        CHECK_ERR(global_mem_.allocate(size, &addr), {
            return err;
        });
        CHECK_ERR(this->mem_access(addr, size, flags), {
            global_mem_.release(addr);
            return err;
        });
#ifdef VM_ENABLE
        std::cout << "physical addr: " << std::hex << *dev_addr << std::endl;
        // VM address translation
        map_local_mem(size, dev_addr);
        std::cout << "virtual addr: " << std::hex << *dev_addr << std::endl;
#endif
        *dev_addr = addr;
        return 0;
    }

  int mem_reserve(uint64_t dev_addr, uint64_t size, int flags) {
    CHECK_ERR(global_mem_.reserve(dev_addr, size), {
      return err;
    });
    CHECK_ERR(this->mem_access(dev_addr, size, flags), {
      global_mem_.release(dev_addr);
      return err;
    });
    return 0;
  }

  int mem_free(uint64_t dev_addr) {
    return global_mem_.release(dev_addr);
  }

  int mem_access(uint64_t dev_addr, uint64_t size, int flags) {
    uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
    if (dev_addr + asize > GLOBAL_MEM_SIZE)
      return -1;

    ram_.set_acl(dev_addr, size, flags);
    return 0;
  }

  int mem_info(uint64_t* mem_free, uint64_t* mem_used) const {
    if (mem_free)
      *mem_free = global_mem_.free();
    if (mem_used)
      *mem_used = global_mem_.allocated();
    return 0;
  }

  int upload(uint64_t dest_addr, const void* src, uint64_t size) {
    uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
    if (dest_addr + asize > GLOBAL_MEM_SIZE)
      return -1;

#ifdef VM_ENABLE
        uint64_t pAddr = dest_addr; // map_local_mem overwrites the provided dest_addr, so store away physical destination address
        if (dest_addr >= STARTUP_ADDR) {
            map_local_mem(asize,&dest_addr);
        } else if (dest_addr >= 0x7fff0000)
        {
            map_local_mem(asize,&dest_addr);
        }
        std::cout << "uploading to 0x" << pAddr << "(VA)" << std::endl;
        dest_addr = pAddr;
#endif

    ram_.enable_acl(false);
    ram_.write((const uint8_t*)src, dest_addr, size);
    ram_.enable_acl(true);

    /*DBGPRINT("upload %ld bytes to 0x%lx\n", size, dest_addr);
    for (uint64_t i = 0; i < size && i < 1024; i += 4) {
        DBGPRINT("  0x%lx <- 0x%x\n", dest_addr + i, *(uint32_t*)((uint8_t*)src + i));
    }*/

    return 0;
  }

  int download(void* dest, uint64_t src_addr, uint64_t size) {
    uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
    if (src_addr + asize > GLOBAL_MEM_SIZE)
      return -1;

    ram_.enable_acl(false);
    ram_.read((uint8_t*)dest, src_addr, size);
    ram_.enable_acl(true);

    /*DBGPRINT("download %ld bytes from 0x%lx\n", size, src_addr);
    for (uint64_t i = 0; i < size && i < 1024; i += 4) {
        DBGPRINT("  0x%lx -> 0x%x\n", src_addr + i, *(uint32_t*)((uint8_t*)dest + i));
    }*/

    return 0;
  }

  int start(uint64_t krnl_addr, uint64_t args_addr) {
    // ensure prior run completed
    if (future_.valid()) {
      future_.wait();
    }

    // set kernel info
    this->dcr_write(VX_DCR_BASE_STARTUP_ADDR0, krnl_addr & 0xffffffff);
    this->dcr_write(VX_DCR_BASE_STARTUP_ADDR1, krnl_addr >> 32);
    this->dcr_write(VX_DCR_BASE_STARTUP_ARG0, args_addr & 0xffffffff);
    this->dcr_write(VX_DCR_BASE_STARTUP_ARG1, args_addr >> 32);

    // start new run
    future_ = std::async(std::launch::async, [&]{
      processor_.run();
    });

    // clear mpm cache
    mpm_cache_.clear();

    return 0;
  }

  int ready_wait(uint64_t timeout) {
    if (!future_.valid())
      return 0;
    uint64_t timeout_sec = timeout / 1000;
    std::chrono::seconds wait_time(1);
    for (;;) {
      // wait for 1 sec and check status
      auto status = future_.wait_for(wait_time);
      if (status == std::future_status::ready)
        break;
      if (0 == timeout_sec--)
        return -1;
    }
    return 0;
  }

  int dcr_write(uint32_t addr, uint32_t value) {
    if (future_.valid()) {
      future_.wait(); // ensure prior run completed
    }
    processor_.dcr_write(addr, value);
    dcrs_.write(addr, value);
    return 0;
  }

  int dcr_read(uint32_t addr, uint32_t* value) const {
    return dcrs_.read(addr, value);
  }

  int mpm_query(uint32_t addr, uint32_t core_id, uint64_t* value) {
    uint32_t offset = addr - VX_CSR_MPM_BASE;
    if (offset > 31)
      return -1;
    if (mpm_cache_.count(core_id) == 0) {
      uint64_t mpm_mem_addr = IO_MPM_ADDR + core_id * 32 * sizeof(uint64_t);
      CHECK_ERR(this->download(mpm_cache_[core_id].data(), mpm_mem_addr, 32 * sizeof(uint64_t)), {
        return err;
      });
    }
    *value = mpm_cache_.at(core_id).at(offset);
    return 0;
  }

#ifdef VM_ENABLE
    /* VM Management */
    void set_processor_satp(VA_MODE mode)
    {
        uint32_t satp;
        if (mode == VA_MODE::BARE)
            satp = 0;
        else if (mode == VA_MODE::SV32)
        {
            satp = (alloc_page_table() >> 10) | 0x80000000;
            // satp = 0xFEBFE000 ;
        }
        processor_.set_satp(satp);
    }

    uint32_t get_ptbr()
    {        
        // return processor_.get_satp();
        return processor_.get_satp() & 0x003fffff;
    }

    VA_MODE get_mode()
    {
        return processor_.get_satp() & 0x80000000 ? VA_MODE::SV32 : VA_MODE::BARE;
        // return VA_MODE::SV32; 
    }  

    void update_page_table(uint64_t pAddr, uint64_t vAddr) {
        std::cout << "mapping vpn: " << vAddr << " to ppn:" << pAddr << std::endl;
        //Updating page table with the following mapping of (vAddr) to (pAddr).
        uint64_t ppn_1, pte_addr, pte_bytes;
        uint64_t vpn_1 = bits(vAddr, 10, 19);
        uint64_t vpn_0 = bits(vAddr, 0, 9);

        //Read first level PTE.
        pte_addr = (get_ptbr() << 12) + (vpn_1 * PTE_SIZE);
        pte_bytes = read_pte(pte_addr);     
        std::cout << "[PTE] addr 0x" << std::hex << pte_addr << ", PTE 0x" << std::hex << pte_bytes << std::endl;


        if ( bit(pte_bytes, 0) && ((pte_bytes & 0xFFFFFFFF) != 0xbaadf00d))
        {
            //If valid bit set, proceed to next level using new ppn form PTE.
            std::cout << "PTE valid, continuing the walk..." << std::endl;
            ppn_1 = (pte_bytes >> 10);
        }
        else
        {
            //If valid bit not set, allocate a second level page table
            // in device memory and store ppn in PTE. Set rwx = 000 in PTE
            //to indicate this is a pointer to the next level of the page table.
            ppn_1 = (alloc_page_table() >> 12);
            pte_bytes = ( (ppn_1 << 10) | 0b0000000001) ;
            write_pte(pte_addr, pte_bytes);
        }

        //Read second level PTE.
        pte_addr = (ppn_1 << 12) + (vpn_0 * PTE_SIZE);
        pte_bytes = read_pte(pte_addr); 
        std::cout << "got pte: " << std::hex << pte_bytes << std::endl;       
    
        if ( bit(pte_bytes, 0) && ((pte_bytes & 0xFFFFFFFF) != 0xbaadf00d))
        {
            std::cout << "ERROR, shouldn't be here" << std::endl;
            //If valid bit is set, then the page is already allocated.
            //Should not reach this point, a sanity check.
        }
        else
        {
            //If valid bit not set, write ppn of pAddr in PTE. Set rwx = 111 in PTE
            //to indicate this is a leaf PTE and has the stated permissions.
            pte_bytes = ( (pAddr << 10) | 0b0000001111) ;
            write_pte(pte_addr, pte_bytes);

            //If super paging is enabled.
            if (SUPER_PAGING)
            {
                //Check if this second level Page Table can be promoted to a super page. Brute force 
                //method is used to iterate over all PTE entries of the table and check if they have 
                //their valid bit set.
                bool superpage = true;
                for(int i = 0; i < 1024; i++)
                {
                    pte_addr = (ppn_1 << 12) + (i * PTE_SIZE);
                    pte_bytes = read_pte(pte_addr); 
                  
                    if (!bit(pte_bytes, 0) && ((pte_bytes & 0xFFFFFFFF) != 0xbaadf00d))
                    {
                        superpage = false;
                        break;
                    }
                }
                if (superpage)
                {
                    //This can be promoted to a super page. Set root PTE to the first PTE of the 
                    //second level. This is because the first PTE of the second level already has the
                    //correct PPN1, PPN0 set to zero and correct access bits.
                    pte_addr = (ppn_1 << 12);
                    pte_bytes = read_pte(pte_addr);
                    pte_addr = (get_ptbr() << 12) + (vpn_1 * PTE_SIZE);
                    write_pte(pte_addr, pte_bytes);
                }
            }
        }
    }

    std::pair<uint64_t, uint8_t> page_table_walk(uint64_t vAddr_bits, uint64_t* size_bits)
    {   
        uint64_t LEVELS = 2;
        vAddr_SV32_t vAddr(vAddr_bits);
        uint64_t pte_bytes;

        std::cout << "PTW on vAddr: 0x" << std::hex << vAddr_bits << std::endl;

        //Get base page table.
        uint64_t a = this->processor_.get_satp() << 12;
        std::cout << "PTW SATP: 0x" << a << std::endl;
        int i = LEVELS - 1; 

        while(true)
        {

        //Read PTE.
        std::cout << "reading PTE from RAM addr 0x" << std::hex << (a+vAddr.vpn[i]*PTE_SIZE) << std::endl;
        ram_.read(&pte_bytes, a+vAddr.vpn[i]*PTE_SIZE, sizeof(uint64_t));
        //pte_bytes &= 0x00000000FFFFFFFF;
        PTE_SV32_t pte(pte_bytes);
        std::cout << "got pte: " << std::hex << pte_bytes << std::endl;
        
        //Check if it has invalid flag bits.
        if ( (pte.v == 0) | ( (pte.r == 0) & (pte.w == 1) ) )
        {
            std::cout << "Error on vAddr 0x" << std::hex << vAddr_bits << std::endl;
            throw Page_Fault_Exception("Page Fault : Attempted to access invalid entry. Entry: 0x");
        }

        if ( (pte.r == 0) & (pte.w == 0) & (pte.x == 0))
        {
            //Not a leaf node as rwx == 000
            i--;
            if (i < 0)
            {
            throw Page_Fault_Exception("Page Fault : No leaf node found.");
            }
            else
            {
            //Continue on to next level.
            a = (pte_bytes >> 10 ) << 12;
            std::cout << "next a: " << a << std::endl;
            }
        }
        else
        {
            //Leaf node found, finished walking.
            a = (pte_bytes >> 10 ) << 12;
            break;
        }
        }

        PTE_SV32_t pte(pte_bytes);

        //Check RWX permissions according to access type.
        if (pte.r == 0)
        {
        throw Page_Fault_Exception("Page Fault : TYPE LOAD, Incorrect permissions.");
        }

        uint64_t pfn;
        if (i > 0)
        {
        //It is a super page.
        if (pte.ppn[0] != 0)
        {
            //Misss aligned super page.
            throw Page_Fault_Exception("Page Fault : Miss Aligned Super Page.");

        }
        else
        {
            //Valid super page.
            pfn = pte.ppn[1];
            *size_bits = 22;
        }
        }
        else
        {
        //Regular page.
        *size_bits = 12;
        pfn = a >> 12;
        }
        return std::make_pair(pfn, pte_bytes & 0xff);
    }

    uint64_t alloc_page_table() {
        uint64_t addr;
        global_mem_.allocate(RAM_PAGE_SIZE, &addr);
        std::cout << "address of page table 0x" << std::hex << addr << std::endl;
        init_page_table(addr);
        return addr;
    }


    void init_page_table(uint64_t addr) {
        uint64_t asize = aligned_size(RAM_PAGE_SIZE, CACHE_BLOCK_SIZE);
        uint8_t *src = new uint8_t[RAM_PAGE_SIZE];
        for (uint64_t i = 0; i < RAM_PAGE_SIZE; ++i) {
            src[i] = (0x00000000 >> ((i & 0x3) << 3)) & 0xff;
        }
        ram_.enable_acl(false);
        ram_.write((const uint8_t*)src, addr, asize);
        ram_.enable_acl(true);
    }

    void read_page_table(uint64_t addr) {
        uint8_t *dest = new uint8_t[RAM_PAGE_SIZE];
        download(dest,  addr,  RAM_PAGE_SIZE);
        printf("VXDRV: download %d bytes from 0x%x\n", RAM_PAGE_SIZE, addr);
        for (int i = 0; i < RAM_PAGE_SIZE; i += 4) {
            printf("mem-read: 0x%x -> 0x%x\n", addr + i, *(uint64_t*)((uint8_t*)dest + i));
        }
    }

    void write_pte(uint64_t addr, uint64_t value = 0xbaadf00d) {
        std::cout << "writing pte " << std::hex << value << " to pAddr: " << std::hex << addr << std::endl;
        uint8_t *src = new uint8_t[PTE_SIZE];
        for (uint64_t i = 0; i < PTE_SIZE; ++i) {
            src[i] = (value >> ((i & 0x3) * 8)) & 0xff;
        }
        //std::cout << "writing PTE to RAM addr 0x" << std::hex << addr << std::endl;
        ram_.enable_acl(false);
        ram_.write((const uint8_t*)src, addr, PTE_SIZE);
        ram_.enable_acl(true);
    }

    uint64_t read_pte(uint64_t addr) {
        uint8_t *dest = new uint8_t[PTE_SIZE];
        std::cout << "[read_pte] reading PTE from RAM addr 0x" << std::hex << addr << std::endl;
        ram_.read((uint8_t*)dest, addr, PTE_SIZE);
        return *(uint64_t*)((uint8_t*)dest);
    }
#endif // JAEWON

private:
  Arch                arch_;
  RAM                 ram_;
  Processor           processor_;
  MemoryAllocator     global_mem_;
  DeviceConfig        dcrs_;
  std::future<void>   future_;
  std::unordered_map<uint32_t, std::array<uint64_t, 32>> mpm_cache_;
#ifdef VM_ENABLE
    std::unordered_map<uint64_t, uint64_t> addr_mapping;
#endif
};

#include <callbacks.inc>
