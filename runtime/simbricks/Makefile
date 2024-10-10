include ../common.mk

DESTDIR ?= $(CURDIR)

SRC_DIR := $(VORTEX_HOME)/runtime/simbricks

CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic -Wfatal-errors
CXXFLAGS += -I$(INC_DIR) -I$(COMMON_DIR) -I$(ROOT_DIR)/hw -I$(COMMON_DIR) -I$(SIM_DIR)/common
CXXFLAGS += -I$(SRC_DIR)
CXXFLAGS += -DXLEN_$(XLEN)

# Position independent code
CXXFLAGS += -fPIC

# Add external configuration
CXXFLAGS += $(CONFIGS)

LDFLAGS += -shared -pthread
LDFLAGS += -L$(DESTDIR)

SRCS := $(SRC_DIR)/vortex.cpp $(COMMON_DIR)/utils.cpp $(SRC_DIR)/vfio-pci.c

# Debugigng
ifdef DEBUG
	CXXFLAGS += -g -O0
else
	CXXFLAGS += -O2 -DNDEBUG
endif

# Enable perf counters
ifdef PERF
	CXXFLAGS += -DPERF_ENABLE
endif

PROJECT := libvortex.so

all: $(DESTDIR)/$(PROJECT)

$(DESTDIR)/$(PROJECT): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $@

clean:
	rm -rf $(DESTDIR)/$(PROJECT) *.o
