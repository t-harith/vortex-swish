ROOT_DIR := $(realpath ..)
include $(ROOT_DIR)/config.mk

all: stub rtlsim simx opae xrt simbricks

stub:
	$(MAKE) -C stub

simx:
	$(MAKE) -C simx

rtlsim:
	$(MAKE) -C rtlsim

opae:
	$(MAKE) -C opae

xrt:
	$(MAKE) -C xrt

simbricks:
	$(MAKE) -C simbricks

clean:
	$(MAKE) clean -C stub
	$(MAKE) clean -C simx
	$(MAKE) clean -C rtlsim
	$(MAKE) clean -C opae
	$(MAKE) clean -C xrt
	$(MAKE) clean -C simbricks

.PHONY: all stub simx rtlsim opae xrt clean