ROOT_DIR := $(realpath ..)
include $(ROOT_DIR)/config.mk

all:
	$(MAKE) -C simx
	$(MAKE) -C rtlsim
	$(MAKE) -C opaesim
	$(MAKE) -C xrtsim
	$(MAKE) -C simbricks

clean:
	$(MAKE) -C simx clean
	$(MAKE) -C rtlsim clean
	$(MAKE) -C opaesim clean
	$(MAKE) -C xrtsim clean
	$(MAKE) -C simbricks clean