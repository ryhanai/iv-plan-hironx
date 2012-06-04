# -*- Makefile -*-

ARCH=$(shell uname -m)
#ifeq ($(USER), leus)
#	SSH_USER = eus
#else
#	SSH_USER = $(USER)
#endif


PQP_DIR = pqp-1.3/PQP_v1.3
PQP_ACV_DIR = http://gamma.cs.unc.edu/software/downloads/SSV
PQP_ACV = pqp-1.3.tar.gz

VPYTHON_VER = 5.32
VPYTHON_DIR = visual
VPYTHON_ACV_DIR = http://rtm-ros-robotics.googlecode.com/svn/wiki
ifeq ($(ARCH),x86_64)
VPYTHON_ACV = visual-$(VPYTHON_VER)_release.patched.x86_64.tgz
else
VPYTHON_ACV = visual-$(VPYTHON_VER)_release.patched.x86.tgz
endif

IK_DIR = ikfast

SUBDIRS = $(IK_DIR) $(PQP_DIR)


.PHONY: all $(SUBDIRS)

all: download untar $(SUBDIRS) 

download: pqp-download vpython-download
pqp-download:
	$(if $(shell find ./ -name $(PQP_ACV)), $(donothing), $(pqp-download))
vpython-download:
	$(if $(shell find ./ -name $(VPYTHON_ACV)), $(donothing), $(vpython-download))

define pqp-download
	wget $(PQP_ACV_DIR)/$(PQP_ACV)
endef
define vpython-download
	wget $(VPYTHON_ACV_DIR)/$(VPYTHON_ACV)
endef

untar: pqp-untar vpython-untar
pqp-untar:
	$(if $(shell find ./ -maxdepth 1 -name $(PQP_DIR)), $(donothing), $(pqp-untar))
vpython-untar:
	$(if $(shell find ./ -maxdepth 1 -name $(VPYTHON_DIR)), $(donothing), $(vpython-untar))
define pqp-untar
	(tar xvfz $(PQP_ACV) && patch -p0 < pqp.patch && cd $(PQP_DIR) && make)
endef
define vpython-untar
	tar xvfz $(VPYTHON_ACV)
endef

define do-nothing
endef

$(SUBDIRS):
	$(MAKE) -C $@

vpython-src:
	(cd "visual-5.32_release.patched101113" && ./configure && make)

clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean ;\
	done; \
	rm -rf $(PQP_DIR) $(VPYTHON_DIR)

dist-clean: clean
	rm -f $(PQP_ACV) $(VPYTHON_ACV)
