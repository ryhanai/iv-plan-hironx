# -*- Makefile -*-

PQP_DIR = pqp-1.3/PQP_v1.3
PQP_ACV_DIR = http://gamma.cs.unc.edu/software/downloads/SSV
PQP_ACV = pqp-1.3.tar.gz

VPYTHON_VER = 5.74
VPYTHON_DIR = visual-$(VPYTHON_VER)_release
VPYTHON_ACV_DIR = http://vpython.org/contents/download
VPYTHON_ACV = visual-$(VPYTHON_VER)_release.tar.bz2

POLYGON_VER = 2.0.6
POLYGON_DIR = Polygon2-$(POLYGON_VER)
POLYGON_ACV_DIR = https://bitbucket.org/jraedler/polygon2/downloads
POLYGON_ACV = Polygon2-$(POLYGON_VER).zip

FONTTOOLS_VER = 2.3
FONTTOOLS_DIR = fonttools-$(FONTTOOLS_VER)
FONTTOOLS_ACV_DIR = http://sourceforge.net/projects/fonttools/files/$(FONTTOOLS_VER)
FONTTOOLS_ACV = fonttools-$(FONTTOOLS_VER).tar.gz

TTFQUERY_VER = 1.0.5
TTFQUERY_DIR = TTFQuery-$(TTFQUERY_VER)
TTFQUERY_ACV_DIR = http://pypi.python.org/packages/source/T/TTFQuery
TTFQUERY_ACV = TTFQuery-$(TTFQUERY_VER).tar.gz

IK_DIR = ikfast

SUBDIRS = $(IK_DIR) $(PQP_DIR)

.PHONY: all $(SUBDIRS) uncomp

all: download uncomp $(SUBDIRS)


download: pqp-download vpython-download polygon-download fonttools-download ttfquery-download
pqp-download:
	$(if $(shell find ./ -name $(PQP_ACV)), $(donothing), $(pqp-download))
vpython-download:
	$(if $(shell find ./ -name $(VPYTHON_ACV)), $(donothing), $(vpython-download))
polygon-download:
	$(if $(shell find ./ -name $(POLYGON_ACV)), $(donothing), $(polygon-download))
fonttools-download:
	$(if $(shell find ./ -name $(FONTTOOLS_ACV)), $(donothing), $(fonttools-download))
ttfquery-download:
	$(if $(shell find ./ -name $(TTFQUERY_ACV)), $(donothing), $(ttfquery-download))
define pqp-download
	wget $(PQP_ACV_DIR)/$(PQP_ACV)
endef
define vpython-download
	wget $(VPYTHON_ACV_DIR)/$(VPYTHON_ACV)
endef
define polygon-download
	wget $(POLYGON_ACV_DIR)/$(POLYGON_ACV)
endef
define fonttools-download
	wget $(FONTTOOLS_ACV_DIR)/$(FONTTOOLS_ACV)
endef
define ttfquery-download
	wget $(TTFQUERY_ACV_DIR)/$(TTFQUERY_ACV)
endef
define do-nothing
endef


uncomp: pqp-uncomp vpython-uncomp polygon-uncomp fonttools-uncomp ttfquery-uncomp
pqp-uncomp:
	$(if $(shell find ./ -maxdepth 1 -name $(PQP_DIR)), $(donothing), $(pqp-uncomp))
vpython-uncomp:
	$(if $(shell find ./ -maxdepth 1 -name $(VPYTHON_DIR)), $(donothing), $(vpython-uncomp))
polygon-uncomp:
	$(if $(shell find ./ -maxdepth 1 -name $(POLYGON_DIR)), $(donothing), $(polygon-uncomp))
fonttools-uncomp:
	$(if $(shell find ./ -maxdepth 1 -name $(FONTTOOLS_DIR)), $(donothing), $(fonttools-uncomp))
ttfquery-uncomp:
	$(if $(shell find ./ -maxdepth 1 -name $(TTFQUERY_DIR)), $(donothing), $(ttfquery-uncomp))
define pqp-uncomp
	(tar xvfz $(PQP_ACV) && patch -p0 < pqp.patch && cd $(PQP_DIR) && make)
endef
define vpython-uncomp
	(make message && echo "$'\n' if not yet, ctrl+c $'\n'" && read keyin)
	(tar jxf $(VPYTHON_ACV) && cd $(VPYTHON_DIR) && ./configure && make)
	(ln -s $(VPYTHON_DIR) visual)
endef
define polygon-uncomp
	(unzip $(POLYGON_ACV) && cd $(POLYGON_DIR) && \
	python setup.py build && sudo python setup.py install)
endef
define fonttools-uncomp
	(tar xvfz $(FONTTOOLS_ACV) && cd $(FONTTOOLS_DIR) && \
	python setup.py build && sudo python setup.py install)
endef
define ttfquery-uncomp
	(tar xvfz $(TTFQUERY_ACV) && cd $(TTFQUERY_DIR) && \
	python setup.py build && sudo python setup.py install)
endef
define do-nothing
endef

message:
	@echo "$'\n !!! Did you finish the following correction? !!!"
	@echo "==================================================================="
	@echo " --- boost/python/detail/translate_exception.hpp (revision 50228)"
	@echo " +++ boost/python/detail/translate_exception.hpp (working copy)"
	@echo " @@ -9,6 +9,7 @@ $'\n"
	@echo " # include <boost/call_traits.hpp>"
	@echo " # include <boost/type_traits/add_const.hpp>"
	@echo " +# include <boost/type_traits/add_reference.hpp> $'\n"
	@echo " # include <boost/function/function0.hpp>"
	@echo "--------------------------------------"

$(SUBDIRS):
	$(MAKE) -C $@

clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean ;\
	done; \
	rm -rf $(VPYTHON_DIR) $(POLYGON_DIR) $(FONTTOOLS_DIR) $(TTFQUERY_DIR)

dist-clean: clean
	rm -f $(VPYTHON_ACV) $(POLYGON_ACV) $(FONTTOOLS_ACV) $(TTFQUERY_ACV)




