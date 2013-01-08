all: installed

SVN_DIR = build/toro-svn
BUILD_DIR = $(SVN_DIR)/trunk
SVN_URL = https://svn.openslam.org/data/svn/toro
SVN_REVISION = -r 29
SVN_PATCH = ./patch_fpic
include $(shell rospack find mk)/svn_checkout.mk

#patch:
#ifneq (,$(wildcard $(BUILD_DIR)/.patched))
#	echo Patching TORO
#	cd $(BUILD_DIR) && patch -p0 < ../../../patch_namespace
#	touch patched
#else
#	echo TORO previously patched, use 'make wipe' to completely remove build directory.
#endif

installed: $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)
	mkdir -p ./include/ccny_toro
	mkdir -p ./lib
	cd $(BUILD_DIR) && make
	cp $(BUILD_DIR)/*.hh ./include/ccny_toro
	cp $(BUILD_DIR)/*.hxx ./include/ccny_toro
	cp $(BUILD_DIR)/*.a ./lib
	touch installed

clean:
	cd $(BUILD_DIR) && make clean
	rm -rf installed patched

wipe: clean
	rm -rf ./include ./lib ./build $(SVN_DIR)  

