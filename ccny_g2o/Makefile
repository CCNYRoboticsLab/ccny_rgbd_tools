all: g2o

SVN_DIR   = g2o-svn
BUILD_DIR = $(SVN_DIR)/build
TOP_DIR = $(PWD)

SVN_URL =  https://svn.openslam.org/data/svn/g2o/trunk
SVN_REVISION = -r 30

# needed since certificate of openslam expired
SVN_CMDLINE = svn --no-auth-cache --trust-server-cert --non-interactive
#SVN_CMDLINE = svn --non-interactive --config-dir $(TOP_DIR)/svnconf

# patch changes #include <cholmod.h> to <suitesparse/cholmod.h>
SVN_PATCH = ./patch_suitesparse 

include $(shell rospack find mk)/svn_checkout.mk

g2o: $(SVN_DIR) patched
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ../ -DCMAKE_INSTALL_PREFIX=$(TOP_DIR) 
	cd $(BUILD_DIR) && make install
	touch g2o

clean:
	cd $(BUILD_DIR) && make clean 
	rm -rf g2o patched

wipe: clean
	-rm -rf $(SVN_DIR) include lib bin  