all: g2o

SVN_DIR   = g2o-svn
BUILD_DIR = $(SVN_DIR)/build
TOP_DIR = $(PWD)

SVN_URL =  https://svn.openslam.org/data/svn/g2o/trunk
SVN_REVISION = -r 30

# needed since certificate of openslam expired
SVN_CMDLINE = svn --no-auth-cache --trust-server-cert --non-interactive

include $(shell rospack find mk)/svn_checkout.mk

g2o: $(SVN_DIR)
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ../ -DCMAKE_INSTALL_PREFIX=$(TOP_DIR) 
	cd $(BUILD_DIR) && make install
	touch g2o

clean:
	cd $(BUILD_DIR) && make clean 
	rm -rf g2o

wipe:
	-rm -rf $(SVN_DIR) include lib bin  
