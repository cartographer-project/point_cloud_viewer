MAKEFILE_PATH := $(lastword ${MAKEFILE_LIST})
ROOT_DIR := $(abspath $(dir ${MAKEFILE_PATH}))
PROTOBUF_DIR := ${ROOT_DIR}/target/protobuf

.PHONY: protobuf
protobuf: ${PROTOBUF_DIR}/protoc

${PROTOBUF_DIR}/protoc:
	mkdir -p ${PROTOBUF_DIR}
	$(eval TMPDIR := $(shell mktemp -d))
	git clone https://github.com/google/protobuf.git ${TMPDIR}/protobuf
	cd ${TMPDIR} && cmake -G Ninja \
	  -DCMAKE_INSTALL_PREFIX=${PROTOBUF_DIR} \
	  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
	  -DCMAKE_BUILD_TYPE=Release \
	  -Dprotobuf_BUILD_TESTS=OFF \
	  ${TMPDIR}/protobuf/cmake
	cd ${TMPDIR} && ninja
	cd ${TMPDIR} && ninja install
	rm -rf ${TMPDIR}
