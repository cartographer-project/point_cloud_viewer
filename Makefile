.PHONY: protobuf
protobuf: protobuf_build/protoc

protobuf_build/protoc:
	mkdir -p protobuf_build
	cd protobuf_build && cmake -G Ninja \
	  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
	  -DCMAKE_BUILD_TYPE=Release \
	  -Dprotobuf_BUILD_TESTS=OFF \
	  ../third_party/protobuf/cmake
	ninja