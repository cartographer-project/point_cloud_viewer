.PHONY: all
all:
	git submodule update --init --recursive
	yarn build_all
	cargo build --release --all
