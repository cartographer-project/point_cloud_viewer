DEFAULT_GOAL := all

.PHONY: all
all:
	yarn build_all
	cargo build --release --all
