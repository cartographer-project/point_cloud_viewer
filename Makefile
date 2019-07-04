.PHONY: all
all:
	yarn build_all
	cargo build --release --all
