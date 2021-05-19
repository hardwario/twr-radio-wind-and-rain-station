SDK_DIR ?= sdk
VERSION ?= vdev

CFLAGS += -D'VERSION="${VERSION}"'
CFLAGS += -D'TWR_SCHEDULER_INTERVAL_MS=50'

-include sdk/Makefile.mk

.PHONY: all
all: debug

.PHONY: sdk
sdk: sdk/Makefile.mk

.PHONY: update
update:
	@git submodule update --remote --merge sdk
	@git submodule update --remote --merge .vscode

sdk/Makefile.mk:
	@git submodule update --init sdk
	@git submodule update --init .vscode
