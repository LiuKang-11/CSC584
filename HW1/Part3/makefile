rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
src := $(call rwildcard,./,*.cpp)

obj = $(patsubst %.cpp,%.o,$(src))

LDFLAGS = -lsfml-graphics -lsfml-window -lsfml-system 

INTELMAC_INCLUDE=-I/usr/local/include							# Intel mac
APPLESILICON_INCLUDE=-I/opt/homebrew/opt/sfml@2/include					# Apple Silicon
UBUNTU_APPLESILICON_INCLUDE=-I/usr/include 						# Apple Silicon Ubuntu VM
UBUNTU_INTEL_INCLUDE=-I/usr/include 							# Intel Ubuntu VM

INTELMAC_LIB=-L/usr/local/lib									# Intel mac
APPLESILICON_LIB=-L/opt/homebrew/opt/sfml@2/lib							# Apple Silicon
UBUNTU_APPLESILICON_LIB=-L/usr/lib/aarch64-linux-gnu			# Apple Silicon Ubuntu VM
UBUNTU_INTEL_LIB=-L/usr/lib/x86_64-linux-gnu					# Intel Ubuntu VM

MACOS_INCLUDE=$(APPLESILICON_INCLUDE)
MACOS_LIB=$(APPLESILICON_LIB)
UBUNTU_INCLUDE=$(UBUNTU_APPLESILICON_INCLUDE)
UBUNTU_LIB=$(UBUNTU_APPLESILICON_LIB)

MACOS_COMPILER=/usr/bin/clang++
UBUNTU_COMPILER=/usr/bin/g++

all: main

uname_s := $(shell uname -s)
main: $(obj)
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(MACOS_LIB)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(UBUNTU_LIB)
endif

uname_s := $(shell uname -s)
%.o: %.cpp
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -c $^ -o $@ $(MACOS_INCLUDE)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -c $^ -o $@ $(UBUNTU_INCLUDE)
endif

.PHONY: clean
clean:
	rm -f $(obj) main
