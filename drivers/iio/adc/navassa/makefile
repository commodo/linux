# Makefile template for shared library
TEST_DIR = .
C_API_DIR = ../c_src

CC = gcc # C compiler
LD = gcc # C linker

CFLAGS = -g -ggdb -fPIC -Wall -O0 -D_ADI_SV_PLATFORM -DDPI_COMPATIBILITY_VERSION_1800v2005 -I$(TEST_DIR)

#Recursive Wildcard
rwildcard = $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))

#Do not compile or include these files and directories
IGNORE_SRC = $(C_API_DIR)/third_party/fru_tools/operating-system.c \
	$(wildcard $(C_API_DIR)/third_party/jsmn/example/*.c) \
	$(wildcard $(C_API_DIR)/third_party/jsmn/test/*.c)

IGNORE_INCL = $(wildcard $(C_API_DIR)/third_party/jsmn/example/*.h) \
	$(wildcard $(C_API_DIR)/third_party/jsmn/test/*.h)

#Finds all header files and added the directories to CFLAGS with -I option, ignoring specified h files
DEPS = $(filter-out $(IGNORE_INCL), $(call rwildcard, $(C_API_DIR)/, *.h))
INCLUDE_DIRS = $(sort $(dir $(DEPS)))
override CFLAGS += $(patsubst %,-I%,$(subst :, ,$(INCLUDE_DIRS)))

LDFLAGS = -g # linking flags
RM = rm -f  # rm command
TARGET = $(TEST_DIR)/example # target lib

#List of all c files in C_SRC and generated c file, ignoring specified c files
SRCS = $(filter-out $(IGNORE_SRC), $(call rwildcard, $(C_API_DIR)/, *.c)) $(wildcard $(TEST_DIR)/*.c)

#Create object files of all c files
OBJS = $(SRCS:.c=.o)

#Recipes
all: ${TARGET}

$(TARGET): $(OBJS)
	$(LD) ${LDFLAGS} -o $@ $^ -lm

.$(EXT).o:
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY : clean 		#prevents make from getting confused by an actual file called clean

clean:
	rm -rf ${TARGET} ${OBJS} $(TEST_DIR)/example

