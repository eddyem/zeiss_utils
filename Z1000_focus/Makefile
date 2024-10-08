# run `make DEF="-D... -D..."` to add extra defines
PROGRAM := can_focus
LDFLAGS := -fdata-sections -ffunction-sections -Wl,--gc-sections -Wl,--discard-all -pthread
SRCS := $(wildcard *.c)
DEFINES := $(DEF) -D_GNU_SOURCE -D_XOPEN_SOURCE=1111
OBJDIR := mk
CFLAGS += -O2 -std=gnu99 -Wall -Wextra
#ifneq (,$(findstring DEBUG, $(DEF)))
#  CFLAGS += -Werror
#  $(info Warnings considered as errors)
#endif
OBJS := $(addprefix $(OBJDIR)/, $(SRCS:%.c=%.o))
DEPS := $(OBJS:.o=.d)
CC ?= gcc
#CXX = g++

all : $(OBJDIR) $(PROGRAM)

debug: CFLAGS += -DEBUG -Werror
debug: all

$(PROGRAM) : $(OBJS)
	@echo -e "\t\tLD $(PROGRAM)"
	$(CC) $(LDFLAGS) $(OBJS) -o $(PROGRAM)

$(OBJDIR):
	mkdir $(OBJDIR)

ifneq ($(MAKECMDGOALS),clean)
-include $(DEPS)
endif

$(OBJDIR)/%.o: %.c
	@echo -e "\t\tCC $<"
	$(CC) -MD -c $(LDFLAGS) $(CFLAGS) $(DEFINES) -o $@ $<

clean:
	@echo -e "\t\tCLEAN"
	@rm -f $(OBJS) $(DEPS)
	@rmdir $(OBJDIR) 2>/dev/null || true

xclean: clean
	@rm -f $(PROGRAM)

gentags:
	CFLAGS="$(CFLAGS) $(DEFINES)" geany -g $(PROGRAM).c.tags *[hc] 2>/dev/null

.PHONY: gentags clean xclean
