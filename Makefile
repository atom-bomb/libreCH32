
ifeq ($(VERBOSE),true)
  V=
else
  V=@
endif

ARCH?=$(shell uname -m)
MAKEDEPEND?=$(CC) -M -MT$(OBJDIR)/$*.o $(CFLAGS) $(INCS) -o $(DEPDIR)/$*.d $<

CFLAGS=-DENABLE_DEBUG_PRINTS

VPATH=\
  src

INCLUDE_PATHS=\
  inc

SOURCES=\
  libreCH32.c\
  libreCH32_main.c\
  debug_print.c

LIBRARIES=

C_OBJECTS=$(addprefix $(OBJDIR)/,$(SOURCES:.c=.o))
OBJECTS=$(C_OBJECTS)

DEPS=$(addprefix $(DEPDIR)/, $(SOURCES:.c=.d))

INCS=$(addprefix -I, $(INCLUDE_PATHS))

LIBS=$(addprefix -L, $(LIBRARY_PATHS))
LIBS+=$(addprefix -l, $(LIBRARIES))

OBJDIR:=$(ARCH)/objs
DEPDIR:=$(ARCH)/deps
BINDIR:=$(ARCH)/bin
LIBDIR:=$(ARCH)/lib

APP=libreCH32

all $(APP): $(BINDIR)/$(APP)

clean:
	@echo Cleaning
	$(V)rm -rf $(OBJDIR) $(BINDIR) $(DEPDIR) $(LIBDIR)

clean-clear: clean
	@echo Cleaning Clear
	$(V)rm -rf $(TURDFILES)

distclean: clean-clear
	@echo Dist-cleaning
	$(V)rm -rf $(ARCH)

$(OBJDIR):
	$(V)mkdir -p $(OBJDIR)

$(BINDIR):
	$(V)mkdir -p $(BINDIR)

$(DEPDIR):
	$(V)mkdir -p $(DEPDIR)

$(LIBDIR):
	$(V)mkdir -p $(LIBDIR)

$(OBJDIR)/%.o: %.c
	@echo Compiling $(notdir $<)
	$(V)$(MAKEDEPEND)
	$(V)$(CC) $(CFLAGS) $(INCS) -c $< -o $@

$(BINDIR)/$(APP): $(BINDIR) $(OBJDIR) $(DEPDIR) $(OBJECTS)
	@echo Linking $(notdir $@)
	$(V)$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJECTS) $(LIBS)

.PHONY: $(APP)
.PHONY: all clean clean-clear distclean

-include $(DEPS)
