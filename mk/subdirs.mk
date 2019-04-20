.PHONY: all clean
.PHONY: $(SUBDIRS)
.PHONY: $(SUBDIRS:=_clean)

all: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@

clean: $(SUBDIRS:=_clean)
$(SUBDIRS:=_clean):
	$(MAKE) -C $(@:_clean=) clean
