# Collect files recursively so autonomous routines can keep assets in folders.
asset_files_recursive=$(foreach entry,$(wildcard $1*),$(if $(wildcard $(entry)/.),$(call asset_files_recursive,$(entry)/),$(entry)))

# if "template" is in the make command, do not include static.lib files
ifneq (,$(findstring template,$(MAKECMDGOALS)))
ASSET_FILES=$(call asset_files_recursive,static/)
else
ASSET_FILES=$(call asset_files_recursive,static/) $(call asset_files_recursive,static.lib/)
endif

TEMPLATE_FILES+=$(call asset_files_recursive,static/) $(wildcard firmware/hot-cold-asset.mk)

ASSET_OBJ=$(addprefix $(BINDIR)/, $(addsuffix .o, $(ASSET_FILES)) )

GETALLOBJ=$(sort $(call ASMOBJ,$1) $(call COBJ,$1) $(call CXXOBJ,$1)) $(ASSET_OBJ)

.SECONDEXPANSION:
$(ASSET_OBJ): $$(patsubst bin/%,%,$$(basename $$@))
	$(VV)mkdir -p $(dir $@)
	@echo "ASSET $@"
	$(VV)$(OBJCOPY) -I binary -O elf32-littlearm -B arm $^ $@
