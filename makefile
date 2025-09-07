# Makefile for manipulating Modelica packages

# Usage examples:
#   make model path=MyLib.Components name=SpringDamper
#   make function path=MyLib.Utils name=interpolate
#   make package path=MyLib name=SubPkg

# Translate dots to slashes for filesystem paths
FS_PATH := $(subst .,/,$(path))

ifndef path
	$(error You must provide 'path', e.g., path=MyLib.Components)
endif

ifndef name
	$(error You must provide 'name', e.g., name=SpringDamper)
endif

# Helper: parent directory of FS_PATH
PARENT := $(dir $(FS_PATH))

# Default rule
all:
	@echo "Use 'make model', 'make function', or 'make package' with path=... name=..."

# Create a Model
model:
	@mkdir -p $(FS_PATH)
	@echo "within $(path);" > $(FS_PATH)/$(name).mo
	@echo "model $(name)" >> $(FS_PATH)/$(name).mo
	@echo "  // TODO: add contents" >> $(FS_PATH)/$(name).mo
	@echo "end $(name);" >> $(FS_PATH)/$(name).mo
	@echo "Created model: $(FS_PATH)/$(name).mo"
	@if [ -f $(FS_PATH)/package.order ]; then \
	  grep -qx '$(name)' $(FS_PATH)/package.order || echo '$(name)' >> $(FS_PATH)/package.order; \
	  echo "Updated package.order in $(FS_PATH)"; \
	fi

# Create a Function
function:
	@mkdir -p $(FS_PATH)
	@echo "within $(path);" > $(FS_PATH)/$(name).mo
	@echo "function $(name)" >> $(FS_PATH)/$(name).mo
	@echo "  // TODO: add inputs/outputs" >> $(FS_PATH)/$(name).mo
	@echo "end $(name);" >> $(FS_PATH)/$(name).mo
	@echo "Created function: $(FS_PATH)/$(name).mo"
	@if [ -f $(FS_PATH)/package.order ]; then \
	  grep -qx '$(name)' $(FS_PATH)/package.order || echo '$(name)' >> $(FS_PATH)/package.order; \
	  echo "Updated package.order in $(FS_PATH)"; \
	fi

# Create a Package
package:
	@mkdir -p $(FS_PATH)/$(name)
	@echo "within $(path);" > $(FS_PATH)/$(name)/package.mo
	@echo "package $(name)" >> $(FS_PATH)/$(name)/package.mo
	@echo "  // TODO: add contents" >> $(FS_PATH)/$(name)/package.mo
	@echo "end $(name);" >> $(FS_PATH)/$(name)/package.mo
	@touch $(FS_PATH)/$(name)/package.order
	@echo "Created package: $(FS_PATH)/$(name)"
	@if [ -f $(FS_PATH)/package.order ]; then \
	  grep -qx '$(name)' $(FS_PATH)/package.order || echo '$(name)' >> $(FS_PATH)/package.order; \
	  echo "Updated package.order in $(FS_PATH)"; \
	fi
