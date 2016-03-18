RM := rm -rf

# All Target
all: libpmc464.so

OBJS += \
./pmc464.o 

C_DEPS += \
./pmc464.d

# Rules for building sources
%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	gcc -I"../kernel" -O0 -g3 -Wall -c -fPIC -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

# Tool invocations
libpmc464.so: $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	gcc -shared -o"libibs.so" -fPIC $(OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS) $(C_DEPS) libpmc464.so
	-@echo ' '

.PHONY: all clean
