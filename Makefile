COMPILER = g++
CFLAGS   = -Wextra -MMD -MP -O3
LDFLAGS  = -O3
LIBS     =
INCLUDE  = -I ./
TARGET   = $(shell basename `pwd`)
OBJDIR   = ./obj
ifeq "$(strip $(OBJDIR))" ""
  OBJDIR = .
endif
SOURCES  = $(wildcard *.cpp)
OBJECTS  = $(addprefix $(OBJDIR)/, $(SOURCES:.cpp=.o))
DEPENDS  = $(OBJECTS:.o=.d)
 
$(TARGET): $(OBJECTS) $(LIBS)
	$(COMPILER) -o $@ $^ $(LDFLAGS)
 
$(OBJDIR)/%.o: %.cpp
	@[ -d $(OBJDIR) ] || mkdir -p $(OBJDIR)
	$(COMPILER) $(CFLAGS) $(INCLUDE) -o $@ -c $<
 
all: clean $(TARGET)
 
clean:
	rm -rf $(OBJECTS) $(DEPENDS) $(TARGET)
 
-include $(DEPENDS)
