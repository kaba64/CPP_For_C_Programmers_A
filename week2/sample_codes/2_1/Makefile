CC := g++ -std=c++20 -O2 -Wall -fmax-errors=3
TARGET_EXEC := out
SRCS := main.cpp
OBJS=$(SRCS:.cpp=.o)


$(TARGET_EXEC): $(OBJS)
	$(CC) $^ -o $@
$(OBJS): $(SRCS)
	$(CC) -c $<
clean:
	rm $(OBJS) $(TARGET_EXEC) *~
run:
	@./$(TARGET_EXEC)
