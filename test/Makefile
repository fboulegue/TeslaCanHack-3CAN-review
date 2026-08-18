CXX ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O0 -g

.PHONY: check clean

check: test_remote_can_protocol test_remote_can_rx_filter
	./test_remote_can_protocol
	./test_remote_can_rx_filter

test_remote_can_protocol: test_remote_can_protocol.cpp ../esp32/.firmware/remote_can_protocol.h
	$(CXX) $(CXXFLAGS) test_remote_can_protocol.cpp -o $@

test_remote_can_rx_filter: test_remote_can_rx_filter.cpp ../esp32/.firmware/remote_can_rx_filter.h
	$(CXX) $(CXXFLAGS) test_remote_can_rx_filter.cpp -o $@

clean:
	rm -rf test_remote_can_protocol test_remote_can_rx_filter *.dSYM
