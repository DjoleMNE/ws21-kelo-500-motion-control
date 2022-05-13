setup_ubuntu:
	gcc -I"../../SOEM/" -I"../../SOEM/osal/" -I"../../SOEM/osal/linux/" -I"../../SOEM/oshw/linux/" -I"../../kelo_tulip/include/" -L"../../SOEM/build/" EthercatCommunication.c -lsoem -lpthread -lgsl -ldl -lgslcblas -lm -o setup_output
	sudo setcap cap_net_raw+ep setup_output

run_setup:
	./setup_output no_debug

run_debug_setup:
	./setup_output debug
	
clean:
	rm -f setup_output
