all: rover_controller.cpp
	g++ $^ -lrt -lpthread -lrf24 -lserial -lssd1306 -o rover_controller

clean:
	rm rover_controller

install:
	# try to stop the runnning service (note the minus at the beginning of the line)
	-sudo systemctl stop rover_controller.service
	sudo cp rover_controller /usr/bin
	sudo cp rover_controller.service /etc/systemd/system
	sudo systemctl enable rover_controller.service

remove:
	# try to stop the runnning service (note the minus at the beginning of the line)
	-sudo systemctl disable rover_controller.service
	sudo rm /etc/systemd/system/rover_controller.service
	sudo rm /usr/bin/rover_controller

