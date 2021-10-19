all: base_controller.cpp
	g++ $^ -lpthread -lrf24 -lserial -lssd1306 -o base_controller

clean:
	rm base_controller

install:
	# try to stop the runnning service (note the minus at the beginning of the line)
	-sudo systemctl stop base_controller.service
	sudo cp base_controller /usr/bin
	sudo cp base_controller.service /etc/systemd/system
	sudo systemctl enable base_controller.service

remove:
	# try to stop the runnning service (note the minus at the beginning of the line)
	-sudo systemctl disable base_controller.service
	sudo rm /etc/systemd/system/base_controller.service
	sudo rm /usr/bin/base_controller

