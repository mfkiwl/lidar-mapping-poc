#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <RF24/RF24.h>
#include <serial/SerialPort.h>
#include <SSD1306/ssd1306.h>

using namespace std;

// hardware configs
#define GNSS_DEV 			"/dev/ttyTHS1"	//
#define GNSS_MSG_LEN_MAX	1024            // bytes
#define GNSS_MSG_TIMEOUT	5000            // ms
#define RF24_CE_PIN 		15 				// sys_gpio
#define OLED_DEV			"/dev/i2c-1"	//

// shared memory with ROS
#define ROS_POSITION_SHARE_NAME		"R0v3r_Position"
#define ROS_POSITION_SHARE_LENGTH	128

// system status
uint32_t rxRadio = 0;
uint32_t txGnss = 0;

// send GNNS command
void GnssSendCommand(SerialPort& gnss, const char* command, char* response)
{
	cout << command << endl;
	gnss.writeString(command);
	gnss.writeString("\r\n");
	gnss.readString(response, '\n', GNSS_MSG_LEN_MAX, GNSS_MSG_TIMEOUT);
	cout << response;
}

// read response from GNSS
void GnssReadString(SerialPort& gnss, char* response)
{
	gnss.readString(response, '\n', GNSS_MSG_LEN_MAX, GNSS_MSG_TIMEOUT);
	cout << response;
}

// forward radio rtcm message to gnss
void DataForwardingThread() {
	// GNSS port
	SerialPort gnss;
	bool base_fixed = false;
	char response[GNSS_MSG_LEN_MAX] = {0};

	// try to connect to gnss
	char ret = gnss.openDevice(GNSS_DEV, 115200);
    if (ret !=1 ) {	
		cout << "Can NOT connect to " << GNSS_DEV << " !!!" << endl;
		return;
	}

    cout << "Connected to " << GNSS_DEV << "!" << endl;

	GnssSendCommand(gnss, "unlog", response);

	GnssSendCommand(gnss, "versiona", response);
	GnssReadString(gnss, response);

	// set output
	GnssSendCommand(gnss, "gngga com2 1", response); // Position data
	GnssSendCommand(gnss, "gprmc com3 1", response); // Datetime data

	// RF24 transmitter
	/* CE = use a sys_gpio pin
	 * CSN = 0 means spidev0.0
	 * default speed is 10 Mbps 
	 */
	RF24 radio(RF24_CE_PIN, 0);
	uint8_t payload[32]; // max payload of RF24 is 32 bytes

	// try to connect to radio
    if (!radio.begin()) {
        cout << "Radio hardware is not responding!!!" << endl;
        return;
    }

    cout << "RF24 Radio is ready!" << endl;

    // setup radio
    radio.setChannel(100); // 2400 + 100 = 2500 MHz, out of WiFi band
    radio.setPayloadSize(sizeof(payload));
    radio.setPALevel(RF24_PA_MAX);

    // set the address, defaut length is 5
    uint8_t rx_address[6] = "R0v3r"; // write to
    radio.openReadingPipe(1, rx_address); // always uses pipe 1

    // for debugging info
    radio.printDetails();       // (smaller) function that prints raw register values
    radio.printPrettyDetails(); // (larger) function that prints human readable data

    // set radio in RX mode
    radio.startListening();

    // forward data from radio to gnss
    uint8_t pipe;
    int rxRead = 0;
	bool txSent = false;

    while(true) {
    	if(radio.available(&pipe)) {
    		// read from radio
    		rxRead = radio.getPayloadSize();
    		radio.read(payload, rxRead);
    		rxRadio += rxRead;
    		
    		// send on gnss
    		txSent = gnss.writeBytes(payload, rxRead);
        	if (txSent) {
        		txGnss += rxRead;
        	}
        }
    }

    gnss.closeDevice();
}

// display system status on OLED
void SystemStatusThread() {
	// setup shared memory
	int shm_fd;
	void* shm_ptr;

	// open shared memory object
	shm_fd = shm_open(ROS_POSITION_SHARE_NAME, O_RDWR | O_CREAT | O_TRUNC,  0777);
	if (shm_fd == -1) {
		cout << "Can NOT open shared memory!!!" << endl;
		return;
	}

	// allocated file size
	int res = ftruncate(shm_fd, ROS_POSITION_SHARE_LENGTH);
	if (res == -1) {
		cout << "Can NOT allocate shared memory!!!" << endl;
		return;
	}

	// map the memory
	shm_ptr = mmap(NULL, ROS_POSITION_SHARE_LENGTH, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
	if (shm_ptr == MAP_FAILED) {
		cout << "Can NOT map the shared memory!!!" << endl;
		return;
	}

	cout << "Shared memory allocated at " << shm_ptr << endl;

	double* latitude = (double*) shm_ptr;
	double* longitude = (double*) (latitude + sizeof(double));
	int* posfix = (int*) (longitude + sizeof(double));


	*latitude = 0.0;
	*longitude = 0.0;
	*posfix = 0;

	// setup OLED
	if(!SSD1306_Init(OLED_DEV)) {
		//munmap(shm_ptr, ROS_POSITION_SHARE_LENGTH);
		//shm_unlink(ROS_POSITION_SHARE_NAME);
		return;
	}

	char line_1[32] = {0};
	char line_2[32] = {0}; char line_2b[32] = {0};
	char line_3[32] = {0};

	while(true) {
		SSD1306_Clear();

		// show Base location
		sprintf(line_1, "P:%.4f, %.4f", *latitude, *longitude);
		SSD1306_WriteString(0, 0, line_1, &Font_7x10, SSD1306_WHITE, SSD1306_OVERRIDE);

		sprintf(line_2b, "%d", *posfix);
		SSD1306_WriteString(7*(10+4), 0, line_2b, &Font_7x10, SSD1306_WHITE, SSD1306_OVERRIDE);

		// statistic data
		sprintf(line_2, "R:%d", rxRadio);
		SSD1306_WriteString(0, 10, line_2, &Font_7x10, SSD1306_WHITE, SSD1306_OVERRIDE);

		sprintf(line_3, "T:%d", txGnss);
		SSD1306_WriteString(0, 20, line_3, &Font_7x10, SSD1306_WHITE, SSD1306_OVERRIDE);

		// update every second
		SSD1306_Screen_Update();
		this_thread::sleep_for(chrono::seconds(1));
	}

}

int main() {
	// no buffer in stdout
	setbuf(stdout, NULL);

	// run threads
	thread dataThread(DataForwardingThread);
	thread statusThread(SystemStatusThread);

	dataThread.join();
	statusThread.join();

    return 0 ;
}
