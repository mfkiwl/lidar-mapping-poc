#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <chrono>
#include <RF24/RF24.h>
#include <serial/SerialPort.h>
#include <SSD1306/ssd1306.h>

using namespace std;

// hardware configs
#define GNSS_DEV 		"/dev/ttyS0"	// 
#define RF24_CE_PIN 	15 				// sys_gpio
#define OLED_DEV		"/dev/i2c-0"	//

// system status
uint32_t rxGnss = 0;
uint32_t txRadio = 0;
double latitude = 0.0;
double longitude = 0.0;

// read number from string
template <class Type>
Type stringToNum(const string &str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

// split string to an array
vector<string> stringSplit(const string &s, const char delimiter)
{
    vector<string> tokens;
    string token;
    istringstream tokenStream(s);
    while (getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

// forward gnss rtcm message to radio
void DataForwardingThread() {
	// GNSS port
	SerialPort gnss;

	// try to connect to gnss
	char ret = gnss.openDevice(GNSS_DEV, 115200);
    if (ret !=1 ) {	
		cout << "Can NOT connect to " << GNSS_DEV << " !!!" << endl;
		return;
	}

    cout << "Connected to " << GNSS_DEV << "!" << endl;

    bool base_fixed = false;

    while(!base_fixed) {
	    // set GNSS as a base
	    gnss.writeString("mode base time 60 1.0 2.0\r\n");

	    // wait for base coordinate
	    this_thread::sleep_for(chrono::seconds(60));

	    // query position
	    gnss.flushReceiver();
	    gnss.writeString("gngga\r\n");

	    // read response
	    char response[128] = {0};
	    gnss.readString(response, '\n', 128, 5000); // timeout in 5000 ms

	    // expected
	    // $GNGGA,090031.00,2057.59811809,N,10546.17292292,E,1,18,2.2,16.4378,M,-28.2478,M,,*64
	    if (response[0]) {
		    vector<string> message = stringSplit(response, ',');
		    if (message[0] == "$GNGGA" && message[2] != "" && message[4] != "")
		    {
		    	latitude = stringToNum<double>(message[2]);
		    	longitude = stringToNum<double>(message[4]);

		    	cout << "Base location:" << setprecision(20) << latitude << "," << longitude << endl;

		    	base_fixed = true;
		    } else {
		    	cout << "GNSS does not response!!!" << endl;
		    }
		}
	}

    // query RTCM
    gnss.writeString("rtcm1006 10\r\n"); // Base station antenna
    gnss.writeString("rtcm1033 10\r\n"); // Description of receiver
    gnss.writeString("rtcm1074 1\r\n");  // GPS system correction data
    gnss.writeString("rtcm1124 1\r\n");  // BDS system correction data
    gnss.writeString("rtcm1084 1\r\n");  // Glonass system correction data
    gnss.writeString("rtcm1094 1\r\n");  // Galileo system correction data

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
    uint8_t tx_address[6] = "R0v3r"; // write to
    radio.openWritingPipe(tx_address); // always uses pipe 0

    // for debugging info
    radio.printDetails();       // (smaller) function that prints raw register values
    radio.printPrettyDetails(); // (larger) function that prints human readable data

    // set radio in TX mode
    radio.stopListening();

    // forward data from gnss to radio
    int rxRead = 0;
	bool txSent = false;

    while(true) {
    	// read from gnss
    	rxRead = gnss.readBytes(payload, sizeof(payload));
    	rxGnss += rxRead;

    	// send on radio
        txSent = radio.write(&payload, rxRead);
        if(txSent) {
        	txRadio += rxRead;
        }
    }

    gnss.closeDevice();
}

// display system status on OLED
void SystemStatusThread() {
	// setup OLED
	if(!SSD1306_Init(OLED_DEV)) {
		return;
	}

	char line_1[32] = {0};
	char line_2[32] = {0};
	char line_3[32] = {0};

	while(true) {
		SSD1306_Clear();

		// show Base location
		sprintf(line_1, "%8.4f/%8.4f", latitude, longitude);
		SSD1306_WriteString(0, 10, line_1, &Font_7x10, SSD1306_WHITE, SSD1306_OVERRIDE);

		// statistic data
		sprintf(line_2, "RX:%d", rxGnss);
		SSD1306_WriteString(0, 20, line_2, &Font_7x10, SSD1306_WHITE, SSD1306_OVERRIDE);

		sprintf(line_3, "TX:%d", txRadio);
		SSD1306_WriteString(0, 30, line_3, &Font_7x10, SSD1306_WHITE, SSD1306_OVERRIDE);

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
