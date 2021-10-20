#include <string>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sensor_msgs/NavSatFix.h>
#include <serial/SerialPort.h>

// components
ros::Publisher gnss_pub;
SerialPort gnss_serial;

// shared memory with ROS
#define ROS_POSITION_SHARE_NAME     "R0v3r_Position"
#define ROS_POSITION_SHARE_LENGTH   128
double* latitude = nullptr;
double* longitude = nullptr;
int* posfix = nullptr;

// read number from string
template <class Type>
Type stringToNum(const std::string &str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

// split string to an array
std::vector<std::string> stringSplit(const std::string &s, const char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

// convert NMEA DDmm.mm format to deciamal D.d format
double convertNmeaToDouble(const std::string &val, const std::string &dir) {
    int dot = val.find('.');
    std::string degree = val.substr(0, dot-2);
    std::string minute = val.substr(dot-2);

    // ROS_INFO_STREAM("convertNmeaToDouble: " << degree << " / " << minute);

    double ret = stringToNum<double>(degree) + stringToNum<double>(minute)/60;

    if (dir == "S" || dir == "W") {
        ret = -ret;
    }

    return ret;
}

// get coordinate info from GNSS message
void gnssMessageParser(const std::string &msg)
{
    // $GNGGA,090031.00,2057.59811809,N,10546.17292292,E,1,18,2.2,16.4378,M,-28.2478,M,,*64
    ROS_INFO_STREAM(msg);
    // if (gnss_pub.getNumSubscribers() > 0)
    // {
    std::vector<std::string> message = stringSplit(msg, ',');
    if (message[0] == "$GNGGA" && message[2] != "" && message[4] != "")
    {
        *latitude = convertNmeaToDouble(message[2], message[3]);
        *longitude = convertNmeaToDouble(message[4], message[5]);
        *posfix = stringToNum<int>(message[6]);

        sensor_msgs::NavSatFix navfix_msg;
        navfix_msg.latitude = *latitude;
        navfix_msg.longitude = *longitude;
        navfix_msg.altitude = stringToNum<double>(message[9]);
        gnss_pub.publish(navfix_msg);

        // ROS_INFO_STREAM("gnss_parser: " << *posfix << ", " << std::setprecision(20) << navfix_msg.latitude << ", " << navfix_msg.longitude << ", " << navfix_msg.altitude);
    }
    // }
}

int main(int argc, char **argv)
{
    // setup shared memory
    int shm_fd;
    void* shm_ptr;

    // open shared memory object
    shm_fd = shm_open(ROS_POSITION_SHARE_NAME, O_RDWR | O_CREAT | O_TRUNC,  0777);
    if (shm_fd == -1) {
        ROS_INFO_STREAM("Can NOT open shared memory!!!");
        return 1;
    }

    // allocated file size
    int res = ftruncate(shm_fd, ROS_POSITION_SHARE_LENGTH);
    if (res == -1) {
        ROS_INFO_STREAM("Can NOT allocate shared memory!!!");
        return 1;
    }

    // map the memory
    shm_ptr = mmap(NULL, ROS_POSITION_SHARE_LENGTH, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        ROS_INFO_STREAM("Can NOT map the shared memory!!!");
        return 1;
    }

    ROS_INFO_STREAM("Shared memory allocated at " << shm_ptr);

    latitude = (double*) shm_ptr;
    longitude = (double*) (latitude + sizeof(double));
    posfix = (int*) (longitude + sizeof(double));

    // create an ROS node
    ros::init(argc, argv, "gnss_parser");
    ros::NodeHandle node;
    gnss_pub = node.advertise<sensor_msgs::NavSatFix>("gnss_position", 1);

    // read launch configs
    std::string port;
    int32_t baudrate;

    ros::param::get("~port", port);
    ros::param::get("~baudrate", baudrate);

    ROS_INFO_STREAM("gnss_parser: " << "Opening " << port << " with baudrate " << baudrate);
    char ret = gnss_serial.openDevice(port.c_str(), baudrate);
    if (ret !=1 ) { 
        ROS_ERROR_STREAM("gnss_parser: " << "Can NOT connect to " << port << " !!!");
        return 1;
    }

    ROS_INFO_STREAM("gnss_parser: " << "Successfully connected to serial port.");

    // preapre to read data
    char message[1024] = {0};

    //gnss_serial.flushReceiver();
    while(gnss_serial.available()) {
        gnss_serial.readString(message, '\n', 1024);
    }


    // loop forever
    while (ros::ok())
    {
        ros::spinOnce();

        // if there is data
        if (gnss_serial.available()>20)
        {
            gnss_serial.readString(message, '\n', 1024, 5000); // timeout in 5000 ms
            gnssMessageParser(std::string(message));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
