#include "ros/ros.h"

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <unordered_map>
#include "rover_control_msgs/gps.h"

#define IN
#define IN_OUT
#define OUT
#define GET_WORST_OF(X, Y) ((X) < (Y) ? (X) : (Y)) 

#define SIZE_MSG_BUFFER 1000
#define SIZE_NB_ELEMENTS 20
#define SIZE_VALUES 20

enum ErrorCodes
{
    Failure = INT8_MIN,
    NotImplemented,
    InvalidArguments,
    SyntaxError,
    Drop,
    EmptyMessage = 0,
    Success = 1
} ErrorCodes;

enum GPSFixQuality
{
    Invalid = rover_control_msgs::gps::FIX_INVALID,       // Bad
    Standalone = rover_control_msgs::gps::FIX_STANDALONE, // 30m +0m -20m
    DGPS = rover_control_msgs::gps::FIX_DGPS,             // 10m +0m -8m
    RTKFixed = rover_control_msgs::gps::FIX_RTK_FIXED,    // 1.0m +0.0m -0.9m
    RTKFloat = rover_control_msgs::gps::FIX_RTK_FLOAT     // No info
};

enum GGAElements
{
    Latitude = 2,
    LatitudeDir,
    Longitude,
    LongitudeDir,
    GPSFixQuality,
    Satellite,
    Height = 9,
    ReferenceStationID = 14
};

int getInfo(char read_buf[SIZE_MSG_BUFFER]);
int strToInt(IN const char *str, IN uint8_t size);
int findSizesDegreesMinutes(IN const char *str, OUT uint8_t* size_degrees, OUT uint8_t* size_minutes, IN uint8_t max = 100);
float strToFloat(IN const char *str, IN uint8_t skip, IN uint8_t size);
float strToFloat(IN const char *str, IN uint8_t size);
int getLatLong(IN const char elements[SIZE_NB_ELEMENTS][SIZE_VALUES], IN GGAElements data_type,  OUT float* data);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_node");
    ros::NodeHandle nh;

    std::string device_name = nh.param<std::string>(ros::this_node::getName() + "/port", "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0");
    ROS_INFO("%s::main() Selected device: %s", ros::this_node::getName().c_str(), device_name.c_str());

    int serial_port = open(device_name.c_str(), O_RDWR);
    struct termios tty;

    if (tcgetattr(serial_port, &tty) != 0)
    {
        ROS_ERROR("Error on device %s: Failed to get attr: %s\n", device_name.c_str(), strerror(errno));
        return 2;
    }

    // Setting baudrate
    cfsetispeed(&tty, B4800);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    if (serial_port < 0)
    {
        ROS_ERROR("Error on device %s: %s", device_name.c_str(), strerror(errno));
        return 1;
    }

    while (!ros::isShuttingDown())
    {
        char read_buf[SIZE_MSG_BUFFER];
        int n = read(serial_port, &read_buf, sizeof(read_buf));

        if (n == 0)
        {
            ROS_ERROR("Some error occured while reading serial");
        }

        int res = getInfo(read_buf);
        if (res != 1)
        {
            if (res == EmptyMessage)
                ROS_DEBUG("%s:main() Empty message, dropping message", ros::this_node::getName().c_str());
            if (res == SyntaxError)
                ROS_ERROR("%s:main() Error \"%d\" in buffer, dropping message", ros::this_node::getName().c_str(), res);
        }
        else
        {
        }

        for (int i = 0; i < SIZE_MSG_BUFFER; i++)
        {
            read_buf[i] = '\0';
        }
    }

    return 0;
}

int getInfo(char zs_read_buf[SIZE_MSG_BUFFER])
{
    if (zs_read_buf[0] == '\n' || zs_read_buf[0] == '\0')
    {
        return EmptyMessage;
    }

    if (zs_read_buf[0] != '$')
    {
        return SyntaxError;
    }

    char elements[SIZE_NB_ELEMENTS][SIZE_VALUES] = {0};
    uint8_t index_element = 0;
    uint8_t index_character = 0;
    uint16_t cursor = 0;
    for (; zs_read_buf[cursor] != '\n' && cursor < SIZE_MSG_BUFFER && index_element < SIZE_NB_ELEMENTS; cursor++)
    {
        if (zs_read_buf[cursor] == ',')
        {
            elements[index_element][index_character + 1] = '\0';
            index_character = 0;
            index_element++;
        }
        else
        {
            elements[index_element][index_character] = zs_read_buf[cursor];
            index_character++;
        }
    }
    uint8_t size_elements = index_element + 1;

    if (strncmp(elements[0], "$GPGGA", sizeof("$GPGGA") - 1UL) == 0)
    {
        float latitude = 0.0f;
        float longitude = 0.0f;
        int res = Failure;

        res = GET_WORST_OF(res, getLatLong(elements, Latitude, &latitude));
        res = GET_WORST_OF(res, getLatLong(elements, Longitude, &longitude));

        if (res != Success)
        {
            ROS_WARN("%s: Error parsing message - dropping", ros::this_node::getName().c_str());
        }
        else
        {
            ROS_INFO("%s: Latitude is: %f\t Longitude is: %f", ros::this_node::getName().c_str(), latitude, longitude);
        }
    }
    else if (strncmp(elements[0], "$GPRMC", sizeof("$GPRMC") - 1UL) == 0)
    {
        // for (int i = 0; i < size_elements; i++)
        // ROS_INFO("Value #%d: %s", i, elements[i]);
        // ROS_INFO("");
    }
    else if (strncmp(elements[0], "$GPGGA", sizeof("$GPGGA") - 1UL) == 0)
    {
        return NotImplemented;
    }
    else if (strncmp(elements[0], "$GPGGA", sizeof("$GPGGA") - 1UL) == 0)
    {
        return NotImplemented;
    }
    else if (strncmp(elements[0], "$GPGGA", sizeof("$GPGGA") - 1UL) == 0)
    {
        return NotImplemented;
    }
    else if (strncmp(elements[0], "$GPGGA", sizeof("$GPGGA") - 1UL) == 0)
    {
        return NotImplemented;
    }

    return 1;
}

int strToInt(IN const char *str, IN uint8_t size)
{
    char buffer[size + 1] = {'\0'};

    for (uint8_t i = 0; i < size; i++, str++)
    {
        buffer[i] = *str;
    }

    return atoi(buffer);
}

float strToFloat(IN const char *str, IN uint8_t size)
{
    char buffer[size + 1] = {'\0'};

    for (uint8_t i = 0; i < size; i++, str++)
    {
        buffer[i] = *str;
    }

    return atof(buffer);
}

float strToFloat(IN const char *str, IN uint8_t skip, IN uint8_t size)
{
    char buffer[size + 1] = {'\0'};
    str += skip;
    for (uint8_t i = 0; i < size; i++, str++)
    {
        buffer[i] = *str;
    }

    return atof(buffer);
}

int findSizesDegreesMinutes(IN const char *str, OUT uint8_t* size_degrees, OUT uint8_t* size_minutes, IN uint8_t max)
{
    uint8_t i = 0;
    for (; (i < max && *str != '\0'); i++, str++)
    {
        if (*str == '.')
        {
            *size_degrees = i - 2;
            break;
        }
    }

    if (i == max)
    {
        return ErrorCodes::Drop;
    }

    for (; (*str != '\0' && i < max); i++, str++){}

    if (i == max)
    {
        return ErrorCodes::Drop;
    }

    *size_minutes = i - *size_degrees;

    return ErrorCodes::Success;
}

int getLatLong(IN const char elements[SIZE_NB_ELEMENTS][SIZE_VALUES], IN GGAElements data_type,  OUT float* data)
{
    ROS_INFO("%s: Element[data_type]= %s", ros::this_node::getName().c_str(), elements[data_type]);

    if (data == NULL || elements == NULL && (data_type != Longitude && data_type != Latitude))
    {
        return InvalidArguments;
    }

    *data = 0;
    uint8_t size_degrees = 0;
    uint8_t size_minutes = 0;
    if (findSizesDegreesMinutes(elements[data_type], &size_degrees, &size_minutes) != Drop)
    {
        ROS_WARN("size_degrees= %d", size_degrees);
        float deg = static_cast<float>(strToInt(elements[data_type], size_degrees));
        float minutes = strToFloat(elements[data_type], size_degrees, size_minutes);
    
        *data = deg + (minutes/60.0f);
    }
    else
    {
        return Drop;
    }

    if (strncmp(elements[data_type+1], "N", 1) != 0 && strncmp(elements[data_type+1], "E", 1) != 0)
    {
        *data *= -1;
    }

    return Success;
}
