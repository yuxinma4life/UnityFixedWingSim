/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>

serial::Serial ser;

void write_callback(const sensor_msgs::Joy::ConstPtr& msg){
    //ROS_INFO_STREAM("Writing to serial port" << msg->header);
    std_msgs::String rc_out;
    //rc_out.data.push_back('$');
    uint8_t checksum = 0;
    
    for(int i =0; i<5; i++){
    	
    	checksum = 0;
    	uint8_t rc_out_h = (int)((msg->axes[i]+1)*1000/2)/255;
	    uint8_t rc_out_l = (int)((msg->axes[i]+1)*1000/2)%255;
    	int rc_out_a = rc_out_h*255 + rc_out_l+1000;
    	checksum += rc_out_h + rc_out_l + (uint8_t)i;
    	//ROS_INFO("%d", rc_out_a);
    	
    	rc_out.data.clear();
    	rc_out.data.push_back('$');
    	rc_out.data.push_back((uint8_t)i);
    	rc_out.data.push_back(rc_out_h);
    	rc_out.data.push_back(rc_out_l);
    	rc_out.data.push_back(checksum);
    	ser.write(rc_out.data);
    	//ROS_INFO("%d    %d", (uint8_t)i, rc_out_a);
    }
    
    

    
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("joy", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<sensor_msgs::Joy>("joy", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(100);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            //read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

