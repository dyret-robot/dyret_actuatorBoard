#include <ros.h>
#include <dyret_common/Configuration.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

dyret_common::Configuration stateMessage;
ros::Publisher statePub("actuatorStates", &stateMessage);

dyret_common::Configuration debugMessage;
ros::Publisher debugPub("dyret_actuator/actuatorDebug", &stateMessage);

void messageCb(const dyret_common::Configuration& msg){
/*  debugMessage.id = msg.id;
  debugMessage.id_length = msg.id_length;
  debugMessage.distance = msg.distance;
  debugMessage.distance_length = msg.distance_length;
*/
  debugMessage = msg;

  debugPub.publish(&debugMessage);
}

ros::Subscriber<dyret_common::Configuration> s("actuatorCommands",messageCb);

void setup() {
  nh.initNode(); 
  
  nh.subscribe(s);
  nh.advertise(statePub);
  nh.advertise(debugPub);

}

void loop() {
  
  nh.spinOnce();

}
