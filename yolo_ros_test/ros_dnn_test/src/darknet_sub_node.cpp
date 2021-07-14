#include <ros/ros.h>

// darknet_ros_msgs
#include "ros_dnn_test/MsgState.h" 

#include "ros_dnn_test/BoundingBoxes.h"
#include "ros_dnn_test/BoundingBox.h"

#include <pthread.h>

#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std;

string target = "person";
bool detect_target = false;
int state_num = 0; 
ros_dnn_test::MsgState msg;
ros::Publisher state_pub;

void msgCallback(const ros_dnn_test::BoundingBoxes::ConstPtr& msg)
{
    cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[0].Class <<endl;
    if(msg->bounding_boxes[0].Class.compare(target) == 0) {
	cout<<">>>>TARGET (Class):" << msg->bounding_boxes[0].Class <<endl;
        detect_target = true;
    } else {
        detect_target = false;
    }
}

#if 1
void *test(void *data)
{

}
#endif

int main(int argc, char **argv)
{
    cout<<"darknet_sub_node" <<endl;
#if 1

    pthread_t thread_t;
    int status;

    ros::init(argc, argv, "darknet_sub_node");
    ros::NodeHandle nh;
    ros::Subscriber obj_sub = nh.subscribe("/ros_dnn_test/bounding_boxes",100,msgCallback);
 
    string MsgTopicName = string("/ros_dnn_test/percep_info");
	
    state_pub = nh.advertise<ros_dnn_test::MsgState>(MsgTopicName, 100, false);

	
    if (pthread_create(&thread_t, NULL, test, 0) < 0)
    {
        printf("thread create error:");
        exit(0);
    }


    ros::spin();

    pthread_join(thread_t, (void **)&status);
    printf("Thread End %d\n", status);
#endif

	return 0;
}

