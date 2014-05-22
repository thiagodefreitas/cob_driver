#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "cob_srvs/SetString.h"

#include <libphidgets/phidget21.h>
#include <sstream>



class Button
{
	ros::NodeHandle n_;
	ros::Publisher pub_button_;
	int id_, filter_size_;
    bool val_;
	std::string frame_id_;
public:

    Button(const std::string &fr_id, const int id) :
			id_(id), frame_id_(fr_id)
	{
		char buffer[256];
		sprintf(buffer, "button_%d", id);
        pub_button_ = n_.advertise<std_msgs::Bool>(buffer, 0);
	}

	int getId() const
	{
		return id_;
	}

    void update(const bool v)
	{
        val_ = v;
	}

    bool getVal()
    {
        return val_;
    }
};

void display_generic_properties(CPhidgetHandle phid)
{
    int sernum, version;
    const char *deviceptr, *label;
    CPhidget_getDeviceType(phid, &deviceptr);
    CPhidget_getSerialNumber(phid, &sernum);
    CPhidget_getDeviceVersion(phid, &version);
    CPhidget_getDeviceLabel(phid, &label);

    ROS_INFO("%s", deviceptr);
    ROS_INFO("Version: %8d SerialNumber: %10d", version, sernum);
    ROS_INFO("Label: %s", label);
    return;
}

int IFK_AttachHandler(CPhidgetHandle IFK, void *userptr)
{
    //CPhidgetInterfaceKit_setSensorChangeTrigger((CPhidgetInterfaceKitHandle)IFK, 0, 0);
    //printf("Attach handler ran!\n");
    return 0;
}

std::vector<Button>* g_buttons;
std::map<std::string, int> buttons_map;

int CCONV InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *userptr,
        int Index, int State)
{

    g_buttons = (std::vector<Button>*) userptr;
    for (size_t i = 0; i < g_buttons->size(); i++)
        if ((*g_buttons)[i].getId() == Index)
        {
            (*g_buttons)[i].update(State);
            std::cout << State;
        }
    return 0;
}



bool update_button_state(cob_srvs::SetString::Request  &req,
         cob_srvs::SetString::Response &res)
{

  if(buttons_map.find( req.data ) != buttons_map.end())
    ROS_INFO("Setting button state to: %d", (*g_buttons)[buttons_map[req.data]].getVal());

  res.success = true;

  if((*g_buttons)[buttons_map[req.data]].getVal())
    res.errorMessage.data = ("Button ON");

  else
    res.errorMessage.data = "Button OFF";

  return true;
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_phidgets");


	ros::NodeHandle nh_("~");
    std::vector<Button> g_buttons;

    if (nh_.hasParam("buttons"))
	{
		XmlRpc::XmlRpcValue v;
        nh_.param("buttons", v, v);
		for (int i = 0; i < v.size(); i++)
		{
			ROS_ASSERT(v[i].size()>=2);

			int id = v[i][0];
			std::string fr_id = v[i][1];


            g_buttons.push_back(Button(fr_id, id));
            buttons_map[fr_id] = i;
		}
	}
	else
	{
        ROS_ERROR("Parameter buttons not set, shutting down node...");
		nh_.shutdown();
		return false;
	}

	ros::Rate loop_rate(10);

	//init and open phidget
    int numInputs, numOutputs, numbuttons;
	int err;

    ros::ServiceServer service = nh_.advertiseService("button_state",update_button_state);
    ros::ServiceClient client = nh_.serviceClient<cob_srvs::SetString>("button_state");

	CPhidgetInterfaceKitHandle IFK = 0;
	CPhidget_enableLogging(PHIDGET_LOG_VERBOSE, NULL);
	CPhidgetInterfaceKit_create(&IFK);
    CPhidgetInterfaceKit_set_OnInputChange_Handler(IFK,
            InputChangeHandler, (void*) &g_buttons);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle) IFK, IFK_AttachHandler,
			NULL);
	//opening phidget
	CPhidget_open((CPhidgetHandle) IFK, -1);

	//wait 5 seconds for attachment
	ROS_INFO("waiting for phidgets attachement...");
	if ((err = CPhidget_waitForAttachment((CPhidgetHandle) IFK, 10000))
			!= EPHIDGET_OK)
	{
		const char *errStr;
		CPhidget_getErrorDescription(err, &errStr);
		ROS_ERROR("Error waiting for attachment: (%d): %s", err, errStr);
		goto exit;
	}
	ROS_INFO("... attached");

	display_generic_properties((CPhidgetHandle) IFK);
	CPhidgetInterfaceKit_getOutputCount((CPhidgetInterfaceKitHandle) IFK,
			&numOutputs);
	CPhidgetInterfaceKit_getInputCount((CPhidgetInterfaceKitHandle) IFK,
			&numInputs);
	CPhidgetInterfaceKit_getSensorCount((CPhidgetInterfaceKitHandle) IFK,
            &numbuttons);
	//CPhidgetInterfaceKit_setOutputState((CPhidgetInterfaceKitHandle)IFK, 0, 1);
	ROS_INFO(
            "buttons:%d Inputs:%d Outputs:%d", numbuttons, numInputs, numOutputs);

	while (ros::ok())
    {
        //for (size_t i = 0; i < g_buttons.size(); i++)
        //     g_buttons[i].publish();

		ros::spinOnce();
		loop_rate.sleep();
	}

	exit: CPhidget_close((CPhidgetHandle) IFK);
	CPhidget_delete((CPhidgetHandle) IFK);

	return 0;
}
