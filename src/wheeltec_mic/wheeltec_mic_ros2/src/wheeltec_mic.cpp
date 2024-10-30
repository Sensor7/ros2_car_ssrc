/****************************************************************/
/* Copyright (c) 2023 WHEELTEC Technology, Inc   								*/
/* function:Serial port analysis																*/
/* 功能：串口解析																									*/
/****************************************************************/
#include "wheeltec_mic.h"

/**************************************
Function: Get the serial port return field
功能: 获取串口返回字段
***************************************/
bool Wheeltec_Mic::UnPackMsgPacket(const string &content, MsgPacket &data)
{
	if (content.size() < 7 || ((unsigned char)content.at(0) != FRAME_HEADER))
		return false;

	data.uid  = content[1] & 0xff;
	data.type = content[2] & 0xff;
	data.size = (content[3] & 0xff) | ((content[4] << 8 & 0xff00));
	data.sid  = (content[5] & 0xff) | ((content[6] << 8 & 0xff00));

	switch((MsgType)data.type)
	{
		case MsgType::AIUI_MSG:
		{
			string info = content.substr(7,data.size);
			data.bytes  = info;
			return true;
		}
			break;
		case MsgType::CONTROL:
		{
			string info = content.substr(7,data.size);
			data.bytes  = info;
			return true;
		}
			break;
		default:
        	break;
	}
    return false;
}

/**************************************
Function: Verify serial port data and parse information
功能: 校验串口数据并解析信息
***************************************/
int Wheeltec_Mic::process_data(const unsigned char *buf, int len)
{
	if (buf[2] == 0xff)
	{
		return -1;
	}

	//校验码校对
	int sum = std::accumulate(buf, buf + len - 1, 0);
	if (((~sum + 1) & 0xff) != buf[len - 1]) 
    {
      // cout<<"recv data not crc, drop\n"<<endl;
      return -1;
    }

    if (UnPackMsgPacket(string((char *)buf,len),MsgPkg))
    {
    	Json::Reader reader;
    	if ((MsgType)MsgPkg.type == MsgType::AIUI_MSG)
    	{
    		Json::Value Aiui_Msg;
    		Json::Value value_iwv;

    		if (reader.parse(MsgPkg.bytes, Aiui_Msg)){
    			//串口反馈信息显示
				//cout<< "Aiui_Msg: "<<  Aiui_Msg <<endl;
    			if (Aiui_Msg["type"].asString() == "aiui_event")
    			{
	    			std_msgs::msg::Int8 awake_msg;
	    			awake_msg.data = 1;
	    			awake_flag_pub->publish(awake_msg);
		    		Json::Value content = Aiui_Msg["content"];
						if (content["eventType"].asString() == "4")
							{
								std::string iwv_msg = content["info"].asString();
								if (reader.parse(iwv_msg,value_iwv))
				    		{
				    			angle = value_iwv["ivw"]["angle"].asInt();
				    			std_msgs::msg::UInt32 angle_msg;
				    			angle_msg.data = angle;
				    			angle_pub->publish(angle_msg);
				    			cout << ">>>>>唤醒角度为: " << angle << "°"<< endl;

				    			std_msgs::msg::String msg;
									msg.data = "小车唤醒";
									voice_words_pub->publish(msg);
				    		}
				    	else
								cout << "reader json fail!"<< endl;
								}    				
    			}
    			else
    			{
    				device_message = Aiui_Msg["content"].asString();
    				process_result = true;
    			}
    		}
    	return 1;
    	}
    }
    return -1;
}

/**************************************
Function: Receive and filter data
功能: 过滤数据
***************************************/
int Wheeltec_Mic::uart_analyse(unsigned char buffer)
{
	static int count=0, frame_len=0;
	Receive_Data[count] = buffer;
    if(Receive_Data[0] != FRAME_HEADER || (count == 1 && Receive_Data[1] != USER_ID))  
      count = 0,frame_len = 0;
    else 
      count++;
	if (count == 7){  
      frame_len = (Receive_Data[4]<<8 | Receive_Data[3]) + 7 + 1;
	}
	if(count == frame_len){

		process_data(Receive_Data,frame_len);
		count = 0,frame_len = 0;
		memset(Receive_Data, 0, 1024);
		return 1;
	}
	return 0;
}

/**************************************
Function: Receive the information sent by the device
功能: 接收下位机发送的信息
***************************************/
bool Wheeltec_Mic::Get_Serial_Data()
{
		unsigned char buffer[1];
    memset(buffer, 0, 1);

    while(MicArr_Serial.available()>0)
    {
    	if(MicArr_Serial.read(buffer,sizeof(buffer)))
    	{
    		if(uart_analyse(buffer[0]))
    			return true;
   		}
	}
	return false;
}

/**************************************
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void Wheeltec_Mic::run()
{
	while(rclcpp::ok()){
		Get_Serial_Data();

	  rclcpp::spin_some(this->get_node_base_interface());
	}
}

/**************************************
Function: Constructor, executed only once, for initialization
功能: 构造函数, 用于初始化
***************************************/
Wheeltec_Mic::Wheeltec_Mic(const std::string &node_name)
:rclcpp::Node(node_name){
	memset(&Receive_Data, 0, sizeof(Receive_Data));

	setupMicArrayServices();

	this->declare_parameter<string>("usart_port_name","/dev/ttyCH343USB0");
	this->declare_parameter<int>("serial_baud_rate",115200);

	this->get_parameter("usart_port_name",usart_port_name);
	this->get_parameter("serial_baud_rate",serial_baud_rate);

	/***唤醒标志位话题发布者创建***/
	awake_flag_pub = this->create_publisher<std_msgs::msg::Int8>("awake_flag",10);
	/***麦克风设备串口打开标志位话题发布者创建***/
	voice_flag_pub = this->create_publisher<std_msgs::msg::Int8>("voice_flag",10);
	/***唤醒角度话题发布者创建***/
	angle_pub = this->create_publisher<std_msgs::msg::UInt32>("awake_angle",10);
	/***命令词话题发布者创建***/
	voice_words_pub = this->create_publisher<std_msgs::msg::String>("voice_words",10);

	try
	{
		MicArr_Serial.setPort(usart_port_name);
		MicArr_Serial.setBaudrate(serial_baud_rate);
		MicArr_Serial.setFlowcontrol(serial::flowcontrol_none);
		MicArr_Serial.setParity(serial::parity_none);
		MicArr_Serial.setStopbits(serial::stopbits_one);
		MicArr_Serial.setBytesize(serial::eightbits);
		serial::Timeout _time = serial::Timeout::simpleTimeout(1000);
		MicArr_Serial.setTimeout(_time);
		MicArr_Serial.open();
	}
	catch(serial::IOException& e)
	{
		RCLCPP_ERROR(this->get_logger(),"wheeltec_mic can not open serial port,Please check the serial port cable! ");
	}
	if (MicArr_Serial.isOpen())
	{
		sleep(0.1);
		MicArr_Serial.flush();
		RCLCPP_INFO(this->get_logger(),"wheeltec_mic serial port opened");

		sleep(1.0);
		cout << ">>>>>成功打开麦克风设备" << endl;
		cout << ">>>>>以降噪板设置的唤醒词为准[默认:小微小微] " << endl;
		std_msgs::msg::Int8 voice_flag_msg;
		voice_flag_msg.data = 1;
		voice_flag_pub->publish(voice_flag_msg);	
	}
}

Wheeltec_Mic::~Wheeltec_Mic()
{
	RCLCPP_INFO(this->get_logger(),"wheeltec_mic_node over!\n");
	MicArr_Serial.close();
}

int main(int argc,char **argv)
{
	rclcpp::init(argc,argv);
	Wheeltec_Mic mic("wheeltec_mic");
	mic.run();
	return 0;
}