#include "yhs_can_control/yhs_can_control_node.hpp"

namespace yhs
{

	CanControl::CanControl(rclcpp::Node::SharedPtr node)
			: node_(node), if_name_("can0"), can_socket_(-1)
	{

		READ_PARAM(std::string, "can_name", (if_name_), "can0");

		node_->declare_parameter<std::vector<int64_t>>("ultrasonic_number", std::vector<int64_t>{});
		node_->declare_parameter<bool>("io_cmd.enable", true);
		node_->declare_parameter<bool>("io_cmd.lower_beam", false);
		node_->declare_parameter<bool>("io_cmd.upper_beam", false);
		node_->declare_parameter<int>("io_cmd.turn_lamp", 0);
		node_->declare_parameter<bool>("io_cmd.braking_lamp", false);
		node_->declare_parameter<bool>("io_cmd.clearance_lamp", false);
		node_->declare_parameter<bool>("io_cmd.fog_lamp", false);
		node_->declare_parameter<bool>("io_cmd.speaker", false);
		node_->declare_parameter<bool>("io_cmd.discharge", false);
		
		node_->get_parameter("ultrasonic_number", ultrasonic_number_);
		node_->get_parameter("io_cmd.enable", current_io_cmd_.io_cmd_enable);
		node_->get_parameter("io_cmd.lower_beam", current_io_cmd_.io_cmd_lower_beam_headlamp);
		node_->get_parameter("io_cmd.upper_beam", current_io_cmd_.io_cmd_upper_beam_headlamp);
		node_->get_parameter("io_cmd.braking_lamp", current_io_cmd_.io_cmd_braking_lamp);
		node_->get_parameter("io_cmd.clearance_lamp", current_io_cmd_.io_cmd_clearance_lamp);
		node_->get_parameter("io_cmd.fog_lamp", current_io_cmd_.io_cmd_fog_lamp);
		node_->get_parameter("io_cmd.speaker", current_io_cmd_.io_cmd_speaker);

		int turn_lamp_temp;
		node_->get_parameter("io_cmd.turn_lamp", turn_lamp_temp);
		current_io_cmd_.io_cmd_turn_lamp = turn_lamp_temp;
		bool discharge_temp;
		node_->get_parameter("io_cmd.discharge", discharge_temp);
		current_io_cmd_.io_cmd_dis_charge = discharge_temp;

		io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::IoCmd>(
				"io_cmd",
				1,
				std::bind(&CanControl::io_cmd_callback, this, std::placeholders::_1));

		ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::CtrlCmd>(
				"ctrl_cmd",
				1,
				std::bind(&CanControl::ctrl_cmd_callback, this, std::placeholders::_1));

		front_free_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::FrontFreeCtrlCmd>(
				"front_free_ctrl_cmd",
				1,
				std::bind(&CanControl::front_free_ctrl_cmd_callback, this, std::placeholders::_1));

		rear_free_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::RearFreeCtrlCmd>(
				"rear_free_ctrl_cmd",
				1,
				std::bind(&CanControl::rear_free_ctrl_cmd_callback, this, std::placeholders::_1));	

		chassis_info_fb_publisher_ = node_->create_publisher<yhs_can_interfaces::msg::ChassisInfoFb>("chassis_info_fb", 1);

		odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

		odo_fb_pub_ = node_->create_publisher<yhs_can_interfaces::msg::OdoFb>("odo_fb", 1);

		timer_ = node_->create_wall_timer(
			std::chrono::milliseconds(10), 
			std::bind(&CanControl::timer_callback, this));
	}

	void CanControl::timer_callback()
	{
		static int loop_count = 0;
		std::lock_guard<std::mutex> lock(cmd_mutex_);

		send_ctrl_cmd();

		if (loop_count % 2 == 0) {
			send_io_cmd(); // 50Hz 发送 IO
		}
		loop_count++;
	}

	void CanControl::io_cmd_callback(const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg)
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		current_io_cmd_ = *io_cmd_msg;
	}

	void CanControl::send_io_cmd()
	{
		static unsigned char count = 0;
		unsigned char sendData_u_io_[8] = {0};

		sendData_u_io_[0] = current_io_cmd_.io_cmd_enable ? 0x01 : 0x00;
		if (current_io_cmd_.io_cmd_lower_beam_headlamp) 
			sendData_u_io_[1] |= 0x01;
		if (current_io_cmd_.io_cmd_upper_beam_headlamp) 
			sendData_u_io_[1] |= 0x02;
		sendData_u_io_[1] |= current_io_cmd_.io_cmd_turn_lamp << 2;
		if (current_io_cmd_.io_cmd_braking_lamp)   
			sendData_u_io_[1] |= 0x10;
		if (current_io_cmd_.io_cmd_clearance_lamp) 
			sendData_u_io_[1] |= 0x20;
		if (current_io_cmd_.io_cmd_fog_lamp)       
			sendData_u_io_[1] |= 0x40;
		
		sendData_u_io_[2] = current_io_cmd_.io_cmd_speaker ? 0x01 : 0x00;
		sendData_u_io_[5] = current_io_cmd_.io_cmd_dis_charge ? 0x01 : 0x00;

		count++;
		if (count == 16) count = 0;
		sendData_u_io_[6] = count << 4;

		sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

		can_frame send_frame;
		send_frame.can_id = 0x18C4D7D0 | CAN_EFF_FLAG;
		send_frame.can_dlc = 8;
		memcpy(send_frame.data, sendData_u_io_, 8);
		write(can_socket_, &send_frame, sizeof(send_frame));
	}

	void CanControl::ctrl_cmd_callback(const yhs_can_interfaces::msg::CtrlCmd::SharedPtr ctrl_cmd_msg)
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		current_ctrl_cmd_ = *ctrl_cmd_msg;
	}

	void CanControl::send_ctrl_cmd()
	{
		static unsigned char count = 0;
		unsigned char sendData_u_ctrl_[8] = {0};

		short linear_s = static_cast<short>(current_ctrl_cmd_.ctrl_cmd_linear * 1000.0f);
		unsigned short linear = static_cast<unsigned short>(linear_s);
		
		short angular_s = static_cast<short>(current_ctrl_cmd_.ctrl_cmd_angular * 100.0f);
		unsigned short angular = static_cast<unsigned short>(angular_s);

		sendData_u_ctrl_[0] = (0x0f & current_ctrl_cmd_.ctrl_cmd_gear) | ((linear & 0x0f) << 4);
		sendData_u_ctrl_[1] = (linear >> 4) & 0xff;
		sendData_u_ctrl_[2] = ((linear >> 12) & 0x0f) | ((angular & 0x0f) << 4);
		sendData_u_ctrl_[3] = (angular >> 4) & 0xff;
		sendData_u_ctrl_[4] = ((angular >> 12) & 0x0f);

		count = (count + 1) % 16;
		sendData_u_ctrl_[6] = count << 4;

		sendData_u_ctrl_[7] = sendData_u_ctrl_[0] ^ sendData_u_ctrl_[1] ^ sendData_u_ctrl_[2] ^ sendData_u_ctrl_[3] ^ sendData_u_ctrl_[4] ^ sendData_u_ctrl_[5] ^ sendData_u_ctrl_[6];

		can_frame send_frame;
		send_frame.can_id = 0x18C4D1D0 | CAN_EFF_FLAG;
		send_frame.can_dlc = 8;
		memcpy(send_frame.data, sendData_u_ctrl_, 8);
		write(can_socket_, &send_frame, sizeof(send_frame));
	}

	void CanControl::front_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::FrontFreeCtrlCmd::SharedPtr front_free_cmd_msg)
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		unsigned short vel_l = static_cast<unsigned short>(static_cast<short>(front_free_cmd_msg->front_free_ctrl_cmd_velocity_l * 1000.0f));
		unsigned short vel_r = static_cast<unsigned short>(static_cast<short>(front_free_cmd_msg->front_free_ctrl_cmd_velocity_r * 1000.0f));

		static unsigned char count = 0;
		unsigned char sendData_u_front_free_[8] = {0};

		sendData_u_front_free_[0] = (0x0f & front_free_cmd_msg->front_free_ctrl_cmd_gear) | ((vel_l & 0x0f) << 4);
		sendData_u_front_free_[1] = (vel_l >> 4) & 0xff;
		sendData_u_front_free_[2] = ((vel_l >> 12) & 0x0f) | ((vel_r & 0x0f) << 4);
		sendData_u_front_free_[3] = (vel_r >> 4) & 0xff;
		sendData_u_front_free_[4] = ((vel_r >> 12) & 0x0f);

		count = (count + 1) % 16;
		sendData_u_front_free_[6] = count << 4;
		sendData_u_front_free_[7] = sendData_u_front_free_[0]^sendData_u_front_free_[1]^sendData_u_front_free_[2]^sendData_u_front_free_[3]^sendData_u_front_free_[4]^sendData_u_front_free_[5]^sendData_u_front_free_[6];

		can_frame send_frame;
		send_frame.can_id = 0x18C4D2D0 | CAN_EFF_FLAG;
		send_frame.can_dlc = 8;
		memcpy(send_frame.data, sendData_u_front_free_, 8);
		write(can_socket_, &send_frame, sizeof(send_frame));
	}

	void CanControl::rear_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::RearFreeCtrlCmd::SharedPtr rear_free_cmd_msg)
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);
		unsigned short vel_l = static_cast<unsigned short>(static_cast<short>(rear_free_cmd_msg->rear_free_ctrl_cmd_velocity_l * 1000.0f));
		unsigned short vel_r = static_cast<unsigned short>(static_cast<short>(rear_free_cmd_msg->rear_free_ctrl_cmd_velocity_r * 1000.0f));

		static unsigned char count = 0;
		unsigned char sendData_u_rear_free_[8] = {0};

		sendData_u_rear_free_[0] = (0x0f & rear_free_cmd_msg->rear_ctrl_cmd_gear) | ((vel_l & 0x0f) << 4);
		sendData_u_rear_free_[1] = (vel_l >> 4) & 0xff;
		sendData_u_rear_free_[2] = ((vel_l >> 12) & 0x0f) | ((vel_r & 0x0f) << 4);
		sendData_u_rear_free_[3] = (vel_r >> 4) & 0xff;
		sendData_u_rear_free_[4] = ((vel_r >> 12) & 0x0f);

		count = (count + 1) % 16;
		sendData_u_rear_free_[6] = count << 4;
		sendData_u_rear_free_[7] = sendData_u_rear_free_[0]^sendData_u_rear_free_[1]^sendData_u_rear_free_[2]^sendData_u_rear_free_[3]^sendData_u_rear_free_[4]^sendData_u_rear_free_[5]^sendData_u_rear_free_[6];

		can_frame send_frame;
		send_frame.can_id = 0x18C4D3D0 | CAN_EFF_FLAG;
		send_frame.can_dlc = 8;
		memcpy(send_frame.data, sendData_u_rear_free_, 8);
		write(can_socket_, &send_frame, sizeof(send_frame));
	}

	bool CanControl::wait_for_can_frame()
	{
		struct timeval tv;
		fd_set rdfs;
		FD_ZERO(&rdfs);
		FD_SET(can_socket_, &rdfs);
		tv.tv_sec = 0;
		tv.tv_usec = 30000; // 15ms

		int ret = select(can_socket_ + 1, &rdfs, NULL, NULL, &tv);
		if (ret == -1)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Error waiting for CAN frame: " << std::strerror(errno));
			return false;
		}
		else if (ret == 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Timeout waiting for CAN frame! Please check whether the can0 setting is correct,\
whether the can line is connected correctly, and whether the chassis is powered on.");
			return false;
		}
		else
		{
			return true;
		}
		return false;
	}

	void CanControl::can_data_recv_callback()
	{

		can_frame recv_frame;
		yhs_can_interfaces::msg::ChassisInfoFb chassis_info_msg;

		while (rclcpp::ok())
		{
			if (!wait_for_can_frame())
				continue;

			if (read(can_socket_, &recv_frame, sizeof(recv_frame)) >= 0)
			{
				switch (recv_frame.can_id & CAN_EFF_MASK)
				{
				//
				case 0x18C4D1EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::CtrlFb msg;
					msg.ctrl_fb_gear = 0x0f & recv_frame.data[0];

					msg.ctrl_fb_linear = static_cast<float>(static_cast<short>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 | (recv_frame.data[0] & 0xf0) >> 4)) / 1000;

					msg.ctrl_fb_angular = static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 | (recv_frame.data[2] & 0xf0) >> 4)) / 100;

					msg.ctrl_fb_mode = 0x30 & recv_frame.data[0] >> 4;

					msg.ctrl_fb_remote_st = 0x80 & recv_frame.data[0] >> 7;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.header.stamp = node_->get_clock()->now();
						chassis_info_msg.ctrl_fb = msg;
						chassis_info_fb_publisher_->publish(chassis_info_msg);
						publish_odom(msg.ctrl_fb_linear, msg.ctrl_fb_angular / 180 * 3.14);
					}

					break;
				}

				//
				case 0x18C4D6EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::LfWheelFb msg;

					msg.lf_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

					msg.lf_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.lf_wheel_fb = msg;
					}

					break;
				}

				//
				case 0x18C4D7EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::LrWheelFb msg;

					msg.lr_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

					msg.lr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.lr_wheel_fb = msg;
					}

					break;
				}

				//
				case 0x18C4D8EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::RrWheelFb msg;

					msg.rr_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

					msg.rr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.rr_wheel_fb = msg;
					}

					break;
				}

				//
				case 0x18C4D9EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::RfWheelFb msg;

					msg.rf_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

					msg.rf_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.rf_wheel_fb = msg;
					}

					break;
				}

				//
				case 0x18C4DAEF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::IoFb msg;

					msg.io_fb_turn_lamp = (0x0c & recv_frame.data[1]) >> 2;

					msg.io_fb_enable = (recv_frame.data[0] & 0x01) != 0;
					
					// 近光灯开关状态反馈（预留）
					msg.io_fb_lower_beam_headlamp = (recv_frame.data[1] & 0x01) != 0;
					
					// 远光灯开关状态反馈（预留）
					msg.io_fb_upper_beam_headlamp = (recv_frame.data[1] & 0x02) != 0;
					
					msg.io_fb_braking_lamp = (recv_frame.data[1] & 0x10) != 0;
					
					// 示廓灯开关状态反馈（预留）
					msg.io_fb_clearance_lamp = (recv_frame.data[1] & 0x20) != 0;
					 
					// 雾灯开关状态反馈（预留）
					msg.io_fb_fog_lamp = (recv_frame.data[1] & 0x40) != 0;
					
					// 扬声器开关状态反馈（预留）
					msg.io_fb_speaker = (recv_frame.data[2] & 0x01) != 0;
					
					// 前左防撞条开关状态反馈(预留)
					msg.io_fb_fl_impact_sensor = (recv_frame.data[3] & 0x01) != 0;
					
					msg.io_fb_fm_impact_sensor = (recv_frame.data[3] & 0x02) != 0;
					
					// 前右防撞条开关状态反馈（预留）
					msg.io_fb_fr_impact_sensor = (recv_frame.data[3] & 0x04) != 0;
					
					// 后左防撞条开关状态反馈（预留）
					msg.io_fb_rl_impact_sensor = (recv_frame.data[3] & 0x08) != 0;
					
					msg.io_fb_rm_impact_sensor = (recv_frame.data[3] & 0x10) != 0;
					
					// 后右防撞条开关状态反馈（预留）
					msg.io_fb_rr_impact_sensor = (recv_frame.data[3] & 0x20) != 0;
					// 前左跌落传感器状态反馈（预留）
					msg.io_fb_fl_drop_sensor = (recv_frame.data[4] & 0x01) != 0;
					// 前中跌落传感器状态反馈（预留）
					msg.io_fb_fm_drop_sensor = (recv_frame.data[4] & 0x02) != 0;
					// 前右跌落传感器状态反馈（预留）
					msg.io_fb_fr_drop_sensor = (recv_frame.data[4] & 0x04) != 0;
					// 后左跌落传感器状态反馈（预留）
					msg.io_fb_rl_drop_sensor = (recv_frame.data[4] & 0x08) != 0;
					// 后中跌落传感器状态反馈（预留）
					msg.io_fb_rm_drop_sensor = (recv_frame.data[4] & 0x10) != 0;
					// 后右跌落传感器状态反馈（预留）
					msg.io_fb_rr_drop_sensor = (recv_frame.data[4] & 0x20) != 0;
					
					msg.io_fb_dis_charge_flg = (recv_frame.data[5] & 0x01) != 0;					
					msg.io_fb_charge_en = (recv_frame.data[5] & 0x02) != 0;
					msg.io_fb_scram_st = (recv_frame.data[5] & 0x10) != 0;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.io_fb = msg;
					}

					break;
				}

				//
				case 0x18C4E1EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::BmsInfoFb msg;

					msg.bms_info_voltage = static_cast<float>(static_cast<unsigned short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

					msg.bms_info_current = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

					msg.bms_info_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frame.data[5] << 8 | recv_frame.data[4])) / 100;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.bms_info_fb = msg;
					}

					break;
				}

          		case 0x18C4DEEF | CAN_EFF_FLAG:
          		{
            		yhs_can_interfaces::msg::OdoFb msg;
            		msg.odo_fb_accumulative_mileage = static_cast<float>(static_cast<int>(recv_frame.data[3] << 24 | recv_frame.data[2] << 16 | recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

            		// 累计角度（预留）
            		msg.odo_fb_accumulative_angular = static_cast<float>(static_cast<int>(recv_frame.data[7] << 24 | recv_frame.data[6] << 16 | recv_frame.data[5] << 8 | recv_frame.data[4])) / 1000;

            		odo_fb_pub_->publish(msg);

            		break;
          		}
				
				//
				case 0x18C4E2EF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::BmsFlagInfoFb msg;

					msg.bms_flag_info_soc = recv_frame.data[0];

					msg.bms_flag_info_single_ov = (recv_frame.data[1] & 0x01) != 0;
					msg.bms_flag_info_single_uv = (recv_frame.data[1] & 0x02) != 0;
					msg.bms_flag_info_ov = (recv_frame.data[1] & 0x04) != 0;
					msg.bms_flag_info_uv = (recv_frame.data[1] & 0x08) != 0;
					msg.bms_flag_info_charge_ot = (recv_frame.data[1] & 0x10) != 0;
					msg.bms_flag_info_charge_ut = (recv_frame.data[1] & 0x20) != 0;
					msg.bms_flag_info_discharge_ot = (recv_frame.data[1] & 0x40) != 0;
					msg.bms_flag_info_discharge_ut = (recv_frame.data[1] & 0x80) != 0;

					msg.bms_flag_info_charge_oc = (recv_frame.data[2] & 0x01) != 0;
					msg.bms_flag_info_discharge_oc = (recv_frame.data[2] & 0x02) != 0;
					msg.bms_flag_info_short = (recv_frame.data[2] & 0x04) != 0;
					msg.bms_flag_info_ic_error = (recv_frame.data[2] & 0x08) != 0;
					msg.bms_flag_info_lock_mos = (recv_frame.data[2] & 0x10) != 0;

					msg.bms_flag_info_charge_st = (recv_frame.data[2] & 0x60) >> 5;

					msg.bms_flag_info_soc_warning = (recv_frame.data[2] & 0x80) != 0; 
					msg.bms_flag_info_soc_low_protection = (recv_frame.data[3] & 0x01) != 0;;

					msg.bms_flag_info_hight_temperature = static_cast<float>(static_cast<short>(recv_frame.data[4] << 4 | recv_frame.data[3] >> 4)) / 10;

					msg.bms_flag_info_low_temperature = static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 8 | recv_frame.data[5])) / 10;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.bms_flag_info_fb = msg;
					}

					break;
				}

				//
				case 0x18C4EAEF | CAN_EFF_FLAG:
				{
					yhs_can_interfaces::msg::VehDiagFb msg;
					msg.veh_fb_fault_level = 0x0f & recv_frame.data[0];

					msg.veh_fb_auto_can_ctrl_cmd = (0x10 & recv_frame.data[0]) != 0;
					msg.veh_fb_auto_can_io_cmd = (0x20 & recv_frame.data[0]) != 0;
					msg.veh_fb_eps_dis_online = (0x01 & recv_frame.data[1]) != 0;
					msg.veh_fb_eps_fault = (0x02 & recv_frame.data[1]) != 0;
					msg.veh_fb_eps_mosfet_ot = (0x04 & recv_frame.data[1]) != 0;
					msg.veh_fb_eps_warning = (0x08 & recv_frame.data[1]) != 0;
					msg.veh_fb_eps_dis_work = (0x10 & recv_frame.data[1]) != 0;
					msg.veh_fb_eps_over_current = (0x20 & recv_frame.data[1]) != 0;
					
					// 转向系统故障预留
					msg.veh_fb_st_reserve = (recv_frame.data[1] >> 6) | ((recv_frame.data[2] & 0x0f) << 2);
					
					msg.veh_fb_aux_bms_dis_online = (0x10 & recv_frame.data[5]) != 0;
					msg.veh_fb_aux_remote_dis_online = (0x80 & recv_frame.data[5]) != 0;

					// 辅件故障预留
					msg.veh_fb_aux_reserve = recv_frame.data[6] & 0x0f;

					msg.veh_fb_lf_drv_mcu_fault = (recv_frame.data[2] >> 4) | ((recv_frame.data[3] & 0x03) << 4);
					msg.veh_fb_rf_drv_mcu_fault = recv_frame.data[3] >> 2;
					msg.veh_fb_lr_drv_mcu_fault = recv_frame.data[4] & 0x3f;
					msg.veh_fb_rr_drv_mcu_fault = (recv_frame.data[4] >> 6) | ((recv_frame.data[5] & 0x0f) << 2);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.veh_diag_fb = msg;
					}

					break;
				}

					// ultrasonic预留
					static unsigned short ultra_data[8] = {0};
				case 0x18C4E8EF | CAN_EFF_FLAG:
				{
					ultra_data[0] = (unsigned short)((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]);
					ultra_data[1] = (unsigned short)(recv_frame.data[2] << 4 | ((recv_frame.data[1] & 0xf0) >> 4));

					ultra_data[2] = (unsigned short)((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]);
					ultra_data[3] = (unsigned short)(recv_frame.data[5] << 4 | ((recv_frame.data[4] & 0xf0) >> 4));
					break;
				}

				case 0x18C4E9EF | CAN_EFF_FLAG:
				{
					ultra_data[4] = (unsigned short)((recv_frame.data[1] & 0x0f) << 8 | recv_frame.data[0]);
					ultra_data[5] = (unsigned short)(recv_frame.data[2] << 4 | ((recv_frame.data[1] & 0xf0) >> 4));

					ultra_data[6] = (unsigned short)((recv_frame.data[4] & 0x0f) << 8 | recv_frame.data[3]);
					ultra_data[7] = (unsigned short)(recv_frame.data[5] << 4 | ((recv_frame.data[4] & 0xf0) >> 4));

					yhs_can_interfaces::msg::Ultrasonic ultra_msg;

          			ultra_msg.ultrasonic_fb_01 = ultra_data[ultrasonic_number_[0]];
          			ultra_msg.ultrasonic_fb_02 = ultra_data[ultrasonic_number_[1]];
          			ultra_msg.ultrasonic_fb_03 = ultra_data[ultrasonic_number_[2]];
          			ultra_msg.ultrasonic_fb_04 = ultra_data[ultrasonic_number_[3]];

          			ultra_msg.ultrasonic_fb_05 = ultra_data[ultrasonic_number_[4]];
          			ultra_msg.ultrasonic_fb_06 = ultra_data[ultrasonic_number_[5]];
          			ultra_msg.ultrasonic_fb_07 = ultra_data[ultrasonic_number_[6]];
          			ultra_msg.ultrasonic_fb_08 = ultra_data[ultrasonic_number_[7]];

					chassis_info_msg.ultrasonic = ultra_msg;
				}

				default:
					break;
				}
			}
		}
	}

	void CanControl::publish_odom(double linear_vel, double angular_vel)
	{

		static double x_ = 0.0;
		static double y_ = 0.0;
		static double theta_ = 0.0;
		static rclcpp::Time last_time_ = node_->now();

		rclcpp::Time current_time = node_->now();

		double dt = (current_time - last_time_).seconds();

		x_ += linear_vel * cos(theta_) * dt;
		y_ += linear_vel * sin(theta_) * dt;
		theta_ += angular_vel * dt;

		nav_msgs::msg::Odometry odom_msg;
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";

		geometry_msgs::msg::PoseWithCovariance pose_cov;
		pose_cov.pose.position.x = x_;
		pose_cov.pose.position.y = y_;
		pose_cov.pose.position.z = 0.0;
		tf2::Quaternion quat;
		quat.setRPY(0.0, 0.0, theta_);
		pose_cov.pose.orientation.x = quat.x();
		pose_cov.pose.orientation.y = quat.y();
		pose_cov.pose.orientation.z = quat.z();
		pose_cov.pose.orientation.w = quat.w();
		odom_msg.pose = pose_cov;

		geometry_msgs::msg::TwistWithCovariance twist_cov;
		twist_cov.twist.linear.x = linear_vel;
		twist_cov.twist.linear.y = 0.0;
		twist_cov.twist.linear.z = 0.0;
		twist_cov.twist.angular.x = 0.0;
		twist_cov.twist.angular.y = 0.0;
		twist_cov.twist.angular.z = angular_vel;
		odom_msg.twist = twist_cov;

		odom_pub_->publish(odom_msg);

		last_time_ = current_time;
	}

	CanControl::~CanControl()
	{
	}

	bool CanControl::run()
	{
		can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (can_socket_ < 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to open socket: " << strerror(errno));
			return false;
		}

		struct ifreq ifr;
		strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
		ifr.ifr_name[IFNAMSIZ - 1] = '\0';
		if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to get interface index: " << strerror(errno) << " ==> " << if_name_.c_str());
			return false;
		}

		struct sockaddr_can addr;
		memset(&addr, 0, sizeof(addr));
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to bind socket: " << strerror(errno));
			return false;
		}

		thread_ = std::thread(&CanControl::can_data_recv_callback, this);

		return true;
	}

	void CanControl::stop()
	{
		if (can_socket_ >= 0)
		{
			close(can_socket_);
			can_socket_ = -1;
		}

		if (thread_.joinable())
		{
			thread_.join();
		}
	}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("yhs_can_control_node");

	yhs::CanControl cancontrol(node);
	if (!cancontrol.run())
	{
		RCLCPP_ERROR(node->get_logger(), "Failed to initialize yhs_can_control_node");
		return 0;
	}

	RCLCPP_INFO(node->get_logger(), "yhs_can_control_node initialized successfully");

	rclcpp::spin(node);

	cancontrol.stop();
	RCLCPP_INFO(node->get_logger(), "yhs_can_control_node stopped");

	rclcpp::shutdown();

	return 0;
}
