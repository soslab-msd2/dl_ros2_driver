
#ifndef DLROS2DRIVER_H
#define DLROS2DRIVER_H

#include <stdio.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>

#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 


class DL
{
public:
	struct payload_header_t
	{
		size_t img_num;
		uint8_t version;
		uint16_t data_type;
		uint16_t width;
		uint16_t height;
		uint16_t roi_x0;
		uint16_t roi_y0;
		uint16_t roi_x1;
		uint16_t roi_y1;
		uint16_t integration_time_low;
		uint16_t integration_time_mid;
		uint16_t integration_time_high;
		uint16_t mgx;
		uint16_t offset_payload;
	};

	struct distance_amplitude_img_t
	{
		payload_header_t payload_header;
		std::vector<uint8_t> user_data;
		std::vector<uint16_t> distance;
		std::vector<uint16_t> amplitude;
	};


public:
	DL(std::string& dl_ip, int dl_tcp_port, int pc_port);
	~DL();

	void SetIntegrationTimes(int integration_time_low, int integration_time_mid, int integration_time_high, int integration_time_grayscale);
	void SetHDR(uint8_t hdr);
	void SetModulation(uint8_t frequency, uint8_t channel, uint8_t auto_channel);
	void SetMinAmplitude(uint16_t min_amplitude);
	void SetOffset(int16_t offset);
	
	void SetStreamDistanceAmplitude(void);
	void StopStream(void);

	void GetDistanceAmplitudeImage(distance_amplitude_img_t& distance_amplitude_img);
	

private:
	void ThreadCallBack(void);
	bool thread_running = true;
	std::thread th;
};

#endif