
#include "dl_driver.h"


int sockfd_tcp, sockfd_udp;


struct frame_header_t
{
	uint16_t data_number;
	uint32_t total_size;
	uint16_t payload_size;
    uint32_t num_packet;
    uint32_t packet_number;
    uint32_t offset;
};

DL::distance_amplitude_img_t distance_amplitude_img_out;

DL::payload_header_t payload_header_;
std::vector<uint8_t> user_data_;
std::vector<uint8_t> payload_;

uint16_t last_data_number = -1;
size_t packet_num = 0;


//////////////////////////////////////////////////////////////
// Constructor and Deconstructor for DL Class
//////////////////////////////////////////////////////////////

DL::DL(std::string& dl_ip, int dl_tcp_port, int pc_port)
{
    // tcp setup
    if ( (sockfd_tcp=socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1 ) 
    { 
		perror("[ERROR] Socket creation failed"); 
		exit(EXIT_FAILURE); 
	} 

    struct sockaddr_in sockaddr_dl_tcp;
	sockaddr_dl_tcp.sin_family = AF_INET; 
	sockaddr_dl_tcp.sin_port = htons(dl_tcp_port); 
    sockaddr_dl_tcp.sin_addr.s_addr = inet_addr(dl_ip.c_str());

    connect(sockfd_tcp, (struct sockaddr *)&sockaddr_dl_tcp, sizeof(sockaddr_dl_tcp));

    
    // udp setup
    if ( (sockfd_udp=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1 ) 
    { 
		perror("[ERROR] Socket creation failed"); 
		exit(EXIT_FAILURE); 
	} 
    
    struct sockaddr_in sockaddr_pc;
    sockaddr_pc.sin_family = AF_INET; 
	sockaddr_pc.sin_port = htons(pc_port); 
    sockaddr_pc.sin_addr.s_addr = htonl( INADDR_ANY );

    if ( bind(sockfd_udp,(struct sockaddr *)&sockaddr_pc,sizeof(sockaddr_pc)) == -1 ) 
    { 
		perror("[ERROR] Bind failed"); 
		exit(EXIT_FAILURE); 
	} 

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "Socket START [ TCP : " << sockfd_tcp << " UDP : " << sockfd_udp << " ]" << std::endl;

    th = std::thread(&DL::ThreadCallBack,this);
}


DL::~DL()
{
    close(sockfd_tcp); 
    close(sockfd_udp); 

    thread_running = false;
    th.detach();

    std::cout << "Socket END" << std::endl;
}


//////////////////////////////////////////////////////////////
// Functions for Comm
//////////////////////////////////////////////////////////////

void write_packet(uint16_t cmd_id, const std::vector<uint8_t>& data)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::vector<uint8_t> send_packet;

    // start_marker
    std::vector<uint8_t> start_marker = {0xFF, 0xFF, 0xAA, 0x55};
    for(auto& i: start_marker) send_packet.push_back(i);
    
    // payload_length
    uint32_t payload_length = data.size() + 2;
    send_packet.push_back((payload_length>>24)&0xff);
    send_packet.push_back((payload_length>>16)&0xff);
    send_packet.push_back((payload_length>>8)&0xff);
    send_packet.push_back(payload_length&0xff);

    // payload (cmd_id)
    send_packet.push_back((cmd_id>>8)&0xff);
    send_packet.push_back(cmd_id&0xff);

    // payload (data)
    for(auto& i: data) send_packet.push_back(i);

    // end_marker
    std::vector<uint8_t> end_marker = {0xFF, 0xFF, 0x55, 0xAA};
    for(auto& i: end_marker) send_packet.push_back(i);

    send(sockfd_tcp, &send_packet[0], send_packet.size(), MSG_CONFIRM);
}

void GetFrameHeader(frame_header_t& frame_header, const std::vector<uint8_t>& recv_packet)
{
    frame_header.data_number = ( (uint16_t)(recv_packet[0]&0xff) ) << 8;
    frame_header.data_number |= recv_packet[1]&0xff;
    // std::cout << "data_number : " << frame_header.data_number << std::endl;      // 0~

    frame_header.total_size = ( (uint32_t)(recv_packet[2]&0xff) ) << 24;
    frame_header.total_size |= ( (uint32_t)(recv_packet[3]&0xff) ) << 16;
    frame_header.total_size |= ( (uint32_t)(recv_packet[4]&0xff) ) << 8;
    frame_header.total_size |= recv_packet[5]&0xff;
    // std::cout << "total_size : " << frame_header.total_size << std::endl;        // 307225

    frame_header.payload_size = ( (uint16_t)(recv_packet[6]&0xff) ) << 8;
    frame_header.payload_size |= recv_packet[7]&0xff;
    // std::cout << "payload_size : " << frame_header.payload_size << std::endl;    // 1400 or 625(last)

    frame_header.num_packet = ( (uint32_t)(recv_packet[8]&0xff) ) << 24;
    frame_header.num_packet |= ( (uint32_t)(recv_packet[9]&0xff) ) << 16;
    frame_header.num_packet |= ( (uint32_t)(recv_packet[10]&0xff) ) << 8;
    frame_header.num_packet |= recv_packet[11]&0xff;
    // std::cout << "num_packet : " << frame_header.num_packet << std::endl;        // num_packet = num_packet + 1400

    frame_header.packet_number = ( (uint32_t)(recv_packet[12]&0xff) ) << 24;
    frame_header.packet_number |= ( (uint32_t)(recv_packet[13]&0xff) ) << 16;
    frame_header.packet_number |= ( (uint32_t)(recv_packet[14]&0xff) ) << 8;
    frame_header.packet_number |= recv_packet[15]&0xff;
    // std::cout << "packet_number : " << frame_header.packet_number << std::endl;  // 220

    frame_header.offset = ( (uint32_t)(recv_packet[16]&0xff) ) << 24;
    frame_header.offset |= ( (uint32_t)(recv_packet[17]&0xff) ) << 16;
    frame_header.offset |= ( (uint32_t)(recv_packet[18]&0xff) ) << 8;
    frame_header.offset |= recv_packet[19]&0xff;
    // std::cout << "offset : " << frame_header.offset << std::endl;                // 0~219
}

void GetPayloadHeader(const std::vector<uint8_t>& recv_packet)
{
    payload_header_.version = recv_packet[20]&0xff;
    // std::cout << "version : " << std::hex << (int)payload_header_.version << std::dec << std::endl;

    payload_header_.data_type = ( (uint16_t)(recv_packet[21]&0xff) ) << 8;
    payload_header_.data_type |= recv_packet[22]&0xff;
    // std::cout << "data_type : " << payload_header_.data_type << std::endl;

    payload_header_.width = ( (uint16_t)(recv_packet[23]&0xff) ) << 8;
    payload_header_.width |= recv_packet[24]&0xff;
    // std::cout << "width : " << payload_header_.width << std::endl;

    payload_header_.height = ( (uint16_t)(recv_packet[25]&0xff) ) << 8;
    payload_header_.height |= recv_packet[26]&0xff;
    // std::cout << "height : " << payload_header_.height << std::endl;

    payload_header_.roi_x0 = ( (uint16_t)(recv_packet[27]&0xff) ) << 8;
    payload_header_.roi_x0 |= recv_packet[28]&0xff;
    // std::cout << "roi_x0 : " << payload_header_.roi_x0 << std::endl;

    payload_header_.roi_y0 = ( (uint16_t)(recv_packet[29]&0xff) ) << 8;
    payload_header_.roi_y0 |= recv_packet[30]&0xff;
    // std::cout << "roi_y0 : " << payload_header_.roi_y0 << std::endl;

    payload_header_.roi_x1 = ( (uint16_t)(recv_packet[31]&0xff) ) << 8;
    payload_header_.roi_x1 |= recv_packet[32]&0xff;
    // std::cout << "roi_x1 : " << payload_header_.roi_x1 << std::endl;

    payload_header_.roi_y1 = ( (uint16_t)(recv_packet[33]&0xff) ) << 8;
    payload_header_.roi_y1 |= recv_packet[34]&0xff;
    // std::cout << "roi_y1 : " << payload_header_.roi_y1 << std::endl;

    payload_header_.integration_time_low = ( (uint16_t)(recv_packet[35]&0xff) ) << 8;
    payload_header_.integration_time_low |= recv_packet[36]&0xff;
    // std::cout << "integration_time_low : " << payload_header_.integration_time_low << std::endl;

    payload_header_.integration_time_mid = ( (uint16_t)(recv_packet[37]&0xff) ) << 8;
    payload_header_.integration_time_mid |= recv_packet[38]&0xff;
    // std::cout << "integration_time_mid : " << payload_header_.integration_time_mid << std::endl;

    payload_header_.integration_time_high = ( (uint16_t)(recv_packet[39]&0xff) ) << 8;
    payload_header_.integration_time_high |= recv_packet[40]&0xff;
    // std::cout << "integration_time_high : " << payload_header_.integration_time_high << std::endl;

    payload_header_.mgx = ( (uint16_t)(recv_packet[41]&0xff) ) << 8;
    payload_header_.mgx |= recv_packet[42]&0xff;
    // std::cout << "mgx : " << payload_header_.mgx << std::endl;

    payload_header_.offset_payload = ( (uint16_t)(recv_packet[43]&0xff) ) << 8;
    payload_header_.offset_payload |= recv_packet[44]&0xff;
    // std::cout << "offset_payload : " << payload_header_.offset_payload << std::endl;
}

void GetDistanceAmplitudeImg(DL::distance_amplitude_img_t distance_amplitude_img)
{
    int image_size = distance_amplitude_img_out.payload_header.width*distance_amplitude_img_out.payload_header.height;

    if(payload_.size()!=image_size*4) return;

    distance_amplitude_img_out.distance.resize(image_size);
    distance_amplitude_img_out.amplitude.resize(image_size);

    for(int i=0; i<payload_.size(); i+=4)
    {
        uint16_t buff;

        buff = payload_[i]&0xff;
        buff |= (uint16_t)(payload_[i+1]&0xff) << 8;
        distance_amplitude_img_out.distance[(int)(i/4)] = buff;

        buff = payload_[i+2]&0xff;
        buff |= (uint16_t)(payload_[i+3]&0xff) << 8;
        distance_amplitude_img_out.amplitude[(int)(i/4)] = buff;
    }
}

void GetUserData(const std::vector<uint8_t>& recv_packet)
{
    for(int i=45; i<(20+payload_header_.offset_payload); i++)
    {
        user_data_.push_back(recv_packet[i]);
    }
}

void GetPayload(int start_idx, const std::vector<uint8_t>& recv_packet)
{
    for(int i=start_idx; i<recv_packet.size(); i++)
    {
        payload_.push_back(recv_packet[i]);
    }
}

void ProcImageData(const frame_header_t& frame_header)
{
    if(packet_num==frame_header.packet_number)
    {
        distance_amplitude_img_out.payload_header = payload_header_;
        distance_amplitude_img_out.user_data = user_data_;
        if(distance_amplitude_img_out.payload_header.data_type==0) GetDistanceAmplitudeImg(distance_amplitude_img_out);
    }

    payload_header_ = DL::payload_header_t();
    user_data_.clear();
    payload_.clear();

    last_data_number = frame_header.data_number;
    packet_num = 0;
}

void PacketParsing(const std::vector<uint8_t>& recv_packet)
{
    frame_header_t frame_header;
    GetFrameHeader(frame_header, recv_packet);

    if(last_data_number!=frame_header.data_number)
    {
        ProcImageData(frame_header);
    }

    if(frame_header.offset==0)
    {
        GetPayloadHeader(recv_packet);
        payload_header_.img_num = frame_header.data_number;
        GetUserData(recv_packet);
        GetPayload(20+payload_header_.offset_payload, recv_packet);

        payload_.reserve(payload_header_.width*payload_header_.height*4);
    }
    else
    {
        GetPayload(20, recv_packet);
    }

    packet_num++;
}

void DL::ThreadCallBack(void) 
{
    while(thread_running==true)
    {
        std::vector<uint8_t> recv_packet(2000);
        int recv_len = recv(sockfd_udp, &recv_packet[0], recv_packet.size(), MSG_WAITFORONE);
        recv_packet.resize(recv_len);
        if(recv_len>20) PacketParsing(recv_packet);
    }
}


//////////////////////////////////////////////////////////////
// Set DL 
//////////////////////////////////////////////////////////////

void DL::SetIntegrationTimes(int integration_time_low, int integration_time_mid, int integration_time_high, int integration_time_grayscale)
{
    uint16_t cmd_id = 1;
    std::vector<uint8_t> data(8);
    
    data[0] = (integration_time_low>>8)&0xff;
    data[1] = (integration_time_low&0xff);

    data[2] = (integration_time_mid>>8)&0xff;
    data[3] = (integration_time_mid&0xff);

    data[4] = (integration_time_high>>8)&0xff;
    data[5] = (integration_time_high&0xff);

    data[6] = (integration_time_grayscale>>8)&0xff;
    data[7] = (integration_time_grayscale&0xff);

    write_packet(cmd_id, data);
}

void DL::SetHDR(uint8_t hdr)
{
    uint16_t cmd_id = 25;
    std::vector<uint8_t> data = {hdr};

    write_packet(cmd_id, data);
}

void DL::SetModulation(uint8_t frequency, uint8_t channel, uint8_t auto_channel)
{
    uint16_t cmd_id = 23;
    std::vector<uint8_t> data = {frequency, channel, auto_channel};

    write_packet(cmd_id, data);
}

void DL::SetMinAmplitude(uint16_t min_amplitude)
{
    uint16_t cmd_id = 21;
    std::vector<uint8_t> data(2);

    data[0] = (min_amplitude>>8)&0xff;
    data[1] = (min_amplitude&0xff);

    write_packet(cmd_id, data);
}

void DL::SetOffset(int16_t offset)
{
    uint16_t cmd_id = 20;
    std::vector<uint8_t> data = {(uint8_t)offset};

    write_packet(cmd_id, data);
}

void DL::SetStreamDistanceAmplitude(void)
{
    uint16_t cmd_id = 2;
    std::vector<uint8_t> data = {0x01};

    write_packet(cmd_id, data);
}

void DL::StopStream(void)
{
    uint16_t cmd_id = 6;
    std::vector<uint8_t> data = {};

    write_packet(cmd_id, data);
}

void DL::GetDistanceAmplitudeImage(distance_amplitude_img_t& distance_amplitude_img)
{
    distance_amplitude_img = distance_amplitude_img_out;
}