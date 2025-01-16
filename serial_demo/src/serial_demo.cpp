#include <thread>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>

//demo 程序
//功能： 1) 发送状态控制指令到Mini控制启停算法等
//       2)解码Mini发送的姿态数据,打印出来或者打包成ROS话题发送出去

#define LREV_LEN    61
#define LSEND_LEN   34
#define LS_CHECK    LSEND_LEN - 3   //发送校验位序列
#define LR_CHECK    LREV_LEN - 3    //接收校验位置

#define RHEAD_1     0X66    //接收帧头
#define RHEAD_2     0X27
#define RTILE_1     0X08
#define RTILE_2     0X0a   

struct pose_t{
    float px,py,pz,qx,qy,qz,qw;
};

struct speed_t{
    float lx,ly,lz,ax,ay,az;
};

struct odom_t{
    pose_t pose;
    speed_t speed;
};

ros::Publisher pub_odom;
serial::Serial serial_port;
uint8_t send_buffer[LSEND_LEN];
uint8_t receive_buffer[LREV_LEN];

enum algo_state{
    algo_enable = 0x01,     //启动
    algo_disable = 0x02,    //停止
    algo_restart = 0x03,    //重置
    algo_none   = 0x00,
};

//发布里程计话题
void publish_odom(const odom_t& odom){
    ros::Time stamp = ros::Time::now(); 
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = odom.pose.px;
    odom_msg.pose.pose.position.y = odom.pose.py;
    odom_msg.pose.pose.position.z = odom.pose.pz;
    odom_msg.pose.pose.orientation.x = odom.pose.qx;
    odom_msg.pose.pose.orientation.y = odom.pose.qy;
    odom_msg.pose.pose.orientation.z = odom.pose.qz;
    odom_msg.pose.pose.orientation.w = odom.pose.qw;
    odom_msg.twist.twist.linear.x = odom.speed.lx;
    odom_msg.twist.twist.linear.y = odom.speed.ly;
    odom_msg.twist.twist.linear.z = odom.speed.lz;
    odom_msg.twist.twist.angular.x = odom.speed.ax;
    odom_msg.twist.twist.angular.y = odom.speed.ay;
    odom_msg.twist.twist.angular.z = odom.speed.az;
    pub_odom.publish(odom_msg);
}

//解析数据包
void unpack_uart(odom_t& odom,int& frame_id){
    // std::cout << "receive:" << std::endl;
    // for(int i = 0;i < LREV_LEN;i++)  {
    //     printf("0x%02x ",receive_buffer[i]);
    // }
    // std::cout << std::endl;

    if(receive_buffer[0] == RHEAD_1 && receive_buffer[1] == RHEAD_2 && receive_buffer[LREV_LEN - 2] == RTILE_1 && receive_buffer[LREV_LEN - 1] == RTILE_2 ){
        unsigned char check=0;
        //61-3
        for(int i = 2;i < LR_CHECK;i++){
            check += receive_buffer[i];
        }
        if(check == receive_buffer[LR_CHECK]){
            // std::cout<<"======unpack ok======"<<std::endl;
            memcpy(&frame_id,receive_buffer + 2,sizeof(frame_id));
            memcpy(&odom.pose,receive_buffer + 6,sizeof(odom.pose));
            memcpy(&odom.speed,receive_buffer + 6 + sizeof(odom.pose),sizeof(odom.speed));
            // printf("frame_id = %d\n",frame_id);
            // //px,py,pz,qx,qy,qz,qw;
            // printf("pose [%f,%f,%f,%f,%f,%f,%f]\n",odom.pose.px,odom.pose.py,odom.pose.pz,odom.pose.qx,odom.pose.qy,odom.pose.qz,odom.pose.qw);
            // //lx,ly,lz,ax,ay,az;
            // printf("speed [%f,%f,%f,%f,%f,%f]\n",odom.speed.lx,odom.speed.ly,odom.speed.lz,odom.speed.ax,odom.speed.ay,odom.speed.az);
        }
        else{
            std::cout<<"check err"<<std::endl;
            printf("want is %x,but receive:%x\n",check,receive_buffer[LR_CHECK]);
        }
    }
    else{
        std::cout<<"pack format err"<<std::endl;
    }
}

/// @brief 发送控制指令
/// @param t algo_state 控制指令
void send_command(const algo_state t){   
    memset(send_buffer,0,LSEND_LEN);
    send_buffer[0] = 0x67;
    send_buffer[1] = 0x28;
    send_buffer[2] = t;
    char check_sum = 0;
    for(int i = 2;i < LS_CHECK;i++){
        check_sum += send_buffer[i];
    }
    send_buffer[LS_CHECK] = check_sum;
    send_buffer[LSEND_LEN - 2] = 0x09;
    send_buffer[LSEND_LEN - 1] = 0x0d;
    serial_port.write(send_buffer,LSEND_LEN);    //已发送
    // std::cout << "send is" << std::endl;
    // for(int i = 0;i < LSEND_LEN;i++) {
    //     printf("0x%02x ",send_buffer[i]);
    // }
}

//串口初始化
void serial_init(){
	try{
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);
        serial_port.open();

    }catch(serial::IOException& e){
        std::cerr<<"can't open serial /dev/ttyUSB0. " <<std::endl;
        exit(0);
    }
	if(!serial_port.isOpen()){
        std::cerr<<"serial is note opend"<<std::endl;
		exit(0);
    }
}

//串口接收线程
void serial_receive_thread(){
    odom_t mini_odom;
    int frame_id = 0,last_frame_id = 0;
    while(1){
        serial_port.read(&receive_buffer[0],1);
        if(receive_buffer[0] == RHEAD_1){
            serial_port.read(receive_buffer + 1,LREV_LEN - 1);
            unpack_uart(mini_odom,frame_id);
            //拿到了里程计后的操作
            if(frame_id > last_frame_id){
                publish_odom(mini_odom);
                last_frame_id = frame_id; //用于判断里程计是否持续更新，可不要  
            }
        }
    }
}

int main(int argc,char **argv){
    ros::init(argc,argv,"serial_demo");
    ros::NodeHandle nh;
	
	serial_init();
    pub_odom = nh.advertise<nav_msgs::Odometry>("/baton_mini_odometry", 10);

    std::thread receive_thread{serial_receive_thread};
    int v;
    ros::Rate r(10);
    while(ros::ok()){
        ROS_INFO("please input command:");
        std::cin >> v;
        if(v == 1){
            ROS_INFO("algo_enable");
            send_command(algo_enable);
        }
        else if(v == 2){
            ROS_INFO("algo_disable");
            send_command(algo_disable);
        }
        else if(v == 3){
            ROS_INFO("algo_reset");
            send_command(algo_restart);
        }
        else if(v == 4){
            ROS_INFO("exit");
            break;
        }
        r.sleep();
        ros::spinOnce(); 
    }
    serial_port.close();
    receive_thread.join();
    return 0;
}
