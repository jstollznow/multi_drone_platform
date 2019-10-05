#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>

#include "geometry_msgs/PoseStamped.h"

#define CF_SIM 5
#define QUEUE 10
#define OPTI_FREQ 120


struct matrix{
	std::vector< std::vector<double> > rows;
};


matrix readCSV(std::string filename ){
	
	matrix M;

	std::ifstream in(filename);
	std::string line;
	int i = 0;
	std::getline(in, line);
	std::cout<<"Data Format: \n"<<line<<std::endl;
	while (std::getline(in, line)){
		std::stringstream ss(line);
		std::vector<double> row;
		std::string data;
		while (std::getline(ss, data, ',' ) ){
			if(data.length() !=0){
				row.push_back(std::stod(data));
			}
		}
		std::cout<<line<<std::endl;
		if ( row.size() > 0 ) M.rows.push_back(row); 
		i++;
	}
   return M;
}
void write(const matrix &M)
{
   const int w = 12;
   for ( auto row : M.rows )
   {
      for ( auto e : row ) std::cout << std::setw( w ) << e << ' ';
      std::cout << '\n';
   }
}
std::vector<geometry_msgs::PoseStamped> set(const matrix &M, int seq)
{
	std::vector<geometry_msgs::PoseStamped> messages;
	std::vector<double> row = M.rows.at(seq + 2);
	for (int numCF = 0; numCF < CF_SIM; numCF++)
	{
		geometry_msgs::PoseStamped myMsg;
		myMsg.header.seq = seq;
		myMsg.header.stamp.sec = row.at(1);
		myMsg.header.frame_id = "world";
		messages.push_back(myMsg);
	}
	int posCount = 0;
	for(size_t i = 3; i < row.size(); i += 2)
	{
		int ref = (i-2)/6;
		switch(posCount)
		{
			case 0:
				messages.at(ref).pose.position.x = row[i];
				break;
			case 1:
				messages.at(ref).pose.position.y = row[i];
				break;
			case 2:
				messages.at(ref).pose.position.z = row[i];
				posCount = -1;
				break;
		}
		posCount++;
	}
	return messages;
}
int main(int argc, char **argv)
{
	matrix myVals = readCSV("5CFSample.csv");

    ros::init(argc, argv, "/optitrack");

    ros::NodeHandle Node;

    std::vector<ros::Publisher> pubs;
	for (int numCF = 0; numCF < CF_SIM; numCF++)
	{
		const std::string topic = "/optitrack/cf" + std::to_string(numCF + 1) + "/pose";
		ros::Publisher myPub = Node.advertise<geometry_msgs::PoseStamped>(topic,QUEUE);
		pubs.push_back(myPub);
	}

    ros::Rate LoopRate(OPTI_FREQ);
	int count = 0;
    while (ros::ok())
    {
		std::vector<geometry_msgs::PoseStamped> messages = set(myVals,count);

		for (int numCF = 0; numCF < CF_SIM; numCF++)
		{
			pubs.at(numCF).publish(messages.at(numCF));
		}

		count++;
        
		ros::spinOnce();
        LoopRate.sleep();
    }

    return 0;
}