#include <iostream>
#include <fstream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>

#include <boost/filesystem.hpp>

int main(int argc, char** argv) {
	if (argc < 3) {
		std::cout << "Usage: convert <urdf file> <bag files>" << std::endl;
		return 1;
	}

	KDL::Tree robot;
	if (!kdl_parser::treeFromFile(argv[1], robot)) {
		std::cout << "Failed to build tree" << std::endl;
		return 1;
	}
	
	unsigned int nj = robot.getNrOfJoints();
	

	KDL::TreeFkSolverPos_recursive fksolver(robot);
	KDL::Frame cart_pos;
	KDL::JntArray joints(robot.getNrOfJoints());
	
	std::vector<std::string> frames;
	frames.push_back("mico_link_1");
	frames.push_back("mico_link_2");
	frames.push_back("mico_link_3");
	frames.push_back("mico_link_4");
	frames.push_back("mico_link_5");
	frames.push_back("mico_link_hand");
	
	typedef std::map<std::string, unsigned int> JointNumMapType;
	JointNumMapType joint_num_map;
	for (KDL::SegmentMap::const_iterator it = robot.getSegments().begin(); it != robot.getSegments().end(); ++it) {
	    if (it->second.segment.getJoint().getType() != KDL::Joint::None) {
	        std::cout << it->second.segment.getJoint().getName() << ": " << it->second.q_nr << std::endl;
	        joint_num_map[it->second.segment.getJoint().getName()] = it->second.q_nr;
	    }
	}
	
	std::vector<std::string> topics;
	topics.push_back("/joint_states");
	
	for (int i=2; i<argc; ++i) {
	    rosbag::Bag bag(argv[i]);
	    std::cout << "Opened bag: " << argv[i] << std::endl;
	    
	    boost::filesystem::path path(argv[i]);
	    boost::filesystem::path outfile = path.parent_path() / path.stem();
	    outfile += std::string("_export.csv");
	    
	    std::ofstream outf(outfile.native().c_str());
	    outf << "time";
	    for (std::vector<std::string>::const_iterator it = frames.begin(); it != frames.end(); ++it) {
	        outf << "," << *it << "_x";
	        outf << "," << *it << "_y";
	        outf << "," << *it << "_z";
	    }
	    outf << "\n";
	    
	    rosbag::View view(bag, rosbag::TopicQuery(topics));
	    for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
//	         std::cout << "message at " << it->getTime() << " topic: " << it->getTopic() << std::endl;
            outf << it->getTime();
	         sensor_msgs::JointState::ConstPtr joint_state_ptr = it->instantiate<sensor_msgs::JointState>();
	         if (joint_state_ptr) {
//	             std::cout << "read in joint state: " << *joint_state_ptr << std::endl;
	             for (int j=0; j<joint_state_ptr->position.size(); ++j) {
//	                std::cout << "Searching for " << joint_state_ptr->name[j] << std::endl;
	                JointNumMapType::const_iterator joint_num = joint_num_map.find(joint_state_ptr->name[j]);
	                if (joint_num != joint_num_map.end()) {
	                    joints(joint_num->second) = joint_state_ptr->position[j];
//	                    std::cout << "Set " << joint_state_ptr->name[j] << " to " << joint_state_ptr->position[j] << std::endl;
	                } else {
//	                    std::cout << "Failed to find joint for " << joint_state_ptr->name[j] << std::endl;
	                }
	             }
	             
	             for (std::vector<std::string>::const_iterator frame = frames.begin(); frame != frames.end(); ++frame) {
	                int status = fksolver.JntToCart(joints, cart_pos, *frame);
//	                std::cout << *frame << ": ";
	                if (status >= 0) {
//	                    std::cout << cart_pos.p;
                        outf << "," << cart_pos.p(0) << "," << cart_pos.p(1) << "," << cart_pos.p(2);
	                } else {
//	                    std::cout << status;
	                }
//                    std::cout << std::endl;
	             }
	             outf << std::endl;
	         } else {
//	            std::cout << "got empty joint state pointer" << std::endl;
	         }
	    }
	}
    
    
	
	return 0;
}
