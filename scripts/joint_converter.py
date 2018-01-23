

import sys, csv
import itertools
import rosbag
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

BASE_LINK = 'mico_link_base'
END_LINK = 'mico_link_hand'
AXIS_SUFFIX = ['_x', '_y', '_z']

def main(urdf_file, bag_file, out_file):
    robot = URDF.from_xml_file(urdf_file)
    kdl_kin = KDLKinematics(robot, BASE_LINK, END_LINK)
    joint_names = kdl_kin.get_joint_names()
    link_names = kdl_kin.get_link_names()
    
    csv_headers = ['time'] + [ link + axis for link, axis in itertools.product(link_names, AXIS_SUFFIX) ]
    
    with open(out_file, 'wb') as f:
        writer = csv.DictWriter(f, fieldnames=csv_headers, quoting=csv.QUOTE_MINIMAL)
        writer.writeheader()

        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=['/joint_states']):
            joint_vals = {name: val for name, val in zip(msg.name, msg.position)}
            q = [joint_vals[n] for n in joint_names]
            
            vals = {'time': t.secs + 1e-9*t.nsecs}
            for i, link in enumerate(link_names):
                pos = kdl_kin._do_kdl_fk(q, i)[0:3,3].ravel().tolist()[0]
                vals.update( { link + axis: pos[val] for val,axis in enumerate(AXIS_SUFFIX) } )
            
            writer.writerow(vals)
                
    
if __name__ == "__main__":
    if len(sys.argv) < 4:
        print "Usage: joint-converter.py <urdf-file> <bag-file> <out-file>"
    else:
        urdf_file = sys.argv[1]
        bag_file = sys.argv[2]
        out_file= sys.argv[3]
        main(urdf_file, bag_file, out_file)

