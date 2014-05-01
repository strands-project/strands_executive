#! /usr/bin/env python

import sys
import rospy


from mdp_plan_exec.mdp import TopMapMdp








if __name__ == '__main__':
    rospy.init_node('test_client')
    

    top_map_mdp=TopMapMdp('top_floor_simple')
    
    #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
    top_map_mdp.update_nav_statistics()
    top_map_mdp.write_prism_model('/home/bruno/Desktop/teste.prism')