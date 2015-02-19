#! /usr/bin/env python

import sys
import rospy
from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.product_mdp import ProductMdp
from mdp_plan_exec.prism_client import PrismClient

if __name__ == '__main__':
    rospy.init_node('test_client')
    

    top_map_mdp=TopMapMdp('lg_june14_small')

    
    #top_map_mdp.update_nav_statistics()
    top_map_mdp.write_prism_model('/home/bruno/Desktop/teste.prism')
    
    #directory = '/home/bruno/Desktop/'

    #port=8085    
    #prism_client=PrismClient(port, directory)
    #prism_client.add_model('test','/home/bruno/Desktop/teste.prism')
    #prism_client.get_state_vector('test', 'R{"time"}min=? [ (F "WayPoint1") & (F "WayPoint6")]')
    #prism_client.get_policy('test', 'R{"time"}min=? [ (!"WayPoint3" U "WayPoint5")]')
    #prism_client.get_policy('test', 'R{"time"}min=? [ (F "WayPoint1") & (F "WayPoint6") & (F "WayPoint4")]')
    #prism_client.get_policy('test', 'R{"time"}min=? [ (F "WayPoint1") & (F "WayPoint6")]')
    #prism_client.get_policy('test', 'Pmax=? [ ((!"WayPoint3") U "WayPoint5") & ((!"WayPoint3") U "WayPoint6")]')
    

    #product=ProductMdp(top_map_mdp, directory+'test/prod.sta',directory+'test/prod.lab',directory+'test/prod.tra',directory+'test/prod.aut')
    #product.write_prism_model('/home/bruno/Desktop/product.prism')
    
    #prism_client.add_model('test2','/home/bruno/Desktop/product.prism')
    #prism_client.get_policy('test2', 'R{"time"}min=? [ (F ("WayPoint2" & (X "WayPoint3")))]')
    
    #product=ProductMdp(product, directory+'test2/prod.sta',directory+'test2/prod.lab',directory+'test2/prod.tra',directory+'test2/prod.aut')
    #product.write_prism_model('/home/bruno/Desktop/product2.prism')
    
    #prism_client.add_model('new_rew', '/home/bruno/Desktop/product.prism')
    #prism_client.get_policy('new_rew', 'multi(R{"time"}min=? [ (F "dra_acc_state1")], R{"goal1_rew"}max=? [ (F "dra_acc_state1")])')
    ##prism_client.get_policy('new_rew', 'multi(R{"time"}min=? [ (F "dra_acc_state1")],Pmax=? [ (F "dra_acc_state1")])')

    ##doors_top_map_mdp=DoorsTopMapMdp('lg_june14')
    ##doors_top_map_mdp.write_prism_model('/home/bruno/Desktop/doors.prism')

    
    #prism_client.shutdown(False)