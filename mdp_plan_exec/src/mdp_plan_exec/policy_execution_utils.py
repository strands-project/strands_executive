import os
from copy import deepcopy
import rospy

from strands_navigation_msgs.msg import NavRoute

from mdp_plan_exec.policy_mdp import PolicyMdp
from mdp_plan_exec.prism_java_talker import PrismJavaTalker


   
class PolicyExecutionUtils(object): 
    def __init__(self, port, file_dir, file_name, mdp):
        
        self.mdp = mdp
        self.current_nav_policy_state_defs={}
        self.file_dir = file_dir
        self.file_name = file_name
        try:
            os.makedirs(file_dir)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.prism_policy_generator = PrismJavaTalker(port, file_dir, file_name)

    def generate_prism_specification(self, ltl_spec):
        return 'partial(R{"time"}min=? [ (' + ltl_spec + ') ])'
    
    # Returns the parsed policy obtained by prism, or None is there is an error generating the policy.
    def generate_policy_mdp(self, spec, initial_waypoint, epoch):
        specification=self.generate_prism_specification(spec.ltl_task)
        self.mdp.create_top_map_mdp_structure()
        self.mdp.add_extra_domain(spec.vars, spec.actions)
        rospy.loginfo("The specification is: " + specification)
    
        #update initial state
        self.mdp.set_initial_state_from_waypoint(initial_waypoint)
        self.mdp.add_predictions(self.file_dir+self.file_name, epoch)
        prism_call_success=self.prism_policy_generator.call_prism(specification)
        if prism_call_success:
            policy_mdp=PolicyMdp(self.mdp,
                                    self.file_dir + 'prod.sta',
                                    self.file_dir + 'prod.lab',
                                    self.file_dir + 'adv.tra',
                                    self.file_dir + 'guarantees1.vect',
                                    self.file_dir + 'guarantees2.vect',
                                    self.file_dir + 'guarantees3.vect'
                                    )
            self.current_policy_flat_state = policy_mdp.initial_flat_state
            return policy_mdp
        else:
            rospy.logwarn("Error generating policy. Aborting...")
            return None
           
    def shutdown_prism(self, remove_dir):
        self.prism_policy_generator.shutdown(remove_dir)
           
    def generate_current_nav_policy(self, policy_mdp):
        policy_msg = NavRoute()
        current_state_def=policy_mdp.current_state_def
        current_dfa_state=policy_mdp.current_state_def["_da"]
        queue=[policy_mdp.current_flat_state]
        self.current_nav_policy_state_defs={policy_mdp.current_flat_state:current_state_def}
        while queue != []:
            print(queue)
            new_flat_state=queue.pop(0)
            current_state_def=policy_mdp.flat_state_defs[new_flat_state]
            if policy_mdp.flat_state_policy.has_key(new_flat_state):
                action=policy_mdp.flat_state_policy[new_flat_state]
                if action in self.mdp.nav_actions and current_state_def["_da"]==current_dfa_state:
                    source = self.mdp.get_waypoint_prop(current_state_def["waypoint"])
                    policy_msg.source.append(source)
                    policy_msg.edge_id.append(action)
                    for suc_state in policy_mdp.flat_state_sucs[new_flat_state]:
                        if not self.current_nav_policy_state_defs.has_key(suc_state):
                            queue.append(suc_state)
                            self.current_nav_policy_state_defs[suc_state] = policy_mdp.flat_state_defs[suc_state]
        return policy_msg

    def get_next_nav_policy_state(self, waypoint, policy_mdp):
        waypoint_val=self.mdp.get_waypoint_var_val(waypoint)
        next_state = None
        #if got feedback from topo nav in a waypoint, check if need to update state
        if waypoint_val is not None:
            rospy.loginfo("Reached waypoint " + waypoint)
            #waypoint change is on the MDP transition model
            for (flat_state, flat_state_def) in self.current_nav_policy_state_defs.items():
                print(flat_state_def)
                if flat_state_def["waypoint"]==waypoint_val:
                    next_state = flat_state
                    break
        return next_state

    def get_next_state_from_wp_update(self, waypoint, policy_mdp):
        waypoint_val=self.mdp.get_waypoint_var_val(waypoint)
        next_state = None
        if waypoint_val is not None:
            new_state_def=deepcopy(self.current_nav_policy_state_defs[policy_mdp.current_flat_state])
            new_state_def["waypoint"]=waypoint_val
            for (flat_state, flat_state_def) in policy_mdp.flat_state_defs.items():
                if policy_mdp.check_cond_sat(new_state_def, flat_state_def):
                    rospy.loginfo("Found state " + str(flat_state_def) + ". Updating.")
                    next_state = flat_state
                    break
        return next_state

    def get_next_state_from_action_outcome(self, action_outcome, policy_mdp):
        current_state_def=deepcopy(policy_mdp.flat_state_defs[policy_mdp.current_flat_state])
        current_state_def.update(action_outcome)
        next_state = None
        del current_state_def["_da"]
        for flat_suc in policy_mdp.flat_state_sucs[policy_mdp.current_flat_state]:
            if policy_mdp.check_cond_sat(current_state_def, policy_mdp.flat_state_defs[flat_suc]):
                next_state = flat_suc
                break
        return next_state
