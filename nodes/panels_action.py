from __future__ import print_function
from base_action import BaseAction
import numpy as np
import rospy
import time

from vums_arena.msg import ActionData
#from rolling_angular_velocity import RollingAngularVelocity

class PanelsAction(BaseAction):

    index_to_patter_id = {}

     #Possible virtual open loop implementation
#    def update(self,t,angle, ang_velo):
#        msg = super(PanelsAction,self).update(t,angle)  # start and stop occurs here

#        #rospy.logwarn('{}: is_stopped: {}, {} '.format(t, self.is_stopped, self.param['mode']))

#        if not self.is_stopped and self.param['mode'] == 'virtual_open_loop': 
##            gain_amp = 100
##            gain_period = 5.0
##            gain_x = int(gain_amp*np.sin(2.0*np.pi*t/gain_period))
#            self.device.stop()
#            self.device.send_gain_bias( 
#                    gain_x = ang_velo, 
#                    bias_x = self.param['bias_x'],
#                    gain_y = self.param['gain_y'],
#                    bias_y = self.param['bias_y']
#                    )
#            self.device.start()

#        return msg


    def __init__(self,init_angle, device,param,trial_index): #fponce edit 
        #print('panels action __init__')
        super(PanelsAction,self).__init__(device,param)
        self.init_angle = init_angle
        self.pattern_id = None
        self.last_update_t = None 
        self.trial_index = trial_index
        #self.ang_velo = ang_velo

    def stop(self):
        if not self.is_stopped:
            if not ('nostop' in self.param):
                self.panels_off()
            self.is_stopped = True
            

    def start(self):
        #print('panels action start')
        if not self.is_started:
            pattern_id = self.get_pattern_id()
            self.show_pattern(pattern_id)
            self.is_started = True

    def show_pattern(self,pattern_id):

        if pattern_id == 'off':
            self.device.stop()
            #self.device.set_mode('xrate=funcx','yrate=funcy')
            self.panels_off() 
        
#        elif self.param['mode'] == 'gain_from_angvel':
#            self.device.set_pattern_id(pattern_id)  
#            self.device.stop()
#            self.device.send_gain_bias(
#                    gain_x = self.get_gain(self.ang_velo), 
#                    bias_x = self.param['bias_x'],
#                    gain_y = self.param['gain_y'],
#                    bias_y = self.param['bias_y']
#                    )
#            self.device.start()          

        elif self.param['mode'] == 'closed_loop':
            self.device.set_pattern_id(pattern_id)  
            self.device.stop()
            self.device.set_position(1,1)
            time.sleep(0.05)
            self.device.set_mode('xrate=ch0','yrate=funcy')
            self.device.send_gain_bias(
                    gain_x = self.param['gain_x'], 
                    bias_x = self.param['bias_x'],
                    gain_y = self.param['gain_y'],
                    bias_y = self.param['bias_y']
                    )
            self.device.start()

        else:
            print('show pattern', pattern_id)
            self.device.set_pattern_id(pattern_id)      
            self.device.stop()
#            self.device.set_position(1,1)
#            time.sleep(0.05)
            self.device.set_mode('xrate=funcx','yrate=funcy')
            self.device.send_gain_bias(
                    gain_x = self.param['gain_x'], 
                    bias_x = self.param['bias_x'],
                    gain_y = self.param['gain_y'],
                    bias_y = self.param['bias_y']
                    )
            self.device.start()

    def panels_off(self):
        self.device.stop()
        self.device.all_off()
        
#    def get_gain(self, ang_velo):
#        if self.param['mode'] == 'gain_from_angvel':
#            return ang_velo

#        self.pattern_indices = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
#        self.closed_loop_gain = -1.0
#        self.closed_loop_bias = 0.0
#        self.open_loop_gain = 5.0
#        self.open_loop_bias = 0.0
#        self.open_loop_duration = 10.0
#        self.open_loop_wait_period = 30.0 fly must be flying this period before firing the open loop response
#        self.pattern_id = 0
#        self.open_or_closed_loop = 0

#    def closed_loop_mode(self):
#        self.device.set_position(1,1)
#        time.sleep(0.05)
#        self.device.set_mode('xrate=ch0','yrate=funcy')
#        time.sleep(0.05)
#        self.device.send_gain_bias(gain_x = self.closed_loop_gain, bias_x = self.closed_loop_bias,gain_y = 0,bias_y = 0)
#        time.sleep(0.05)
#        self.device.start()
#        time.sleep(0.05)

#    def open_loop_mode(self):
#        self.device.set_position(1,1)
#        time.sleep(0.05)
#        self.device.set_mode('xrate=funcx','yrate=funcy')
#        time.sleep(0.05)
#        self.device.send_gain_bias(gain_x = self.open_loop_gain, bias_x = self.open_loop_bias,gain_y = 0,bias_y = 0)
#        time.sleep(0.05)
#        self.device.start()
#        time.sleep(self.open_loop_duration)
#        self.device.stop()
#        time.sleep(0.05)


    def get_pattern_id(self):
        if self.param['mode'] == 'fixed_pattern':
            pattern_id = self.param['pattern_id']
            #fponce edit##
            self.index_to_patter_id[self.trial_index] = pattern_id
        #fponce edit
        
        elif self.param['mode'] == 'gain_from_angvel':
            pattern_id = self.param['pattern_id']
            self.index_to_patter_id[self.trial_index] = pattern_id       
        
        elif self.param['mode'] == 'inherit_from_last':
            inherit_index = self.trial_index-1
            pattern_id = self.index_to_patter_id[inherit_index]
            self.index_to_patter_id[self.trial_index] = pattern_id

        elif self.param['mode'] == 'inherit':
            inherit_index = self.param['inherit_from']
            pattern_id = self.index_to_patter_id[inherit_index]
            self.index_to_patter_id[self.trial_index] = pattern_id
            
        elif self.param['mode'] == 'inherit_n_set_by_table':
            inherit_index = self.param['inherit_from']
            
            self.prev_pattern = self.index_to_patter_id[inherit_index]
            self.position = self.get_pattern_from_patterntable(self.prev_pattern)
            pattern_id =  self.position
            self.index_to_patter_id[self.trial_index] = self.position

        elif self.param['mode'] == 'closed_loop':
            pattern_id = self.param['pattern_id']
            #fponce edit##
            self.index_to_patter_id[self.trial_index] = pattern_id
            
#        elif self.param['mode'] == 'virtual_open_loop':
#            pattern_id = self.param['pattern_id']
#            #fponce edit##
#            self.index_to_patter_id[self.trial_index] = pattern_id
            
        #####
        else:
            angle_pair_list = [angle_pair for angle_pair,pair_id in self.param['pattern_table']]
            pair_id_list  = [pair_id for angle,pair_id in self.param['pattern_table']]
            pattern_id = 'off'
            for angle_pair, pair_id in zip(angle_pair_list, pair_id_list):
                lower_angle, upper_angle = angle_pair
                if lower_angle <= self.init_angle and self.init_angle <= upper_angle:
                    pattern_id = pair_id
                    #fponce edit
                    self.index_to_patter_id[self.trial_index] = pattern_id
                    break
        return pattern_id
        
######################
    #fponce edit# get a pattern based on what pattern was shown in the past in a different trial
    def get_pattern_from_patterntable(self,prev_position): 
        prev_pattern_list = [prev_pattern for prev_pattern,pattern_index in self.param['pattern_to_pattern_table']]
        pattern_index_list  = [pattern_index for old_pattern,pattern_index in self.param['pattern_to_pattern_table']]
        position = None
        for prev_pattern, pattern_index in zip(prev_pattern_list, pattern_index_list):
            if prev_pattern == prev_position:
                position = pattern_index
        return position

