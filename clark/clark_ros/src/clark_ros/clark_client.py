#!/usr/bin/env python
#
#  Copyright (c) 2016 MIT. All rights reserved.
#
#   author: Pedro Santana
#   e-mail: psantana@mit.edu
#   website: people.csail.mit.edu/psantana
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#  3. Neither the name(s) of the copyright holders nor the names of its
#     contributors or of the Massachusetts Institute of Technology may be
#     used to endorse or promote products derived from this software
#     without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
"""
Example Python node that interacts with the CLARK server.

@author: Pedro Santana (psantana@mit.edu).
"""
import rospy
from clark_ros.srv import CLARKSrv,CLARKSrvRequest
import clark
import time
from os.path import join

class CLARKClient(object):
    """Class encapsulating the CLARK server functionality."""
    def __init__(self):
        rospy.loginfo('Waiting for CLARK service to be available')
        rospy.wait_for_service('clark_service')
        self._clark_srv = rospy.ServiceProxy('clark_service', CLARKSrv)
        rospy.loginfo('CLARK service found!')

    def request_service(self,stype,params):
        """Request a policy from CLARK."""
        start_time = time.time()
        req = CLARKSrvRequest()

        if stype.lower() in ['pddl','rmpyl','crmpl','ccpomdp']:
            req.type = stype.lower()
            req.params = params

            req.header.stamp = rospy.Time.now()
            resp = self._clark_srv(req)
            elapsed_time = time.time()-start_time
        else:
            rospy.logwarn(stype+' is not a valid type of CLARK service.')
            resp=None

        if resp == None:
            rospy.logwarn('CLARK service failed')
        else:
            rospy.loginfo('CLARK service replied!')

        print('\nTotal time (including ROS overhead): %.4f s\n'%(elapsed_time))
        return resp

if __name__=='__main__':
    rospy.init_node('clark_client_node')
    rospy.loginfo('Initializing CLARK client')
    clark_client = CLARKClient()

    clark_path = clark.__path__[0] #Path to CLARK module

    stype = 'pddl'
    params = '%s %s %s --model %s --duration %s --schedule --makespan --svg'%(
                 join(clark_path,'examples/pddl/domain.pddl'),
                 join(clark_path,'examples/pddl/problem.pddl'),
                 '/tmp/clark_pddl_output.tpn',
                 join(clark_path,'examples/pddl/boeing_demo_duration_model.py'),
                 'duration_func')

    resp = clark_client.request_service(stype,params)

    if resp != None:
        print('\n##### TPN:')
        print(resp.tpn)

        print('\n##### Performance stats:')
        for keyvalue in resp.performance_dict:
            print('%s: %s'%(keyvalue.key,keyvalue.value))        

    rospy.loginfo('Press ctrl+c to terminate.')
    rospy.spin()
    rospy.loginfo('CLARK client terminated')
