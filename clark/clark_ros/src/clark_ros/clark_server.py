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
ROS node that allows CLARK to be called as a service.

@author: Pedro Santana (psantana@mit.edu).
"""
import rospy
from clark_ros.srv import CLARKSrv,CLARKSrvResponse
from clark_ros.msg import KeyValue
from clark.clark_system import CLARK


class CLARKServer(object):
    """Class encapsulating the CLARK server functionality."""
    def __init__(self):
        rospy.loginfo('Creating CLARK service')
        self._clark_service = rospy.Service('clark_service', CLARKSrv,self._clark_service_cb)
        rospy.loginfo('CLARK service created!')
        rospy.loginfo('Creating CLARK instance')
        self._clark = CLARK()
        rospy.loginfo('CLARK instance created! Use ctrl+c to interrupt.')

    def _clark_service_cb(self,req):
        """Handler of CLARK service requests."""
        resp = CLARKSrvResponse()

        resp.success,output_dict = self._clark.call(req.type,req.params)

        if resp.success:
            rospy.loginfo('Call to CLARK completed successfully!')
            resp.tpn = output_dict['rmpyl'].to_ptpn(filename=None)
            resp.performance_dict = [KeyValue(str(k),str(v))for k,v in output_dict['performance'].items()]
        else:
            rospy.logwarn('Call to CLARK failed...')

        resp.header.stamp = rospy.Time.now() #Records time when solution was obtained

        return resp

if __name__=='__main__':

    rospy.init_node('clark_service_node')
    rospy.loginfo('Initializing the CLARK server')
    clark_server = CLARKServer()
    rospy.spin()
    rospy.loginfo('CLARK server terminated')
