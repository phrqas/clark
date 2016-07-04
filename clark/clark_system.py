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
High-level interface to the CLARK system.
"""
import clark.pddl_clark as pddlc
import clark.rmpyl_clark as rmpylc
import clark.ccpomdp_clark as ccpomdpc
from rao.export import policy_to_dot
import sys

class CLARK(object):
    """
    High-level interface to the CLARK system.
    """
    def __init__(self,debug=False):
        self.debug=debug

    def call(self,req_type,params):
        """
        Calls CLARK with different types of inputs and parameters.
        """
        req_type = req_type.lower()
        if req_type == 'pddl':
            parser = pddlc.get_argument_parser()
            call_func = pddlc.call_clark
        elif req_type in ['crmpl','rmpyl']:
            parser = rmpylc.get_argument_parser()
            call_func = rmpylc.call_clark
        elif req_type == 'ccpomdp':
            parser = ccpomdpc.get_argument_parser()
            call_func = ccpomdpc.call_clark
        else:
            print(req_type+' is an invalid request type.')
            _print_clark_usage()

        _add_common_parsing(parser) #Adds common arguments

        try:
            args = parser.parse_args(params.split())
            success,output_dict = call_func(args)

            #If requested, writes the policy in graphical SVG format.
            if args.svg:
                policy_file = args.output[:args.output.rfind('.')]
                dot_policy = policy_to_dot(output_dict['explicit'],output_dict['policy'])
                dot_policy.write(policy_file+'.svg',format='svg')

            #Generates output tpn
            output_dict['rmpyl'].to_ptpn(filename=args.output)           

        except:
            success,output_dict = False,None
            print('\n##### ERROR: could not process request\n\tType: %s\n\tParams: %s\n'%(req_type,params))
            if self.debug:
                import ipdb; ipdb.set_trace()
            raise

        return success,output_dict

def _add_common_parsing(parser):
    """
    Adds argument parsing fields that are common to all types of calls.
    """
    parser.add_argument("output",type=str,help="Plan output file.")
    parser.add_argument("--cc",type=float,default=1.0,help="Chance constraint value (between 0 and 1).")
    parser.add_argument("--cctype",type=str,default='overall',help="Type of chance constraint (overall or everywhere).")
    parser.add_argument("--terminalprob",type=float,default=1.0,help="Probability threshold for terminal nodes.")
    parser.add_argument("--random",type=float,default=0.0,help="Probability of randomly choosing a suboptimal node to explore.")
    parser.add_argument("--noriskpropag",action="store_true",help="Disables risk propagation.")
    parser.add_argument("--singleexpand",action="store_true",help="Expands a single node per iteration.")
    parser.add_argument("--verbose",type=int,default=1,help="Verbosity level (0, 1, or 2).")
    parser.add_argument("--log",action="store_true",help="Whether to log the quantities.")
    parser.add_argument("--svg",action="store_true",help="Whether to output the policy in SVG format.")
    parser.add_argument("--animation",action="store_true",help="Whether to create a sequence of images animating policy generation.")

def _print_clark_usage():
    """
    Help string for the CLARK system
    """
    print('\n##### CLARK command line usage:')
    print('./clark_system -h: prints this help menu')
    print('./clark_system <req-type> [param]: calls CLARK with one of its available input types and parameters.')

    print('\nSee below the parameters for different input types:')

    for req_type,mod_obj in zip(['pddl','rmpyl','ccpomdp'],[pddlc,rmpylc,ccpomdpc]):
        print('\n##### "%s" requests:'%(req_type))
        parser = mod_obj.get_argument_parser()
        _add_common_parsing(parser) #Adds common arguments

        try:
            parser.parse_args(['-h'])
        #Prevents the argparse from terminating execution with sys.exit()
        except SystemExit:
            pass
        print('#'*40)

if __name__=='__main__':

    if len(sys.argv)<=2 or sys.argv[1].lower() in ['-h','--help']:
        _print_clark_usage()
    else:
        clark = CLARK()
        clark.call(sys.argv[1],' '.join(sys.argv[2:]))
