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
Script allowing CLARK to be called with a generic CC-POMDP model.

@author: Pedro Santana (psantana@mit.edu).
"""
from rao.raostar import RAOStar
from rao.export import policy_to_rmpyl
from clark.clark_utils import rmpyl_post_processing,import_function_from_module_file
import argparse

def ccpomdp_clark(ccpomdp_model,b0,out_file,svg,cc,cc_type,terminal_prob,randomization,
                  propagate_risk,expand_all_open,verbose,log,constraint_fields,
                  det_schedule,stnu_ref,animation):
    """
    Calls CLARK with a generic CC-POMDP model.
    """
    planner = RAOStar(ccpomdp_model,node_name='id',cc=cc,cc_type=cc_type,
                      terminal_prob=terminal_prob,randomization=randomization,
                      propagate_risk=propagate_risk,expand_all_open=expand_all_open,
                      verbose=verbose,log=log,animation=animation)

    policy,explicit,performance = planner.search(b0)

    if len(policy)>0:
        rmpyl_policy = policy_to_rmpyl(explicit,policy,
                                       constraint_fields=constraint_fields,
                                       global_end=ccpomdp_model.global_end_event)

        #Function to allow RMPyL policies to be manually modified to suit the needs
        #of whoever is calling CLARK (oh, Enterprise... why do you torture me?)
        rmpyl_post_processing(rmpyl_policy,stnu_ref,det_schedule)

        return True,{'policy':policy,'explicit':explicit,'performance':performance,
                     'rmpyl':rmpyl_policy}
    else:
        return False,{'policy':policy,'explicit':explicit,'performance':performance,
                     'rmpyl':None}

def get_argument_parser():
    """
    Returns an argument parser object.
    """
    parser = argparse.ArgumentParser(description="Allows CLARK to be called using a generic CC-POMDP input.")
    parser.add_argument("mfile",type=str,help="Path to Python file containing the CC-POMDP planning model.")
    parser.add_argument("mfunc",type=str,help="Function in the model file that outputs the CC-POMDP model and initial belief b0.")
    parser.add_argument("--modelargs",nargs='*',default=[],help="List of arguments that should be given to model function.")
    parser.add_argument("--detschedule",nargs=2,help="Makes all temporal constraints controllable with given bounds. Format is [lb ub].")
    parser.add_argument("--stnuref",nargs=3,help="Performs STNU reformulation in the output policy. Format is [pseudocontrol makespan cc].")
    parser.add_argument("--constraints",nargs='*',default=['constraints'],help="State fields containing constraints.")

    return parser

def call_clark(args):
    """
    Makes the appropriate call to CLARK.
    """
    #Calls the function that returns the CC-POMDP model object and initial belief
    model_func = import_function_from_module_file(args.mfile,args.mfunc)
    ccpomdp_model,b0 = model_func(*args.modelargs)

    return ccpomdp_clark(ccpomdp_model,b0,args.output,args.svg,args.cc,args.cctype,args.terminalprob,
                         args.random,not args.noriskpropag,not args.singleexpand,args.verbose,
                         args.log,args.constraints,args.detschedule,args.stnuref,
                         args.animation)

if __name__=='__main__':
    parser = get_argument_parser()
    args = parser.parse_args() #Gets the arguments from the command line
    call_clark(args)
