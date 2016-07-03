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
Script allowing CLARK to generate optimal conditional plans from an RMPyL program.
If conditional scheduling is required, uses the unconditional scheduling
strategy first presented at ICAPS14, combined with PARIS.

@author: Pedro Santana (psantana@mit.edu).
"""
from rao.raostar import RAOStar
from rao.export import policy_to_rmpyl
from rao.models.rmpylmodel import BaseRMPyLModel,StrongStrongRMPyLModel

from clark.clark_utils import rmpyl_post_processing,import_function_from_module_file

import argparse

def rmpyl_clark(prog,out_file,svg,schedule,cc,cc_type,terminal_prob,randomization,
                propagate_risk,expand_all_open,verbose,log,det_schedule,stnu_ref,
                animation):
    """
    Used CLARK to generate optimal conditional plans from an RMPyL program.
    If conditional scheduling is required, uses the unconditional scheduling
    strategy first presented at ICAPS14, combined with PARIS.
    """
    if schedule:
        rmpyl_model = StrongStrongRMPyLModel(prog,verbose=verbose)
    else:
        rmpyl_model = BaseRMPyLModel(prog,verbose=verbose)

    b0 = rmpyl_model.get_initial_belief()

    planner = RAOStar(rmpyl_model,node_name='id',cc=cc,cc_type=cc_type,
                      terminal_prob=terminal_prob,randomization=randomization,
                      propagate_risk=propagate_risk,expand_all_open=expand_all_open,
                      verbose=verbose,log=log,animation=animation)

    policy,explicit,performance = planner.search(b0)

    if len(policy)>0:
        rmpyl_policy = policy_to_rmpyl(explicit,policy)

        #Function to allow RMPyL policies to be manually modified to suit the needs
        #of whoever is calling CLARK (oh, Enterprise... why do you torture me?)
        rmpyl_post_processing(rmpyl_policy,stnu_ref,det_schedule)
    else:
        rmpyl_policy = None

    return len(policy)>0,{'policy':policy,'explicit':explicit,'performance':performance,
                          'rmpyl':rmpyl_policy}

def get_argument_parser():
    """
    Returns an argument parser object.
    """
    parser = argparse.ArgumentParser(description="Allows CLARK to be called using a cRMPL (RMPyL) input.")
    parser.add_argument("mfile",type=str,help="Path to Python file containing the RMPyL model.")
    parser.add_argument("mfunc",type=str,help="Function in the model file that outputs the RMPyL program.")
    parser.add_argument("--modelargs",nargs='*',default=[],help="List of arguments that should be given to model function.")
    parser.add_argument("--schedule",action="store_true",help="Whether to perform scheduling.")
    parser.add_argument("--detschedule",nargs=2,help="Makes all temporal constraints controllable with given bounds. Format is [lb ub].")
    parser.add_argument("--stnuref",nargs=3,help="Performs STNU reformulation in the output policy. Format is [pseudocontrol makespan cc].")
    return parser

def call_clark(args):
    """
    Makes the appropriate call to CLARK.
    """
    #Calls the function that returns the RMPyL program
    model_func = import_function_from_module_file(args.mfile,args.mfunc)
    prog = model_func(*args.modelargs)

    return rmpyl_clark(prog,args.output,args.svg,args.schedule,args.cc,args.cctype,
                       args.terminalprob,args.random, not args.noriskpropag,
                       not args.singleexpand,args.verbose,args.log,args.detschedule,
                       args.stnuref,args.animation)

if __name__=='__main__':
    parser = get_argument_parser()
    args = parser.parse_args() #Gets the arguments from the command line
    call_clark(args)
