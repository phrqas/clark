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
Miscellaneous tools for the CLARK system.

@author: Pedro Santana (psantana@mit.edu).
"""
from pytemporal.paris import PARIS

import os.path as pa
import imp

def rmpyl_post_processing(rmpyl_policy,stnu_ref,det_schedule):
    """
    Function to allow RMPyL policies to be manually modified to suit the needs
    of whoever is calling CLARK (oh, Enterprise... why do you torture me?)
    """
    modified_policy=False
    #Performs an STNU reformulation for the schedule and potentially
    if stnu_ref != None:
        modified_policy=True
        pseudocontrol = not (stnu_ref[0].lower() in ['0','false','no'])
        makespan = not (stnu_ref[1].lower() in ['0','false','no'])
        cc = float(stnu_ref[2])
        strong_stnu_reformulation(rmpyl_policy,pseudocontrol,makespan,cc)

    #Turns all temporal constraints in the policy to controllable ones with
    #given bounds.
    if det_schedule!=None:
        modified_policy=True
        determinize_schedule(rmpyl_policy,float(det_schedule[0]),float(det_schedule[1]))

    if modified_policy:
        print('\nWARNING: policy has been manually altered.')


def import_function_from_module_file(filename,funcname):
    """
    Returns a function object from a given Python module.
    """
    head,tail = pa.split(filename)
    module_name = tail.partition('.py')[0]
    return getattr(imp.load_source(module_name,filename),funcname)

def strong_stnu_reformulation(rmpyl_policy,pseudocontrol=False,makespan=False,cc=-1.0):
    """
    Performs the STNU reformulation for chance-constrained strong controllability.
    It also converts all uncontrollable durations to controllable, if pseudo-controllability
    is desired. It alters the RMPyL policy given as input.
    """
    paris = PARIS()
    risk_bound,sc_schedule = paris.stnu_reformulation(rmpyl_policy,makespan=makespan,cc=cc)

    if pseudocontrol:
        for tc in rmpyl_policy.temporal_constraints:
            tc.type = 'controllable'

    return risk_bound,sc_schedule

def determinize_schedule(rmpyl_policy,new_lb=0.1,new_ub=float('inf')):
    """
    Turns every temporal constraint into a controllable temporal constraint,
    effectively rendering the schedule fully controllable.
    """
    for tc in rmpyl_policy.temporal_constraints:
        tc.set_stc(new_lb,new_ub)

    rmpyl_policy.simplify_temporal_constraints()

def dict_to_str(dictio):
    """
    Converts a dictionary to a single string, where key:value become key;;value\n.
    """
    return '\n'.join(['%s;;%s'%(str(k),str(v)) for k,v in dictio.items()])

def str_to_dict(dict_str):
    """
    Converts a key,value\n to a dictionary.
    """    
    return {p[0]:p[1] for p in [t.split(';;') for t in dict_str.split('\n')]}
