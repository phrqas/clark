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
Script allowing CLARK to be called as a PDDL planner. It assumes that STRIPS
problem and domain files are given, and allows its to be extended with all
types of temporal constraints supported by PARIS through a function pointer.

@author: Pedro Santana (psantana@mit.edu).
"""
from rao.raostar import RAOStar
from rao.external_planners.pysat import PySAT
from rao.pddl.model_parser import model_parser
from rao.export import policy_to_rmpyl
from rao.models.pddlmodel import DurativePDDL
from rmpyl.rmpyl import RMPyL
from rmpyl.episodes import Episode
from rmpyl.constraints import TemporalConstraint
from rmpyl.defs import Event
from pytemporal.paris import PARIS

from clark.clark_utils import rmpyl_post_processing,import_function_from_module_file

import argparse
import time

def clark_planner(domain_file,problem_file,max_steps,cc,duration_func,schedule,
                  timewindow,animation):
    """
    Uses CLARK as the planner.
    """
    pddl_model = DurativePDDL(domain_file=domain_file,prob_file=problem_file,
                              perform_scheduling=schedule,
                              duration_func=duration_func,
                              max_steps=max_steps,
                              verbose=1)

    initial_constraints=[]
    if timewindow != None:
        initial_constraints.append(TemporalConstraint(start=pddl_model.global_start_event,
                                                      end=pddl_model.global_end_event,
                                                      ctype='controllable',lb=0.0,ub=timewindow))

    b0 = pddl_model.get_initial_belief(constraints=initial_constraints)

    planner = RAOStar(pddl_model,node_name='id',cc=cc,cc_type='overall',
                      terminal_prob=1.0,randomization=0.0,propagate_risk=True,
                      expand_all_open=True,verbose=1,log=False,animation=animation)

    policy,explicit,performance = planner.search(b0)

    if (len(policy)>0) and (performance['optimal_value'] != float('inf')):
        rmpyl_policy = policy_to_rmpyl(explicit,policy,
                                       constraint_fields=['constraints'],
                                       global_end=pddl_model.global_end_event)

        #Combines controllable temporal constraints.
        rmpyl_policy.simplify_temporal_constraints()
    else:
        rmpyl_policy = None

    return {'policy':policy,'explicit':explicit,'performance':performance,
            'rmpyl':rmpyl_policy}

def pysat_planner(dom_file,prob_file,max_steps,duration_func):
    """
    Uses PySAT as the planner.
    """
    py_sat = PySAT(dom_file,prob_file,precompute_steps=max_steps,remove_static=True,
                   write_dimacs=False,verbose=True)

    domain,problem,task = model_parser(dom_file,prob_file,remove_static=True)

    print('\n##### Determining optimal plan length!\n')
    start = time.time()
    min_steps = len(task.goals-task.initial_state)
    plans = py_sat.plan(task.initial_state,task.goals,time_steps=max_steps,
                        find_shortest=True,min_steps=min_steps)
    elapsed = time.time()-start
    print('\n##### All solving took %.4f s'%(elapsed))

    if len(plans)>0:
        plan = plans[0]
        print('\n##### Plan found!\n')
        for t,action in enumerate(plan):
            print('%d: %s'%(t,action))

        prog = RMPyL(name='run()')
        if duration_func!=None:
            prog.plan = prog.sequence(*[Episode(start=Event(name='start-of-'+op),
                                                end=Event(name='end-of-'+op),
                                                action=op,
                                                duration=duration_func(op)) for op in plan])
        else:
            prog.plan = prog.sequence(*[Episode(start=Event(name='start-of-'+op),
                                                end=Event(name='end-of-'+op),
                                                action=op) for op in plan])
    else:
        prog = None

    return {'policy':None,'explicit':None,'performance':None,'rmpyl':prog}

def pddl_clark(dom_file,prob_file,out_file,use_pysat,maxsteps,cc,duration_func,
               schedule,makespan,timewindow,det_schedule,stnu_ref,animation):
    """
    Calls CLARK as a PDDL planner. It assumes that STRIPS problem and domain files
    are given, and allows its to be extended with all types of temporal constraints
    supported by PARIS through a function pointer.
    """
    if use_pysat:
        output_dict = pysat_planner(dom_file,prob_file,maxsteps,duration_func)
    else:
        output_dict = clark_planner(dom_file,prob_file,maxsteps,cc,duration_func,
                                     schedule,timewindow,animation)

    if output_dict['rmpyl'] != None:
        #Function to allow RMPyL policies to be manually modified to suit the needs
        #of whoever is calling CLARK (oh, Enterprise... why do you torture me?)
        rmpyl_post_processing(output_dict['rmpyl'],stnu_ref,det_schedule)

        if schedule:
            paris = PARIS()
            risk_bound,sc_schedule = paris.stnu_reformulation(output_dict['rmpyl'],makespan=makespan,cc=cc)
            if risk_bound != None:
                risk_bound = min(risk_bound,1.0)
                print('\nSuccessfully performed STNU reformulation with scheduling risk %f %%!'%(risk_bound*100.0))
                print('\nThis is the schedule:')
                for e,t in sorted([(e,t) for e,t in sc_schedule.items()],key=lambda x: x[1]):
                    print('\t%s: %.2f s'%(e,t))
            else:
                print('\nFailed to perform STNU reformulation...')
                sc_schedule=risk_bound=None
    else:
        sc_schedule=risk_bound=None
        domain,problem,task = model_parser(dom_file,prob_file,remove_static=False)
        print('Plan not found!')
        initial_conditions = sorted(list(task.initial_state))
        print('Initial conditions:')
        for p in initial_conditions:
            print('\t%s'%(str(p)))

    output_dict['sc_schedule']=sc_schedule
    output_dict['sc_risk_bound']=risk_bound

    return output_dict['rmpyl']!=None,output_dict

def get_argument_parser():
    """
    Returns an argument parser object.
    """
    parser = argparse.ArgumentParser(description="Allows CLARK to be used as a PDDL-like planner.")
    parser.add_argument("domain",type=str,help="PDDL domain file.")
    parser.add_argument("problem",type=str,help="PDDL problem file.")
    parser.add_argument("--model",type=str,help="Path to Python file containing the planning model.")
    parser.add_argument("--duration",type=str,default='duration_func',help="Name of activity duration function in planning model.")
    parser.add_argument("--schedule",action="store_true",help="Whether to perform scheduling (uses PARIS).")
    parser.add_argument("--detschedule",nargs=2,help="Makes all temporal constraints controllable with given bounds. Format is [lb ub].")
    parser.add_argument("--stnuref",nargs=3,help="Performs STNU reformulation in the output policy. Format is [pseudocontrol makespan cc].")
    parser.add_argument("--timewindow",type=float,help="Chance constraint value (between 0 and 1).")
    parser.add_argument("--pysat",action="store_true",help="Whether to use PySAT instead of CLARK.")
    parser.add_argument("--makespan",action="store_true",help="Whether to return minimum makespan schedules.")
    parser.add_argument("--maxsteps",type=int,default=30,help="Maximum number of steps allowed for the plan.")
    return parser

def call_clark(args):
    """
    Makes the appropriate call to CLARK.
    """
    #Verifies if a reference to a duration function was provided
    if args.model==None:
        duration_func=None
    else:
        duration_func = import_function_from_module_file(args.model,args.duration)

    return pddl_clark(args.domain,args.problem,args.output,args.pysat,args.maxsteps,
                      args.cc,duration_func,args.schedule,args.makespan,args.timewindow,
                      args.detschedule,args.stnuref,args.animation)

if __name__=='__main__':
    parser = get_argument_parser()
    args = parser.parse_args() #Gets arguments from the command line
    call_clark(args)
