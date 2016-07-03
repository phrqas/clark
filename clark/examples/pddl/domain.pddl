(define (domain mitsubishi-domain)
  (:requirements :strips :typing)
    (:types
        agent - object
        manipulator - object
        thing - object
        location - object)
    (:predicates
                    (empty ?manip - manipulator)
                    (clear ?obj - thing)
                    (at ?obj - thing ?loc - location)
                    (holding ?obj - thing ?manip - manipulator)
                    (accessible ?loc - location)
                    (unknownposition ?obj - thing)

                    (isclean ?loc - location)
                    (soldered ?obj - thing)
                    (not-soldered ?obj - thing)
                    (not-picked ?obj - thing)

                    (available ?manip - manipulator)
                    (not-moving ?a - agent)

                    (reachable ?loc - location ?manip - manipulator)
                    (isSolder ?obj - thing)
                    (isCleaner ?obj - thing)
                    (isHuman ?manip - manipulator)
                    (isComponent ?obj - thing)
                    (canplace ?obj - thing ?loc - location)
		                (canpick ?manip - manipulator ?obj - thing)
                    (canclean ?loc - location)
		                (part-of ?manip - manipulator ?a - agent)
                    (placeable ?obj - thing ?loc - location)
                    )

    (:action pick
      :parameters (?obj - thing ?manip - manipulator ?loc - location ?a - agent)
      :precondition (and (part-of ?manip ?a) (clear ?obj) (at ?obj ?loc) (empty ?manip) (reachable ?loc ?manip))
      :effect (and (holding ?obj ?manip) (accessible ?loc) (not (clear ?obj)) (not (at ?obj ?loc)) (not (empty ?manip)))
      )

    (:action place
      :parameters  (?obj - thing ?manip - manipulator ?loc - location ?a - agent)
      :precondition (and (part-of ?manip ?a) (holding ?obj ?manip) (accessible ?loc) (placeable ?obj ?loc))
      :effect (and (clear ?obj) (empty ?manip) (at ?obj ?loc) (not (holding ?obj ?manip)) (not (accessible ?loc)))
      )

    (:action clean
      :parameters  (?loc - location ?manip - manipulator ?obj - thing ?a - agent)
      :precondition (and (part-of ?manip ?a) (holding ?obj ?manip) (accessible ?loc) (isCleaner ?obj))
      :effect (isclean ?loc)
      )

    (:action solder
      :parameters  (?obj - thing ?manip - manipulator ?solderinglocation - location ?sold - thing ?a - agent)
      :precondition (and (part-of ?manip ?a) (at ?obj ?solderinglocation) (clear ?obj) (holding ?sold ?manip)
                          (isclean ?solderinglocation) (isComponent ?obj) (isSolder ?sold) (isHuman ?manip))
      :effect (soldered ?obj)
      )

    (:action recover
      :parameters (?obj - thing ?manip - manipulator ?a - agent)
      :precondition
        (and
          (part-of ?manip ?a)
          (available ?manip)
          (unknownposition ?obj)
          (isHuman ?manip)
          (empty ?manip))
    :effect
      (and
        (holding ?obj ?manip)
        (not (empty ?manip)))
  )

    ;;(:action pass
    ;;  :parameters (?obj - thing ?manip1 - manipulator ?manip2 - manipulator ?a - agent)
    ;;  :precondition (and (holding ?obj ?manip1) (empty ?manip2) (isHuman ?manip2))
    ;;  :effect (and (holding ?thing ?manip2) (not (holding ?thing ?manip1)) (not (empty ?manip2)) (empty ?manip1))
    ;;  )
      )
