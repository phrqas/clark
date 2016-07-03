(define (domain mitsubishi-domain)
  (:requirements :strips :typing :durative-actions)
    (:types
        agent - thing
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

    (:durative-action pick
      :parameters (?obj - thing ?manip - manipulator ?loc - location ?a - agent)
      :duration (= ?duration 40)
      :condition
        (and
          (at start (part-of ?manip ?a))
          (at start (clear ?obj))
          (at start (at ?obj ?loc))
          (at start (empty ?manip))
          (at start (reachable ?loc ?manip)))
      :effect
        (and
          (at end (holding ?obj ?manip) )
          (at end (accessible ?loc) )
          (at end (not (clear ?obj)))
          (at end (not (at ?obj ?loc)))
          (at end (not (empty ?manip))))
      )

    (:durative-action place
      :parameters  (?obj - thing ?manip - manipulator ?loc - location ?a - agent)
      :duration (= ?duration 40)
      :condition
        (and
          (at start (part-of ?manip ?a))
          (at start (holding ?obj ?manip))
          (at start (placeable ?obj ?loc))
          (at start (accessible ?loc)))

      :effect
        (and
          (at end (clear ?obj))
          (at end (empty ?manip))
          (at end (at ?obj ?loc))
          (at end (not (holding ?obj ?manip)))
          (at end (not (accessible ?loc))))
      )

    (:durative-action clean
      :parameters  (?loc - location ?manip - manipulator ?obj - thing ?a - agent)
      :duration (= ?duration 40)
      :condition
        (and
          (at start (part-of ?manip ?a))
          (at start (holding ?obj ?manip))
          (at start (accessible ?loc))
          (at start (isCleaner ?obj)))
      :effect (and (at end (isclean ?loc)))
      )

    (:durative-action solder
      :parameters  (?obj - thing ?manip - manipulator ?solderinglocation - location ?sold - thing ?a - agent)
      :duration (= ?duration 40)
      :condition
        (and
          (at start (part-of ?manip ?a))
          (at start (at ?obj ?solderinglocation) )
          (at start (clear ?obj) )
          (at start (holding ?sold ?manip) )
          (at start (isclean ?solderinglocation) )
          (at start (isComponent ?obj) )
          (at start (isSolder ?sold))
          (at start (isHuman ?manip)))
      :effect (and (at end (soldered ?obj)))
      )

      (:durative-action recover
        :parameters (?obj - thing ?manip - manipulator ?a - agent)
        :duration (= ?duration 40)
        :condition
          (and
            (at start (part-of ?manip ?a))
        		(at start (available ?manip))
        		(at start (not-moving ?a))
        		(at start (unknownposition ?obj))
        		(at start (isHuman ?manip))
        		(at start (empty ?manip)))
      :effect
        (and
      		(at end (not-moving ?a))
      		(at end (holding ?obj ?manip))
          (at end (not (empty ?manip)))
  		)
    )

    ;;(:durative-action pass
    ;;  :parameters (?obj - thing ?manip1 - manipulator ?manip2 - manipulator ?a - agent)
    ;;  :duration (= ?duration 40)
    ;;  :condition
    ;;    (and
    ;;      (at start (holding ?obj ?manip1) )
    ;;      (at start (empty ?manip2))
    ;;      (at start (isHuman ?manip2)))
    ;;  :effect
    ;;    (and
    ;;      (at end (holding ?thing ?manip2) )
    ;;      (at end (not (holding ?thing ?manip1)) )
    ;;      (at end (not (empty ?manip2)))
    ;;      (at end (empty ?manip1)))
    ;;  )

      )
