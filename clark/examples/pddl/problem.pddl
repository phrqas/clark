(define (problem mitsubishi-problem)
        (:domain mitsubishi-domain)
        (:objects
            Baxter - agent
            Human - agent
            Hand - manipulator
            BaxterLeft - manipulator
            BaxterRight - manipulator
            RedComponent - thing
            BlueComponent - thing
            GreenComponent - thing
            YellowComponent - thing
            Cleaner - thing
            Solder - thing
            RedTarget - location
            BlueTarget - location
            GreenTarget - location
            YellowTarget - location
            CleanBin  - location
            SolderBin - location
            RedBin  - location
            BlueBin - location
            YellowBin - location
            GreenBin - location)

        (:init
                (at RedComponent RedBin)
                (at RedComponent RedBin)
                (at BlueComponent BlueBin)
                (at GreenComponent GreenBin)
                (at YellowComponent YellowBin)
                (at Solder SolderBin)
                (at Cleaner CleanBin )

                (accessible RedTarget)
                (accessible BlueTarget)
                (accessible GreenTarget)
                (accessible YellowTarget)

                ;(clean RedTarget)
                ;(clean BlueTarget)
                ;(clean GreenTarget)
                ;(clean YellowTarget)

                (clear RedComponent)
                (clear BlueComponent)
                (clear YellowComponent)
                (clear GreenComponent)
                (clear Cleaner)
                (clear Solder)

                (empty Hand)
                (available Hand)
                (part-of Hand Human)
                (empty BaxterLeft)
                (available BaxterLeft)
                (part-of BaxterLeft Baxter)
                (empty BaxterRight)
                (available BaxterRight)
                (part-of BaxterRight Baxter)

                (not-moving Human)
        		    (not-moving Baxter)

                (isHuman Hand)
                (isComponent RedComponent)
                (isComponent BlueComponent)
                (isComponent GreenComponent)
                (isComponent YellowComponent)
                (isCleaner Cleaner)
                (isSolder Solder)

                (reachable RedTarget Hand)
                (reachable BlueTarget Hand)
                (reachable GreenTarget Hand)
                (reachable YellowTarget Hand)
                (reachable CleanBin  Hand)
                (reachable SolderBin Hand)
                (reachable RedBin Hand)
                (reachable BlueBin Hand)
                (reachable YellowBin Hand)
                (reachable GreenBin Hand)

                (reachable RedTarget BaxterLeft)
                (reachable BlueTarget BaxterLeft)
                (reachable GreenTarget BaxterLeft)
                (reachable YellowTarget BaxterLeft)
                (reachable CleanBin  BaxterLeft)
                (reachable SolderBin BaxterLeft)
                ;(reachable RedBin BaxterLeft)
                ;(reachable BlueBin BaxterLeft)
                ;(reachable YellowBin BaxterLeft)
                ;(reachable GreenBin BaxterLeft)

                (reachable RedTarget BaxterRight)
                (reachable BlueTarget BaxterRight)
                (reachable GreenTarget BaxterRight)
                (reachable YellowTarget BaxterRight)
                ;(reachable CleanBin  BaxterRight)
                ;(reachable SolderBin BaxterRight)
                (reachable RedBin BaxterRight)
                (reachable BlueBin BaxterRight)
                (reachable YellowBin BaxterRight)
                (reachable GreenBin BaxterRight)

                (placeable BlueComponent BlueBin)
                (placeable BlueComponent BlueTarget)

                (placeable RedComponent RedBin)
                (placeable RedComponent RedTarget)

                (placeable YellowComponent YellowBin)
                (placeable YellowComponent YellowTarget)

                (placeable GreenComponent GreenBin)
                (placeable GreenComponent GreenTarget)

            )

        (:goal
                (and
                    (at RedComponent RedTarget)
                    (at BlueComponent BlueTarget)
                    ;(at GreenComponent GreenTarget)
                    ;(at YellowComponent YellowTarget)
                    (soldered RedComponent)
                    (soldered BlueComponent)
                    ;(soldered GreenComponent)
                    ;(soldered YellowComponent)
                    )))
