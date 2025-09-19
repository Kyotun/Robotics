(define (problem task3-example)
  (:domain pyrobosim-pickplace)

  (:objects
    my_robot - robot
    kitchen living_room hall bedroom - location
    table1 counter1 - place
    apple1 - item
  )

  (:init
    (at my_robot kitchen)
    (locationof table1 bedroom)
    (locationof counter1 hall)
    (handempty my_robot)
    (on apple1 table1)

    ; bidirectional connectivity
    (connected kitchen living_room)  (connected living_room kitchen)
    (connected kitchen hall)  (connected hall kitchen)
    (connected hall bedroom)  (connected bedroom hall)
    (connected bedroom living_room)  (connected living_room bedroom)
  )

  (:goal
    (and (on apple1 counter1))
  )
)
