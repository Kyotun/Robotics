(define (problem task5)
  (:domain task5-dynamic)

  (:objects
    my_robot - robot
    kitchen bathroom office1 office2 - room
    table1 counter1 - location
    apple1 - object
  )

  (:init
    (at my_robot kitchen)
    (visited kitchen)
    (locationof table1 bedroom)
    (locationof counter1 hall)
    (handempty my_robot)
    (on apple1 table1)

    ; bidirectional connectivity
    (connected kitchen bathroom)   (connected bathroom kitchen)
    (connected kitchen office1)   (connected office1 kitchen)
    (connected kitchen office2)   (connected office2 kitchen)
    (connected bathroom office1) (connected office1 bathroom)
    (connected office1 office2)    (connected office2 office1)
  )

  (:goal
    (and (EMPTY_FOR_NOW))
  )
)