(define (problem visit-all-example)
  (:domain visit-all)

  (:objects
    my_robot - robot
    kitchen living_room hall bedroom - room
  )

  (:init
    (at my_robot kitchen)
    (visited kitchen)   ; count the start room as already visited

    ; bidirectional connectivity between rooms
    (connected kitchen living_room)  (connected living_room kitchen)
    (connected kitchen hall)         (connected hall kitchen)
  )

  (:goal
    (and
      (visited kitchen)
      (visited living_room)
      (visited hall)
      (visited bedroom)
    )
  )
)
