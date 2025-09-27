(define (problem visit-all-example)
  (:domain visit-all)

  (:objects
    my_robot - robot
    kitchen office1 office2 bathroom - room
  )

  (:init
    (at my_robot kitchen)
    (visited kitchen)   ; count the start room as already visited

    ; bidirectional connectivity between rooms
    (connected kitchen office1)  (connected office1 kitchen)
    (connected kitchen office2)  (connected office2 kitchen)
    (connected office1 bathroom) (connected bathroom office1)
  )

  (:goal
    (and
      (visited kitchen)
      (visited office1)
      (visited office2)
      (visited bathroom)
    )
  )
)
