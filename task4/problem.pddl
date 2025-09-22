(define (problem visit-four-rooms)
  (:domain visit-all)

  (:objects
    r1 - robot
    office1 office2 kitchen bathroom - room
  )

  (:init
    (at r1 office1)          
    (visited office1)        

    (connected office1 office2)
    (connected office2 office1)
    (connected office1 bathroom)
    (connected bathroom office1)
    (connected bathroom kitchen)
    (connected kitchen bathroom)

  )

  (:goal
    (and
      (visited office1)
      (visited office2)
      (visited kitchen)
      (visited bathroom)
    )
  )
)
