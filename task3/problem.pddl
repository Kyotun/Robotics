(define (problem move-mug-to-kitchen)
  (:domain pyrobosim-min)

  (:objects
    r1 - robot
    office1 hall kitchen - location
    mug0 - item
  )

  (:init
    (at r1 office1)
    (handempty r1)
    (at-obj mug0 office1)
    (graspable mug0)

    (surface office1)
    (surface kitchen)

    (connected office1 hall)
    (connected hall office1)
    (connected hall kitchen)
    (connected kitchen hall)
  )

  (:goal
    (and (at-obj mug0 kitchen))
  )
)
