(define (problem task3-example)
  (:domain pyrobosim-pickplace)

  (:objects
    r1 - robot
    l_kitchen l_hall - location
    table1 counter1 - receptacle
    apple1 - object
  )

  (:init
    (at r1 l_kitchen)
    (handempty r1)
    (obj-at apple1 table1)

    ;; simple connectivity (bidirectional edges)
    (connected l_kitchen table1)  (connected table1 l_kitchen)
    (connected l_kitchen l_hall)  (connected l_hall l_kitchen)
    (connected l_hall counter1)   (connected counter1 l_hall)
  )

  (:goal
    (and (obj-at apple1 counter1))
  )
)