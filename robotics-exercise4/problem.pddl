(define (problem move-example)
  (:domain robot-navigation)

  (:objects
    cool_robot - robot
    roomA roomB roomC roomD - room)


  (:init
    (at cool_robot roomA)

    ;; bidirectional corridor map
    (connected roomA roomB)  (connected roomB roomA)
    (connected roomB roomD)  (connected roomD roomB)
    (connected roomA roomC)  (connected roomC roomA)
    (connected roomC roomD)  (connected roomD roomC))


  (:goal
    (at cool_robot roomD)
  )
)