(define (domain visit-all)
  (:requirements :strips :typing)

  (:types
    robot
    room
  )

  (:predicates
    (at ?r - robot ?rm - room)
    (connected ?a - room ?b - room)
    (visited ?rm - room)
  )

  (:action move
    :parameters (?r - robot ?from - room ?to - room)
    :precondition (and (at ?r ?from)
                       (connected ?from ?to))
    :effect (and (at ?r ?to)
                 (visited ?to)
                 (not (at ?r ?from)))
  )
)
