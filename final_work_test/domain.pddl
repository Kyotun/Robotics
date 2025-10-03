(define (domain robot_logistics)
  (:requirements :typing)

  (:types
    robot location - object
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (object_at ?o - object ?l - location)
    (holding ?r - robot ?o - object)
    (hand_empty ?r - robot)
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (robot_at ?r ?from)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )

  (:action pick
    :parameters (?r - robot ?o - object ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (object_at ?o ?l)
      (hand_empty ?r)
    )
    :effect (and
      (not (object_at ?o ?l))
      (not (hand_empty ?r))
      (holding ?r ?o)
    )
  )

  (:action place
    :parameters (?r - robot ?o - object ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (holding ?r ?o)
    )
    :effect (and
      (not (holding ?r ?o))
      (hand_empty ?r)
      (object_at ?o ?l)
    )
  )
)