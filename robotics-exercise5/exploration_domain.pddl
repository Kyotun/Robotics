(define (domain exploration-pickplace)
  (:requirements :strips :typing)

  (:types
    robot
    room
    place
    item
  )

  (:predicates
    (at ?r - robot ?rm - room)
    (connected ?a - room ?b - room)
    (locationof ?p - place ?rm - room)
    (on ?o - item ?p - place)
    (holding ?r - robot ?o - item)
    (handempty ?r - robot)
  )

  (:action move
    :parameters (?r - robot ?from - room ?to - room)
    :precondition (and (at ?r ?from) (connected ?from ?to))
    :effect (and (at ?r ?to) (not (at ?r ?from)))
  )

  (:action pick
    :parameters (?r - robot ?o - item ?p - place ?rm - room)
    :precondition (and (at ?r ?rm)
                       (locationof ?p ?rm)
                       (on ?o ?p)
                       (handempty ?r))
    :effect (and (holding ?r ?o)
                 (not (handempty ?r))
                 (not (on ?o ?p)))
  )

  (:action put
    :parameters (?r - robot ?o - item ?p - place ?rm - room)
    :precondition (and (at ?r ?rm)
                       (locationof ?p ?rm)
                       (holding ?r ?o))
    :effect (and (on ?o ?p)
                 (handempty ?r)
                 (not (holding ?r ?o)))
  )
)