(define (domain task5-dynamic)
  (:requirements :strips :typing)

  (:types
    robot
    room         ; navigable locations
    location     ; placeable surfaces (table, counter, shelf...)
    item
  )

  (:predicates
    (at ?r - robot ?rm - room)
    (connected ?from - room ?to - room)
    (visited ?rm - room)

    ; mapping from receptacles to their containing location
    (locationof ?l - location ?rm - room)

    ; item state
    (on ?i - item ?l - location)
    (holding ?r - robot ?i - item)
    (handempty ?r - robot)
  )

  (:action move
    :parameters (?r - robot ?from - room ?to - room)
    :precondition (and (at ?r ?from)
                       (connected ?from ?to))
    :effect (and (at ?r ?to)
                 (not (at ?r ?from)))
  )

  (:action pick
    :parameters (?r - robot ?i - item ?l - location ?rm - room)
    :precondition (and (at ?r ?rm)
                       (locationof ?l ?rm)
                       (on ?i ?l)
                       (handempty ?r))
    :effect (and (holding ?r ?i)
                 (not (handempty ?r))
                 (not (on ?i ?l)))
  )

  (:action place
    :parameters (?r - robot ?i - item ?l - location ?rm - room)
    :precondition (and (at ?r ?rm)
                       (locationof ?l ?rm)
                       (holding ?r ?i))
    :effect (and (on ?i ?l)
                 (handempty ?r)
                 (not (holding ?r ?i)))
  )
)
