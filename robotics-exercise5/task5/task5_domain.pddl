(define (domain task5-dynamic)
  (:requirements :strips :typing)

  (:types
    robot
    room         ; navigable locations
    location     ; placeable surfaces (table, counter, shelf...)
    object
  )

  (:predicates
    (at ?r - robot ?rm - room)
    (connected ?from - room ?to - room)
    (visited ?rm - room)

    ; mapping from receptacles to their containing location
    (locationof ?l - location ?rm - room)

    ; object state
    (on ?o - object ?l - location)
    (holding ?r - robot ?o - object)
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
    :parameters (?r - robot ?o - object ?l - location ?rm - room)
    :precondition (and (at ?r ?rm)
                       (locationof ?l ?rm)
                       (on ?o ?l)
                       (handempty ?r))
    :effect (and (holding ?r ?o)
                 (not (handempty ?r))
                 (not (on ?o ?l)))
  )

  (:action put
    :parameters (?r - robot ?o - object ?l - location ?rm - room)
    :precondition (and (at ?r ?rm)
                       (locationof ?l ?rm)
                       (holding ?r ?o))
    :effect (and (on ?o ?l)
                 (handempty ?r)
                 (not (holding ?r ?o)))
  )
)
