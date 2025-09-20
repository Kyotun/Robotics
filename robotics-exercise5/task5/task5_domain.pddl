(define (domain task5-dynamic)
  (:requirements :strips :typing)

  (:types
    robot
    location  ; navigable locations
    placeable     ; placeable surfaces (table, counter, shelf...)
    item
  )

  (:predicates
    (at ?r - robot ?l - location)
    (connected ?from - location ?to - location)
    (visited ?l - location)

    ; mapping from receptacles to their containing location
    (locationof ?p - placeable ?l - location)

    ; object state
    (on ?o - item ?s - placeable)
    (holding ?r - robot ?o - item)
    (handempty ?r - robot)
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from)
                       (connected ?from ?to))
    :effect (and (at ?r ?to)
                 (not (at ?r ?from)))
  )

  (:action pick
    :parameters (?r - robot ?o - item ?p - placeable ?l - location)
    :precondition (and (at ?r ?l)
                       (locationof ?p ?l)
                       (on ?o ?p)
                       (handempty ?r))
    :effect (and (holding ?r ?o)
                 (not (handempty ?r))
                 (not (on ?o ?p)))
  )

  (:action put
    :parameters (?r - robot ?o - item ?p - placeable ?l - location)
    :precondition (and (at ?r ?l)
                       (locationof ?p ?l)
                       (holding ?r ?o))
    :effect (and (on ?o ?p)
                 (handempty ?r)
                 (not (holding ?r ?o)))
  )
)
