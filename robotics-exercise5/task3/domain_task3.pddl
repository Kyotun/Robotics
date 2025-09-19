(define (domain pyrobosim-pickplace)
  (:requirements :strips :typing)

  (:types
    robot
    location
    receptacle - location
    item
  )

  (:predicates
    (at ?r - robot ?l - location)
    (connected ?from - location ?to - location)
    (obj-at ?o - item ?l - location)
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
    :parameters (?r - robot ?o - item ?l - location)
    :precondition (and (at ?r ?l)
                       (obj-at ?o ?l)
                       (handempty ?r))
    :effect (and (holding ?r ?o)
                 (not (handempty ?r))
                 (not (obj-at ?o ?l)))
  )

  (:action place
    :parameters (?r - robot ?o - item ?l - receptacle)
    :precondition (and (at ?r ?l)
                       (holding ?r ?o))
    :effect (and (obj-at ?o ?l)
                 (handempty ?r)
                 (not (holding ?r ?o)))
  )
)