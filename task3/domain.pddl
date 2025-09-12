(define (domain pyrobosim-min)
  (:requirements :typing :strips)
  (:types robot location item)

  (:predicates
    (at ?r - robot ?l - location)
    (at-obj ?o - item ?l - location)
    (connected ?from - location ?to - location)
    (handempty ?r - robot)
    (holding ?r - robot ?o - item)
    (surface ?l - location)
    (graspable ?o - item)
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from) (connected ?from ?to))
    :effect (and
      (not (at ?r ?from))
      (at ?r ?to)
    )
  )

  (:action pick
    :parameters (?r - robot ?o - item ?l - location)
    :precondition (and
      (at ?r ?l)
      (at-obj ?o ?l)
      (handempty ?r)
      (graspable ?o)
    )
    :effect (and
      (holding ?r ?o)
      (not (handempty ?r))
      (not (at-obj ?o ?l))
    )
  )

  (:action place
    :parameters (?r - robot ?o - item ?l - location)
    :precondition (and
      (at ?r ?l)
      (holding ?r ?o)
      (surface ?l)
    )
    :effect (and
      (at-obj ?o ?l)
      (handempty ?r)
      (not (holding ?r ?o))
    )
  )
)
