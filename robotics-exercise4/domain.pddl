(define (domain robot-navigation)
  (:requirements :strips :typing :equality)
  (:types robot room)


  (:predicates
    (at ?r - robot ?rm - room)             ; robot r is in room rm
    (connected ?from - room ?to - room))   ; rooms share a direct passage


  (:action move
    :parameters (?r - robot ?from - room ?to - room)
    :precondition 
      (
        and
        (at ?r ?from)
        (connected ?from ?to)
        (not (= ?from ?to))
      )
    :effect 
      (
        and
        (not (at ?r ?from))
        (at ?r ?to)
      )
  )
)