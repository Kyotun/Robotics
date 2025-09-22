(define (domain visit-all)
  (:requirements :typing :strips)
  (:types robot room)

  (:predicates
    (at ?r - robot ?l - room)    
    (connected ?a - room ?b - room) 
    (visited ?l - room)          
  )

  (:action move
    :parameters (?r - robot ?from - room ?to - room)
    :precondition (and (at ?r ?from) (connected ?from ?to))
    :effect (and
      (not (at ?r ?from))
      (at ?r ?to)
      (visited ?to)              
    )
  )
)
