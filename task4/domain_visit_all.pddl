(define (domain visit-all)
  (:requirements :typing :strips)
  (:types robot room)

  (:predicates
    (at ?r - robot ?l - room)     ; 机器人在某房间
    (connected ?a - room ?b - room) ; 房间之间可达（双向自己建）
    (visited ?l - room)           ; 已访问
  )

  (:action move
    :parameters (?r - robot ?from - room ?to - room)
    :precondition (and (at ?r ?from) (connected ?from ?to))
    :effect (and
      (not (at ?r ?from))
      (at ?r ?to)
      (visited ?to)               ; 一到达就标记为已访问
    )
  )
)
