(define (problem visit-four-rooms)
  (:domain visit-all)

  (:objects
    r1 - robot
    office1 office2 kitchen bathroom - room
  )

  (:init
    (at r1 office1)          ; 初始在 office1
    (visited office1)        ; 起点默认已访问

    ; 房间拓扑（双向连通，按你的地图改）
    (connected office1 office2)
    (connected office2 office1)
    (connected office2 kitchen)
    (connected kitchen office2)
    (connected office1 bathroom)
    (connected bathroom office1)
  )

  (:goal
    (and
      (visited office1)
      (visited office2)
      (visited kitchen)
      (visited bathroom)
    )
  )
)
