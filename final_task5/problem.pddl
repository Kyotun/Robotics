(define (problem collect_all_discovered_objects)
  (:domain robot_logistics)

  (:objects
    robot0 - robot
    
  
    banana0 apple1 apple2 apple0 water1 water0 banana1 - object

    desk0 counter0 table1 - location 
    table0 - location             
    kitchen - location      
  )

  (:init
    (robot_at robot0 kitchen)
    (hand_empty robot0)

    (object_at apple0 desk0)
    (object_at apple1 table0)
    (object_at apple2 table0)
    (object_at water0 counter0)
    (object_at water1 desk0)
    (object_at banana0 table0)
    (object_at banana1 counter0)
  )

  (:goal (and
    (object_at apple0 table0)
    (object_at apple1 table0)
    (object_at apple2 table0)
    (object_at banana0 table0)
    (object_at banana1 table0)
    (object_at water0 table0)
    (object_at water1 table0)
  ))
)