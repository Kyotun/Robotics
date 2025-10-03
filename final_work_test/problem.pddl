(define (problem collect_all_discovered_objects)
  (:domain robot_logistics)

  (:objects
    robot0 - robot
    
  
    cosmetic laptop trash0 trash1 trash2 trash3 food toothbrush  - object

    desk0 counter0 table0 - location 
    kitchen - location      
  	trash_can - location
	charger0 - location
  )

  (:init
    (robot_at robot0 kitchen)
    (hand_empty robot0)

    (object_at food desk0)
    (object_at trash0 desk0)
    (object_at cosmetic table0)
    (object_at laptop table0)
    (object_at trash2 table0)
    (object_at trash1 charger0)    
    
  )

  (:goal (and
    (object_at food table0)
    (object_at trash0 trash_can)
    (object_at trash1 trash_can)
    (object_at trash2 trash_can)
    (object_at cosmetic counter0)
    (object_at laptop desk0)
    
  ))
)