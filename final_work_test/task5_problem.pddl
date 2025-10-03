(define (problem task5)
  (:domain task5-dynamic)
   (:objects
		alan_turing - robot
		kitchen bedroom0 bathroom bedroom1 - room
		table0 counter0 desk0 charger0 trash_can nav_kitchen nav_bedroom0 nav_bathroom nav_bedroom1 - location
		trash0 food cosmetic laptop trash1 trash2 - item
		   )
   (:init		
		(at alan_turing kitchen)
		(visited kitchen)
		(handempty alan_turing)
		(locationof table0 kitchen)
		(locationof counter0 bathroom)
		(locationof desk0 bedroom0)
		(locationof charger0 bedroom1)
		(locationof trash_can kitchen)
		(locationof nav_kitchen kitchen)
		(locationof nav_bedroom0 bedroom0)
		(locationof nav_bathroom bathroom)
		(locationof nav_bedroom1 bedroom1)
		(on trash0 desk0)
		(on food desk0)
		(on cosmetic table0)
		(on laptop table0)
		(on trash1 charger0)
		(on trash2 table0)
		(connected kitchen bathroom) (connected bathroom kitchen)
		(connected bathroom bedroom0) (connected bedroom0 bathroom)
		(connected kitchen bedroom0) (connected bedroom0 kitchen)
		(connected bedroom0 bedroom1) (connected bedroom1 bedroom0)
		   )
   (:goal		
		(and
			(on trash0 trash_can)
			(on food table0)
			(on cosmetic counter0)
			(on laptop desk0)
			(on trash1 trash_can)
			(on trash2 trash_can)
			)   )
)