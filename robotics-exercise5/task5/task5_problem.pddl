(define (problem task5)
  (:domain task5-dynamic)
   (:objects
		alan_turing - robot
		kitchen office1 bathroom office2 - room
		table0 counter0 desk0 charger0 trash nav_kitchen nav_office1 nav_bathroom nav_office2 - location
		apple0 water_bottle0 soda0 apple1 - item
		   )
   (:init		
		(at alan_turing office2)
		(visited office2)
		(handempty alan_turing)
		(locationof table0 kitchen)
		(locationof counter0 bathroom)
		(locationof desk0 office1)
		(locationof charger0 office2)
		(locationof trash kitchen)
		(locationof nav_kitchen kitchen)
		(locationof nav_office1 office1)
		(locationof nav_bathroom bathroom)
		(locationof nav_office2 office2)
		(on apple0 desk0)
		(on water_bottle0 desk0)
		(on soda0 desk0)
		(on apple1 trash)
		(connected kitchen bathroom) (connected bathroom kitchen)
		(connected bathroom office1) (connected office1 bathroom)
		(connected kitchen office1) (connected office1 kitchen)
		(connected office1 office2) (connected office2 office1)
		   )
   (:goal		
		(and
			(on apple0 nav_office2)
			(on water_bottle0 nav_office2)
			(on soda0 nav_office2)
			(on apple1 nav_office2)
			)   )
)