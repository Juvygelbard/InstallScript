(define (domain service_robot)
(:requirements :adl :durative-actions)
(:types
	obj
)
(:predicates 
				(robot_at ?loc - obj)
				(robot_at_zone ?zone - obj)
				(loc_in_zone ?loc - obj ?zone - obj)
				(K_BUTTON_LOC ?button - obj)
				(K_CAN_LOC)
				(has_door_button ?loc - obj ?door - obj)
				(grabbed_can)
				(arm_free)
				(open_door ?door - obj)
				(door_entrance ?door - obj ?zone - obj ?loc - obj)
				(has_coke_stand ?loc - obj)
				(coke_at ?loc - obj)
)
(:durative-action move
:parameters (?curr_loc - obj ?loc - obj ?curr_zone - obj)
:duration ( = ?duration 10)
:condition (and (at start (robot_at ?curr_loc)) (over all (robot_at_zone ?curr_zone)) 
					(over all (loc_in_zone ?loc ?curr_zone)))
:effect (and (at start (not (robot_at ?curr_loc))) (at end (robot_at ?loc)) (forall (?e - obj) (at end (not (K_BUTTON_LOC ?e)))) (at end (not (K_CAN_LOC)))))

(:durative-action observe_door_button
:parameters (?curr_loc - obj ?door - obj)
:duration ( = ?duration 10)
:condition (and (over all (robot_at ?curr_loc)) (over all (has_door_button ?curr_loc ?door)))
:effect (at end (K_BUTTON_LOC ?door)))

(:durative-action press_door_button
:parameters (?curr_loc - obj ?door - obj)
:duration ( = ?duration 10)
:condition (and (over all (arm_free)) (over all (robot_at ?curr_loc)) (over all (has_door_button ?curr_loc ?door)) (at start (K_BUTTON_LOC ?door)))
:effect (and (at end (not (K_BUTTON_LOC ?door))) (at end (open_door ?door))))

(:durative-action move_through_door
:parameters (?door - obj ?curr_loc - obj ?curr_zone - obj ?goal_loc - obj ?goal_zone - obj)
:duration ( = ?duration 10)
:condition (and (at start (has_door_button ?curr_loc ?door)) (at start (robot_at ?curr_loc)) (over all (robot_at_zone ?curr_zone)) (over all (open_door ?door)) (over all (door_entrance ?door ?goal_zone ?goal_loc)))
:effect (and (at end (not (robot_at ?curr_loc))) (at end (not (robot_at_zone ?curr_zone))) (at end (robot_at_zone ?goal_zone)) (at end (robot_at ?goal_loc))))

(:durative-action order_coke
:parameters (?curr_loc - obj)
:duration ( = ?duration 10)
:condition (and (over all (robot_at ?curr_loc)) (over all (has_coke_stand ?curr_loc)))
:effect (at end (coke_at ?curr_loc)))

(:durative-action observe_coke_can
:parameters (?curr_loc - obj)
:duration ( = ?duration 10)
:condition (and (over all (robot_at ?curr_loc)) (over all (coke_at ?curr_loc)))
:effect (at end (K_CAN_LOC)))

(:durative-action grab_coke_can
:parameters (?curr_loc - obj)
:duration ( = ?duration 10)
:condition (and (over all (robot_at ?curr_loc)) (over all (coke_at ?curr_loc)) (over all (K_CAN_LOC)))
:effect (and (at end (grabbed_can)) (at end (not (arm_free))) (at end (not (coke_at ?curr_loc))) (at end (not (K_CAN_LOC)))))

(:durative-action request_open_door
:parameters (?curr_loc - obj ?door - obj)
:duration ( = ?duration 10)
:condition (and (over all (grabbed_can)) (over all (robot_at ?curr_loc)) (over all (has_door_button ?curr_loc ?door)))
:effect (at end (open_door ?door)))

(:durative-action serve_can
:parameters (?curr_loc - obj)
:duration ( = ?duration 10)
:condition (and (over all (robot_at ?curr_loc)) (over all (grabbed_can)))
:effect (and (at end (coke_at ?curr_loc)) (at end (not (grabbed_can))) (at end (arm_free))))
)