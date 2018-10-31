export function add(amount) {
	return { type: 'ADD', payload: amount };
}

export function test(data) {
		return {
			type: "TRANSMIT",
			payload: {
				msg: "TEST",
				data: data
			}
		};
}
//TAKEOFF", "LAND", "FAILSAFE", "THROTTLE_CUT"
export const DroneStates = {
  TAKEOFF: 'TAKEOFF',
  LAND: 'LAND',
  FAILSAFE: 'FAILSAFE',
	THROTTLE_CUT: "THROTTLE_CUT"
}

export function change_drone_state(drone_state) {
		if (DroneStates[drone_state]){
			return {
				type: "TRANSMIT",
				payload: {
					msg: "CHANGE_DRONE_STATE",
					data: drone_state
				}
			};
		} else{
			console.log("Invalid Drone state: " + drone_state); //TODO: idk redux that well, should it throw an exception?
		}

}
