import dotProp from "dot-prop-immutable";

const initialState = {
  interopIp: "000.00.000.00.0000",
  groundIp: 0,
  username: "",
  password: "",
  lat: 0,
  lng: 0
};

export default {
  reducer: (state = initialState, action) => {
    switch (action.type) {
      case "UPDATE_SETTINGS": {
        return {...state, ...action.payload};
      }
      default: {
        return state;
      }
    }
  }
}
