// import dotProp from "dot-prop-immutable";

const initialState = {
  connectedIp: "none",
  interopIp: "000.00.000.00.0000",
  groundIp: 0,
  username: "",
  password: "",
  lat: 0,
  lng: 0
};

export default function reducer(state=initialState, action) {
  switch (action.type) {
    case "UPDATE_SETTINGS": {
      return {...state, ...action.payload};
    }
    case "INTEROP_CONNECTION_SUCCESS": {
      return {...state, connectedIp: action.payload};
    }
    case "INTEROP_CONNECTION_ERROR": {
      return {...state, connectedIp: "none"};
    }
    default: {
      return state;
    }
  }
}
