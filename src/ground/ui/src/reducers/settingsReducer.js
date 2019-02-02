import dotProp from "dot-prop-immutable";

const initialState = {
  ip: "000.00.000.00.0000",
  gip: 0,
  un: "",
  pw: "",
  lat: 0,
  long: 0
};

export default function reducer(state = initialState, action) {
  switch (action.type) {
    case "UPDATE_SETTINGS": {
      return action.payload;
    }
    default: {
      return state;
    }
  }
}
